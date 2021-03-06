#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from DXLProxy import *
from colab_reachy_control.msg import Telemetry
from threading import Lock
import time

from colab_reachy_control.srv import Grasp, GraspResponse, SetGripperPos, SetGripperPosResponse, Recover, RecoverResponse, Relax, RelaxResponse, RestPose, RestPoseResponse, Zero, ZeroResponse

class CommandServer:
    def __init__(self):
        self.telemDict = {}
        self.telemLock = Lock()
        self.telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.telemetryCallback)

        self.zeroServer = rospy.Service('zero', Zero, self.zero)
        self.recoverServer = rospy.Service('recover', Recover, self.recoverCallback)
        self.gripperServer = rospy.Service('set_gripper_pos', SetGripperPos, self.setGripperPosCallback)
        self.graspServer = rospy.Service('grasp', Grasp, self.graspCallback)
        self.relaxServer = rospy.Service('relax', Relax, self.relaxCallback)
        self.restPoseServer = rospy.Service('rest_pose', RestPose, self.restPoseCallback)

        self.enableRighArmJointStateTelem = rospy.ServiceProxy('right_arm_controller/enable_joint_state_telem', SetBool)
        self.enableLeftArmJointStateTelem = rospy.ServiceProxy('left_arm_controller/enable_joint_state_telem', SetBool)
        self.enableExtraTelem = rospy.ServiceProxy('colab_reachy_control/enable_extra_telem', SetBool)

        self.dxlProxy = DXLProxy()

    def zero(self, zeroMsg):
        resp = ZeroResponse()
        side = zeroMsg.side
        resp.result = 'failure'
        ids = [10, 11, 12, 13, 14, 15, 16]
        if side == 'left':
            ids = [20, 21, 22, 23, 24, 25, 26]
        self.dxlProxy.writeRegisters(ids, [RAM_GOAL_POSITION] * 7, [0] * 7)
        resp.result = 'success'
        return resp

    def recoverCallback(self, recoverMsg):
        if recoverMsg.dxl_ids[0] < 20:
            self.enableRightArmJointStateTelem(False)
        else:
            self.enableLeftArmJointStateTelem(False)
        self.enableExtraTelem(False)
        resp = RecoverResponse()
        resp.result = 'failure'
        idCount = len(recoverMsg.dxl_ids)
        rospy.Rate(1)
        self.dxlProxy.writeRegisters([recoverMsg.dxl_ids], [RAM_TORQUE_ENABLE] * idCount, [0] * idCount)
        rospy.sleep()
        self.dxlProxy.writeRegisters([recoverMsg.dxl_ids], [RAM_TORQUE_LIMIT] * idCount, [1023] * idCount)
        rospy.sleep()
        self.dxlProxy.writeRegisters([recoverMsg.dxl_ids], [RAM_TORQUE_ENABLE] * idCount, [1] * idCount)
        rospy.sleep()
        resp.result = 'success'
        for id in recoverMsg.dxl_ids:
            if self.telemDict[id]['error_bits'] != 0:
                resp.result = 'failure'
                return resp
        self.enableExtraTelem(True)
        if recoverMsg.dxl_ids[0] < 20:
            self.enableRightArmJointStateTelem(True)
        else:
            self.enableLeftArmJointStateTelem(True)
        return resp

    def setGripperPosCallback(self, gripperPosMsg):
        side = gripperPosMsg.side
        angle = gripperPosMsg.angle
        id = 17 if side == 'right' else 27
        self.dxlProxy.writeRegisters([id], [RAM_GOAL_POSITION], [angle])
        resp = SetGripperPosResponse()
        resp.result = 'success'
        return resp

    def graspCallback(self, graspMsg):
        side = graspMsg.side
        gripperId = 17
        if side == 'left':
            gripperId = 27
        targetLoad = graspMsg.target_load * 1023.0
        gripperSpeed = graspMsg.gripper_speed / 30.0
        rate = rospy.Rate(30)
        gripping = True
        resp = GraspResponse()
        startTime = rospy.Time.now()
        if side == 'left':
            presentPosition = 0.7
        else:
            presentPosition = -0.7
            gripperSpeed = -gripperSpeed

        self.dxlProxy.writeRegisters([gripperId, gripperId], [RAM_MOVING_SPEED, RAM_GOAL_POSITION],  [0.2, presentPosition])
        
        while(gripping and (rospy.Time.now() - startTime) < rospy.Duration(graspMsg.timeout)):
            # posArray = self.dxlProxy.readRegisters([gripperId], [RAM_PRESENT_POSITION])
            # presentPosition = posArray[0]
            # print(presentPosition)
            with self.telemLock:
                presentLoad = self.telemDict[gripperId]['present_load']
                presentVel = self.telemDict[gripperId]['present_speed']
            print(f"present load : {presentLoad}, present vel: {presentVel}")
            resp.present_load = presentLoad
            resp.result = "failure"
            if presentVel == 0 and presentLoad > targetLoad:
                gripping = False
                resp.result = "success"
            else:
                self.dxlProxy.writeRegisters([gripperId], [RAM_GOAL_POSITION], [presentPosition + gripperSpeed])
            rate.sleep()
        return resp

    def restPoseCallback(self, restPoseMsg):
        side = restPoseMsg.side
        speed = restPoseMsg.speed
        ids = [id + (20 if (side == 'left') else 10) for id in range(0, 7)]
        # set speed to 0.2 revolutions per second
        cmds = [RAM_MOVING_SPEED] * len(ids)
        vals = [speed] * 8
        self.dxlProxy.writeRegisters(ids, cmds, vals)

        cmds = [RAM_GOAL_POSITION] * len(ids)
        vals = [0.0] * 8
        self.dxlProxy.writeRegisters(ids, cmds, vals)

        startTime = rospy.get_time()
        cmds = [RAM_PRESENT_POSITION] * len(ids)
        atRest = False
        rate = rospy.Rate(1)
        while(not atRest):
            atRest = True
            vals, results, errors = self.dxlProxy.readRegisters(ids, cmds)
            for val in vals:
                if abs(val) > 0.004363323129986:
                    atRest = False
            rate.sleep()
            if rospy.get_time() - startTime > 60.0:
                atRest = True

        time.sleep(5.0)

        cmds = [RAM_MOVING_SPEED] * len(ids)
        self.dxlProxy.writeRegisters(ids, cmds, vals)

        resp = RestPoseResponse()
        resp.result = "success"
        return resp

    def relaxCallback(self, relaxMsg):
        side = relaxMsg.side
        ids = [id + (20 if (side == "left") else 10) for id in range(0, 7)]
        cmds = [RAM_TORQUE_ENABLE] * len(ids)
        vals = [0] * len(ids)
        self.dxlProxy.writeRegisters(ids, cmds, vals)
        resp = RelaxResponse()
        resp.result = "success"
        return resp

    def telemetryCallback(self, telemetry):
        for i, id in enumerate(telemetry.dxl_ids):
            torqueLimit = telemetry.torque_limits[i]
            presentSpeed = telemetry.present_speeds[i]
            presentLoad = telemetry.present_loads[i]
            presentVoltage = telemetry.present_voltages[i]
            presentTemperature = telemetry.present_temperatures[i]
            errorBits = telemetry.error_bits[i]
            with self.telemLock:
                self.telemDict[id] = {
                    'torque_limit': torqueLimit,
                    'present_speed': presentSpeed,
                    'present_load': presentLoad,
                    'present_voltage': presentVoltage,
                    'present_temperature': presentTemperature,
                    'error_bits': errorBits }
            '''
            if errorBits != 0:
                rospy.loginfo(f'Error on servo {id}; attempting to recover')
                self.dxlProxy.writeRegisters([id, id, id], [RAM_TORQUE_ENABLE, RAM_TORQUE_LIMIT, RAM_TORQUE_ENABLE], [0, 1023, 1])
            '''

commandServer = CommandServer()

def main():
    rospy.init_node("reachy_action_server")
    spinRate = rospy.Rate(30)

    while not rospy.is_shutdown():
        spinRate.sleep();

if __name__ == '__main__':
    main()
