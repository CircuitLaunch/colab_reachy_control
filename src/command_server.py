#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from DXLProxy import *
from colab_reachy_control.msg import Telemetry
from threading import Lock

from colab_reachy_control.srv import Grasp, GraspResponse, Recover, RecoverResponse, Relax, RelaxResponse, Zero, ZeroResponse

class CommandServer:
    def __init__(self):
        self.telemDict = {}
        self.telemLock = Lock()
        self.telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.telemetryCallback)

        self.zeroServer = rospy.Service('zero', Zero, self.zero)
        self.recoverServer = rospy.Service('recover', Recover, self.recoverCallback)
        self.graspServer = rospy.Service('grasp', Grasp, self.graspCallback)
        self.relaxServer = rospy.Service('relax', Relax, self.relaxCallback)

        self.enableRighArmJointStateTelem = rospy.ServiceProxy('right_arm_controller/enable_joint_state_telem', SetBool)
        self.enableLeftArmJointStateTelem = rospy.ServiceProxy('left_arm_controller/enable_joint_state_telem', SetBool)
        self.enableExtraTelem = rospy.ServiceProxy('colab_reachy_control/enable_extra_telem', SetBool)

        self.dxlProxy = DXLProxy()

    def zero(self, zeroMsg):
        resp = ZeroResponse()
        side = zeroMsg.side
        resp.result = 'failure'
        ids = [10, 11, 12, 13, 14, 15, 16, 17]
        if side == 'left':
            ids = [20, 21, 22, 23, 24, 25, 26, 27]
        self.dxlProxy.writeRegisters(ids, [RAM_GOAL_POSITION] * 8, [0] * 8)
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

    def graspCallback(self, graspMsg):
        side = graspMsg.side
        gripperId = 18
        if side > 20:
            gripperId = 28
        targetLoad = graspMsg.target_load * 1023.0
        gripperSpeed = graspMsg.gripper_speed / 30.0
        rate = rospy.Rate(30);
        gripping = true
        resp = GraspResponse()
        startTime = rospy.Time.now()
        while(gripping and (rospy.Time.now() - startTime) < rospy.Duration(graspMsg.timeout)):
            with self.telemLock:
                presentPosition = self.telemDict[gripperId]['present_position']
                presentLoad = self.telemDict[gripperId]['present_load']
            resp.present_load = presentLoad
            resp.result = "failure"
            if presentLoad > targetLoad:
                gripping = false
                resp.result = "success"
            else:
                self.dxlProxy.writeRegisters([gripperId], [RAM_GOAL_POSITION], [presentPosition + gripperSpeed])
            rate.sleep()
        return resp

    def relaxCallback(self, relaxMsg):
        side = relaxMsg.side
        ids = [id + (20 if (side == "left") else 10) for id in range(0, 9)]
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
