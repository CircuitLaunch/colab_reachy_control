#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from DXLProxy import *
from colab_reachy_control.msg import Telemetry
from threading import Lock

from colab_reachy_control.srv import Telemetry, Grasp

class CommandServer:
    def __init__(self):
        self.telemDict = {}
        self.telemLock = Lock()
        self.telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.telemetryCallback)

        self.recoverServer = rospy.ServiceProxy('recover', Recover, self.recoverCallback)
        self.graspServer = rospy.ServiceProxy('grasp', Grasp, self.graspCallback)
        self.restServer = rospy.ServiceProxy('rest', String, self.restCallback)
        self.relaxServer = rospy.ServiceProxy('relax', String, self.relaxCallback)

        self.dxlProxy = DXLProxy()

    def recoverCallback(self, recoverMsg):
        resp = RecoverResponse()
        resp.result = 'failure'
        for id in recoverMsg.dxl_ids:
            self.dxlProxy.writeRegisters([id, id, id], [RAM_TORQUE_ENABLE, RAM_TORQUE_LIMIT, RAM_TORQUE_ENABLE], [0, 1023, 1])
        resp.result = 'succes'
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
        while(gripping && (rospy.Time.now() - startTim)e < rospy.Duration(graspMsg.timeout)):
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

    def restCallback(self, restMsg):
        side = restMsg.data

    def relaxCallback(self, relaxMsg):
        side = relaxMsg.data

    def telemetryCallback(self, telemetry):
        for i, id in enumerate(telemetry.dxl_ids):
            torqueLimit = telemetry.torque_limits[i]
            presentSpeed = telemetry.present_speed[i]
            presentLoad = telemetry.present_load[i]
            presentVoltage = telemetry.present_voltage[i]
            presentTemperature = telementry.present_temperature[i]
            errorBits = telemetry.error_bits[i]
            with self.telemLock:
                telemDict[id] = {
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

autoRecover = AutoRecover()

def main():
    rospy.init_node("reachy_action_server")
    spinRate = rospy.Rate(30)

    while not rospy.is_shutdown():
        spinRate.sleep();

if __name__ == '__main__':
    main()
