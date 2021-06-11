#!/usr/bin/env python3

import rospy
from DXLProxy import *
from colab_reachy_control.msg import Telemetry

class AutoRecover:
    def __init__(self):
        self.telemSub = rospy.Subscriber('dxl_telemetry', Telemetry, self.telemetryCallback)
        self.dxlProxy = DXLProxy()

    def telemetryCallback(self, telemetry):
        for i, id in enumerate(telemetry.dxl_ids):
            torqueLimit = telemetry.torque_limits[i]
            errorBits = telemetry.error_bits[i]
            if errorBits != 0:
                rospy.loginfo(f'Error on servo {id}; attempting to recover')
                self.dxlProxy.writeRegisters([id, id, id], [RAM_TORQUE_ENABLE, RAM_TORQUE_LIMIT, RAM_TORQUE_ENABLE], [0, 1023, 1])

autoRecover = AutoRecover()

def main():
    rospy.init_node("reachy_action_server")
    spinRate = rospy.Rate(30)

    while not rospy.is_shutdown():
        spinRate.sleep();

if __name__ == '__main__':
    main()
