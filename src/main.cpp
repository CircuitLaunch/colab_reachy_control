#include "ros/ros.h"
#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <deque>
#include <vector>
#include "DXL.hpp"
#include "Interpolator.hpp"
#include "DXLCommandServer.hpp"
#include "DXLTelemetryPublisher.hpp"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpolator");

  ros::NodeHandle n;

  vector<string> rightArmJointNames;
  rightArmJointNames.push_back("r_shoulder_pitch");
  rightArmJointNames.push_back("r_shoulder_roll");
  rightArmJointNames.push_back("r_arm_yaw");
  rightArmJointNames.push_back("r_elbow_pitch");
  rightArmJointNames.push_back("r_forearm_yaw");
  rightArmJointNames.push_back("r_wrist_pitch");
  rightArmJointNames.push_back("r_wrist_roll");
  rightArmJointNames.push_back("r_gripper");

  vector<string> leftArmJointNames;
  leftArmJointNames.push_back("l_shoulder_pitch");
  leftArmJointNames.push_back("l_shoulder_roll");
  leftArmJointNames.push_back("l_arm_yaw");
  leftArmJointNames.push_back("l_elbow_pitch");
  leftArmJointNames.push_back("l_forearm_yaw");
  leftArmJointNames.push_back("l_wrist_pitch");
  leftArmJointNames.push_back("l_wrist_roll");
  leftArmJointNames.push_back("l_gripper");

  initKludgyDXLDict();

  string deviceName;
  if(!n.getParam("/interpolator/device", deviceName)) deviceName = "/dev/serial/by-id/usb-Xevelabs_USB2AX_74031303437351011190-if00";

  try {
    DXLPort port(deviceName);

    if(!port.isValidConnection()) {
      ROS_INFO("Could not connect to Dynamixel control hardware at %s; using a dummy interface.", deviceName.c_str());
    } else {
      ROS_INFO("Connected successfully to Dynamixel control hardware at %s", deviceName.c_str());
    }

    Interpolator rai(port, n, string("right_arm_controller/joint_states"), string("action_server/right_arm_trajectory"), rightArmJointNames, 10);
    Interpolator lai(port, n, string("left_arm_controller/joint_states"), string("action_server/left_arm_trajectory"), leftArmJointNames, 10);

    DXLCommandServer cmdSrv(port, n);
    DXLTelemetryPublisher telPub(port, n);

    ros::Timer rightTimer = n.createTimer(ros::Duration(1.0 / REPORT_HZ), &Interpolator::timerTick, &rai);
    ros::Timer leftTimer = n.createTimer(ros::Duration(1.0 / REPORT_HZ), &Interpolator::timerTick, &lai);
    ros::Timer telTimer = n.createTimer(ros::Duration(1.0/ REPORT_HZ), &DXLTelemetryPublisher::timerTick, &telPub);

    ros::AsyncSpinner spinner(5);
    spinner.start();
    ros::waitForShutdown();

  } catch (exception &e) {
    ROS_FATAL("Failed to connect to reachy %s", e.what());
    exit(-1);
  }
}
