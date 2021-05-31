#include "ros/ros.h"
#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <deque>
#include <vector>
#include <colab_reachy_control/Trajectory.h>
#include <colab_reachy_control/TrajectoryService.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "BSpline/BSpline.hpp"
#include "BSpline/Parametizer.hpp"

using namespace std;
using namespace sensor_msgs;

#define DEBUG 0
#define PASSTHROUGH 1
#define CONTROL_HZ 30.0
#define REPORT_HZ 30.0

#define TO_DEGREES(r) (r * 57.295779513)

class Interpolator
{
  public:
    Interpolator(ros::NodeHandle &iNH, const string &iJointStateTopic, const string &iTrajectoryTopic, vector<string> &iJointNames, uint32_t iQueueSize = 10);

    bool trajectoryService(colab_reachy_control::TrajectoryService::Request &iReq, colab_reachy_control::TrajectoryService::Response &iRes);

    void drive(vector<int> &dxl_ids, float *ioJointState);
    vector<float> getJointState();
    void remapJointState(vector<int> &dxl_ids, float *iJointState);

    void timerTick(const ros::TimerEvent &iEvent) const;

  public:
    ros::NodeHandle &nh;
    ros::Publisher jointStatePub;
    ros::ServiceServer trajectorySrv;

    vector<string> jointNames;

    float passThroughJointState[8];
};

Interpolator::Interpolator(ros::NodeHandle &iNH, const string &iJointStateTopic, const string &iTrajectoryTopic, vector<string> &iJointNames, uint32_t iQueueSize)
: nh(iNH),
  jointStatePub(nh.advertise<JointState>(iJointStateTopic, iQueueSize)),
  trajectorySrv(nh.advertiseService<Interpolator, colab_reachy_control::TrajectoryService::Request, colab_reachy_control::TrajectoryService::Response>(iTrajectoryTopic, &Interpolator::trajectoryService, this)),
  jointNames(iJointNames)
{
  int i = 8;
  while(i--)
    passThroughJointState[i] = 0.0;
}

bool Interpolator::trajectoryService(colab_reachy_control::TrajectoryService::Request &iReq, colab_reachy_control::TrajectoryService::Response &iRes)
{
  #ifdef DEBUG
  ROS_INFO("trajectoryService");
  #endif

  int jointCount = iReq.dxl_ids.size();
  float goalTimeTol = iReq.goal_time_tolerance.toSec();
  float duration = iReq.time_from_start.toSec();
  int controlPointCount = iReq.control_points.size() / jointCount;
  float knots[controlPointCount + 4];

  #if DEBUG
  {
    ostringstream ss;
    bool first = true;
    for(auto id : iReq.dxl_ids) {
      if(!first) ss << ", ";
      ss << id;
      first = false;
    }
    ROS_INFO("Trajectory ids: %s", ss.str().c_str());
  }

  {
    ROS_INFO("Trajectory:");
    for(int j = 0; j < controlPointCount; j++) {
      ostringstream ss;
      for(int i = 0; i < jointCount; i++) {
        ss << TO_DEGREES(iReq.control_points[j * jointCount + i]) << ", ";
      }
      ROS_INFO(ss.str().c_str());
    }
  }
  #endif

  BSpline spline(&iReq.control_points[0], knots, controlPointCount);
  spline.init(jointCount, controlPointCount);
  Parametizer parametizer(spline);
  parametizer.init();

  int divisions = int(duration * CONTROL_HZ);
  vector<float> parametization = parametizer.parametizeSigmoidal(divisions);

  #if DEBUG
  {
    ostringstream ss;
    bool first = true;
    for(auto time : parametization) {
      if(!first) ss << ", ";
      ss << time;
      first = false;
    }
    ROS_INFO("Parametization: %s", ss.str().c_str());
  }
  #endif

  int paramIndex = 0;
  ros::Time startTime = ros::Time::now();
  bool interpolating = true;

  float jointState[8];
  ros::Rate spinRate(int(CONTROL_HZ));
  while(interpolating) {
    double elapsed = (ros::Time::now() - startTime).toSec();
    int newParamIndex = int(elapsed * CONTROL_HZ);
    if(newParamIndex < divisions) {
      if(newParamIndex > paramIndex) {
        float currentParam = parametization[newParamIndex];
        spline.eval(currentParam, jointState);
        #if PASSTHROUGH
        remapJointState(iReq.dxl_ids, jointState);
        #else
        drive(iReq.dxl_ids, jointState);
        #endif
      }
    } else {
      interpolating = false;
    }
    spinRate.sleep();
  }

  iRes.result = string("SUCCESS");

  return true;
}

void Interpolator::drive(vector<int> &dxl_ids, float *iJointState)
{
}

vector<float> Interpolator::getJointState()
{
  vector<float> jointState;
  return jointState;
}

void Interpolator::remapJointState(vector<int> &dxl_ids, float *iJointState)
{
  for(int src = 0; src < dxl_ids.size(); src++) {
    int dst = dxl_ids[src] - ((dxl_ids[src] < 20) ? 10 : 20);
    passThroughJointState[dst] = iJointState[src];
  }
}

void Interpolator::timerTick(const ros::TimerEvent &iEvent) const
{
  JointState jntState;
  jntState.header.stamp = ros::Time::now();
  jntState.name = jointNames;
  #if PASSTHROUGH
  jntState.position = vector<double>(passThroughJointState, passThroughJointState + 8);
  #else
  jntState.position = getJointState();
  #endif
  jointStatePub.publish(jntState);
}

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

  Interpolator rai(n, string("right_arm_controller/joint_states"), string("action_server/right_arm_trajectory"), rightArmJointNames, 10);
  Interpolator lai(n, string("left_arm_controller/joint_states"), string("action_server/left_arm_trajectory"), leftArmJointNames, 10);

  ros::Timer rightTimer = n.createTimer(ros::Duration(1.0 / REPORT_HZ), &Interpolator::timerTick, &rai);
  ros::Timer leftTimer = n.createTimer(ros::Duration(1.0 / REPORT_HZ), &Interpolator::timerTick, &lai);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}
