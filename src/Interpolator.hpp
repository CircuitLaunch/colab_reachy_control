#ifndef __INTERPOLATOR_HPP__
#define __INTERPOLATOR_HPP__

#include "ros/ros.h"
#include <colab_reachy_control/WriteRegisters.h>
#include <colab_reachy_control/ReadRegisters.h>
#include "DXL.hpp"
#include <sensor_msgs/JointState.h>
#include <colab_reachy_control/Trajectory.h>

using namespace std;

#define DEBUG 0
#define PASSTHROUGH 0
#define CONTROL_HZ 30.0
#define REPORT_HZ 30.0

class Interpolator : DXLErrorHandler
{
  public:
    Interpolator(DXLPort &iPort, ros::NodeHandle &iNH, const string &iJointStateTopic, const string &iTrajectoryService, vector<string> &iJointNames, uint32_t iQueueSize = 10);
    virtual ~Interpolator();

    bool trajectoryService(colab_reachy_control::Trajectory::Request &iReq, colab_reachy_control::Trajectory::Response &iRes);

    void drive(vector<int> &dxl_ids, float *ioJointState);
    vector<double> getJointState() const;
    void remapJointState(vector<int> &dxl_ids, float *iJointState);

    void timerTick(const ros::TimerEvent &iEvent) const;

  protected:
    virtual void handleError(DXL &iDXL, int iCommResult, uint8_t iErrorStatus, const string &iErrorMsg) const;

  public:
    DXLPort &port;
    ros::NodeHandle &nh;
    ros::Publisher jointStatePub;
    ros::ServiceServer trajectorySrv;

    vector<string> jointNames;
    unordered_map<string, int> jointDict;

    float passThroughJointState[8];
};

void initKludgyDXLDict();

extern unordered_map<string, int> kludgyDXLDict;

#endif
