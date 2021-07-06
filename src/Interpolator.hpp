#ifndef __INTERPOLATOR_HPP__
#define __INTERPOLATOR_HPP__

#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <colab_reachy_control/WriteRegisters.h>
#include <colab_reachy_control/ReadRegisters.h>
#include "DXL.hpp"
#include <sensor_msgs/JointState.h>
#include <colab_reachy_control/Trajectory.h>
#include <mutex>

#define DEBUG 0
#define PASSTHROUGH 0
#define CONTROL_HZ 30.0
#define REPORT_HZ 5.0

using namespace std;
using namespace ros;
using namespace std_srvs;
using namespace sensor_msgs;
using namespace colab_reachy_control;

class Interpolator : DXLErrorHandler
{
  public:
    Interpolator(DXLPort &iPort, NodeHandle &iNH, const string &iSide, vector<string> &iJointNames, uint32_t iQueueSize = 10);
    virtual ~Interpolator();

    bool trajectorySrvCB(Trajectory::Request &iReq, Trajectory::Response &iRes);
    bool enableJointStateTelemSrvCB(SetBool::Request &iReq, SetBool::Response &iResp);

    void drive(vector<int> &dxl_ids, float *ioJointState);
    vector<double> getJointState() const;
    void remapJointState(vector<int> &dxl_ids, float *iJointState);

    void timerTick(const TimerEvent &iEvent);

    void enableJointStateTelemetry(bool iEnable);
    bool isJointStateTelemetryEnabled();

  protected:
    virtual void handleError(DXL &iDXL, int iCommResult, uint8_t iErrorStatus, const string &iErrorMsg) const;

  public:
    DXLPort &port;
    NodeHandle &nh;
    Publisher jointStatePub;
    ServiceServer trajectorySrv;

    mutex enableJointStateTelemMutex;
    bool enableJointStateTelem;
    ServiceServer enableJointStateTelemSrv;

    vector<string> jointNames;
    unordered_map<string, int> jointDict;

    float passThroughJointState[8];
};

void initKludgyDXLDict();

extern unordered_map<string, int> kludgyDXLDict;

#endif
