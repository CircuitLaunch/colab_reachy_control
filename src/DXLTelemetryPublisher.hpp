#ifndef __DXLTELEMETRYPUBLISHER_HPP__
#define __DXLTELEMETRYPUBLISHER_HPP__

#include "DXL.hpp"
#include "ros/ros.h"
#include <std_srvs/SetBool.h>

using namespace ros;
using namespace std;
using namespace std_srvs;

class DXLTelemetryPublisher
{
  public:
    DXLTelemetryPublisher(DXLPort &iPort, ros::NodeHandle &iNH, int iQueueSize = 10);
    virtual ~DXLTelemetryPublisher() { }

    void enableTelemetry(bool iEnable);
    bool isTelemetryEnabled();
    bool enableTelemetrySrvCB(SetBool::Request &iReq, SetBool::Response &oResp);

    void timerTick(const TimerEvent &iEvent);

  protected:
    DXLPort &port;
    NodeHandle &nh;
    Publisher telemetryPub;

    mutex enableTelemMutex;
    bool enableTelem;
    ServiceServer enableTelemetrySrv;
};

#endif
