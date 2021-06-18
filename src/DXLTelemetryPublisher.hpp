#ifndef __DXLTELEMETRYPUBLISHER_HPP__
#define __DXLTELEMETRYPUBLISHER_HPP__

#include "DXL.hpp"
#include "ros/ros.h"

class DXLTelemetryPublisher
{
  public:
    DXLTelemetryPublisher(DXLPort &iPort, ros::NodeHandle &iNH, int iQueueSize = 10);
    virtual ~DXLTelemetryPublisher() { }

    void timerTick(const ros::TimerEvent &iEvent) const;

  protected:
    DXLPort &port;
    ros::NodeHandle &nh;
    ros::Publisher telemetryPub;
};

#endif
