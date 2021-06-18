#ifndef __DXLCOMMANDSERVER_HPP__
#define __DXLCOMMANDSERVER_HPP__

#include "ros/ros.h"
#include <colab_reachy_control/WriteRegisters.h>
#include <colab_reachy_control/ReadRegisters.h>
#include "DXL.hpp"

class DXLCommandServer
{
  public:
    DXLCommandServer(DXLPort &iPort, ros::NodeHandle &iNH);
    virtual ~DXLCommandServer();

    bool writeService(colab_reachy_control::WriteRegisters::Request &iReq, colab_reachy_control::WriteRegisters::Response &iResp);
    bool readService(colab_reachy_control::ReadRegisters::Request &iReq, colab_reachy_control::ReadRegisters::Response &iResp);

  protected:
    DXLPort &port;
    ros::NodeHandle &nh;
    ros::ServiceServer writeServer;
    ros::ServiceServer readServer;
};

#endif
