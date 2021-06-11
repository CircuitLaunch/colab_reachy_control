#include "DXLTelemetryPublisher.hpp"

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <colab_reachy_control/Telemetry.h>
#include <ros/console.h>

using namespace std;

DXLTelemetryPublisher::DXLTelemetryPublisher(DXLPort &iPort, ros::NodeHandle &iNH, int iQueueSize)
: port(iPort), nh(iNH),
  telemetryPub(nh.advertise<colab_reachy_control::Telemetry>("dxl_telemetry", iQueueSize))
{ }

void DXLTelemetryPublisher::timerTick(const ros::TimerEvent &iEvent) const
{
  colab_reachy_control::Telemetry telemetry;
  for(int i = 10; i < 18; i++) {
    DXL &dxl = port.getDXL(i);
    telemetry.dxl_ids.push_back(i);
    telemetry.torque_limits.push_back(dxl.getTorqueLimit());
    telemetry.present_speeds.push_back(dxl.getPresentSpeed());
    telemetry.present_loads.push_back(dxl.getPresentLoad());
    telemetry.present_voltages.push_back(dxl.getPresentVoltage());
    telemetry.present_temperatures.push_back(dxl.getPresentTemperature());
    telemetry.error_bits.push_back(dxl.getError());
  }

  for(int i = 20; i < 28; i++) {
    DXL &dxl = port.getDXL(i);
    telemetry.dxl_ids.push_back(i);
    telemetry.torque_limits.push_back(dxl.getTorqueLimit());
    telemetry.present_speeds.push_back(dxl.getPresentSpeed());
    telemetry.present_loads.push_back(dxl.getPresentLoad());
    telemetry.present_voltages.push_back(dxl.getPresentVoltage());
    telemetry.present_temperatures.push_back(dxl.getPresentTemperature());
    telemetry.error_bits.push_back(dxl.getError());
  }
  telemetryPub.publish(telemetry);
}
