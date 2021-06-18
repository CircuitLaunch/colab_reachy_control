#include "DXLCommandServer.hpp"

#include <ros/console.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <deque>
#include <vector>

using namespace std;

DXLCommandServer::DXLCommandServer(DXLPort &iPort, ros::NodeHandle &iNH)
: port(iPort), nh(iNH),
  writeServer(nh.advertiseService<DXLCommandServer, colab_reachy_control::WriteRegisters::Request, colab_reachy_control::WriteRegisters::Response>("dxl_write_registers_service", &DXLCommandServer::writeService, this)),
  readServer(nh.advertiseService<DXLCommandServer, colab_reachy_control::ReadRegisters::Request, colab_reachy_control::ReadRegisters::Response>("dxl_read_registers_service", &DXLCommandServer::readService, this))
{ }

DXLCommandServer::~DXLCommandServer()
{ }

bool DXLCommandServer::writeService(colab_reachy_control::WriteRegisters::Request &iReq, colab_reachy_control::WriteRegisters::Response &iResp)
{
  for(int i = 0; i < iReq.dxl_ids.size(); i++) {
    uint8_t id = iReq.dxl_ids[i];
    uint8_t reg = iReq.registers[i];
    uint8_t regSize = DXLPort::registerSize(reg);
    uint32_t val = iReq.values[i];
    uint8_t error;
    int result;
    if(regSize == 1)
      result = port.writeUInt8(id, reg, uint8_t(val), error);
    else
      result = port.writeUInt16(id, reg, uint16_t(val), error);
    iResp.results.push_back(result);
    iResp.error_bits.push_back(error);
  }
  return true;
}

bool DXLCommandServer::readService(colab_reachy_control::ReadRegisters::Request &iReq, colab_reachy_control::ReadRegisters::Response &iResp)
{
  for(int i = 0; i < iReq.dxl_ids.size(); i++) {
    uint8_t id = iReq.dxl_ids[i];
    uint8_t reg = iReq.registers[i];
    uint8_t regSize = DXLPort::registerSize(reg);
    uint8_t error;
    uint32_t val;
    int result;
    if(regSize == 1) {
      uint8_t byteVal;
      result = port.writeUInt8(id, reg, byteVal, error);
      val = uint32_t(byteVal);
    } else {
      uint16_t wordVal;
      result = port.writeUInt16(id, reg, wordVal, error);
      val = uint32_t(wordVal);
    }
    iResp.results.push_back(result);
    iResp.error_bits.push_back(error);
    iResp.values.push_back(val);
  }
  return true;
}
