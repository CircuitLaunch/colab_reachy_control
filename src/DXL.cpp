#include "DXL.hpp"

DXLPort::DXLPort(const string &iAddress, int iBaud)
{
  portHandler = dxl::PortHandler::getPortHandler(iAddress.c_str());
  if(portHandler == nullptr)
    throw invalid_argument("Failed to open port to Dynamixel controller");

  portHandler->openPort();
  portHandler->setBaudRate(iBaud);

  packetHandler = dxl::PacketHandler::getPacketHandler(1.0);
  if(packetHandler == nullptr)
    throw invalid_argument("Failed to obtain packet handler for Dynamixel protocol 1.0");
}

DXLPort::~DXLPort()
{
  if(portHandler) portHandler->closePort();
  while(actuators.size()) {
    auto i = actuators.begin();
    actuators.erase(i);
    delete i->second;
  }
}

int DXLPort::getModel(uint8_t iId, uint16_t &oModel, uint8_t &oError)
{
  return readUInt16(iId, 0, oModel, oError);
}

int DXLPort::ping(uint8_t iId, uint8_t &oError)
{
  return packetHandler->ping(portHandler, iId, &oError);
}

DXL &DXLPort::getDXL(uint8_t iId)
{
  return getDXL(iId, *this);
}

DXL &DXLPort::getDXL(uint8_t iId, DXLErrorHandler &iErrorHandler)
{
  auto i = actuators.find(iId);
  if(i != actuators.end())
    return *(i->second);

  uint16_t model;
  uint8_t error;
  int result = getModel(iId, model, error);
  if(result != COMM_SUCCESS)
    throw invalid_argument("Failed to get model for actuator");

  DXL *newDXL;
  switch(model) {
    case MODEL_NUMBER_AX12A:
    case MODEL_NUMBER_AX12W:
    case MODEL_NUMBER_AX18A:
    case MODEL_NUMBER_RX10:
    case MODEL_NUMBER_RX24F:
    case MODEL_NUMBER_RX28:
    case MODEL_NUMBER_RX64:
      newDXL = new DXL_AX(*this, iId, model, iErrorHandler);
      break;
    case MODEL_NUMBER_EX106:
      newDXL = new DXL_EX(*this, iId, model, iErrorHandler);
      break;
    case MODEL_NUMBER_MX12W:
    case MODEL_NUMBER_MX28:
      newDXL = new DXL_MX(*this, iId, model, iErrorHandler);
      break;
    case MODEL_NUMBER_MX64:
    case MODEL_NUMBER_MX106:
      newDXL = new DXL_MX64(*this, iId, model, iErrorHandler);
      break;
    default:
      throw invalid_argument("Unsupported model");
  }

  actuators[iId] = newDXL;

  return *newDXL;
}

int DXLPort::readUInt8(uint8_t iId, uint16_t iRegister, uint8_t &oValue, uint8_t &oError)
{
  return packetHandler->read1ByteTxRx(portHandler, iId, iRegister, &oValue, &oError);
}

int DXLPort::readUInt16(uint8_t iId, uint16_t iRegister, uint16_t &oValue, uint8_t &oError)
{
  return packetHandler->read2ByteTxRx(portHandler, iId, iRegister, &oValue, &oError);
}

int DXLPort::writeUInt8(uint8_t iId, uint16_t iRegister, uint8_t iValue, uint8_t &oError)
{
  return packetHandler->write1ByteTxRx(portHandler, iId, iRegister, iValue, &oError);
}

int DXLPort::writeUInt16(uint8_t iId, uint16_t iRegister, uint16_t iValue, uint8_t &oError)
{
  return packetHandler->write2ByteTxRx(portHandler, iId, iRegister, iValue, &oError);
}

void DXLPort::syncWriteInit(uint16_t iRegister, uint16_t iDataLen)
{
  syncWriteRegister = iRegister;
  syncWriteHandler = new dxl::GroupSyncWrite(portHandler, packetHandler, iRegister, iDataLen);
}

int DXLPort::syncWriteComplete()
{
  int result = syncWriteHandler->txPacket();
  delete syncWriteHandler;
  syncWriteHandler = nullptr;
  return result;
}

void DXLPort::handleError(DXL &iActuator, uint8_t iCommResult, uint8_t iErrorStatus)
{
}

DXL::DXL(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler)
: port(iPort), id(iId), model(iModel), errorHandler(iErrorHandler)
{ }

uint8_t DXL::getFirmwareVersion()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_FIRMWARE_VERSION, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getReturnDelayTime()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_RETURN_DELAY_TIME, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setReturnDelayTime(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_RETURN_DELAY_TIME, iValue, error);
  errorHandler.handleError(*this, result, error);
}

float DXL::getCWAngleLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_CW_ANGLE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return toAngle(oValue, 0.0);
}

void DXL::setCWAngleLimit(float iValue)
{
  uint16_t steps = fromAngle(iValue, 0.0);
  result = port.writeUInt16(id, EEPROM_CW_ANGLE_LIMIT, steps, error);
  errorHandler.handleError(*this, result, error);
}

float DXL::getCCWAngleLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_CCW_ANGLE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return toAngle(oValue, 0.0);
}

void DXL::setCCWAngleLimit(float iValue)
{
  uint16_t steps = fromAngle(iValue, 0.0);
  result = port.writeUInt16(id, EEPROM_CCW_ANGLE_LIMIT, steps, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getTemperatureLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_TEMPERATURE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getMinVoltageLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_MIN_VOLTAGE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setMinVoltageLimit(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_MIN_VOLTAGE_LIMIT, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getMaxVoltageLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_MAX_VOLTAGE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setMaxVoltageLimit(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_MAX_VOLTAGE_LIMIT, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL::getMaxTorque()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_MAX_TORQUE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setMaxTorque(uint16_t iValue)
{
  result = port.writeUInt16(id, EEPROM_MAX_TORQUE, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getStatusReturnLevel()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_STATUS_RETURN_LEVEL, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setStatusReturnLevel(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_STATUS_RETURN_LEVEL, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getShutdown()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_SHUTDOWN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setShutdown(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_SHUTDOWN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getAlarmLED()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_ALARM_LED, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setAlarmLED(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_ALARM_LED, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getTorqueEnable()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_TORQUE_ENABLE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setTorqueEnable(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_TORQUE_ENABLE, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL::getLED()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_LED, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setLED(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_LED, iValue, error);
  errorHandler.handleError(*this, result, error);
}

float DXL::getGoalPosition()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_GOAL_POSITION, oValue, error);
  errorHandler.handleError(*this, result, error);
  return polarity * toAngle(oValue);
}

void DXL::setGoalPosition(float iValue)
{
  uint16_t steps = fromAngle(polarity * iValue);
  result = port.writeUInt16(id, RAM_GOAL_POSITION, steps, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL::getMovingSpeed()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_MOVING_SPEED, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setMovingSpeed(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_MOVING_SPEED, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL::getTorqueLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_TORQUE_LIMIT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setTorqueLimit(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_TORQUE_LIMIT, iValue, error);
  errorHandler.handleError(*this, result, error);
}

float DXL::getPresentPosition()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_POSITION, oValue, error);
  errorHandler.handleError(*this, result, error);
  return polarity * toAngle(oValue);
}

uint16_t DXL::getPresentSpeed()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_SPEED, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint16_t DXL::getPresentLoad()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_LOAD, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getPresentVoltage()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_PRESENT_VOLTAGE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getPresentTemperature()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_PRESENT_TEMPERATURE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getRegistered()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_REGISTERED, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getMoving()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_MOVING, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint8_t DXL::getLock()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_LOCK, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setLock(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_LOCK, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL::getPunch()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PUNCH, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL::setPunch(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_PUNCH, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL::fromAngle(float iAngle, float iUseOffset)
{
  return getCenterOffset() + uint16_t((iAngle + (offset * iUseOffset)) / getStepResolution());
}

float DXL::toAngle(uint16_t iSteps, float iUseOffset)
{
  return float(iSteps - getCenterOffset()) * getStepResolution() - offset * iUseOffset;
}

DXL_AX::DXL_AX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler)
: DXL(iPort, iId, iModel, iErrorHandler)
{ }

uint8_t DXL_AX::getCWComplianceMargin()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CW_COMPLIANCE_MARGIN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_AX::setCWComplianceMargin(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CW_COMPLIANCE_MARGIN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_AX::getCCWComplianceMargin()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CCW_COMPLIANCE_MARGIN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_AX::setCCWComplianceMargin(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CCW_COMPLIANCE_MARGIN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_AX::getCWComplianceSlope()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CW_COMPLIANCE_SLOPE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_AX::setCWComplianceSlope(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CW_COMPLIANCE_SLOPE, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_AX::getCCWComplianceSlope()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CCW_COMPLIANCE_SLOPE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_AX::setCCWComplianceSlope(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CCW_COMPLIANCE_SLOPE, iValue, error);
  errorHandler.handleError(*this, result, error);
}

DXL_EX::DXL_EX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler)
: DXL_AX(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_EX::getSensedCurrent()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EX_RAM_SENSED_CURRENT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

DXL_MX::DXL_MX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler)
: DXL(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_MX::getMultiTurnOffset()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_EEPROM_MULTI_TURN_OFFSET, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setMultiTurnOffset(uint16_t iValue)
{
  result = port.writeUInt16(id, MX_EEPROM_MULTI_TURN_OFFSET, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_MX::getResolutionDivider()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_EEPROM_RESOLUTION_DIVIDER, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setResolutionDivider(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_EEPROM_RESOLUTION_DIVIDER, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_MX::getDGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_D_GAIN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setDGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_D_GAIN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_MX::getIGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_I_GAIN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setIGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_I_GAIN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_MX::getPGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_P_GAIN, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setPGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_P_GAIN, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL_MX::getRealtimeTick()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_RAM_REALTIME_TICK, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

uint16_t DXL_MX::getGoalAcceleration()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_RAM_GOAL_ACCELERATION, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX::setGoalAcceleration(uint16_t iValue)
{
  result = port.writeUInt16(id, MX_RAM_GOAL_ACCELERATION, iValue, error);
  errorHandler.handleError(*this, result, error);
}

DXL_MX64::DXL_MX64(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler)
: DXL_MX(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_MX64::getCurrent()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX64_RAM_CURRENT, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX64::setCurrent(uint16_t iValue)
{
  result = port.writeUInt16(id, MX64_RAM_CURRENT, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint8_t DXL_MX64::getTorqueCtlModeEnable()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX64_RAM_TORQUE_CTRL_MODE_ENABLE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX64::setTorqueCtlModeEnable(uint8_t iValue)
{
  result = port.writeUInt8(id, MX64_RAM_TORQUE_CTRL_MODE_ENABLE, iValue, error);
  errorHandler.handleError(*this, result, error);
}

uint16_t DXL_MX64::getTorque()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX64_RAM_GOAL_TORQUE, oValue, error);
  errorHandler.handleError(*this, result, error);
  return oValue;
}

void DXL_MX64::setTorque(uint16_t iValue)
{
  result = port.writeUInt16(id, MX64_RAM_GOAL_TORQUE, iValue, error);
  errorHandler.handleError(*this, result, error);
}
