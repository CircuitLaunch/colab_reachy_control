#include "DXL.hpp"
#include <iostream>
#include <sstream>

using namespace std;

uint8_t DXLPort::registerSize(uint8_t iRegister)
{
  switch(iRegister) {
    case EEPROM_FIRMWARE_VERSION:
    case EEPROM_RETURN_DELAY_TIME:
    case EEPROM_TEMPERATURE_LIMIT:
    case EEPROM_MIN_VOLTAGE_LIMIT:
    case EEPROM_MAX_VOLTAGE_LIMIT:
    case EEPROM_STATUS_RETURN_LEVEL:
    case EEPROM_ALARM_LED:
    case EEPROM_SHUTDOWN:
    case RAM_TORQUE_ENABLE:
    case RAM_LED:
    case RAM_PRESENT_TEMPERATURE:
    case RAM_PRESENT_VOLTAGE:
    case RAM_REGISTERED:
    case RAM_MOVING:
    case RAM_LOCK:
    case AX_RAM_CW_COMPLIANCE_MARGIN:
    case AX_RAM_CCW_COMPLIANCE_MARGIN:
    case AX_RAM_CW_COMPLIANCE_SLOPE:
    case AX_RAM_CCW_COMPLIANCE_SLOPE:
    case MX_EEPROM_RESOLUTION_DIVIDER:
    /*
    case MX_RAM_D_GAIN:
    case MX_RAM_I_GAIN:
    case MX_RAM_P_GAIN:
    */
    case MX64_RAM_TORQUE_CTRL_MODE_ENABLE:
      return 1;
    case EEPROM_CW_ANGLE_LIMIT:
    case EEPROM_CCW_ANGLE_LIMIT:
    case EEPROM_MAX_TORQUE:
    case RAM_GOAL_POSITION:
    case RAM_MOVING_SPEED:
    case RAM_TORQUE_LIMIT:
    case RAM_PRESENT_POSITION:
    case RAM_PRESENT_SPEED:
    case RAM_PRESENT_LOAD:
    case RAM_PUNCH:
    case EX_RAM_SENSED_CURRENT:
    case MX_EEPROM_MULTI_TURN_OFFSET:
    case MX_RAM_REALTIME_TICK:
    case MX_RAM_GOAL_ACCELERATION:
    case MX64_RAM_CURRENT:
    case MX64_RAM_GOAL_TORQUE:
      return 2;
  }
  return 0;
}


DXLPort::DXLPort(const string &iAddress, int iBaud)
: lock(), syncWriteLock(), actuatorLock()
{
  portHandler = sdk::PortHandler::getPortHandler(iAddress.c_str());

  if(portHandler->openPort()) {
    if(portHandler->setBaudRate(iBaud)) {
      packetHandler = sdk::PacketHandler::getPacketHandler(1.0);
      if(!packetHandler)
        throw invalid_argument("Failed to obtain packet handler for Dynamixel protocol 1.0");
    } else {
      delete portHandler;
      throw invalid_argument("Invalid baud rate");
    }
  } else {
    delete portHandler;
    portHandler = nullptr;
  }
}

DXLPort::~DXLPort()
{
  if(portHandler) {
    portHandler->closePort();
    delete portHandler;
    portHandler = nullptr;
  }
  while(actuators.size()) {
    auto i = actuators.begin();
    actuators.erase(i);
    delete i->second;
  }
}

bool DXLPort::isValidConnection()
{
  return portHandler != nullptr;
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

DXL &DXLPort::getDXL(uint8_t iId, const DXLErrorHandler &iErrorHandler)
{
  actuatorLock.lock();
  auto i = actuators.find(iId);
  actuatorLock.unlock();
  if(i != actuators.end())
    return *(i->second);

  int result;
  uint8_t error;
  DXL *newDXL = nullptr;
  if(portHandler) {
    uint16_t model;
    result = getModel(iId, model, error);
    if(result == COMM_SUCCESS) {
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
      }
    }
  }
  if(!newDXL) {
    newDXL = new DXL_DUMMY(*this, iId, MODEL_NUMBER_DUMMY, iErrorHandler);
  }

  newDXL->setReturnDelayTime(125);

  if(result != COMM_SUCCESS || error != 0)
    iErrorHandler.handleError(*newDXL, result, error, "");

  actuatorLock.lock();
  actuators[iId] = newDXL;
  actuatorLock.unlock();
  return *newDXL;
}

int DXLPort::readUInt8(uint8_t iId, uint16_t iRegister, uint8_t &oValue, uint8_t &oError)
{
  lock.lock();
  int result = packetHandler->read1ByteTxRx(portHandler, iId, iRegister, &oValue, &oError);
  lock.unlock();
  return result;
}

int DXLPort::readUInt16(uint8_t iId, uint16_t iRegister, uint16_t &oValue, uint8_t &oError)
{
  lock.lock();
  int result = packetHandler->read2ByteTxRx(portHandler, iId, iRegister, &oValue, &oError);
  lock.unlock();
  return result;
}

int DXLPort::writeUInt8(uint8_t iId, uint16_t iRegister, uint8_t iValue, uint8_t &oError)
{
  lock.lock();
  int result = packetHandler->write1ByteTxRx(portHandler, iId, iRegister, iValue, &oError);
  lock.unlock();
  return result;
}

int DXLPort::writeUInt16(uint8_t iId, uint16_t iRegister, uint16_t iValue, uint8_t &oError)
{
  lock.lock();
  int result = packetHandler->write2ByteTxRx(portHandler, iId, iRegister, iValue, &oError);
  lock.unlock();
  return result;
}

void DXLPort::syncWriteInit(uint16_t iRegister, uint16_t iDataLen)
{
  syncWriteLock.lock();
  syncWriteRegister = iRegister;
  syncWriteHandler = new sdk::GroupSyncWrite(portHandler, packetHandler, iRegister, iDataLen);
}

int DXLPort::syncWriteComplete()
{
  int result = syncWriteHandler->txPacket();
  delete syncWriteHandler;
  syncWriteHandler = nullptr;
  syncWriteLock.unlock();
  return result;
}

void DXLPort::handleError(DXL &iActuator, int iCommResult, uint8_t iErrorStatus, const string &iErrorMsg) const
{
}

DXL::DXL(DXLPort &iPort, uint8_t iId, uint16_t iModel, const DXLErrorHandler &iErrorHandler)
: port(iPort), id(iId), model(iModel), errorHandler(iErrorHandler), offset(0.0), polarity(1.0)
{ }

DXL::~DXL()
{ }

uint8_t DXL::getFirmwareVersion()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_FIRMWARE_VERSION, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_FIRMWARE_VERSION, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getReturnDelayTime()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_RETURN_DELAY_TIME, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_RETURN_DELAY_TIME, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setReturnDelayTime(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_RETURN_DELAY_TIME, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_RETURN_DELAY_TIME, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

float DXL::getCWAngleLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_CW_ANGLE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt16(EEPROM_CW_ANGLE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return toAngle(oValue, 0.0);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------
void DXL::setCWAngleLimit(float iValue)
{
  uint16_t steps = fromAngle(iValue, 0.0);
  result = port.writeUInt16(id, EEPROM_CW_ANGLE_LIMIT, steps, error);
  ostringstream os;
  os << "writeUInt16(EEPROM_CW_ANGLE_LIMIT, " << steps << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

float DXL::getCCWAngleLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_CCW_ANGLE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt16(EEPROM_CCW_ANGLE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return toAngle(oValue, 0.0);
}

void DXL::setCCWAngleLimit(float iValue)
{
  uint16_t steps = fromAngle(iValue, 0.0);
  result = port.writeUInt16(id, EEPROM_CCW_ANGLE_LIMIT, steps, error);
  ostringstream os;
  os << "readUInt16(EEPROM_CCW_ANGLE_LIMIT, " << steps << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getTemperatureLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_TEMPERATURE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_TEMPERATURE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getMinVoltageLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_MIN_VOLTAGE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_MIN_VOLTAGE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setMinVoltageLimit(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_MIN_VOLTAGE_LIMIT, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_MIN_VOLTAGE_LIMIT, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getMaxVoltageLimit()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_MAX_VOLTAGE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_MAX_VOLTAGE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setMaxVoltageLimit(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_MAX_VOLTAGE_LIMIT, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_MAX_VOLTAGE_LIMIT, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL::getMaxTorque()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EEPROM_MAX_TORQUE, oValue, error);
  ostringstream os;
  os << "readUInt16(EEPROM_MAX_TORQUE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setMaxTorque(uint16_t iValue)
{
  result = port.writeUInt16(id, EEPROM_MAX_TORQUE, iValue, error);
  ostringstream os;
  os << "writeUInt16(EEPROM_MAX_TORQUE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getStatusReturnLevel()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_STATUS_RETURN_LEVEL, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_STATUS_RETURN_LEVEL, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setStatusReturnLevel(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_STATUS_RETURN_LEVEL, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_STATUS_RETURN_LEVEL, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getShutdown()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_SHUTDOWN, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_SHUTDOWN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setShutdown(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_SHUTDOWN, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_SHUTDOWN, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getAlarmLED()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, EEPROM_ALARM_LED, oValue, error);
  ostringstream os;
  os << "readUInt8(EEPROM_ALARM_LED, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setAlarmLED(uint8_t iValue)
{
  result = port.writeUInt8(id, EEPROM_ALARM_LED, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_ALARM_LED, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getTorqueEnable()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_TORQUE_ENABLE, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_TORQUE_ENABLE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setTorqueEnable(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_TORQUE_ENABLE, iValue, error);
  ostringstream os;
  os << "writeUInt8(RAM_TORQUE_ENABLE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL::getLED()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_LED, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_LED, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setLED(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_LED, iValue, error);
  ostringstream os;
  os << "writeUInt8(RAM_LED, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

float DXL::getGoalPosition()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_GOAL_POSITION, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_GOAL_POSITION, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return polarity * toAngle(oValue);
}

void DXL::setGoalPosition(float iValue)
{
  uint16_t steps = fromAngle(polarity * iValue);
  result = port.writeUInt16(id, RAM_GOAL_POSITION, steps, error);
  ostringstream os;
  os << "writeUInt16(RAM_GOAL_POSITION, " << steps << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL::getMovingSpeed()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_MOVING_SPEED, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_MOVING_SPEED, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setMovingSpeed(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_MOVING_SPEED, iValue, error);
  ostringstream os;
  os << "writeUInt16(RAM_MOVING_SPEED, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL::getTorqueLimit()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_TORQUE_LIMIT, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_TORQUE_LIMIT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setTorqueLimit(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_TORQUE_LIMIT, iValue, error);
  ostringstream os;
  os << "writeUInt16(RAM_TORQUE_LIMIT, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

float DXL::getPresentPosition()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_POSITION, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_PRESENT_POSITION, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return polarity * toAngle(oValue);
}

uint16_t DXL::getPresentSpeed()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_SPEED, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_PRESENT_SPEED, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint16_t DXL::getPresentLoad()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PRESENT_LOAD, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_PRESENT_LOAD, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getPresentVoltage()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_PRESENT_VOLTAGE, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_PRESENT_VOLTAGE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getPresentTemperature()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_PRESENT_TEMPERATURE, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_PRESENT_TEMPERATURE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getRegistered()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_REGISTERED, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_REGISTERED, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getMoving()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_MOVING, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_MOVING, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint8_t DXL::getLock()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, RAM_LOCK, oValue, error);
  ostringstream os;
  os << "readUInt8(RAM_LOCK, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setLock(uint8_t iValue)
{
  result = port.writeUInt8(id, RAM_LOCK, iValue, error);
  ostringstream os;
  os << "writeUInt8(RAM_LOCK, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL::getPunch()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, RAM_PUNCH, oValue, error);
  ostringstream os;
  os << "readUInt16(RAM_PUNCH, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL::setPunch(uint16_t iValue)
{
  result = port.writeUInt16(id, RAM_PUNCH, iValue, error);
  ostringstream os;
  os << "writeUInt16(RAM_PUNCH, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL::fromAngle(float iAngle, float iUseOffset)
{
  return getCenterOffset() + uint16_t((iAngle + (offset * iUseOffset)) / getStepResolution());
}

float DXL::toAngle(uint16_t iSteps, float iUseOffset)
{
  return float(iSteps - getCenterOffset()) * getStepResolution() - offset * iUseOffset;
}

DXL_AX::DXL_AX(DXLPort &iPort, uint8_t iId, uint16_t iModel, const DXLErrorHandler &iErrorHandler)
: DXL(iPort, iId, iModel, iErrorHandler)
{ }

uint8_t DXL_AX::getCWComplianceMargin()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CW_COMPLIANCE_MARGIN, oValue, error);
  ostringstream os;
  os << "readUInt8(AX_RAM_CW_COMPLIANCE_MARGIN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_AX::setCWComplianceMargin(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CW_COMPLIANCE_MARGIN, iValue, error);
  ostringstream os;
  os << "writeUInt8(AX_RAM_CW_COMPLIANCE_MARGIN, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_AX::getCCWComplianceMargin()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CCW_COMPLIANCE_MARGIN, oValue, error);
  ostringstream os;
  os << "readUInt8(AX_RAM_CCW_COMPLIANCE_MARGIN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_AX::setCCWComplianceMargin(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CCW_COMPLIANCE_MARGIN, iValue, error);
  ostringstream os;
  os << "writeUInt8(AX_RAM_CCW_COMPLIANCE_MARGIN, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_AX::getCWComplianceSlope()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CW_COMPLIANCE_SLOPE, oValue, error);
  ostringstream os;
  os << "readUInt8(AX_RAM_CW_COMPLIANCE_SLOPE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_AX::setCWComplianceSlope(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CW_COMPLIANCE_SLOPE, iValue, error);
  ostringstream os;
  os << "writeUInt8(AX_RAM_CW_COMPLIANCE_SLOPE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_AX::getCCWComplianceSlope()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, AX_RAM_CCW_COMPLIANCE_SLOPE, oValue, error);
  ostringstream os;
  os << "readUInt8(AX_RAM_CCW_COMPLIANCE_SLOPE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_AX::setCCWComplianceSlope(uint8_t iValue)
{
  result = port.writeUInt8(id, AX_RAM_CCW_COMPLIANCE_SLOPE, iValue, error);
  ostringstream os;
  os << "writeUInt8(AX_RAM_CCW_COMPLIANCE_SLOPE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

DXL_EX::DXL_EX(DXLPort &iPort, uint8_t iId, uint16_t iModel, const DXLErrorHandler &iErrorHandler)
: DXL_AX(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_EX::getSensedCurrent()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, EX_RAM_SENSED_CURRENT, oValue, error);
  ostringstream os;
  os << "readUInt16(EX_RAM_SENSED_CURRENT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

DXL_MX::DXL_MX(DXLPort &iPort, uint8_t iId, uint16_t iModel, const DXLErrorHandler &iErrorHandler)
: DXL(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_MX::getMultiTurnOffset()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_EEPROM_MULTI_TURN_OFFSET, oValue, error);
  ostringstream os;
  os << "readUInt16(MX_EEPROM_MULTI_TURN_OFFSET, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setMultiTurnOffset(uint16_t iValue)
{
  result = port.writeUInt16(id, MX_EEPROM_MULTI_TURN_OFFSET, iValue, error);
  ostringstream os;
  os << "writeUInt16(MX_EEPROM_MULTI_TURN_OFFSET, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_MX::getResolutionDivider()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_EEPROM_RESOLUTION_DIVIDER, oValue, error);
  ostringstream os;
  os << "readUInt8(MX_EEPROM_RESOLUTION_DIVIDER, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setResolutionDivider(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_EEPROM_RESOLUTION_DIVIDER, iValue, error);
  ostringstream os;
  os << "writeUInt8(MX_EEPROM_RESOLUTION_DIVIDER, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_MX::getDGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_D_GAIN, oValue, error);
  ostringstream os;
  os << "readUInt8(MX_RAM_D_GAIN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setDGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_D_GAIN, iValue, error);
  ostringstream os;
  os << "writeUInt8(writeUInt8, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_MX::getIGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_I_GAIN, oValue, error);
  ostringstream os;
  os << "writeUInt8(MX_RAM_I_GAIN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setIGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_I_GAIN, iValue, error);
  ostringstream os;
  os << "writeUInt8(MX_RAM_I_GAIN, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_MX::getPGain()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX_RAM_P_GAIN, oValue, error);
  ostringstream os;
  os << "readUInt8(MX_RAM_P_GAIN, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setPGain(uint8_t iValue)
{
  result = port.writeUInt8(id, MX_RAM_P_GAIN, iValue, error);
  ostringstream os;
  os << "writeUInt8(EEPROM_CW_ANGLE_LIMIT, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL_MX::getRealtimeTick()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_RAM_REALTIME_TICK, oValue, error);
  ostringstream os;
  os << "readUInt16(MX_RAM_REALTIME_TICK, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

uint16_t DXL_MX::getGoalAcceleration()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX_RAM_GOAL_ACCELERATION, oValue, error);
  ostringstream os;
  os << "readUInt16(MX_RAM_GOAL_ACCELERATION, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX::setGoalAcceleration(uint16_t iValue)
{
  result = port.writeUInt16(id, MX_RAM_GOAL_ACCELERATION, iValue, error);
  ostringstream os;
  os << "readUInt16(MX_RAM_GOAL_ACCELERATION, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

DXL_MX64::DXL_MX64(DXLPort &iPort, uint8_t iId, uint16_t iModel, const DXLErrorHandler &iErrorHandler)
: DXL_MX(iPort, iId, iModel, iErrorHandler)
{ }

uint16_t DXL_MX64::getCurrent()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX64_RAM_CURRENT, oValue, error);
  ostringstream os;
  os << "readUInt16(MX64_RAM_CURRENT, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX64::setCurrent(uint16_t iValue)
{
  result = port.writeUInt16(id, MX64_RAM_CURRENT, iValue, error);
  ostringstream os;
  os << "writeUInt16(MX64_RAM_CURRENT, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint8_t DXL_MX64::getTorqueCtlModeEnable()
{
  uint8_t oValue = 0;
  result = port.readUInt8(id, MX64_RAM_TORQUE_CTRL_MODE_ENABLE, oValue, error);
  ostringstream os;
  os << "readUInt8(MX64_RAM_TORQUE_CTRL_MODE_ENABLE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX64::setTorqueCtlModeEnable(uint8_t iValue)
{
  result = port.writeUInt8(id, MX64_RAM_TORQUE_CTRL_MODE_ENABLE, iValue, error);
  ostringstream os;
  os << "writeUInt8(MX64_RAM_TORQUE_CTRL_MODE_ENABLE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

uint16_t DXL_MX64::getGoalTorque()
{
  uint16_t oValue = 0;
  result = port.readUInt16(id, MX64_RAM_GOAL_TORQUE, oValue, error);
  ostringstream os;
  os << "readUInt16(MX64_RAM_GOAL_TORQUE, " << oValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
  return oValue;
}

void DXL_MX64::setGoalTorque(uint16_t iValue)
{
  result = port.writeUInt16(id, MX64_RAM_GOAL_TORQUE, iValue, error);
  ostringstream os;
  os << "readUInt16(MX64_RAM_GOAL_TORQUE, " << iValue << ")";
  errorHandler.handleError(*this, result, error, os.str());
}

DXL_DUMMY::~DXL_DUMMY()
{ }

DXLErrorHandler::~DXLErrorHandler()
{ }

void DXLErrorHandler::resultCodeToString(int iCode, string &oString)
{
  ostringstream ss;
  switch(iCode) {
    case COMM_SUCCESS: oString = "SUCCESS"; return;
    case COMM_PORT_BUSY: oString = "PORT_BUSY"; return;
    case COMM_TX_FAIL: oString = "TX_FAIL"; return;
    case COMM_RX_FAIL: oString = "RX_FAIL"; return;
    case COMM_TX_ERROR: oString = "TX_ERROR"; return;
    case COMM_RX_WAITING: oString = "RX_WAITING"; return;
    case COMM_RX_TIMEOUT: oString = "RX_TIMEOUT"; return;
    case COMM_RX_CORRUPT: oString = "RX_CORRUPT"; return;
    case COMM_NOT_AVAILABLE: oString = "NOT_AVAILABLE"; return;
  }
  oString = "UNKNOWN_CODE";
}

const char *errorBitDescriptors[] =
{
  "INSTRUCTION",
  "OVERLOAD",
  "CHECKSUM",
  "RANGE",
  "OVERHEAT",
  "ANGLE",
  "VOLTAGE"
};

void DXLErrorHandler::errorFlagsToString(uint8_t iFlags, string &oString)
{
  ostringstream ss;
  bool space = false;
  ss << "[";
  for(int i = 0; i < 7; i++) {
    if(iFlags & (1 << i)) {
      if(space) ss << " ";
      ss << errorBitDescriptors[i];
      space = true;
    }
  }
  ss << "]";
  oString = ss.str();
}
