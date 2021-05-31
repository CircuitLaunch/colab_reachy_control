#include <dynamixel_sdk/dynamixel_sdk.h>
#include <unordered_map>
#include <stdexcept>

using namespace std;

enum {
  MODEL_NUMBER_AX12A = 12,
  MODEL_NUMBER_AX12W = 300,
  MODEL_NUMBER_AX18A = 18,
  MODEL_NUMBER_EX106 = 107,
  MODEL_NUMBER_RX10 = 10,
  MODEL_NUMBER_RX24F = 24,
  MODEL_NUMBER_RX28 = 28,
  MODEL_NUMBER_RX64 = 64,
  MODEL_NUMBER_MX12W = 360,
  MODEL_NUMBER_MX28 = 29,
  MODEL_NUMBER_MX64 = 310,
  MODEL_NUMBER_MX106 = 320,
  MODEL_NUMBER_MX28_2 = 30,
  MODEL_NUMBER_MX64_2 = 311,
  MODEL_NUMBER_MX106_2 = 321
};

enum {
  EEPROM_FIRMWARE_VERSION = 2,
  EEPROM_RETURN_DELAY_TIME = 5,
  EEPROM_CW_ANGLE_LIMIT = 6,
  EEPROM_CCW_ANGLE_LIMIT = 8,
  EEPROM_TEMPERATURE_LIMIT = 11,
  EEPROM_MIN_VOLTAGE_LIMIT = 12,
  EEPROM_MAX_VOLTAGE_LIMIT = 13,
  EEPROM_MAX_TORQUE = 14,
  EEPROM_STATUS_RETURN_LEVEL = 16,
  EEPROM_ALARM_LED = 17,
  EEPROM_SHUTDOWN = 18,

  MX_EEPROM_MULTI_TURN_OFFSET = 20,
  MX_EEPROM_RESOLUTION_DIVIDER = 22
};

enum {
  RAM_TORQUE_ENABLE = 24,
  RAM_LED = 25,
  RAM_GOAL_POSITION = 30,
  RAM_MOVING_SPEED = 32,
  RAM_TORQUE_LIMIT = 34,
  RAM_PRESENT_POSITION = 36,
  RAM_PRESENT_SPEED = 38,
  RAM_PRESENT_LOAD = 40,
  RAM_PRESENT_VOLTAGE = 42,
  RAM_PRESENT_TEMPERATURE = 43,
  RAM_REGISTERED = 44,
  RAM_MOVING = 46,
  RAM_LOCK = 47,
  RAM_PUNCH = 48,

  AX_RAM_CW_COMPLIANCE_MARGIN = 26,
  AX_RAM_CCW_COMPLIANCE_MARGIN = 27,
  AX_RAM_CW_COMPLIANCE_SLOPE = 28,
  AX_RAM_CCW_COMPLIANCE_SLOPE = 29,

  EX_RAM_SENSED_CURRENT = 56,

  MX_RAM_D_GAIN = 26,
  MX_RAM_I_GAIN = 27,
  MX_RAM_P_GAIN = 28,
  MX_RAM_REALTIME_TICK = 50,
  MX_RAM_GOAL_ACCELERATION = 73,

  MX64_RAM_CURRENT = 68,
  MX64_RAM_TORQUE_CTRL_MODE_ENABLE = 70,
  MX64_RAM_GOAL_TORQUE = 71
};

enum {
  ERROR_BIT_VOLTAGE = 1,
  ERROR_BIT_ANGLE_LIMIT = 2,
  ERROR_BIT_OVERHEATING = 4,
  ERROR_BIT_RANGE = 8,
  ERROR_BIT_CHECKSUM = 16,
  ERROR_BIT_OVERLOAD = 32,
  ERROR_BIT_INSTRUCTION = 64
};

#define dxl dynamixel

class DXL;

class DXLErrorHandler
{
  public:
    virtual void handleError(DXL &iDXL, uint8_t iComResult, uint8_t iErrorStatus) = 0;
};

class DXLPort : public DXLErrorHandler
{
  public:
    DXLPort(const string &iAddress, int iBaud = 1000000);
    virtual ~DXLPort();

    int getModel(uint8_t iId, uint16_t &oModel, uint8_t &oError);
    int ping(uint8_t iId, uint8_t &oError);

    DXL &getDXL(uint8_t iId);
    DXL &getDXL(uint8_t iId, DXLErrorHandler &iErrorHandler);

    int readUInt8(uint8_t iId, uint16_t iRegister, uint8_t &oValue, uint8_t &oError);
    int readUInt16(uint8_t iId, uint16_t iRegister, uint16_t &oValue, uint8_t &oError);
    int writeUInt8(uint8_t iId, uint16_t iRegister, uint8_t iValue, uint8_t &oError);
    int writeUInt16(uint8_t iId, uint16_t iRegister, uint16_t iValue, uint8_t &oError);

    template <typename T>
    int syncWrite(uint16_t iRegister, uint16_t iDataLen, unordered_map<uint8_t, T> &iData);

    virtual void handleError(DXL &iDXL, uint8_t iCommResult, uint8_t iErrorStatus);

  protected:
    void syncWriteInit(uint16_t iRegister, uint16_t iDataLen);
    template <typename T>
    void syncWritePush(DXL &iDXL, T iValue);
    int syncWriteComplete();

  protected:
    dxl::PortHandler *portHandler;
    dxl::PacketHandler *packetHandler;
    dxl::GroupSyncWrite *syncWriteHandler;
    uint16_t syncWriteRegister;
    unordered_map<uint8_t, DXL *> actuators;
};

template <typename T>
int DXLPort::syncWrite(uint16_t iRegister, uint16_t iDataLen, unordered_map<uint8_t, T> &iData)
{
  syncWriteInit(iRegister, iDataLen);
  typename unordered_map<uint8_t, T>::iterator i;
  for(i = iData.begin(); i != iData.end(); i++) {
    uint8_t id = i->first;
    uint32_t value = i->second;
    syncWritePush<T>(actuators[id], value);
  }
  return syncWriteComplete();
}

template <typename T>
void DXLPort::syncWritePush(DXL &iDXL, T iValue)
{
  uint8_t id = iDXL.id;
  uint32_t value = iDXL.convertForWrite(syncWriteRegister, iValue);
  uint8_t param[4];
  param[0] = DXL_LOBYTE(DXL_LOWORD(value));
  param[1] = DXL_HIBYTE(DXL_LOWORD(value));
  param[2] = DXL_LOBYTE(DXL_HIWORD(value));
  param[3] = DXL_HIBYTE(DXL_HIWORD(value));
  syncWriteHandler->addParam(id, param);
}

class DXL
{
  friend class DXLPort;

  public:
    DXL(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler);

    uint8_t getFirmwareVersion();

    uint8_t getReturnDelayTime();
    void setReturnDelayTime(uint8_t);

    float getCWAngleLimit();
    void setCWAngleLimit(float iAngle);

    float getCCWAngleLimit();
    void setCCWAngleLimit(float iAngle);

    uint8_t getTemperatureLimit();

    uint8_t getMinVoltageLimit();
    void setMinVoltageLimit(uint8_t iVolts);

    uint8_t getMaxVoltageLimit();
    void setMaxVoltageLimit(uint8_t iVolts);

    uint16_t getMaxTorque();
    void setMaxTorque(uint16_t iTorque);

    uint8_t getStatusReturnLevel();
    void setStatusReturnLevel(uint8_t iLevel);

    uint8_t getShutdown();
    void setShutdown(uint8_t iFlags);

    uint8_t getAlarmLED();
    void setAlarmLED(uint8_t iFlags);

    uint8_t getTorqueEnable();
    void setTorqueEnable(uint8_t iEnable);

    uint8_t getLED();
    void setLED(uint8_t iBool);

    float getGoalPosition();
    void setGoalPosition(float iAngle);

    uint16_t getMovingSpeed();
    void setMovingSpeed(uint16_t iAnglePerSec);

    uint16_t getTorqueLimit();
    void setTorqueLimit(uint16_t iTorque);

    float getPresentPosition();
    uint16_t getPresentSpeed();
    uint16_t getPresentLoad();
    uint8_t getPresentVoltage();
    uint8_t getPresentTemperature();
    uint8_t getRegistered();
    uint8_t getMoving();

    uint8_t getLock();
    void setLock(uint8_t iBool);

    uint16_t getPunch();
    void setPunch(uint16_t iPunch);

  protected:
    virtual float getStepResolution() = 0;
    virtual uint16_t getCenterOffset() = 0;

  protected:
    template <typename T>
    uint32_t convertForWrite(uint8_t iRegister, T iValue);

    template <typename T>
    T convertForRead(uint8_t iRegister, uint32_t iValue);

    uint16_t fromAngle(float iAngle, float useOffset = 1.0);
    float toAngle(uint16_t iSteps, float useOffset = 1.0);

  protected:
    DXLPort &port;
    uint8_t id;
    uint16_t model;
    uint8_t result;
    uint8_t error;
    float offset;
    float polarity;
    DXLErrorHandler &errorHandler;
};

template <typename T>
uint32_t DXL::convertForWrite(uint8_t iRegister, T iValue)
{
  switch(iRegister) {
    case RAM_GOAL_POSITION:
    case RAM_PRESENT_POSITION:
      return fromAngle(polarity * iValue);
    case EEPROM_CW_ANGLE_LIMIT:
    case EEPROM_CCW_ANGLE_LIMIT:
      return fromAngle(polarity * iValue, 0.0);
  }
  return iValue;
}

template <typename T>
T DXL::convertForRead(uint8_t iRegister, uint32_t iValue)
{
  switch(iRegister) {
    case RAM_GOAL_POSITION:
    case RAM_PRESENT_POSITION:
      return polarity * toAngle(iValue);
    case EEPROM_CW_ANGLE_LIMIT:
    case EEPROM_CCW_ANGLE_LIMIT:
      return polarity * toAngle(iValue, 0.0);
  }
  return iValue;
}

class DXL_AX : public DXL
{
  public:
    DXL_AX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler);

    uint8_t getCWComplianceMargin();
    void setCWComplianceMargin(uint8_t iMargin);
    uint8_t getCCWComplianceMargin();
    void setCCWComplianceMargin(uint8_t iMargin);
    uint8_t getCWComplianceSlope();
    void setCWComplianceSlope(uint8_t iMargin);
    uint8_t getCCWComplianceSlope();
    void setCCWComplianceSlope(uint8_t iMargin);

  protected:
    virtual float getStepResolution() { return 0.29; }
    virtual uint16_t getCenterOffset() { return 512; }
};

class DXL_EX : public DXL_AX
{
  public:
    DXL_EX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler);

    uint16_t getSensedCurrent();

  protected:
    virtual float getStepResolution() { return 0.06127; }
    virtual uint16_t getCenterOffset() { return 2048; }
};

class DXL_MX : public DXL
{
  public:
    DXL_MX(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler);

    uint16_t getMultiTurnOffset();
    void setMultiTurnOffset(uint16_t iOffset);
    uint8_t getResolutionDivider();
    void setResolutionDivider(uint8_t iDivider);
    uint8_t getDGain();
    void setDGain(uint8_t iGain);
    uint8_t getIGain();
    void setIGain(uint8_t iGain);
    uint8_t getPGain();
    void setPGain(uint8_t iGain);
    uint16_t getRealtimeTick();
    uint16_t getGoalAcceleration();
    void setGoalAcceleration(uint16_t iAccel);

  protected:
    virtual float getStepResolution() { return 0.088; }
    virtual uint16_t getCenterOffset() { return 2048; }
};

class DXL_MX64 : public DXL_MX
{
  public:
    DXL_MX64(DXLPort &iPort, uint8_t iId, uint16_t iModel, DXLErrorHandler &iErrorHandler);

    uint16_t getCurrent();
    void setCurrent(uint16_t iCurrent);
    uint8_t getTorqueCtlModeEnable();
    void setTorqueCtlModeEnable(uint8_t iEnable);
    uint16_t getTorque();
    void setTorque(uint16_t iTorque);
};
