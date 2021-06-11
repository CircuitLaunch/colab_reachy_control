import rospy
from colab_reachy_control.srv import ReadRegisters, ReadRegistersRequest, ReadRegistersResponse, WriteRegisters, WriteRegistersRequest, WriteRegistersResponse

EEPROM_FIRMWARE_VERSION = 2
EEPROM_RETURN_DELAY_TIME = 5
EEPROM_CW_ANGLE_LIMIT = 6
EEPROM_CCW_ANGLE_LIMIT = 8
EEPROM_TEMPERATURE_LIMIT = 11
EEPROM_MIN_VOLTAGE_LIMIT = 12
EEPROM_MAX_VOLTAGE_LIMIT = 13
EEPROM_MAX_TORQUE = 14
EEPROM_STATUS_RETURN_LEVEL = 16
EEPROM_ALARM_LED = 17
EEPROM_SHUTDOWN = 18

MX_EEPROM_MULTI_TURN_OFFSET = 20
MX_EEPROM_RESOLUTION_DIVIDER = 22

RAM_TORQUE_ENABLE = 24
RAM_LED = 25
RAM_GOAL_POSITION = 30
RAM_MOVING_SPEED = 32
RAM_TORQUE_LIMIT = 34
RAM_PRESENT_POSITION = 36
RAM_PRESENT_SPEED = 38
RAM_PRESENT_LOAD = 40
RAM_PRESENT_VOLTAGE = 42
RAM_PRESENT_TEMPERATURE = 43
RAM_REGISTERED = 44
RAM_MOVING = 46
RAM_LOCK = 47
RAM_PUNCH = 48

AX_RAM_CW_COMPLIANCE_MARGIN = 26
AX_RAM_CCW_COMPLIANCE_MARGIN = 27
AX_RAM_CW_COMPLIANCE_SLOPE = 28
AX_RAM_CCW_COMPLIANCE_SLOPE = 29

EX_RAM_SENSED_CURRENT = 56

MX_RAM_D_GAIN = 26
MX_RAM_I_GAIN = 27
MX_RAM_P_GAIN = 28
MX_RAM_REALTIME_TICK = 50
MX_RAM_GOAL_ACCELERATION = 73

MX64_RAM_CURRENT = 68
MX64_RAM_TORQUE_CTRL_MODE_ENABLE = 70
MX64_RAM_GOAL_TORQUE = 71

ERROR_BIT_VOLTAGE = 1
ERROR_BIT_ANGLE_LIMIT = 2
ERROR_BIT_OVERHEATING = 4
ERROR_BIT_RANGE = 8
ERROR_BIT_CHECKSUM = 16
ERROR_BIT_OVERLOAD = 32
ERROR_BIT_INSTRUCTION = 64

class DXLProxy:
    def __init__(self):
        self.readRegSrvProx = rospy.ServiceProxy('dxl_read_registers_service', ReadRegisters)
        self.writeRegSrvProx = rospy.ServiceProxy('dxl_write_registers_service', WriteRegisters)

    def readRegisters(self, ids, registers):
        req = ReadRegistersRequest()
        req.dxl_ids = ids
        req.registers = registers
        resp = self.readRegSrvProx(req)
        return resp.values, resp.results, resp.error_bits

    def writeRegisters(self, ids, registers, values):
        req = WriteRegistersRequest()
        req.dxl_ids = ids
        req.registers = registers
        req.values = values
        resp = self.writeRegSrvProx(req)
        return resp.results, resp.error_bits
