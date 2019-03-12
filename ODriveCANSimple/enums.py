from enum import IntEnum

MSG_CO_NMT_CTRL = 0x000  # CANOpen NMT Message REC
MSG_CO_HEARTBEAT_CMD = 0x700  # CANOpen NMT Heartbeat SEND
MSG_ODRIVE_HEARTBEAT = 0x001  # uint32 AxisError, uint32 AxisCurrentState
MSG_ODRIVE_ESTOP = 0x002
MSG_GET_MOTOR_ERROR = 0x003  # uint32 MotorError
MSG_GET_ENCODER_ERROR = 0x004  # uint32 EncoderError
MSG_GET_SENSORLESS_ERROR = 0x005  # uint32 SensorlessError
MSG_SET_AXIS_NODE_ID = 0x006  # uint16 AxisCANNodeID
MSG_SET_AXIS_REQUESTED_STATE = 0x007  # uint32 AxisRequestedState
MSG_SET_AXIS_STARTUP_CONFIG = 0x008
MSG_GET_ENCODER_ESTIMATES = 0x009  # float32 EncoderPosEstimate, float32 EncoderVelEstimate
MSG_GET_ENCODER_COUNT = 0x00A  # uint32 EncoderShadowCount, uint32 EncoderCountCPR
MSG_MOVE_TO_POS = 0x00B  # int32 GoalPosition
MSG_SET_POS_SETPOINT = 0x00C  # int32 PosSetpoint, int16 VelFF, int16 CurFF
MSG_SET_VEL_SETPOINT = 0x00D  # int32 VelSetpoint, int16 Curff
MSG_SET_CUR_SETPOINT = 0x00E  # int32 CurSetPoint
MSG_SET_VEL_LIMIT = 0x00F  # float32 VelLimit
MSG_START_ANTICOGGING = 0x010
MSG_SET_TRAJ_VEL_LIMIT = 0x011  # float32 TrajVelLimit
MSG_SET_TRAJ_ACCEL_LIMITS = 0x012  # float32 TrajAccelLimit, float32 TrajDecelLImit
MSG_SET_TRAJ_A_PER_CSS = 0x013  # float32 TrajAperCSS
MSG_GET_IQ = 0x014  # float32 IqSetpoint, float32 IqMeasured
MSG_GET_SENSORLESS_ESTIMATES = 0x015  # float32 SensorlessPosEstimate, float32 SensorlessVelEstiamte
MSG_RESET_ODRIVE = 0x016
MSG_GET_VBUS_VOLTAGE = 0x017  # float32 VbusVoltage
MSG_SET_ENCODER_OFFSET = 0x01F

class DataType(IntEnum):
    UINT32 = 1
    INT32 = 2
    INT16 = 3
    FLOAT = 4

