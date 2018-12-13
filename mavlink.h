#ifndef MAVLINK_H
#define MAVLINK_H

#include <kernel/list.h>


typedef struct 
{
	unsigned char Seq, SysId, CompId, MsgId;
} mavlink_msg_hdr_t;

typedef struct
{
	mavlink_msg_hdr_t Hdr;
	unsigned char Payload[0];
} mavlink_msg_t;

typedef struct __packed
{
	unsigned int CustomMode;
	unsigned char Type;
	unsigned char Autopilot;
	unsigned char BaseMode;
	unsigned char SystemStatus;	// MAV_STATE_
	unsigned char MavlinkVersion;
} mavlink_msg_heartbeat_t;

typedef struct __packed
{
	unsigned short MsgRate;
	unsigned char TargetSysId, TargetCompId;
	unsigned char StreamId;
	unsigned char StartStop;
} mavlink_msg_request_data_stream_t;

typedef struct __packed
{
	unsigned int TimeBootMs;
	float Roll, Pitch, Yaw;
	float RollSpeed, PitchSpeed, YawSpeed;
} mavlink_msg_attitude_t;

typedef struct __packed
{
	unsigned int TimeBootMs;
	signed short XAcc, YAcc, ZAcc;
	signed short XGyro, YGyro, ZGyro;
	signed short XMag, YMag, ZMag;
} mavlink_msg_scaled_imu2_t;

typedef struct __packed
{
	unsigned long long time_usec;
	float XAcc, YAcc, ZAcc;
	float XGyro, YGyro, ZGyro;
	float Xmag,  YMag, ZMag;
	float AbsPressure, DiffPressure, PressureAlt;
	float Temperature;
	unsigned short FieldsUpdated;
} mavlink_msg_highres_imu_t;

typedef struct __packed
{
	float Param1, Param2, Param3, Param4, Param5, Param6, Param7;
	unsigned short CmdId;
	unsigned char TargetSysId, TargetCompId;
	unsigned char Confirmation;
} mavlink_msg_cmd_long_t;

typedef struct __packed
{
	unsigned short CmdId;
	unsigned char Result;
} mavlink_msg_cmd_ack_t;

typedef struct  {
	unsigned int time_boot_ms; // [ms] Timestamp (time since system boot).
	signed int lat;			// [degE7] Latitude, expressed
	signed int lon;			// [degE7] Longitude, expressed
	signed int alt;			// [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	signed int relative_alt; // [mm] Altitude above ground
	signed short vx;       // [cm/s] Ground X Speed (Latitude, positive north)
	signed short vy;       // [cm/s] Ground Y Speed (Longitude, positive east)
	signed short vz;       // [cm/s] Ground Z Speed (Altitude, positive down)
	unsigned short hdg;     // [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
} mavlink_global_position_int_t;

typedef enum
{
	MAV_AUTOPILOT_GENERIC = 0, // 0
	MAV_AUTOPILOT_RESERVED = 1, // 1
	MAV_AUTOPILOT_SLUGS = 2, // 2
	MAV_AUTOPILOT_ARDUPILOTMEGA = 3, // 3
	MAV_AUTOPILOT_OPENPILOT = 4, // 4
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5, // 5
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6, // 6
	MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7, // 7
	MAV_AUTOPILOT_INVALID = 8, // 8
	MAV_AUTOPILOT_PPZ = 9, // 9
	MAV_AUTOPILOT_UDB = 10, // 10
	MAV_AUTOPILOT_FP = 11, // 11
	MAV_AUTOPILOT_PX4 = 12, // 12
	MAV_AUTOPILOT_SMACCMPILOT = 13, // 13
	MAV_AUTOPILOT_AUTOQUAD = 14, // 14
	MAV_AUTOPILOT_ARMAZILA = 15, // 15
	MAV_AUTOPILOT_AEROB = 16, // 16
	MAV_AUTOPILOT_ASLUAV = 17, // 17
	MAV_AUTOPILOT_SMARTAP = 18, // 18
	MAV_AUTOPILOT_AIRRAILS = 19, // 19
} mavlink_autopilot_t;


typedef enum
{
	MAV_STATE_UNINIT = 0,
	MAV_STATE_BOOT,  // 1
    MAV_STATE_CALIBRATING,  // 2
    MAV_STATE_STANDBY,  // 3
	MAV_STATE_ACTIVE,  // 4
    MAV_STATE_CRITICAL,  // 5
	MAV_STATE_EMERGENCY,  // 6
	MAV_STATE_POWEROFF,  // 7 
	MAV_STATE_FLIGHT_TERMINATION, // 8
} mavlink_state_t;

typedef enum
{
	MAVLINK_MSG_ID_HEARTBEAT = 0,
	MAVLINK_MSG_ID_SYS_STATUS = 1,
    MAVLINK_MSG_ID_PING = 4,
    MAVLINK_MSG_ID_PARAM_REQUEST_LIST = 21,
    MAVLINK_MSG_ID_PARAM_VALUE = 22,
    MAVLINK_MSG_ID_PARAM_SET = 23,
    MAVLINK_MSG_ID_GPS_STATUS = 25,
    MAVLINK_MSG_ID_SCALED_IMU = 26,
    MAVLINK_MSG_ID_RAW_IMU = 27,
    MAVLINK_MSG_ID_RAW_PRESSURE = 28,
    MAVLINK_MSG_ID_SCALED_PRESSURE = 29,
    MAVLINK_MSG_ID_ATTITUDE = 30,
    MAVLINK_MSG_ID_ATTITUDE_QUATERNION = 31,
	MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33,
    MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 34,
    MAVLINK_MSG_ID_RC_CHANNELS_RAW = 35,
    MAVLINK_MSG_ID_SERVO_OUTPUT_RAW = 36,
	// [...]
	MAVLINK_MSG_ID_REQUEST_DATA_STREAM = 66,
	MAVLINK_MSG_ID_DATA_STREAM = 67,
	// [...]
    MAVLINK_MSG_ID_COMMAND_LONG = 76,
    MAVLINK_MSG_ID_COMMAND_ACK = 77,
	// [...]
    MAVLINK_MSG_ID_HIGHRES_IMU = 105,
   	// [...]
	MAVLINK_MSG_ID_SCALED_IMU2 = 116,
	MAVLINK_MSG_ID_AHRS2 = 178,	// NOTE: arducopter specific
} mavlink_msg_id_t;

typedef enum
{
	MAV_CMD_DO_FLIGHTTERMINATION = 185,
	MAV_CMD_GET_MESSAGE_INTERVAL = 510,
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
} mavlink_cmd_t;

typedef enum
{
	MAV_DATA_STREAM_ALL = 0,
	MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_RC_CHANNELS,
    MAV_DATA_STREAM_RAW_CONTROLLER,
    MAV_DATA_STREAM_POSITION = 6,
    MAV_DATA_STREAM_EXTRA1 = 10,
    MAV_DATA_STREAM_EXTRA2,
    MAV_DATA_STREAM_EXTRA3,
} mavlink_data_stream_t;


typedef enum{
	MAV_MODE_FLAG_SAFETY_ARMED = 128,	// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,	//0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED = 32,		// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED = 16, // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED = 8, // 0b00001000 guided mode enabled, system flies waypoints / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED = 4, // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED = 2, //0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1  // 0b00000001 Reserved for future use.
} mavlink_mode_flag_t;

typedef enum
{
	MAV_TYPE_GENERIC = 0,  
	MAV_TYPE_FIXED_WING = 1,  
	MAV_TYPE_QUADROTOR = 2, 
	MAV_TYPE_COAXIAL = 3, 
	MAV_TYPE_HELICOPTER = 4, 
	MAV_TYPE_ANTENNA_TRACKER = 5, 
	MAV_TYPE_GCS = 6, 
	MAV_TYPE_AIRSHIP = 7, 
	MAV_TYPE_FREE_BALLOON = 8, 
	MAV_TYPE_ROCKET = 9, 
	MAV_TYPE_GROUND_ROVER = 10,
	MAV_TYPE_SURFACE_BOAT = 11,
	MAV_TYPE_SUBMARINE = 12, 
	MAV_TYPE_HEXAROTOR = 13, 
	MAV_TYPE_OCTOROTOR = 14,
	MAV_TYPE_TRICOPTER = 15, 
	MAV_TYPE_FLAPPING_WING = 16, 
	MAV_TYPE_KITE = 17, 
	MAV_TYPE_ONBOARD_CONTROLLER = 18,
	MAV_TYPE_VTOL_DUOROTOR = 19, 
	MAV_TYPE_VTOL_QUADROTOR = 20, 
	MAV_TYPE_VTOL_TILTROTOR = 21,
	//-- Entries up to 25 reserved for other VTOL airframes --
	MAV_TYPE_VTOL_RESERVED2 = 22,
	MAV_TYPE_VTOL_RESERVED3 = 23, 
	MAV_TYPE_VTOL_RESERVED4 = 24, 
	MAV_TYPE_VTOL_RESERVED5 = 25, 
	MAV_TYPE_GIMBAL = 26, 
	MAV_TYPE_ADSB = 27, 
	MAV_TYPE_PARAFOIL = 28, 
	MAV_TYPE_DODECAROTOR = 29, 
	MAV_TYPE_CAMERA = 30,
	MAV_TYPE_CHARGING_STATION = 31,
	MAV_TYPE_FLARM = 32,
} mavlink_type_t;

typedef enum
{
      MAV_COMP_ID_ALL = 0,
      MAV_COMP_ID_AUTOPILOT1 = 1,
      MAV_COMP_ID_CAMERA = 100, 
      MAV_COMP_ID_SERVO1 = 140,
      MAV_COMP_ID_SERVO2 = 141,
      MAV_COMP_ID_SERVO3 = 142,
      MAV_COMP_ID_SERVO4 = 143,
      MAV_COMP_ID_SERVO5 = 144,
      MAV_COMP_ID_SERVO6 = 145,
      MAV_COMP_ID_SERVO7 = 146,
      MAV_COMP_ID_SERVO8 = 147,
      MAV_COMP_ID_SERVO9 = 148,
      MAV_COMP_ID_SERVO10 = 149,
      MAV_COMP_ID_SERVO11 = 150,
      MAV_COMP_ID_SERVO12 = 151,
      MAV_COMP_ID_SERVO13 = 152,
      MAV_COMP_ID_SERVO14 = 153,
      MAV_COMP_ID_GIMBAL = 154,
      MAV_COMP_ID_LOG = 155, 
      MAV_COMP_ID_ADSB = 156,
      MAV_COMP_ID_OSD = 157, 
      MAV_COMP_ID_PERIPHERAL = 158, 
      MAV_COMP_ID_QX1_GIMBAL = 159, 
      MAV_COMP_ID_FLARM = 160, 
      MAV_COMP_ID_MAPPER = 180, 
      MAV_COMP_ID_MISSIONPLANNER = 190, 
      MAV_COMP_ID_PATHPLANNER = 195, 
      MAV_COMP_ID_IMU = 200, 
      MAV_COMP_ID_IMU_2 = 201, 
      MAV_COMP_ID_IMU_3 = 202, 
      MAV_COMP_ID_GPS = 220, 
      MAV_COMP_ID_GPS2 = 221, 
      MAV_COMP_ID_UDP_BRIDGE = 240,
      MAV_COMP_ID_UART_BRIDGE = 241, 
      MAV_COMP_ID_SYSTEM_CONTROL = 250,
} mavlin_component_t;

typedef struct __mavlink_handler mavlink_handler_t;
typedef void (*mavlink_handler_func_t)(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);

struct __mavlink_handler
{
	node_t Node;
	unsigned char MsgId;
	mavlink_handler_func_t Func;
};

#include <kernel/dispatcher.h>

void mavlink_initialize(dispatcher_context_t *context);
void mavlink_add_handler(mavlink_handler_t *handler);
bool mavlink_started();
void mavlink_send_msg(mavlink_msg_id_t msg_id, void *msg, unsigned length);

#endif // MAVLINK_H