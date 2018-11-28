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
	signed int lat; // [degE7] Latitude, expressed
	signed int lon; // [degE7] Longitude, expressed
	signed int alt; // [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	signed int relative_alt; // [mm] Altitude above ground
	signed short vx;       // [cm/s] Ground X Speed (Latitude, positive north)
	signed short vy;       // [cm/s] Ground Y Speed (Longitude, positive east)
	signed short vz;       // [cm/s] Ground Z Speed (Altitude, positive down)
	unsigned short hdg;     // [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
} mavlink_global_position_int_t;

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