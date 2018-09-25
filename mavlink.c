#include "mavlink.h"
#include "datalog.h"
#include "board.h"
#include "stdio.h"
#include <kernel/mutex.h>

static unsigned int _mavlink_baudrate = 57600;

static event_t _rx_event;
static uart_control_block_t _cb;
static dispatcher_t _rx_dispatcher;
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);
static dispatcher_t _heartbeat_dispatcher;
static void _heartbeat_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);

static const unsigned char _final[] = {
	50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 
	0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 
	185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 
	41, 39, 0, 0, 0, 0, 15, 3, 0, 0, 0, 0, 0, 153, 183, 51, 
	82, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 
	0, 106, 49, 22, 29, 12, 241, 233, 0, 231, 183, 63, 54, 0, 0, 0, 
	0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 211, 108, 32, 185, 84, 0, 
	0, 124, 119, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 
	226, 0, 29, 223, 85, 6, 229, 203, 1, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 42, 49, 0, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 
	78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138, 
	234, 240, 47, 189, 52, 174, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
	0, 0, 0, 0, 0, 0, 0, 0, 8, 204, 49, 170, 44, 83, 46, 0};

static list_t _handler_list;
static mutex_t _handler_mutex;
static bool _started = false;
static unsigned char _send_seq = 1;

void mavlink_initialize(dispatcher_context_t *context)
{
	event_create(&_rx_event, EVENTF_AUTORESET);
	static unsigned char _mavlink_input_buffer[64];
	static unsigned char _mavlink_output_buffer[64];
	_cb.Baudrate = _mavlink_baudrate;
	stream_buffer_create(&_cb.Input, _mavlink_input_buffer, sizeof(_mavlink_input_buffer), &_rx_event, nullptr);
	stream_buffer_create(&_cb.Output, _mavlink_output_buffer, sizeof(_mavlink_output_buffer), nullptr, nullptr);

	board_mavlink_init(context, &_cb);

	list_initialize(&_handler_list);
	mutex_create(&_handler_mutex);
	dispatcher_create(&_rx_dispatcher, &_rx_event, _rx_dispatch, nullptr);
	dispatcher_add(context, &_rx_dispatcher, TIMEOUT_NEVER);

	dispatcher_create(&_heartbeat_dispatcher, nullptr, _heartbeat_dispatch, nullptr);
	dispatcher_add(context, &_heartbeat_dispatcher, 5000);
}

bool mavlink_started()
{
	return _started;
}

void mavlink_add_handler(mavlink_handler_t *handler)
{
	mutex_lock(&_handler_mutex);
	list_add_tail(&_handler_list, &handler->Node);
	mutex_unlock(&_handler_mutex);
}

static void _heartbeat_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	mavlink_msg_heartbeat_t cmd = (mavlink_msg_heartbeat_t) { .Type = 18,	// MAV_TYPE_
		.Autopilot = 0,	// MAV_AUTOPILOT_
		.BaseMode = 0, // MAV_MODE_
		.SystemStatus = 3 };	// MAV_STATE_STANDBY
	mavlink_send_msg(MAVLINK_MSG_ID_HEARTBEAT, &cmd, sizeof(cmd));

	_started = true;
	printf("<- heartbeat\n");

	dispatcher_add(context, dispatcher, 10000); 
}

static void _do_msg(const mavlink_msg_t *msg, unsigned length)
{
	static const unsigned char *_states[] = { "NONE", "BOOT", "CAL", "STANDBY", "ACTIVE", "CRITICAL", "EMERGENCY", "POFF", "TERM" }; 

	const mavlink_msg_hdr_t *hdr = &msg->Hdr;
	printf("%02x %d:%d [%d] ", hdr->Seq, hdr->SysId, hdr->CompId, hdr->MsgId);

	datalog_add_event ((unsigned char*)msg, length );

	mavlink_msg_heartbeat_t *beat;
	mavlink_msg_cmd_ack_t *ack;
	// FIXME: check hdr seq, sender, etc.
	switch(hdr->MsgId)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
			beat = (mavlink_msg_heartbeat_t *)msg->Payload;
			printf("%d-%d-%d-%d (%d) %s\n", beat->Autopilot, beat->BaseMode, beat->Type, beat->MavlinkVersion, beat->SystemStatus, _states[beat->SystemStatus]);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			printf("att\n");
			break;
		case MAVLINK_MSG_ID_SCALED_IMU:
			printf("imu\n");
			break;
		case MAVLINK_MSG_ID_SCALED_IMU2:
			printf("imu2\n");
			break;
		case MAVLINK_MSG_ID_HIGHRES_IMU:
			printf("hires imu\n");
			break;
		case MAVLINK_MSG_ID_COMMAND_ACK:
			ack = (mavlink_msg_cmd_ack_t *)msg->Payload;
			printf("ack cmd %d -> %d\n", ack->CmdId, ack->Result);
			break;
		default:
			printf("id %d\n", hdr->MsgId);
			break;
	}

	mutex_lock(&_handler_mutex);
	FOREACH(node, &_handler_list)
	{
		mavlink_handler_t *handler = (mavlink_handler_t *)node;
		if (handler->MsgId == hdr->MsgId)
		{
			handler->Func(handler, msg, length);
			break;
		}
	}
	mutex_unlock(&_handler_mutex);
}

static inline unsigned short _crc16(unsigned short crc, unsigned char c)
{
	unsigned short data = c;
	unsigned tmp = data ^ (crc & 0xff);
	tmp ^= (tmp << 4) & 0xff;
	crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ ((tmp >> 4) & 0xf);
	return crc;
} 

static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static enum { ML_IDLE = 0, ML_SYNC, ML_DATA, ML_END1, ML_END2 } _state = ML_IDLE;
	static unsigned _length, _done;
	static unsigned short _crc, _checksum;
	static unsigned char _msg[64];

	unsigned char buffer[16];
	while(true)
	{
		int read = board_mavlink_read(buffer, sizeof(buffer));
		if (read <= 0) 
			break;

		for (unsigned i = 0; i < read; i++)
		{
			unsigned char c = buffer[i];
			switch (_state)
			{
				case ML_IDLE:
					if (c == 0xfe)
						_state = ML_SYNC;
					break;
				case ML_SYNC:
					_length = c + 4;
					_done = 0;
					
					_crc = 0xffff;
					_crc = _crc16(_crc, c);
					_state = ML_DATA;
					break;
				case ML_DATA:
					if (_done < sizeof(_msg))
						_msg[_done] = c;
					_done++;
					_crc = _crc16(_crc, c);

					if (_done == _length)
						_state = ML_END1;
					break;
				case ML_END1:
					_checksum = c;
					_state = ML_END2;
					break;
				case ML_END2:
					_checksum |= c << 8;
					_crc = _crc16(_crc, _final[_msg[3]]);
					if (_checksum == _crc)
						_do_msg((mavlink_msg_t *)_msg, _done);

					_state = ML_IDLE;
					break;
			}
		}
	}

	dispatcher_add(context, dispatcher, TIMEOUT_NEVER);
}

static unsigned short _compute_checksum(unsigned short checksum, unsigned char *buffer, unsigned length)
{
	for (unsigned i = 0; i < length; i++)
		checksum = _crc16(checksum, buffer[i]);
	return checksum;
}

void mavlink_send_msg(mavlink_msg_id_t msg_id, void *msg, unsigned length)
{
	unsigned char sync_buffer[] = { 0xfe, length }; 
	board_mavlink_write(sync_buffer, 2);
	unsigned short checksum = _crc16(0xffff, length);

	mavlink_msg_hdr_t hdr = (mavlink_msg_hdr_t) { .Seq = _send_seq++, 
		.SysId = MAVLINK_SYS_ID, .CompId = MAVLINK_COMP_ID,
		.MsgId = msg_id };
	board_mavlink_write((unsigned char *)&hdr, sizeof(hdr));
	checksum = _compute_checksum(checksum, (unsigned char *)&hdr, sizeof(hdr));

	board_mavlink_write(msg, length);
	checksum = _compute_checksum(checksum, msg, length);
	checksum = _crc16(checksum, _final[msg_id]);

	unsigned char checksum_buffer[] = { checksum, checksum >> 8 }; 
	board_mavlink_write(checksum_buffer, 2);
}




