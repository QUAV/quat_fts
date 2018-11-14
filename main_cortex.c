#include "usb_comms.h"
//#include "can_comms.h"
//#include "persist.h"
//#include "power.h"
#include "board.h"
#include "mavlink.h"
#include "detonator_link.h"
#include <kernel/dispatcher.h>
#include <kernel/panic.h>
#include <support/stm32f1/wdt.h>
#include <meta/config.h>
#include <stdio.h>

#include "datalog.h"

//#define ENABLE_WATCHDOG

static mavlink_state_t _state = MAV_STATE_UNINIT;
static bool _detonator = false;
static bool _armed = false;
static bool _already_fired = false;
static int _fire_cause = FIRED_NOT;
static led_mask_t _current_color = LED_RED;
static int _current_flash_time = 500;


static struct {
	struct { float pitch_min, pitch_max, roll_min, roll_max; } warn_angle, panic_angle;
	unsigned int warn_to_panic_ms;
	} _conf = { 
	.warn_angle = { .pitch_min = -0.8f, .pitch_max = 0.8f, .roll_min = -0.8f, .roll_max = 0.8f },
	.panic_angle = { .pitch_min = -2.0f, .pitch_max = 2.0f, .roll_min = -2.0f, .roll_max = 2.0f },
	.warn_to_panic_ms = 2000 };


static dispatcher_t _led_off_dispatcher;
static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _led_flash(led_mask_t mask, unsigned time);

static dispatcher_t _fire_off_dispatcher;

static void _arm (bool arm);
static void _fire();
static void _fire_off();

static dispatcher_t _mavlink_dispatcher;
static void _mavlink_handler(dispatcher_context_t *context, dispatcher_t *dispatcher);
static dispatcher_t _deadman_dispatcher;
static void _deadman_handler(dispatcher_context_t *context, dispatcher_t *dispatcher);

static void _heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
static void _attitude(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
static void _imu2(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
//static void _highres_imu(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);

static void _request_motor_disabling();
static void _detonator_fire();

static dispatcher_context_t _context;

void main()
{
	//board_set_led(-1, 0); // switch off all leds
	//board_set_led(-1, LED_GREEN); // enabled

	// LED test
	//board_set_led(-1, LED_GREEN); 
	//thread_sleep(1000);
	//board_set_led(-1, 0);

	// BUZZER test
	//board_set_buzzer(true); 
	//thread_sleep(1000);
	//board_set_buzzer(false);

	printf("\n\nBoot!\n");

	_detonator = board_detect_lines (0) && board_detect_lines (1);
	printf ("detonator lines: %d %d\n",  board_detect_lines (0), board_detect_lines (1));
    //if (_state == MAV_STATE_BOOT || _state == MAV_STATE_CALIBRATING || _state == MAV_STATE_STANDBY)
	//	_request_motor_disabling (); 

	
   	/* Fire test
	thread_sleep(1000);
	board_enable_charges(true);
	board_fire(1);
	thread_sleep(800);
	board_fire(0);
  	board_enable_charges(false);*/

	dispatcher_context_create(&_context);
	dispatcher_create(&_led_off_dispatcher, nullptr, _led_off, nullptr);
	dispatcher_create(&_fire_off_dispatcher, nullptr, _fire_off, nullptr);

	mavlink_initialize(&_context);
	dispatcher_create(&_mavlink_dispatcher, nullptr, _mavlink_handler, nullptr);
	dispatcher_create(&_deadman_dispatcher, nullptr, _deadman_handler, nullptr);

	mavlink_handler_t heartbeat_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_HEARTBEAT, .Func = _heartbeat };
	mavlink_add_handler(&heartbeat_handler);
	mavlink_handler_t att_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_ATTITUDE, .Func = _attitude };
	mavlink_add_handler(&att_handler);
	mavlink_handler_t imu2_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_SCALED_IMU2, .Func = _imu2 };
	mavlink_add_handler(&imu2_handler);
	//mavlink_handler_t highres_imu_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_HIGHRES_IMU, .Func = _highres_imu };
	//mavlink_add_handler(&highres_imu_handler);

	detonator_link_initialize(&_context, _detonator_fire);
	//board_uavcan_init ();
	 

#ifdef ENABLE_WATCHDOG
	wdt_initialize(100);
#endif

	while(1)
	{
#ifdef ENABLE_WATCHDOG
		wdt_reload();
#endif
		if (!dispatcher_dispatch(&_context, 1600)) //_current_flash_time))
			_led_flash(_current_color, 1600); //_current_flash_time);
	}
}


static void _mavlink_handler(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	// this configures mavlink stream

//		mavlink_msg_cmd_long_t cmd = (mavlink_msg_cmd_long_t) { .TargetSysId = 1, .TargetCompId = 1, 
//			.CmdId = MAV_CMD_SET_MESSAGE_INTERVAL, 
//			.Param1 = MAVLINK_MSG_ID_ATTITUDE,
//			.Param2 = 250 };
//		mavlink_send_msg(MAVLINK_MSG_ID_COMMAND_LONG, &cmd, sizeof(cmd));

	mavlink_msg_request_data_stream_t req1 = (mavlink_msg_request_data_stream_t) { .TargetSysId = 1, .TargetCompId = 1,
		.StreamId = MAV_DATA_STREAM_EXTRA1,	// requests attitude msg
		.MsgRate = 10, .StartStop = 1 };
	mavlink_send_msg(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, &req1, sizeof(req1));

	mavlink_msg_request_data_stream_t req2 = (mavlink_msg_request_data_stream_t) { .TargetSysId = 1, .TargetCompId = 1,
		.StreamId = MAV_DATA_STREAM_RAW_SENSORS,
		.MsgRate = 5, .StartStop = 1 };
	mavlink_send_msg(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, &req2, sizeof(req2));

	dispatcher_add(context, dispatcher, 5000);
}


static void _detonator_fire()
{
	_arm (true);	// manual detonation happens no matter what

	_fire_cause = FIRED_MANUAL;
	_fire();

	printf("MANUAL DETONATION\n");
}

static void _deadman_handler(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	if (!_already_fired) 
	{
		_fire_cause = FIRED_DEADMAN;
		_fire();
	}
	printf("DEADMAN TIMEOUT\n");
}

static void _heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	static const unsigned char *_autopilot[] = { "generic", "rsvd1", "slugs", "ardupilot", "openpilot", "gen5" ,"gen6", "gen7", "invalid", "ppz", "udb", "flexipilot", "px4", "smacc", "autoquad", "armazila", "aerob", "asluav", "smartap" };
	static const unsigned char *_frame_type[] = { "generic", "fixed-wing", "quad", "coaxial", "heli", "tracker", "gcs", "airship", "ballon", "rocket", "rover", "boat", "submarine", "hexa", "octo", "tri", "flapping-wing", "kite", "onboard", "vtol-duo", "vtol-quad", "vtol-tilt", "res22", "res23", "res24" ,"res25", "gimbal", "adsb" }; 

	static bool _stand_by_delay = false;

	mavlink_msg_heartbeat_t *data = (mavlink_msg_heartbeat_t *)msg->Payload;
	mavlink_state_t state = data->SystemStatus;
	
	dispatcher_add(&_context, &_deadman_dispatcher, 1800); // 1500	

	switch(state)
	{
		case MAV_STATE_BOOT:
			_led_flash(LED_RED, 50);
            //printf("* to boot\n"); 
			_state = MAV_STATE_BOOT;
			break;
		case MAV_STATE_CALIBRATING:
			_led_flash(LED_RED | LED_GREEN | LED_BLUE, 50);	
            //printf("* to calibration\n"); 
			_state = MAV_STATE_CALIBRATING;
			break;
		case MAV_STATE_STANDBY:
			_led_flash(LED_GREEN, 500);	

			if (_state != MAV_STATE_STANDBY)
			{
				if (_stand_by_delay)
					_stand_by_delay = false;
				else
				{
#ifdef DEBUG
					printf("autopilot: %s\n", _autopilot[data->Autopilot]);
					printf("type: %s\n", _frame_type[data->Type]);
					printf("mavlink version %d\n", data->MavlinkVersion);
#endif
					dispatcher_add(&_context, &_mavlink_dispatcher, 100);

					_arm (false);

					//printf("* to standby\n"); 
					_state = MAV_STATE_STANDBY;
				}
			}
			break;

		case MAV_STATE_ACTIVE:				
			_led_flash(LED_RED | LED_GREEN, 500); 

			if (_state == MAV_STATE_STANDBY)
			{
				_arm (true);
			}

			_stand_by_delay = true;
			_state = MAV_STATE_ACTIVE;
			break;

		case MAV_STATE_EMERGENCY:			
		case MAV_STATE_CRITICAL:			// NOTE: arducopter sends this in failsafe
			if (_armed)
				_led_flash(LED_RED | LED_GREEN, 50);	
			else
				_led_flash(LED_GREEN, 50);	
 
			_state = MAV_STATE_EMERGENCY;
			break;

		case MAV_STATE_FLIGHT_TERMINATION:	// NOTE: arducopter NEVER sends this state
			if (!_already_fired)
			{
				_fire_cause = FIRED_FLIGHT_CONTROLLER;
				_fire();
			}
			printf("FLIGHT CONTROLLER TERMINATION\n");
			_state = state;
			break;

		case MAV_STATE_POWEROFF:
			// NOTHING
			break;
	}
}

static void _attitude(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{	
	static unsigned warn_time = 0;
	mavlink_msg_attitude_t *data = (mavlink_msg_attitude_t *)msg->Payload;

	bool warn_ready = data->Pitch > _conf.warn_angle.pitch_min && 
					  data->Pitch < _conf.warn_angle.pitch_max &&
					  data->Roll > _conf.warn_angle.roll_min && 
					  data->Roll < _conf.warn_angle.roll_max;

	warn_time = warn_ready ? 0 : (warn_time + 100);

	bool panic_ready =  data->Pitch > _conf.panic_angle.pitch_min && 
						data->Pitch < _conf.panic_angle.pitch_max &&
						data->Roll > _conf.panic_angle.roll_min &&
						data->Roll < _conf.panic_angle.roll_max;

	if (!panic_ready || warn_time >= _conf.warn_to_panic_ms)
	{
		if (!_already_fired)
		{
			_fire_cause = FIRED_BAD_ATTITUDE;
			if (!panic_ready)
				_fire_cause = FIRED_ATTITUDE_ROLL;
			_fire();
		}
	}

	//if (!warn_ready)
	//	_led_flash(LED_AMBER, 500);	// only on pixhawk boards

	dispatcher_add(&_context, &_mavlink_dispatcher, 1000);	
}


static void _imu2(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	static unsigned fall_time = 0;

	mavlink_msg_scaled_imu2_t *data = (mavlink_msg_scaled_imu2_t *)msg->Payload;
	signed int acc = (((signed int)data->XAcc) * ((signed int)data->XAcc)) +
		(((signed int)data->YAcc) * ((signed int)data->YAcc)) +
		(((signed int)data->ZAcc) * ((signed int)data->ZAcc));
	

	// IMU telemetry  frequency must be 5 Hz; scale is cm
	int min_acc = 26;	// cm/sg^2 experimental measure while falling
	bool acc_ready = acc > (min_acc * min_acc);
	fall_time = acc_ready ? 0 : (fall_time + 200);
	if (fall_time > _conf.warn_to_panic_ms)
	{ 
		if (!_already_fired)
		{
			_fire_cause = FIRED_FALL;
			_fire();
		}
       	printf("FALLING\n");
	}

	//if (!acc_ready)
	//	_led_flash(LED_AMBER, 500);	// only in pixhawk boards
}
/*
// APM sends this message at just 1 Hz!
static void _highres_imu(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	static unsigned fall_time = 0;
	mavlink_msg_highres_imu_h *data = (mavlink_msg_highres_imu_h *)msg->Payload;

	float acc = data->XAcc * data->XAcc + data->YAcc * data->YAcc + data->ZAcc * data->ZAcc; 

	bool acc_ready = acc > 300.0f;
	fall_time = acc_ready ? 0 : (fall_time + 100);
	if (fall_time > _conf.warn_to_panic_ms)
	{
		if (!_already_fired)
		{
			_fire_cause = FIRED_FALL;
			_fire();
		}
       	printf("FALLING\n");
	}

	if (!acc_ready)
		_led_flash(LED_AMBER, 500); // only in pixhawk boards
}
*/
static void _request_motor_disabling()
{
	mavlink_msg_cmd_long_t cmd = (mavlink_msg_cmd_long_t) { .TargetSysId = 1, .TargetCompId = 1, 
		.CmdId = MAV_CMD_DO_FLIGHTTERMINATION, 
		.Param1 = 1.0f };
	// Tell flight controller to stop motor inmediately 
	mavlink_send_msg(MAVLINK_MSG_ID_COMMAND_LONG, &cmd, sizeof(cmd));
}

				
static void _arm (bool arm)
{
	board_enable_charges(arm);
	_armed = arm;
}

static void _fire()
{
	if (_already_fired)
	{
		printf("new fire condition, already fired\n");
	}
	else
	{
        if (_armed)
		{
			datalog_recording (false);
			board_set_buzzer (true);	// Alarm! Until physical disconnect or battery depletion 
			_request_motor_disabling();
       		_led_flash(LED_BLUE, 500);
           	board_fire(true);

			_already_fired = true; 
            printf("FIRE!\n");	
		}
		else
		{
			printf("FIRE but NOT ARMED\n");
		}

 		dispatcher_add(&_context, &_fire_off_dispatcher, 600);

		datalog_flash(_fire_cause);
	}
}

static void _fire_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_fire(false);
}

static led_mask_t _leds_on = 0;

static void _led_flash(led_mask_t mask, unsigned time)
{
	_current_color = mask;
	_current_flash_time = time;
	if (mask & LED_RGB)
	{
		board_set_led(LED_RGB, mask);
		_leds_on |= LED_RGB;
	}
	else
	{
		board_set_led(mask, -1);
		_leds_on |= mask;
	}
	dispatcher_add(&_context, &_led_off_dispatcher, time);
}

static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_set_led(_leds_on, 0);
	_leds_on = 0;
}
 