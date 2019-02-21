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

typedef enum
{
	BUZZER_OFF = 0,
	BUZZER_WARNING = 1,
	BUZZER_FULL_ALARM = 2,
} buzzer_mode_t;

static mavlink_state_t _prev_state = MAV_STATE_UNINIT;
static mavlink_state_t _curr_state = MAV_STATE_UNINIT;
static bool _detonation_lines = false;
static bool _armed = false;
static bool _already_fired = false;
static int _fire_cause = FIRED_NOT;
static int _buzzer_warning_count = 0;

static struct {
	struct { float pitch_min, pitch_max, roll_min, roll_max; } warn_angle, panic_angle;
	int without_fc_warning;  // seconds
	int automatic_power_off; // min
	//int safety_height_mm; // not doable for now, needs a distance sensor
	int deadman_ms;
	int max_fall_mm;
	unsigned int warn_to_panic_ms;

	} _conf = { 
    .automatic_power_off = 6,
	.without_fc_warning = 15,
	.warn_angle = { .pitch_min = -0.8f, .pitch_max = 0.8f, .roll_min = -0.8f, .roll_max = 0.8f },
	.panic_angle = { .pitch_min = -2.0f, .pitch_max = 2.0f, .roll_min = -2.0f, .roll_max = 2.0f },
	//.safety_height_mm = 30000,
	.deadman_ms = 2000,
	.max_fall_mm = 10000,
	.warn_to_panic_ms = 1700};	// APM will go from ACTIVE to STANDBY after 2 seconds of low gas, even changing state to standby
	                            // we try to detect the falling case a bit earlier
	//.max_fall_mm = 4000, // small fall field testing
	//.warn_to_panic_ms = 800};


static dispatcher_t _led_flash_dispatcher;
static void _led_flash(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _led_flash_conf(led_mask_t mask, unsigned time);

static dispatcher_t _buzzer_dispatcher;
static dispatcher_t _buzzer_off_dispatcher;
static void _buzzer(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _buzzer_off(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _buzzer_conf(buzzer_mode_t mode);

static dispatcher_t _fire_off_dispatcher;

static void _set_armed (bool arm);
static void _fire();
static void _fire_off();

static dispatcher_t _mavlink_dispatcher;
static void _mavlink_handler(dispatcher_context_t *context, dispatcher_t *dispatcher);
static dispatcher_t _deadman_dispatcher;
static void _deadman_handler(dispatcher_context_t *context, dispatcher_t *dispatcher);
static dispatcher_t _shutdown_dispatcher;
static void _shutdown_handler(dispatcher_context_t *context, dispatcher_t *dispatcher);

static void _fc_heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
static void _attitude(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
static void _imu2(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
//static void _highres_imu(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);
static void _position(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);

static void _request_motor_disabling();
static void _detonator_fire();
static void _shutdown ();

static dispatcher_context_t _context;


void main()
{
	board_set_led(-1, 0); // switch off all leds
	board_set_led(-1, LED_GREEN); // enabled

	// LED test
	//board_set_led(-1, LED_GREEN); 
	//thread_sleep(1000);
	//board_set_led(-1, 0);

	// BUZZER test
	//board_set_buzzer(true); 
	//thread_sleep(1000);
	//board_set_buzzer(false);

	printf("\n\nBoot!\n");

	//int batt_v = board_battery_voltage ();	// in tenths of volt
	board_enable_battery(true);
	// WARNING: Line detection only works when the battery is enabled
	thread_sleep(100);

	_detonation_lines = board_detect_lines (0) && board_detect_lines (1);
	printf ("Detonator lines: %d %d\n",  board_detect_lines (0), board_detect_lines (1));
	if(!_detonation_lines)
		_buzzer_conf(BUZZER_WARNING);

   	/* Fire test
	thread_sleep(1000);
	board_enable_charges(true);
	board_fire(1);
	thread_sleep(800);
	board_fire(0);
  	board_enable_charges(false);*/

	dispatcher_context_create(&_context);
	dispatcher_create(&_led_flash_dispatcher, nullptr, _led_flash, nullptr);
	dispatcher_create(&_buzzer_dispatcher, nullptr, _buzzer, nullptr);
	dispatcher_create(&_buzzer_off_dispatcher, nullptr, _buzzer_off, nullptr);
	dispatcher_create(&_fire_off_dispatcher, nullptr, _fire_off, nullptr);

	mavlink_initialize(&_context);
	dispatcher_create(&_mavlink_dispatcher, nullptr, _mavlink_handler, nullptr);

	dispatcher_create(&_deadman_dispatcher, nullptr, _deadman_handler, nullptr);
	dispatcher_create(&_shutdown_dispatcher, nullptr, _shutdown_handler, nullptr);

	mavlink_handler_t heartbeat_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_HEARTBEAT, .Func = _fc_heartbeat };
	mavlink_add_handler(&heartbeat_handler);
	mavlink_handler_t att_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_ATTITUDE, .Func = _attitude };
	mavlink_add_handler(&att_handler);
	mavlink_handler_t position_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_GLOBAL_POSITION_INT, .Func = _position };
	mavlink_add_handler(&position_handler);
	//mavlink_handler_t imu2_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_SCALED_IMU2, .Func = _imu2 };
	//mavlink_add_handler(&imu2_handler);
	//mavlink_handler_t highres_imu_handler = (mavlink_handler_t) { .MsgId = MAVLINK_MSG_ID_HIGHRES_IMU, .Func = _highres_imu };
	//mavlink_add_handler(&highres_imu_handler);

	detonator_link_initialize(&_context, _detonator_fire);
	//board_uavcan_init ();
	 
	_led_flash_conf(LED_GREEN, 1600);	// start flashing the leds
	dispatcher_add(&_context, &_led_flash_dispatcher, 1600);

	dispatcher_add(&_context, &_buzzer_dispatcher, 500);

	// Check if we have to shutdown the FTS
	dispatcher_add(&_context, &_shutdown_dispatcher, 1000);	

#ifdef ENABLE_WATCHDOG
	wdt_initialize(100);
#endif

	while(1)
	{
#ifdef ENABLE_WATCHDOG
		wdt_reload();
#endif
		dispatcher_dispatch(&_context, 1600);
	}
}

static void _shutdown ()
{
	board_enable_battery(false);

	while (1);	// world ends here and now
}


// Configures mavlink stream

/*mavlink_msg_cmd_long_t cmd = (mavlink_msg_cmd_long_t) { .TargetSysId = 1, .TargetCompId = 1, 
	.CmdId = MAV_CMD_SET_MESSAGE_INTERVAL, 
	.Param1 = MAVLINK_MSG_ID_ATTITUDE,
	.Param2 = 250 };
mavlink_send_msg(MAVLINK_MSG_ID_COMMAND_LONG, &cmd, sizeof(cmd));*/

/*mavlink_msg_request_data_stream_t req2 = (mavlink_msg_request_data_stream_t) { .TargetSysId = 1, .TargetCompId = 1,
	.StreamId = MAV_DATA_STREAM_RAW_SENSORS,
	.MsgRate = 5, .StartStop = 1 };
mavlink_send_msg(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, &req2, sizeof(req2));*/

static void _mavlink_handler(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	mavlink_msg_request_data_stream_t req1 = (mavlink_msg_request_data_stream_t) { .TargetSysId = 1, .TargetCompId = 1,
		.StreamId = MAV_DATA_STREAM_EXTRA1,	// requests attitude msg
		.MsgRate = 10, .StartStop = 1 };
	mavlink_send_msg(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, &req1, sizeof(req1));

	mavlink_msg_request_data_stream_t req2 = (mavlink_msg_request_data_stream_t) { .TargetSysId = 1, .TargetCompId = 1,
		.StreamId = MAV_DATA_STREAM_POSITION,
		.MsgRate = 10, .StartStop = 1 };
	mavlink_send_msg(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, &req2, sizeof(req2));

	dispatcher_add(context, dispatcher, 5000);
}


static void _detonator_fire()
{
	_set_armed (true);	// manual detonation happens no matter what

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

	
static int _shutdown_count = 0;

static void _shutdown_handler(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	if (!board_detect_ext_power ())
		_shutdown_count++;
	else
		_shutdown_count = 0;	// got power, reset time counter

	if (_shutdown_count >= (_conf.automatic_power_off * 60))
		_shutdown();

	dispatcher_add(&_context, &_shutdown_dispatcher, 1000);
}

static void _countdown_resets ()
{
	_buzzer_warning_count = 0;
	_shutdown_count = 0;
}


static void _fc_heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	static const unsigned char *_autopilot[] = { "generic", "rsvd1", "slugs", "ardupilot", "openpilot", "gen5" ,"gen6", "gen7", "invalid", "ppz", "udb", "flexipilot", "px4", "smacc", "autoquad", "armazila", "aerob", "asluav", "smartap" };
	static const unsigned char *_frame_type[] = { "generic", "fixed-wing", "quad", "coaxial", "heli", "tracker", "gcs", "airship", "ballon", "rocket", "rover", "boat", "submarine", "hexa", "octo", "tri", "flapping-wing", "kite", "onboard", "vtol-duo", "vtol-quad", "vtol-tilt", "res22", "res23", "res24" ,"res25", "gimbal", "adsb" }; 

	mavlink_msg_heartbeat_t *data = (mavlink_msg_heartbeat_t *)msg->Payload;
	mavlink_state_t state = data->SystemStatus;

	_countdown_resets ();

	// WARNING: APM flight controller is reported to stop the heartbeat for seconds in his startup routines
	dispatcher_add(&_context, &_deadman_dispatcher, _conf.deadman_ms);	
	_curr_state = state;
	_set_armed((data->BaseMode & MAV_MODE_FLAG_SAFETY_ARMED) != 0);
	printf("arm %d, fire %d, st %d\n", _armed, _already_fired,  _curr_state); 
	switch(state)
	{
		case MAV_STATE_BOOT:
			_led_flash_conf(LED_RED, 50); 
			_prev_state = MAV_STATE_BOOT;
			break;
		case MAV_STATE_CALIBRATING:
			_led_flash_conf(LED_RED | LED_GREEN | LED_BLUE, 50);	 
			_prev_state = MAV_STATE_CALIBRATING;
			break;
		case MAV_STATE_STANDBY:
			_led_flash_conf(LED_GREEN, 500);	

			if (_prev_state != MAV_STATE_STANDBY)
			{
				printf("autopilot: %s\n", _autopilot[data->Autopilot]);
				printf("type: %s\n", _frame_type[data->Type]);
				printf("mavlink version %d\n", data->MavlinkVersion);

				dispatcher_add(&_context, &_mavlink_dispatcher, 100);
				_prev_state = MAV_STATE_STANDBY;
			}
			break;

		case MAV_STATE_ACTIVE:	
		case MAV_STATE_CRITICAL:			// NOTE: arducopter sends this in failsafe
			_led_flash_conf(LED_RED | LED_GREEN, 500); 
			_prev_state = state;
			break;

		case MAV_STATE_EMERGENCY:
			// should we just fire, here?
			if (_armed)
				_led_flash_conf(LED_RED | LED_GREEN, 50);	
			else
				_led_flash_conf(LED_GREEN, 50);	
 
			_prev_state = state;
			break;

		case MAV_STATE_FLIGHT_TERMINATION:	// NOTE: arducopter NEVER sends this state
			// should we just fire, here?
			_prev_state = state;
			break;

		case MAV_STATE_POWEROFF:
			// NOTHING
			break;
		default:
			// UNKNOWN
			break;
	}
}


/*static void _imu2(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
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
		if ((!_already_fired) && (_last_height >= _conf.safety_height_mm))
		{
			_fire_cause = FIRED_FALL;
			_fire();
		}
       	printf("TIP OVER\n");
	}
}

static void _highres_imu(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	static unsigned fall_time = 0;
	mavlink_msg_highres_imu_h *data = (mavlink_msg_highres_imu_h *)msg->Payload;
	float acc = data->XAcc * data->XAcc + data->YAcc * data->YAcc + data->ZAcc * data->ZAcc; 
	bool acc_ready = acc > 300.0f;
	fall_time = acc_ready ? 0 : (fall_time + 100);
	if (fall_time > _conf.warn_to_panic_ms)
	{
		if ((!_already_fired) && (_last_height >= _conf.safety_height_mm))
		{
			_fire_cause = FIRED_FALL;
			_fire();
		}
       	printf("FALLING\n");
	}
}
*/
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

	if (!panic_ready || warn_time >= 10) //_conf.warn_to_panic_ms)
	{
		if (!_already_fired)
		{
			_fire_cause = FIRED_BAD_ATTITUDE;
			if (!panic_ready)
				_fire_cause = FIRED_ATTITUDE_ROLL;
			_fire();
           	printf("TIP OVER\n");
		}
	}

	dispatcher_add(&_context, &_mavlink_dispatcher, 1000);	
}


typedef struct
{
	unsigned int time;
	signed int   alt;
} altitudes_t;


// Cyclical array
static altitudes_t _alt_historical[32] = {};
static int         _alt_idx = 0; 
static bool        _alt_init = false;

static void _position(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	mavlink_global_position_int_t *data = (mavlink_global_position_int_t *)msg->Payload;
	
	if (!_alt_init)
	{
		int i;
		for (i=0; i<32; i++)
		{
			_alt_historical [i].time = data->time_boot_ms;
			_alt_historical [i].alt  = data->alt;
		}
		_alt_init = true;
	}

	const int pos_hz = 10;
    const int time_margin = 100;	// ms
	int span = _conf.warn_to_panic_ms / (1000 / pos_hz);
	
	int curr_idx = _alt_idx & 31;
	int prev_idx = (_alt_idx - span) & 31; 
	_alt_historical [curr_idx].time = data->time_boot_ms;
	_alt_historical [curr_idx].alt  = data->alt;

	_alt_idx = (_alt_idx + 1) & 31;

	unsigned int elapsed = _alt_historical [curr_idx].time - _alt_historical [prev_idx].time;
	if ((elapsed > (_conf.warn_to_panic_ms - time_margin)) && (elapsed < (_conf.warn_to_panic_ms + time_margin)))	// boot situation or communications problem
	{
		int delta_alt = _alt_historical [curr_idx].alt - _alt_historical [prev_idx].alt;
		delta_alt = -delta_alt; // fall sign
		// Try to be sure that delta altitude is not measurement error: check the sign of the historical deltas
  		int correct_signs = 0;
		int i;
		for (i=0; i < (span - 1); i++)
		{
			int d = _alt_historical [(prev_idx + i + 1) & 31].alt - _alt_historical [(prev_idx + i) & 31].alt;
			correct_signs += (d < 0) ? 1 : 0;
		}
       	//printf ("state %d armed %d delta %d, signs %d\n", _curr_state, _armed, delta_alt, correct_signs);

		if ((delta_alt > _conf.max_fall_mm) && (correct_signs > (int)(span * 0.9f)))
		{
			if (!_already_fired)
			{
				_fire_cause = FIRED_FALL;
				_fire();
			}
			printf("FALLING (%d)\n", correct_signs);
		}
	}
}




// Tell flight controller to stop motor inmediately 
static void _request_motor_disabling()
{
#if 1
	mavlink_msg_cmd_long_t cmd = (mavlink_msg_cmd_long_t) { .TargetSysId = 1, .TargetCompId = 1, 
		.CmdId = MAV_CMD_DO_FLIGHTTERMINATION, .Param1 = 1.0f };
	mavlink_send_msg(MAVLINK_MSG_ID_COMMAND_LONG, &cmd, sizeof(cmd));
#else
	// Trying to really stop rotors by ussing manual control
	// (tentative, not working)

	// MAV_CMD_DO_SET_MODE 
	// MAV_MODE_FLAG_MANUAL_INPUT_ENABLED ? ponerlo a 0?

	mavlink_msg_rc_channels_override_t cmd = (mavlink_msg_rc_channels_override_t) 
	{ 
		.TargetSysId = 1, 
		.TargetCompId = 1, 
	};

	int i;
	for (i=0; i<8; i++)
		cmd.chann_raw[i] = 1000;	// minimal throttle
	for (i=8; i<18; i++)
		cmd.chann_raw[i] = 0xffff;	// don't touch

	mavlink_send_msg(MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, &cmd, sizeof(cmd));
#endif
}

				
static void _set_armed (bool arm)
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
            _buzzer_conf(BUZZER_FULL_ALARM); // Alarm! Until physical disconnect or battery depletion 
			_request_motor_disabling();
       		_led_flash_conf(LED_BLUE, 500);
           	board_fire(true);

			dispatcher_add(&_context, &_fire_off_dispatcher, 600);

			datalog_flash(_fire_cause);

			_already_fired = true; 
            printf("FIRE!\n");	
		}
		else
		{
			printf("FIRE but NOT ARMED\n");
		}
	}
}

static void _fire_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_fire(false);
}

static led_mask_t _current_color = LED_GREEN;
static int _current_flash_time = 500;
static led_mask_t _leds_on = 0;

#define LED_FIRED_COLOR  LED_BLUE 

static void _led_flash_conf(led_mask_t color, unsigned time)
{
	static bool fired_latch = false;
	_current_color = color;
	_current_flash_time = time;

	if (_current_color == LED_FIRED_COLOR)
		fired_latch = true;

	if (fired_latch)	// once fired, will not change anymore
		_current_color = LED_FIRED_COLOR;
	else
		_leds_on = _current_color;
}

static void _led_flash(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static int on_off = 0;

	if (on_off)
		board_set_led(_current_color, -1);
	else
		board_set_led(_leds_on, 0);

    on_off ^= 1;

	dispatcher_add(&_context, &_led_flash_dispatcher, _current_flash_time);
}

static buzzer_mode_t _buzzer_mode = BUZZER_OFF;

static void _buzzer_conf(buzzer_mode_t mode)
{
	// Once the alarm is set, it cannot be stopped
	if (_buzzer_mode != BUZZER_FULL_ALARM)
		_buzzer_mode = mode;
}

static void _buzzer(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	_buzzer_warning_count++;
	if (_buzzer_warning_count > (_conf.without_fc_warning * 2))
		_buzzer_conf(BUZZER_WARNING);

	if (_buzzer_mode != BUZZER_OFF)
		board_set_buzzer(true);

	dispatcher_add(&_context, &_buzzer_off_dispatcher, 50);
}

static void _buzzer_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	if (_buzzer_mode != BUZZER_FULL_ALARM)
		board_set_buzzer(false);

	dispatcher_add(&_context, &_buzzer_dispatcher, 450);
}

