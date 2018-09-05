#include "board.h"
#include "comms.h"
#include <kernel/dispatcher.h>
#include <kernel/panic.h>
#include <stdio.h>

static unsigned int _fmu_baudrate = 115200;

static dispatcher_context_t _context;

static dispatcher_t _led_off_dispatcher;
static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _led_flash(led_mask_t mask, unsigned time);

static event_t _rx_event;
static uart_control_block_t _cb;
static dispatcher_t _rx_dispatcher;
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);

static bool _armed = false;
static bool _fired = false;

void main()
{
	board_set_led(-1, LED_OFF);
	printf("\n\nBoot!\n");

	dispatcher_context_create(&_context);
	dispatcher_create(&_led_off_dispatcher, nullptr, _led_off, nullptr);

	event_create(&_rx_event, EVENTF_AUTORESET);
	static unsigned char _input_buffer[64];
	static unsigned char _output_buffer[64];
	_cb.Baudrate = _fmu_baudrate;
	stream_buffer_create(&_cb.Input, _input_buffer, sizeof(_input_buffer), &_rx_event, nullptr);
	stream_buffer_create(&_cb.Output, _output_buffer, sizeof(_output_buffer), nullptr, nullptr);
	board_comms_init(&_context, &_cb);

	dispatcher_create(&_rx_dispatcher, &_rx_event, _rx_dispatch, nullptr);
	dispatcher_add(&_context, &_rx_dispatcher, TIMEOUT_NEVER);

	while(1)
	{
		if (!dispatcher_dispatch(&_context, 1500))
		{
			_led_flash(LED_AMBER, 50);
			// TODO send 'f' if fail status is detected
			board_comms_write((unsigned char[]) { 'r' }, 1);
		}
	}

	//TODO check servo power
}

static led_mask_t _leds_on = 0;

static void _led_flash(led_mask_t mask, unsigned time)
{
	board_set_led(mask, -1);
	_leds_on |= mask;
	dispatcher_add(&_context, &_led_off_dispatcher, time);
}

static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_set_led(_leds_on, LED_OFF);
	_leds_on = 0;
}

static void _do_msg(dispatcher_context_t *context, unsigned char *msg, unsigned length)
{
	if (length < 1)
		return;
	
	comms_cmd_t cmd = (comms_cmd_t)msg[0];
	switch(cmd)
	{
		case COMMS_CMD_WAKEUP:
			_led_flash(LED_AMBER, 500);
			break;
		case COMMS_CMD_ARM:
			if (!_fired)
			{
				board_set_servo(0, 2000);	// NOTE: M1 (servo) max value 
				board_set_servo(1, 1000);	// NOTE: M2 (igniter) idle value
				_led_flash(LED_BLUE | LED_AMBER, 500);
			}
			_armed = true;
			break;
		case COMMS_CMD_DISARM:
			if (!_fired)
			{
				board_set_servo(0, 0);		// NOTE: M1 (servo) off
				board_set_servo(1, 0);		// NOTE: M2 (igniter) off
				_led_flash(LED_BLUE, 500);
			}
			_armed = false;
			break;
		case COMMS_CMD_FIRE:
			if (_armed)
			{
				// FIRE!
				board_set_servo(0, 1000);	// NOTE: M1 (servo) min value
				board_set_servo(1, 2000);	// NOTE: M2 (igniter) active value
				_led_flash(LED_BLUE, 10000);
				_fired = true;
				_armed = false;
			}
			break;
	}
}

static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static enum { ML_IDLE = 0, ML_SYNC, ML_LENGTH, ML_DATA, ML_CHECKSUM } _state = ML_IDLE;
	static unsigned _length, _done;
	static unsigned _checksum;
	static unsigned char _msg[16];

	unsigned char buffer[16];
	while(true)
	{
		int read = board_comms_read(buffer, sizeof(buffer));
		if (read <= 0) 
			break;

		for (unsigned i = 0; i < read; i++)
		{
			unsigned char c = buffer[i];
			switch (_state)
			{
				case ML_IDLE:
					if (c == 'A')
					{
//						_led_flash(LED_BLUE, 50);
						_state = ML_SYNC;
					}
					break;
				case ML_SYNC:
					_length = c & 0xf;
					_done = 0;
					_checksum = 0;
					_state = ML_DATA;
					break;
				case ML_DATA:
					if (_done < sizeof(_msg))
						_msg[_done] = c;
					_done++;
					_checksum += c;
					if (_done == _length)
						_state = ML_CHECKSUM;
					break;
				case ML_CHECKSUM:
//					_checksum += _done;
					_checksum = 0;	// FIXME: implement checksum
					if (_checksum == c)
						_do_msg(context, _msg, _done);

					_state = ML_IDLE;
					break;
			}
		}
	}

	dispatcher_add(context, dispatcher, TIMEOUT_NEVER);
}
