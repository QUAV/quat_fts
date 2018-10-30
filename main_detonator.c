#include "usb_comms.h"
//#include "can_comms.h"
//#include "persist.h"
//#include "power.h"
#include "board.h"
#include "mavlink.h"
#include <kernel/dispatcher.h>
#include <kernel/panic.h>
#include <support/stm32f1/wdt.h>
#include <meta/config.h>
#include <stdio.h>

#include "datalog.h"

//#define ENABLE_WATCHDOG

static dispatcher_t _led_off_dispatcher;
static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _led_flash(led_mask_t mask, unsigned time);

static void _heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length);

static dispatcher_context_t _context;


static event_t _rx_event;
static uart_control_block_t _cb;
static dispatcher_t _rx_dispatcher;
//static mutex_t _handler_mutex;
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);

// Just bridge the messages to the log

int board_fts_read(unsigned char *buffer, unsigned long length);
int board_fts_write(unsigned char *buffer, unsigned long length);
void board_fts_init(dispatcher_context_t *context, uart_control_block_t *cb);

static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static unsigned _length = 0, _done = 0;
	static unsigned char _msg[64];
 
	unsigned char buffer[16];

	int read = board_fts_read(buffer, sizeof(buffer));
	//if (read > 0)
	// printf ("%s", buffer);

	dispatcher_add(context, dispatcher, TIMEOUT_NEVER);
}


void main()
 {
	board_set_led(-1, 0); // switch off all leds
	board_set_led(-1, LED_GREEN); // enabled

	printf("\n\nDetonator board boot!\n");

	//printf ("detonator lines: %d %d\n",  board_detect_lines (0), board_detect_lines (1));

   	// Fire test
	/*thread_sleep(1000);
	board_enable_charges(true);
	board_fire(1);
	thread_sleep(800);
	board_fire(0);
  	board_enable_charges(false);*/

	dispatcher_context_create(&_context);
	dispatcher_create(&_led_off_dispatcher, nullptr, _led_off, nullptr);

	event_create(&_rx_event, EVENTF_AUTORESET);
	static unsigned char _fts_input_buffer[64];
	static unsigned char _fts_output_buffer[64];
	_cb.Baudrate = 57600;
	stream_buffer_create(&_cb.Input, _fts_input_buffer, sizeof(_fts_input_buffer), &_rx_event, nullptr);
	stream_buffer_create(&_cb.Output, _fts_output_buffer, sizeof(_fts_output_buffer), nullptr, nullptr);

	board_fts_init(&_context, &_cb);

	//mutex_create(&_handler_mutex);
	dispatcher_create(&_rx_dispatcher, &_rx_event, _rx_dispatch, nullptr);
	dispatcher_add(&_context, &_rx_dispatcher, TIMEOUT_NEVER);


#ifdef ENABLE_WATCHDOG
	wdt_initialize(100);
#endif

	while(1)
	{
#ifdef ENABLE_WATCHDOG
		wdt_reload();
#endif
		if (!dispatcher_dispatch(&_context, 300))
		{
			if (board_detect_lines (0))
				board_fts_write("FIRE", 6);

        	printf ("uga %d %d\n", board_detect_lines (0), board_detect_lines (1));
			_led_flash(LED_RGB, 300);
		}
	}
}

static void _heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	//printf ("heart of hearts\n");
	//dispatcher_add(&_context, &_deadman_dispatcher, 1800); // 1500	
}



static led_mask_t _leds_on = 0;

static void _led_flash(led_mask_t mask, unsigned time)
{
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
 