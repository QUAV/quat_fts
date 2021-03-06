#include "usb_comms.h"
//#include "can_comms.h"
//#include "persist.h"
//#include "power.h"
#include "board.h"
#include <kernel/dispatcher.h>
#include <kernel/panic.h>
#include <support/stm32f1/wdt.h>
#include <meta/config.h>
#include <stdio.h>

//#define ENABLE_WATCHDOG

static dispatcher_t _led_on_dispatcher;
static dispatcher_t _led_off_dispatcher;
static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher);
static void _led_on(dispatcher_context_t *context, dispatcher_t *dispatcher);

static led_mask_t _leds_on = LED_GREEN;

static dispatcher_context_t _context;

static event_t _rx_event;
static uart_control_block_t _cb;
static dispatcher_t _rx_dispatcher;

static dispatcher_t _firestarter_dispatcher;


// Just bridge the messages to the log
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);

int board_fts_read(unsigned char *buffer, unsigned long length);
int board_fts_write(unsigned char *buffer, unsigned long length);
void board_fts_init(dispatcher_context_t *context, uart_control_block_t *cb);

static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static unsigned _length = 0, _done = 0;

	unsigned char buffer[64];

	int read = board_fts_read(buffer, sizeof(buffer) - 1);
	if (read > 0)
	{
		buffer [read] = 0;
		printf ("%s", buffer);
	}

	dispatcher_add(context, dispatcher, TIMEOUT_NEVER);
}

static void _firestarter_handler(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static int _shutdown_count = 0;
	bool a = board_detect_lines (0);
	bool b = board_detect_lines (1);

	// Is detonation being requested
	if (a && b)
	{
		//if (board_battery_voltage() > 60)	// Try to be certain about battery presence >6.0V
		{
			_leds_on = LED_RED;
			board_fts_write("DETONATE\n", 9);
		}
	}
	else	
	{
		// Is shutdown being requested
		if (!a && b)
		{
			if (_shutdown_count > 20)	// 1 second
			{
				board_enable_battery(false);
				while (1);	// world ends here and now
			}

			_shutdown_count++;
		}
		else
			_shutdown_count = 0;
	}

	dispatcher_add(&_context, &_firestarter_dispatcher, 50);
}

void main()
{
	board_set_led(-1, 0); // switch off all leds
	board_set_led(-1, LED_GREEN); // enabled

	printf("\n\nDetonator board boot!\n");
 
	// Auto-start
	// A physical power bypass enables the board. We switch the power transistor so the board is
	// correctly feed through the battery DC-DC
	board_enable_battery(true);
	thread_sleep(500);	// We need to be sure the battery DC-DC is working or the detonator buttons 
	                    // will be badly read

	dispatcher_context_create(&_context);
	dispatcher_create(&_led_on_dispatcher, nullptr, _led_on, nullptr);
	dispatcher_create(&_led_off_dispatcher, nullptr, _led_off, nullptr);

	event_create(&_rx_event, EVENTF_AUTORESET);
	static unsigned char _fts_input_buffer[128];
	static unsigned char _fts_output_buffer[128];
	_cb.Baudrate = 57600;
	stream_buffer_create(&_cb.Input, _fts_input_buffer, sizeof(_fts_input_buffer), &_rx_event, nullptr);
	stream_buffer_create(&_cb.Output, _fts_output_buffer, sizeof(_fts_output_buffer), nullptr, nullptr);

	board_fts_init(&_context, &_cb);

	dispatcher_create(&_rx_dispatcher, &_rx_event, _rx_dispatch, nullptr);
	dispatcher_add(&_context, &_rx_dispatcher, TIMEOUT_NEVER);
	dispatcher_create(&_firestarter_dispatcher, nullptr, _firestarter_handler, nullptr);

	dispatcher_add(&_context, &_firestarter_dispatcher, 50);
	dispatcher_add(&_context, &_led_off_dispatcher, 300);
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
			_leds_on = LED_WHITE;
		}
	}
}

/*static void _heartbeat(const mavlink_handler_t *handler, const mavlink_msg_t *msg, unsigned length)
{
	printf ("heart of hearts\n");
	dispatcher_add(&_context, &_deadman_dispatcher, 1800); // 1500	
}*/




static void _led_on(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_set_led(LED_WHITE, _leds_on);

	dispatcher_add(&_context, &_led_off_dispatcher, 300);
}

static void _led_off(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	board_set_led(_leds_on, 0);
    dispatcher_add(&_context, &_led_on_dispatcher, 300);
}
 