#include "output.h"
#include "board.h"
#include "comms.h"

static unsigned _io_baudrate = 115200;

static bool _io_ready = false;

static event_t _rx_event;
static uart_control_block_t _cb;
static dispatcher_t _rx_dispatcher;
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);

void output_initialize(dispatcher_context_t *context)
{
	event_create(&_rx_event, EVENTF_AUTORESET);
	static unsigned char _input_buffer[64];
	static unsigned char _output_buffer[64];
	_cb.Baudrate = _io_baudrate;
	stream_buffer_create(&_cb.Input, _input_buffer, sizeof(_input_buffer), &_rx_event, nullptr);
	stream_buffer_create(&_cb.Output, _output_buffer, sizeof(_output_buffer), nullptr, nullptr);
	board_comms_init(context, &_cb);

	dispatcher_create(&_rx_dispatcher, &_rx_event, _rx_dispatch, nullptr);
	dispatcher_add(context, &_rx_dispatcher, 1000);
}

static void _cmd1(comms_cmd_t cmd)
{
	unsigned char buf[] = { 'A', 1, cmd, 0x00 };	// FIXME: calc checksum

	board_comms_write(buf, sizeof(buf));
}

void output_disarm()
{
	_cmd1(COMMS_CMD_DISARM); 
}

void output_rearm()
{
	_cmd1(COMMS_CMD_ARM); 
}

void output_fire()
{
	_cmd1(COMMS_CMD_FIRE); 
}

static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	bool ready = false;
	if (dispatcher->State == DISPATCHER_TIMEOUT)
	{
		_cmd1(COMMS_CMD_WAKEUP);
	}
	else
	{
		unsigned char buffer[16];
		while(true)
		{
			int read = board_comms_read(buffer, sizeof(buffer));
			if (read <= 0) 
				break;

			for(int i = 0; i < read; i++)
			{
				unsigned char c = buffer[i];
				switch(c)
				{
					case 'r':
						ready = true;
						break;
				}
			}
		}
	}

	_io_ready = ready;

	dispatcher_add(context, dispatcher, 5000);
}

