#include "detonator_link.h"
#include "datalog.h"
#include "board.h"
#include "stdio.h"
#include <kernel/mutex.h>
#include <kernel/panic.h>
#include <string.h>

detonator_link_fire_func_t _fire_func = 0;

int board_detonator_link_read(unsigned char *buffer, unsigned long length);

static event_t _rx_event;
static dispatcher_t _rx_dispatcher;
static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher);


void detonator_link_initialize(dispatcher_context_t *context, detonator_link_fire_func_t fire_func)
{
#ifndef STDOUT_STREAM
#error "This application requieres stdin/stdout enabled"
#endif
	ASSERT(stdin != nullptr, KERNEL_ERROR_NULL_POINTER);

	_fire_func = fire_func;

	dispatcher_create(&_rx_dispatcher, &stdin->io.InputNotEmpty, _rx_dispatch, nullptr);
	dispatcher_add(context, &_rx_dispatcher, TIMEOUT_NEVER);
}


static void _rx_dispatch(dispatcher_context_t *context, dispatcher_t *dispatcher)
{
	static unsigned _length = 0;
	static unsigned char _msg[16];
	unsigned char buffer[16];
	while(true)
	{
		int read = fread(buffer, 1, sizeof(buffer), stdin);
		//int read = board_detonator_link_read(buffer, sizeof(buffer));

		if (read <= 0) 
			break;

		for (unsigned i = 0; i < read; i++)
		{
			unsigned char c = buffer[i];
			if (c == '\n')
			{
				if(_length == 8)
				{
					_msg[_length] = 0;
					if (strcmp(_msg, "DETONATE") == 0)
					{
						_fire_func ();
					}
				}
				_length = 0;
			}
			else
			{
				_msg [_length] = c;
                _length++;
                if (_length ==  sizeof(_msg))	// unrecognized messages are comming
					_length = 0;
			}
		}
	}

	dispatcher_add(context, dispatcher, TIMEOUT_NEVER);
}
