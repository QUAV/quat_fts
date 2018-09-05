#include "output.h"
#include "board.h"	// NOTE: board_... function prototypes are defined here!

static bool _local_armed_state = false;

void output_initialize(dispatcher_context_t *context)
{
	// TODO

	board_enable_charges(false);
	_local_armed_state = false;
}

void output_disarm()
{
	_local_armed_state = false;
	board_enable_charges(false);
}

void output_rearm()
{
	_local_armed_state = true;
	board_enable_charges(true);
}

void output_fire()
{
	board_fire(_local_armed_state);
}


