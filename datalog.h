#ifndef DATALOG_H
#define DATALOG_H

#include <stdbool.h>

typedef enum
{
	FIRED_NOT = 1,
	FIRED_DEADMAN = 2,
	FIRED_FLIGHT_CONTROLLER = 3,
	FIRED_BAD_ATTITUDE = 4,
	FIRED_ATTITUDE_ROLL = 5,
	FIRED_FALL = 6,
} fired_cause_t;


void datalog_recording (bool enable);
void datalog_flash (fired_cause_t cause);
void datalog_add_event (unsigned char* data, unsigned length);

#endif // DATALOG_H

