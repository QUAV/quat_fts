#ifndef DATALOG_H
#define DATALOG_H

typedef enum
{
	FIRED_NOT = 1,
	FIRED_DEADMAN = 2,
	FIRED_FLIGHT_CONTROLLER = 3,
	FIRED_BAD_ATTITUDE = 1,
	FIRED_ATTITUDE_ROLL = 1,
	FIRED_FALL = 1,
} fired_cause_t;


void datalog_flash (fired_cause_t cause);
void datalog_add_event (unsigned char* data, unsigned length);

#endif // DATALOG_H

