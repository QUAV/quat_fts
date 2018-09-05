#ifndef FTS_COMMS_H
#define FTS_COMMS_H


typedef enum
{
	COMMS_CMD_WAKEUP = 'w',
	COMMS_CMD_ARM = 'a',
	COMMS_CMD_DISARM = 'd',
	COMMS_CMD_FIRE = 'f',
} comms_cmd_t;

#endif // FTS_COMMS_H


