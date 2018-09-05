#ifndef BOARD_H
#define BOARD_H

#include <kernel/event.h>
#include <support/misc/eeprom.h>
#include <kernel/dispatcher.h>
#include <support/uart_hal.h>

typedef enum
{
	LED_OFF = 0,
	LED_RED = (1<<0),
	LED_GREEN = (1<<1),
	LED_BLUE = (1<<2),
	LED_RGB = LED_RED|LED_GREEN|LED_BLUE,
	LED_AMBER = (1<<3),
} led_mask_t;

typedef struct
{
	unsigned short VSW;
	unsigned short BAT;
	unsigned short LM;
} board_analog_input_t;

#define BOARD_ANF_VSW (1<<0)
#define BOARD_ANF_BAT (1<<1)
#define BOARD_ANF_LM (1<<2)

typedef enum
{
	OUTPUTF_PSW_EN = (1<<0),
	OUTPUTF_FET_EN = (1<<1),
	OUTPUTF_KILL = (1<<2),
} output_mask_t;

void board_eeprom_create_context(eeprom_context_t *context);

void board_mavlink_init(dispatcher_context_t *context, uart_control_block_t *cb);
int board_mavlink_read(unsigned char *buffer, unsigned long length);
int board_mavlink_write(unsigned char *buffer, unsigned long length);

void board_init_pwm(dispatcher_context_t *context, dispatcher_callback_t callback);

void board_set_servo(unsigned ch, unsigned value);
void board_comms_init(dispatcher_context_t *context, uart_control_block_t *cb);
int board_comms_read(unsigned char *buffer, unsigned long length);
int board_comms_write(unsigned char *buffer, unsigned long length);

void board_set_led(led_mask_t mask, led_mask_t value);
void board_set_output(output_mask_t mask, output_mask_t value);

// NOTE: for board with local charge output
void board_enable_charges(bool enable);
void board_fire(bool fire);

static inline signed _lim(signed value, signed min, signed max)
{
	if (value < min ) value = min;
	else if (value > max) value = max;
	return value;
}

#endif // BOARD_H
