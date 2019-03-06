#include "board.h"
#include <machine/hal.h>
#include <support/stm32f1/cpu.h>
#include <support/stm32f1/gpio.h>
#include <support/gpio_hal.h>
#include <support/i2c_hal.h>
#include <support/pwm_hal.h>

#define LED_INVERSION_MASK (LED_RGB)

void __board_initialize()
{
	gpio_pin_config(PC6, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // led red
	#define LED_R_PIN PC8
	gpio_pin_config(PC7, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // led green
	#define LED_B_PIN PC7
	gpio_pin_config(PC8, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // led blue
	#define LED_G_PIN PC6

	gpio_pin_config(PA2, GPIO_MODE_ALT_FUNC | GPIO_MODEF_MED_SPEED);	// USART2_TX REMAP 0	(SBUS) NOTE: inverted signal
	gpio_pin_config(PA3, GPIO_MODE_INPUT);								// USART2_Rx REMAP 0	(SBUS) NOTE: inverted signal
	#define FTS_UART_MODULE 0	// USART

	// Debug UART
	gpio_pin_config(PA9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART1_TX REMAP 0	(SBUS) NOTE: inverted signal
	gpio_pin_config(PA10, GPIO_MODE_INPUT);								// USART1_Rx REMAP 0	(SBUS) NOTE: inverted signal

	gpio_pin_config(PB7, GPIO_MODE_INPUT);	// Detect 1 
	gpio_pin_config(PB5, GPIO_MODE_INPUT);	// Detect 2
	#define  DETECT_PIN_1 PB7
	#define  DETECT_PIN_2 PB5

    gpio_pin_config(PC0, GPIO_MODE_INPUT | GPIO_MODEF_PULL_UP);	// Batt sense (adc)
	#define  BATT_SENSE_PIN PC0	

	gpio_pin_config(PC5, GPIO_MODE_INPUT);	// Ext. 12V detect 0/1
	#define  EXTP_PIN PC5	

	gpio_pin_config(PC4, GPIO_MODE_OUTPUT | GPIO_MODEF_PULL_UP);	// Battery switch
	#define  BATT_PIN PC4	

   	//gpio_pin_config(PA8, GPIO_MODE_ALT_FUNC);	// TIM1_CHN1
	//#define PPM_IN_PIN PA8	// TIM2
}


void board_set_led(led_mask_t mask, led_mask_t value)
{
	// Note: amber led not supported on this board
#ifdef LED_INVERSION_MASK
	value ^= LED_INVERSION_MASK;
#endif
	if (mask & LED_RED)
		hal_gpio_pin_set(LED_R_PIN, value & LED_RED);

	if (mask & LED_GREEN)
		hal_gpio_pin_set(LED_G_PIN, value & LED_GREEN);

	if (mask & LED_BLUE)
		hal_gpio_pin_set(LED_B_PIN, value & LED_BLUE);
}

bool board_detect_lines (unsigned line)
{
	if (line > 2) 
		return false;

	return !((line == 0) ? hal_gpio_pin(DETECT_PIN_1) : hal_gpio_pin(DETECT_PIN_2));
}

bool board_detect_ext_power ()
{
	return hal_gpio_pin(EXTP_PIN);
}

void board_enable_battery(bool enable)
{
	hal_gpio_pin_set(BATT_PIN, enable);
}

int board_battery_voltage ()	// in tenths of volt
{
	bool threshold = hal_gpio_pin(BATT_SENSE_PIN);
	return threshold ? 84 : 60;	// hack until we have adc 
}

#ifdef FTS_UART_MODULE

int board_fts_read(unsigned char *buffer, unsigned long length)
{
	return uart_read(FTS_UART_MODULE, buffer, length);
}


void board_fts_init(dispatcher_context_t *context, uart_control_block_t *cb)
{
	uart_initialize(FTS_UART_MODULE, cb);
}


int board_fts_write(unsigned char *buffer, unsigned long length)
{
	int done = uart_write(FTS_UART_MODULE, buffer, length);
	ASSERT(done == length, KERNEL_ERROR_KERNEL_PANIC);
	return done;
}

#endif


#ifdef PWM_IN_TIMER_MODULE

static event_t _match_event;

static void _cap_handler(unsigned module, unsigned channel)
{
	// TODO 
	event_set(&_match_event);
}

void board_init_pwm(dispatcher_context_t *context, dispatcher_callback_t callback)
{
	static dispatcher_t dispatcher;

	event_create(&_match_event, EVENTF_AUTORESET);
	dispatcher_create(&dispatcher, &_match_event, callback, nullptr);
	timer_initialize(PWM_IN_TIMER_MODULE, 1000000UL, _cap_handler, TIMER_CTRLF_NONE);
	timer_capture_setup(PWM_IN_TIMER_MODULE, PWM_IN_TIMER_CHANNEL, TIMER_CAPF_INTERRUPT | TIMER_CAPF_RISE | TIMER_CAPF_FALL);
}

#endif




