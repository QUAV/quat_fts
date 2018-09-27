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
	#define LED_R_PIN PC6
	gpio_pin_config(PC7, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // led green
	#define LED_G_PIN PC7
	gpio_pin_config(PC8, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // led blue
	#define LED_B_PIN PC8

	gpio_pin_config(PA2, GPIO_MODE_ALT_FUNC | GPIO_MODEF_MED_SPEED);	// USART2_TX REMAP 0	(SBUS) NOTE: inverted signal
	gpio_pin_config(PA3, GPIO_MODE_INPUT);								// USART2_Rx REMAP 0	(SBUS) NOTE: inverted signal
	#define MAVLINK_UART_MODULE 1	// USART

	// Debug UART
	gpio_pin_config(PA9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART1_TX REMAP 0	(SBUS) NOTE: inverted signal
	gpio_pin_config(PA10, GPIO_MODE_INPUT);								// USART1_Rx REMAP 0	(SBUS) NOTE: inverted signal

	gpio_pin_config(PB0, GPIO_MODE_OUTPUT | GPIO_MODEF_PULL_DOWN);	// Fire 1
	gpio_pin_config(PB1, GPIO_MODE_OUTPUT | GPIO_MODEF_PULL_DOWN);	// Fire 2
	#define FIRE_1_PIN PB0
	#define FIRE_2_PIN PB1

	gpio_pin_config(PB7, GPIO_MODE_INPUT | GPIO_MODEF_PULL_UP);	// Detect 1 
	gpio_pin_config(PB5, GPIO_MODE_INPUT | GPIO_MODEF_PULL_UP);	// Detect 2
	#define  DETECT_PIN_1 PB7	
	#define  DETECT_PIN_2 PB5	

	gpio_pin_config(PB6, GPIO_MODE_OUTPUT | GPIO_MODEF_PULL_DOWN);	// ARMED
	#define  ARMED_PIN PB6

	gpio_pin_config(PA11, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// CAN1 RX REMAP 0
	gpio_pin_config(PA12, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// CAN1 TX REMAP 0
	#define UAV_CAN1  1

	gpio_pin_config(PB12, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// CAN2 RX REMAP 0
	gpio_pin_config(PB13, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// CAN2 TX REMAP 0
	#define UAV_CAN2  2

   	gpio_pin_config(PA8, GPIO_MODE_ALT_FUNC);	// TIM1_CHN1
	#define PPM_IN_PIN PA8	// TIM2

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

void board_enable_charges(bool enable)
{
	hal_gpio_pin_set(ARMED_PIN, enable);
}

void board_fire(bool fire)
{
	hal_gpio_pin_set(FIRE_1_PIN, fire);
	hal_gpio_pin_set(FIRE_2_PIN, fire);
}


#ifdef MAVLINK_UART_MODULE

void board_mavlink_init(dispatcher_context_t *context, uart_control_block_t *cb)
{
	uart_initialize(MAVLINK_UART_MODULE, cb);
}

int board_mavlink_read(unsigned char *buffer, unsigned long length)
{
	return uart_read(MAVLINK_UART_MODULE, buffer, length);
}

int board_mavlink_write(unsigned char *buffer, unsigned long length)
{
	int done = uart_write(MAVLINK_UART_MODULE, buffer, length);
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

