#include "board.h"
#include <machine/hal.h>
#include <support/stm32f1/cpu.h>
#include <support/stm32f1/gpio.h>
#include <support/gpio_hal.h>
#include <support/i2c_hal.h>
#include <support/pwm_hal.h>

void __board_initialize()
{
#if defined BOARD_NAZE32

	gpio_pin_config(PB4, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // LED0
	#define LED_R_PIN PB4
	gpio_pin_config(PB3, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // LED1
	#define LED_G_PIN PB3

	gpio_pin_config(PA9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART1_TX REMAP 0
	gpio_pin_config(PA10, GPIO_MODE_INPUT);								// USART1_RX REMAP 0

#elif defined BOARD_PIXHAWK

	gpio_pin_config(PB14, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // ACT (blue)
	#define LED_B_PIN PB14
	gpio_pin_config(PB15, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN); // B/E (amber)
	#define LED_AMBER_PIN PB15

	gpio_pin_config(PB10, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART3_TX REMAP 0	(SBUS) NOTE: inverted signal
	gpio_pin_config(PB11, GPIO_MODE_INPUT);								// USART3_Rx REMAP 0	(SBUS) NOTE: inverted signal
	hal_gpio_pin_config(PB4, GPIOF_OUTPUT | GPIOF_OPEN_DRAIN);			// _SBUS_OUTPUT_EN
	hal_gpio_pin_set(PB4, 0);

	gpio_pin_config(PA9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART1_TX REMAP 0	(debug/FMU)
	gpio_pin_config(PA10, GPIO_MODE_INPUT);								// USART1_RX REMAP 0	(debug/spectrum/DSM)

	gpio_pin_config(PA2, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// USART2_TX REMAP 0	(connected to FMU)
	gpio_pin_config(PA3, GPIO_MODE_INPUT);								// USART2_RX REMAP 0	(connected to FMU)
	#define FMU_UART_MODULE 1 // USART2

	gpio_pin_config(PA0, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// TIM2_CH1	(SERVO M1)
	gpio_pin_config(PA1, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED);	// TIM2_CH2	(SERVO M2)
	#define PWM_OUT_MODULE 1	// TIM2

#else
#error "Board not supported"
#endif

#ifdef EEPROM_I2C_MODULE
	hal_i2c_initialize(EEPROM_I2C_MODULE, 100000);
#endif

#ifdef PWM_OUT_MODULE
	unsigned const rate = 100;
	unsigned const range = 1000000UL / rate;
	ASSERT(range > 2500, KERNEL_ERROR_KERNEL_PANIC);
	hal_pwm_initialize(PWM_OUT_MODULE, range, rate, 2);
#endif
}

void board_set_led(led_mask_t mask, led_mask_t value)
{
#ifdef LED_R_PIN
	if (mask & LED_RED)
		hal_gpio_pin_set(LED_R_PIN, !(value & LED_RED));
#endif
#ifdef LED_G_PIN
	if (mask & LED_GREEN)
		hal_gpio_pin_set(LED_G_PIN, !(value & LED_GREEN));
#endif
#ifdef LED_B_PIN
	if (mask & LED_BLUE)
		hal_gpio_pin_set(LED_B_PIN, !(value & LED_BLUE));
#endif
#ifdef LED_AMBER_PIN
	if (mask & LED_AMBER)
		hal_gpio_pin_set(LED_AMBER_PIN, !(value & LED_AMBER));
#endif
}

void board_create_eeprom_context(eeprom_context_t *context)
{
	static const eeprom_geometry_t _geometry = { .Size = 256, .PageSize = 8, .Address = 0x50 };
	eeprom_i2c_context_create(context, &_geometry);
}

#ifdef PWM_OUT_MODULE

void board_set_servo(unsigned ch, unsigned value)
{
	hal_pwm_set_output(PWM_OUT_MODULE, ch, value);
}

#endif


#ifdef FMU_UART_MODULE

void board_comms_init(dispatcher_context_t *context, uart_control_block_t *cb)
{
	uart_initialize(FMU_UART_MODULE, cb);
}

int board_comms_read(unsigned char *buffer, unsigned long length)
{
	return uart_read(FMU_UART_MODULE, buffer, length);
}

int board_comms_write(unsigned char *buffer, unsigned long length)
{
	return uart_write(FMU_UART_MODULE, buffer, length);
}

#endif
