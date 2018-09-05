#include "board.h"
#include <machine/hal.h>
#include <support/stm32f4/cpu.h>
#include <support/stm32f4/gpio.h>
#include <support/stm32f4/dma.h>
#include <support/stm32f4/fmc.h>
#include <support/stm32f4/timer.h>
#include <support/gpio_hal.h>
#include <support/i2c_hal.h>
#include <support/spi_hal.h>
#include <support/uart_hal.h>
#include <kernel/memory.h>

void __board_initialize()
{
#if defined BOARD_PIXHAWK

	gpio_port_enable(GPIO_PORTF_A | GPIO_PORTF_B | GPIO_PORTF_C | GPIO_PORTF_D | GPIO_PORTF_E);
	gpio_pin_config(PE12, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN, 0); // (amber) LED in PE12
	#define LED_AMBER_PIN PE12
	#define LED_INVERSION_MASK LED_AMBER
	#define LED_SD_PIN PE12

	gpio_pin_config(PA0, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART4_TX		(SERIAL3-GPS)
	gpio_pin_config(PA1, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART4_RX

	gpio_pin_config(PD3, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART2_RTS	(SERIAL1-TELEM1)
	gpio_pin_config(PD4, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART2_CTS
	gpio_pin_config(PD5, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART2_TX 
	gpio_pin_config(PD6, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART2_RX
	#define MAVLINK_UART_MODULE 1	// USART2

	gpio_pin_config(PD8, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART3_TX 	(SERIAL2-TELEM2)
	gpio_pin_config(PD9, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART3_RX
	gpio_pin_config(PD12, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART3_RTS
	gpio_pin_config(PD11, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 7);		// USART3_CTS

	gpio_pin_config(PE7, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART7_RX		(SERIAL5)
	gpio_pin_config(PE8, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART7_TX
	gpio_pin_config(PE0, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART8_RX		(SERIAL4)
	gpio_pin_config(PE1, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART8_TX

	gpio_pin_config(PC6, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART6_TX		(to IO cpu)
	gpio_pin_config(PC7, GPIO_MODE_ALT_FUNC |  GPIO_MODEF_PULL_UP, 8);		// UART6_RX
	#define IO_UART_MODULE 5 // UART6

	gpio_pin_config(PC8, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D0
	gpio_pin_config(PC9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D1
	gpio_pin_config(PC10, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D2
	gpio_pin_config(PC11, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D3
	gpio_pin_config(PC12, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_CK
	gpio_pin_config(PD2, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_CMD

	gpio_pin_config(PB10, GPIO_MODE_ALT_FUNC | GPIO_MODEF_OPEN_DRAIN, 4);		// I2C2_SCL
	gpio_pin_config(PB11, GPIO_MODE_ALT_FUNC | GPIO_MODEF_OPEN_DRAIN, 4);		// I2C2_SDA
	#define LED_I2C_MODULE 1	// I2C2
	hal_i2c_initialize(LED_I2C_MODULE, 100000);

//	gpio_pin_config(, GPIO_MODE_ALT_FUNC | GPIO_MODEF_OPEN_DRAIN, );	// pwm input
	#define PWM_IN_TIMER_MODULE 0	// TIM1
	#define PWM_IN_TIMER_CHANNEL 0

#elif defined BOARD_SMART_LOGGER

	gpio_port_enable(GPIO_PORTF_A | GPIO_PORTF_B | GPIO_PORTF_C | GPIO_PORTF_D);
	gpio_pin_config(PB3, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN, 0);		// LED_B in PB3
	#define LED_B_PIN PB3
	gpio_pin_config(PB4, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN, 0);		// LED_R in PB4
	#define LED_R_PIN PB4
	gpio_pin_config(PB5, GPIO_MODE_OUTPUT | GPIO_MODEF_OPEN_DRAIN, 0);		// LED_G in PB5
	#define LED_G_PIN PB5
	#define LED_INVERSION_MASK (LED_RED|LED_GREEN|LED_BLUE)

	gpio_pin_config(PC8, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D0
	gpio_pin_config(PC9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D1
	gpio_pin_config(PC10, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D2
	gpio_pin_config(PC11, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_D3
	gpio_pin_config(PC12, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_CK
	gpio_pin_config(PD2, GPIO_MODE_ALT_FUNC | GPIO_MODEF_HIGH_SPEED, 12);		// SDIO_CMD

	gpio_pin_config(PB10, GPIO_MODE_ALT_FUNC | GPIO_MODEF_PULL_UP, 7);		// USART3_TX 
	gpio_pin_config(PB11, GPIO_MODE_ALT_FUNC | GPIO_MODEF_PULL_UP, 7);		// USART3_RX

	gpio_pin_config(PB8, GPIO_MODE_ALT_FUNC | GPIO_MODEF_PULL_UP, 9);		// CAN1_RX 
	gpio_pin_config(PB9, GPIO_MODE_ALT_FUNC | GPIO_MODEF_PULL_UP, 9);		// CAN1_TX

	gpio_pin_config(PB6, GPIO_MODE_ALT_FUNC | GPIO_MODEF_OPEN_DRAIN, 4);	// I2C1_SCL
	gpio_pin_config(PB7, GPIO_MODE_ALT_FUNC | GPIO_MODEF_OPEN_DRAIN, 4);	// I2C1_SDA
	#define EEPROM_I2C_MODULE 0	// I2C1
	hal_i2c_initialize(EEPROM_I2C_MODULE, 100000);

#else
#error "Board not supported"
#endif

	dma_initialize();
}


void board_set_led(led_mask_t mask, led_mask_t value)
{
#ifdef LED_INVERSION_MASK
	value ^= LED_INVERSION_MASK;
#endif
#ifdef LED_R_PIN
	if (mask & LED_RED)
		hal_gpio_pin_set(LED_R_PIN, value & LED_RED);
#endif
#ifdef LED_G_PIN
	if (mask & LED_GREEN)
		hal_gpio_pin_set(LED_G_PIN, value & LED_GREEN);
#endif
#ifdef LED_B_PIN
	if (mask & LED_BLUE)
		hal_gpio_pin_set(LED_B_PIN, value & LED_BLUE);
#endif
#ifdef LED_AMBER_PIN
	if (mask & LED_AMBER)
		hal_gpio_pin_set(LED_AMBER_PIN, value & LED_AMBER);
#endif


#ifdef LED_I2C_MODULE
	static i2c_context_t led_i2c_context;
	static unsigned char led_cmd[] = { 0x01 };	// AI + PWM1
	hal_i2c_init_context(&led_i2c_context, 0x55, led_cmd, sizeof(led_cmd));
#ifdef DEBUG
#define LED_INTENSITY 1
#else
#define LED_INTENSITY 15
#endif
	static led_mask_t rgb;
	if (mask & (LED_RED|LED_GREEN|LED_BLUE))
	{
		rgb = (rgb & ~mask) | (mask & value);
		hal_i2c_write(LED_I2C_MODULE, &led_i2c_context, 
#ifndef ALTERNATE_LED
			(unsigned char[]) { (rgb & LED_BLUE) ? LED_INTENSITY : 0, (rgb & LED_GREEN) ? LED_INTENSITY : 0, (rgb & LED_RED) ? LED_INTENSITY : 0, 3 }, 
#else
			(unsigned char[]) { (rgb & LED_GREEN) ? LED_INTENSITY : 0, (rgb & LED_RED) ? LED_INTENSITY : 0, (rgb & LED_BLUE) ? LED_INTENSITY : 0, 3 }, 
#endif
			4);
	}
#endif
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


#ifdef IO_UART_MODULE

void board_comms_init(dispatcher_context_t *context, uart_control_block_t *cb)
{
	uart_initialize(IO_UART_MODULE, cb);
}

int board_comms_read(unsigned char *buffer, unsigned long length)
{
	return uart_read(IO_UART_MODULE, buffer, length);
}

int board_comms_write(unsigned char *buffer, unsigned long length)
{
	int done = uart_write(IO_UART_MODULE, buffer, length);
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


#ifdef SDCARD_SPI_MODULE

void sd_spi_power_control(int power)
{
	// TODO
}

void sd_spi_cs_assert()
{
//	hal_gpio_pin_set(SDCARD_SEL_PIN, 0);
}

void sd_spi_cs_release()
{
//	hal_gpio_pin_set(SDCARD_SEL_PIN, 1);
}

#endif

void sd_led_set(bool led)
{
#ifdef LED_SD_PIN
	hal_gpio_pin_set(LED_SD_PIN, !led);
#else
	board_set_led(LED_BLUE, led ? LED_BLUE : 0);
#endif
}





