#include "board.h"
#include <machine/hal.h>
#include <support/lpc15/gpio.h>
#include <support/lpc15/swm.h>
#include <support/lpc15/inmux.h>
#include <support/lpc15/sctipu.h>
#include <support/lpc15/sct.h>
#include <support/lpc15/adc.h>
#include <support/lpc15/iap.h>
#include <support/pwm_hal.h>
#include <support/spi_hal.h>
#include <support/dac_hal.h>
#include <support/uart_hal.h>
#include <kernel/panic.h>
#include <kernel/dispatcher.h>
#include <meta/config.h>


static void _pwmin_handle(unsigned module, unsigned ev_flags, unsigned state);
static void _init_pwmin();
static void _adc_handler(unsigned module, adc_seq_t seq);

void __board_initialize()
{
	lpc_swm_initialize();
	lpc_inmux_enable();

	LPC_SYSCON->PRESETCTRL0 |= PRESETCTRL0_EEPROM;
	LPC_SYSCON->SYSAHBCLKCTRL0 |= CLKCTRL0_EEPROM;
	LPC_SYSCON->PRESETCTRL0 ^= PRESETCTRL0_EEPROM;

	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPHY;	// Power-up USB PHY
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPLL;	// Power-up USB PLL
	LPC_SYSCON->USBPLLCLKSEL = 1;	// System oscillator 12 MHz

	LPC_SYSCON->USBPLLCTRL = 0x83;	// M = 4; P = 4
	while (!(LPC_SYSCON->USBPLLSTAT & 0x01));	// Wait until PLL is locked

	LPC_SYSCON->USBCLKSEL = 0x02;	// Select USB PLL
	LPC_SYSCON->USBCLKDIV = 1;

#if defined BOARD_LPCXPRESSO

	// PIO0_14 is PPM_IN/IPU_IN_A0->IPU_SAMPLE0->SCT0 <- FIXME
	lpc_sctipu_sample_config(0, SCTIPU_SAMPLE_INPUT_A, SCTIPU_SAMPLE_TRANSPARENT);
	LPC_INMUX->SCT0_INMUX[0] = SCT0_INP_SCTIPU_SAMPLE0;
	LPC_IOCON->PIO0_14 = IOCON_MODE_PULL_UP | IOCON_HYSTERESIS | IOCON_CLKDIV_BY2 | IOCON_INV_INPUT;
	#define PPM_SCT_MODULE 0
	#define PPM_MAX_CHANNEL_COUNT 10	// NOTE: big SCT allows 6+ channels

	#define LED0_PIN PIO0_25	// RED
	hal_gpio_pin_config(LED0_PIN, GPIOF_OUTPUT | GPIOF_OPEN_DRAIN);
   	#define LED1_PIN PIO0_3		// GREEN
	hal_gpio_pin_config(LED1_PIN, GPIOF_OUTPUT | GPIOF_OPEN_DRAIN);
   	#define LED2_PIN PIO1_1		// BLUE
	hal_gpio_pin_config(LED2_PIN, GPIOF_OUTPUT | GPIOF_OPEN_DRAIN);

	hal_gpio_pin_config(PIO1_8, GPIOF_INPUT | GPIOF_PULLUP);	// MAVLINK_RX
	lpc_swm_assign(LPC_SWMA_UART0_RXD, LPC_SWM_PIO1_8);
	lpc_swm_assign(LPC_SWMA_UART0_TXD, LPC_SWM_PIO1_7);
	#define MAVLINK_UART_MODULE	0	// UART0 for MAVLINK

	hal_gpio_pin_config(PIO0_13, GPIOF_INPUT | GPIOF_PULLUP);
	lpc_swm_assign(LPC_SWMA_UART1_RXD, LPC_SWM_PIO0_13);		// ISP_RX
	lpc_swm_assign(LPC_SWMA_UART1_TXD, LPC_SWM_PIO0_18);		// ISP_TX

#elif defined BOARD_GOV_R2

	lpc_sctipu_initialize();

	hal_pwm_initialize(3, 20000, 50, 2);	// SCT3 for PWM OUT (S0-1)
	lpc_swm_assign(LPC_SWMA_SCT3_OUT0, LPC_SWM_PIO0_0);
	lpc_swm_assign(LPC_SWMA_SCT3_OUT1, LPC_SWM_PIO0_30);
	hal_pwm_set_output(3, 0, 0);
	hal_pwm_set_output(3, 1, 0);
	#define PWM_SCT_MODULE 3

	hal_gpio_pin_config(PIO1_0, GPIOF_INPUT | GPIOF_PULLUP);	// F1
	hal_gpio_pin_config(PIO0_1, GPIOF_INPUT | GPIOF_PULLUP);	// F2
	hal_gpio_pin_config(PIO0_2, GPIOF_INPUT | GPIOF_PULLUP);	// F3
	hal_gpio_pin_config(PIO0_3, GPIOF_INPUT | GPIOF_PULLUP);	// F4
	hal_gpio_pin_config(PIO0_29, GPIOF_INPUT | GPIOF_PULLUP);	// F5 <- BUTTON_OFF
	#define BUTTON_OFF_PIN	PIO0_29
	hal_gpio_pin_config(PIO0_28, GPIOF_INPUT | GPIOF_PULLUP);	// F6

	hal_gpio_pin_config(PIO0_27, GPIOF_INPUT | GPIOF_PULLUP);	// RPM_IN
	// TODO

	hal_gpio_pin_config(PIO0_14, GPIOF_INPUT | GPIOF_PULLUP);	// PPM_IN/IPU_IN_A0->IPU_SAMPLE0->SCT0
	lpc_sctipu_sample_config(0, SCTIPU_SAMPLE_INPUT_A, SCTIPU_SAMPLE_TRANSPARENT);
	LPC_INMUX->SCT0_INMUX[0] = SCT0_INP_SCTIPU_SAMPLE0;
	LPC_IOCON->PIO0_14 = IOCON_MODE_PULL_UP | IOCON_HYSTERESIS | IOCON_CLKDIV_BY2 | IOCON_INV_INPUT;
	#define PPM_SCT_MODULE 0
	#define PPM_MAX_CHANNEL_COUNT 10	// NOTE: big SCT allows 6+ channels
//	#define SCT_DEBUG_OUTPUT 2
//	lpc_swm_assign(LPC_SWMA_SCT0_OUT2, LPC_SWM_PIO0_30);	// connect debug output to trim out

	hal_gpio_pin_config(PIO0_4, GPIOF_INPUT | GPIOF_PULLUP);	// PWM_IN
	LPC_INMUX->SCT2_INMUX[0] = SCT2_INP_PIO0_4;
	#define PWMIN_SCT_MODULE 2

	hal_gpio_pin_config(PIO0_5, GPIOF_INPUT | GPIOF_PULLUP);	// DSM2_IN
//	LPC_IOCON->PIO0_5 = IOCON_MODE_PULL_UP /*| IOCON_INV_INPUT*/ | IOCON_CLKDIV_PCLK; 
	lpc_swm_assign(LPC_SWMA_UART0_RXD, LPC_SWM_PIO0_5);
	#define DSM2_INPUT_MODULE	0	// UART0 for DSM2

	#define PS_ERROR_PIN PIO0_7
	hal_gpio_pin_config(PS_ERROR_PIN, GPIOF_INPUT | GPIOF_PULLUP);	// ERROR
	#define PS_FAIL1_PIN PIO0_15
	hal_gpio_pin_config(PS_FAIL1_PIN, GPIOF_INPUT | GPIOF_PULLUP);	// FAIL1
	#define PS_FAIL2_PIN PIO0_16
	hal_gpio_pin_config(PS_FAIL2_PIN, GPIOF_INPUT | GPIOF_PULLUP);	// FAIL2
	#define PS_FAIL3_PIN PIO1_3
	hal_gpio_pin_config(PS_FAIL3_PIN, GPIOF_INPUT | GPIOF_PULLUP);	// FAIL3

	#define MOTOREN_PIN PIO0_25
	hal_gpio_pin_config(MOTOREN_PIN, GPIOF_OUTPUT);
	hal_gpio_pin_set(MOTOREN_PIN, true);	// enable buffer

	LPC_IOCON->PIO0_8 = IOCON_MODE_PULL_NONE;	//PIO0_8 is BAT (ADC0_0)
	lpc_swm_enable(LPC_SWME_ADC0_0, true);
	#define ADC0_BAT 0
	LPC_IOCON->PIO0_10 = IOCON_MODE_PULL_NONE;	//PIO0_10 is LM (ADC1_2)
	lpc_swm_enable(LPC_SWME_ADC1_2, true);
	#define ADC1_LM 2
	LPC_IOCON->PIO0_11 = IOCON_MODE_PULL_NONE;	//PIO0_11 is VSW (ADC1_3)
	lpc_swm_enable(LPC_SWME_ADC1_3, true);
	#define ADC1_VSW 3

	#define ADC0_MASK (1<<ADC0_BAT)
	#define ADC1_MASK (1<<ADC1_LM) | (1<<ADC1_VSW)

	#define LED0_PIN PIO1_4
	hal_gpio_pin_config(LED0_PIN, GPIOF_OUTPUT); //PIO1_4 is LEDR
	hal_gpio_pin_set(LED0_PIN, 0);
	hal_gpio_pin_set(LED0_PIN, 1);

	#define LED1_PIN PIO1_5
	hal_gpio_pin_config(LED1_PIN, GPIOF_OUTPUT); //PIO1_5 is LEDG
	hal_gpio_pin_set(LED1_PIN, 0);
	hal_gpio_pin_set(LED1_PIN, 1);

	#define LED2_PIN PIO1_6
	hal_gpio_pin_config(LED2_PIN, GPIOF_OUTPUT); //PIO1_6 is LEDB
	hal_gpio_pin_set(LED2_PIN, 0);
	hal_gpio_pin_set(LED2_PIN, 1);

	LPC_IOCON->PIO0_17 = IOCON_MODE_PULL_NONE | IOCON_CLKDIV_BY2; 
	#define OUTPUT_PSWEN_PIN PIO0_17
	hal_gpio_pin_config(OUTPUT_PSWEN_PIN, GPIOF_OUTPUT); //PIO0_17 is POWER_SWITCH (PSWEN)
	hal_gpio_pin_set(OUTPUT_PSWEN_PIN, 0);
	#define OUTPUT_FETEN_PIN PIO1_2
	hal_gpio_pin_config(OUTPUT_FETEN_PIN, GPIOF_OUTPUT); //PIO1_17 is FET_SWITCH (FETEN)
	hal_gpio_pin_set(OUTPUT_FETEN_PIN, 0);

	LPC_IOCON->PIO0_12 = IOCON_MODE_PULL_NONE | IOCON_CLKDIV_BY2; 
	lpc_swm_enable(LPC_SWME_DAC_OUT, true);	// PIO0_12 is CURRENT_OUT

	lpc_swm_assign(LPC_SWMA_CAN_RD1, LPC_SWM_PIO0_13);	// P0_13 (CAN_RD1)
	lpc_swm_assign(LPC_SWMA_CAN_TD1, LPC_SWM_PIO0_31);	// P0_31 (CAN_TD1)

	#define OUTPUT_KILL_PIN PIO0_26
	hal_gpio_pin_config(OUTPUT_KILL_PIN, GPIOF_OUTPUT); //PIO0_26 is KILLN
	hal_gpio_pin_set(OUTPUT_KILL_PIN, 0);	// default KILLN low = ENGINE POWER off 

#else
#error "Board not defined"
#endif

#ifdef ADC0_MASK
	adc_initialize(0, 1000000, ADC_INITF_NONE);
	adc_config_seq_irqmode(0, ADC_SEQ_A, &(adc_seq_config_t) 
		{ .Channels = ADC0_MASK },
		_adc_handler);
#endif
#ifdef ADC1_MASK
	adc_initialize(1, 1000000, ADC_INITF_NONE);
	adc_config_seq_irqmode(1, ADC_SEQ_A, &(adc_seq_config_t) 
		{ .Channels = ADC1_MASK },
		_adc_handler);
#endif

#ifdef PWMIN_SCT_MODULE
	_init_pwmin();
#endif

	hal_dac_initialize();

//	lpc_inmux_disable();
}


void board_eeprom_create_context(eeprom_context_t *context)
{
	ASSERT(context != nullptr, KERNEL_ERROR_NULL_POINTER);
	static eeprom_geometry_t _geometry = { .Size = 1024, .PageSize = 0 };
	eeprom_context_create(context, &_geometry, __lpc15_eeprom_driver, nullptr);
}

static event_t *_adc_event = nullptr;

static void _adc_handler(unsigned module, adc_seq_t seq)
{
	if (_adc_event != nullptr)
		event_set(_adc_event);
}

static void _adc_update(unsigned module)
{
	event_t ev;
	event_create(&ev, EVENTF_NONE);
	_adc_event = &ev;
	adc_start(module, ADC_SEQ_A);
	event_wait(&ev, TIMEOUT_NEVER);
	_adc_event = nullptr;
}

unsigned board_analog_update(board_analog_input_t *input)
{
	unsigned done = 0;
	unsigned short vsw, bat, lm;
#ifdef ADC0_MASK
	_adc_update(0);
#ifdef ADC0_VSW
	if (adc_read(0, ADC0_VSW, &vsw)) done |= BOARD_ANF_VSW;
#endif
#ifdef ADC0_BAT
	if (adc_read(0, ADC0_BAT, &bat)) done |= BOARD_ANF_BAT;
#endif
#ifdef ADC0_LM
	if (adc_read(0, ADC0_LM, &lm)) done |= BOARD_ANF_LM;
#endif
#endif

#ifdef ADC1_MASK
	_adc_update(1);
#ifdef ADC1_VSW
	if (adc_read(1, ADC1_VSW, &vsw)) done |= BOARD_ANF_VSW;
#endif
#ifdef ADC1_BAT
	if (adc_read(1, ADC1_BAT, &bat)) done |= BOARD_ANF_BAT;
#endif
#ifdef ADC1_LM
	if (adc_read(1, ADC1_LM, &lm)) done |= BOARD_ANF_LM;
#endif
#endif

#ifdef VSW_DIVIDER_RATIO
	input->VSW = (vsw * 3300 * VSW_DIVIDER_RATIO) >> 16;
#else
	input->VSW = (vsw * 3300) >> 16;
#endif
	input->BAT = (bat * 3300) >> 16;
	input->LM = (lm * 3300) >> 16;
	return done;
}

input_mask_t board_get_input(input_mask_t mask)
{
	input_mask_t input = 0;

#ifdef BUTTON_OFF_PIN
	if (mask & INPUTF_BUTTON_OFF)
	{
		if (!hal_gpio_pin(BUTTON_OFF_PIN))
			input |= INPUTF_BUTTON_OFF;
	}
#endif
#ifdef PS_ERROR_PIN
	if (mask & INPUTF_ERROR)
	{
		if (!hal_gpio_pin(PS_ERROR_PIN))
			input |= INPUTF_ERROR;
	}
#endif
#ifdef PS_FAIL1_PIN
	if (mask & INPUTF_FAIL1)
	{
		if (hal_gpio_pin(PS_FAIL1_PIN))
			input |= INPUTF_FAIL1;
	}
#endif
#ifdef PS_FAIL2_PIN
	if (mask & INPUTF_FAIL2)
	{
		if (hal_gpio_pin(PS_FAIL2_PIN))
			input |= INPUTF_FAIL2;
	}
#endif
#ifdef PS_FAIL3_PIN
	if (mask & INPUTF_FAIL3)
	{
		if (hal_gpio_pin(PS_FAIL3_PIN))
			input |= INPUTF_FAIL3;
	}
#endif

	return input;
}

void board_set_dac(unsigned output, unsigned value)
{
	unsigned sample = (value * 1271) >> 6;	// NOTE: input 0-3300 (mV)
	if (sample > 65535) sample = 65535;

	switch(output)
	{
		case 0:
			hal_dac_write_sample(sample);
			break;
	}
}

void board_set_pwm(unsigned ch, unsigned time)
{
#ifdef PWM_SCT_MODULE
	switch(ch)
	{
		case 0: // NOTE: S0	-> throttle
		case 1:	// NOTE: S1 -> generator
			hal_pwm_set_output(PWM_SCT_MODULE, ch, (time >= 800 && time <= 2200) ? time : 0);
			break;
	}
#endif
}

#ifdef PWMIN_SCT_MODULE
static void _init_pwmin()
{
	sct_initialize(PWMIN_SCT_MODULE, _pwmin_handle, SCT_INITF_NONE);
	sct_timer_setup(PWMIN_SCT_MODULE, 1000000, 0, SCT_TMRF_NONE);

	unsigned event = 0;
	unsigned event_start, event_stop;
	sct_event_setup(PWMIN_SCT_MODULE, event_start = event++, &(sct_ev_setup_t) {
		.IOSel = 0, .IOCond = SCT_EV_IO_RISE, .CombMode = SCT_EV_COMB_IO, 
		.StateVal = 1, .Flags = SCT_EVF_STATE_LOAD | SCT_EVF_INPUT | SCT_EVF_LIMIT,	
		}, (1<<0));
	sct_event_setup(PWMIN_SCT_MODULE, event_stop = event++, &(sct_ev_setup_t) {
		.IOSel = 0, .IOCond = SCT_EV_IO_FALL, .CombMode = SCT_EV_COMB_IO, 
		.StateVal = 0, .Flags = SCT_EVF_STATE_LOAD | SCT_EVF_INPUT | SCT_EVF_IRQEN,	
		}, (1<<1));
	sct_capture_setup(PWMIN_SCT_MODULE, 0, 1 << event_stop, SCT_TMRF_NONE);
	sct_control(PWMIN_SCT_MODULE, SCT_CTRLF_RUN);
}

static bool _pwm_in_ready = false;
static unsigned char _pwm_selector = 0;
static unsigned char _pwm_thr = 0;
static bool _pwm_kill = 0;

static void _pwmin_handle(unsigned module, unsigned ev_flags, unsigned state)
{
	ASSERT(module == PWMIN_SCT_MODULE, KERNEL_ERROR_KERNEL_PANIC);
	unsigned int cap = sct_capture(PWMIN_SCT_MODULE, 0, SCT_TMRF_NONE);
	if (cap >= 800 && cap <= 2200)
	{
		_pwm_in_ready = true;

		unsigned p1000 = _lim(cap, 1000, 2000) - 1000;
		_pwm_selector = (p1000 >= 950) ? 2 : 1;	// NOTE: 1 = governor manual; 2 = governor auto (generator current output)
		
		if (p1000 < 100)
			_pwm_thr = 0;
		else if (p1000 > 900)
			_pwm_thr = 100;
		else
			_pwm_thr = (p1000 - 100) / 8;

		_pwm_kill = p1000 < 50;
	}
	else 
	{
		_pwm_in_ready = false;
	}
}

#endif


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


static unsigned char _aux_selector = 1;
static unsigned char _aux_thr = 0;
static bool _aux_kill = false;
static unsigned char _aux_ready = 0;

void board_set_remote_data(unsigned sel, unsigned thr, bool kill)
{
	_aux_selector = sel;
	_aux_thr = thr;
	_aux_kill = kill;
	_aux_ready = 20;
}

bool board_get_remote_input(remote_input_t input, unsigned int *pvalue)
{
	unsigned cap;

#ifdef PWMIN_SCT_MODULE
	if (_pwm_in_ready)
	{
		switch (input)
		{
			case BOARD_REMOTE_SELECTOR:	*pvalue = _pwm_selector;		return true;
			case BOARD_REMOTE_THROTTLE:	*pvalue = _pwm_thr;		return true;
			case BOARD_REMOTE_KILL:		*pvalue = _pwm_kill ? 2000 : 1000;	return true;
		}
	}
#endif

#ifdef DSM2_INPUT_PIN
	if (_aux_ready)
	{
		switch (input)
		{
			case BOARD_REMOTE_SELECTOR:	*pvalue = _aux_selector;	return true;
			case BOARD_REMOTE_THROTTLE:	*pvalue = _aux_thr;		return true;
			case BOARD_REMOTE_KILL:		*pvalue = _aux_kill ? 2000 : 1000;	return true;
		}

		_aux_ready--;
	}
#endif

	return false;
}




void board_set_led(led_mask_t mask, led_mask_t value)
{
#ifdef LED0_PIN
	if (mask & LED_RED)
		hal_gpio_pin_set(LED0_PIN, !(value & LED_RED));
#endif
#ifdef LED1_PIN
	if (mask & LED_GREEN)
		hal_gpio_pin_set(LED1_PIN, !(value & LED_GREEN));
#endif
#ifdef LED2_PIN
	if (mask & LED_BLUE)
		hal_gpio_pin_set(LED2_PIN, !(value & LED_BLUE));
#endif
}

void board_set_output(output_mask_t mask, output_mask_t value)
{
#ifdef OUTPUT_PSWEN_PIN
	if (mask & OUTPUTF_PSW_EN)
		hal_gpio_pin_set(OUTPUT_PSWEN_PIN, value & OUTPUTF_PSW_EN);
#endif
#ifdef OUTPUT_FETEN_PIN
	if (mask & OUTPUTF_FET_EN)
		hal_gpio_pin_set(OUTPUT_FETEN_PIN, value & OUTPUTF_FET_EN);
#endif
#ifdef OUTPUT_KILL_PIN
	if (mask & OUTPUTF_KILL)
		hal_gpio_pin_set(OUTPUT_KILL_PIN, !(value & OUTPUTF_KILL));	// active low
#endif
}


void board_enable_charges(bool enable)
{
	// TODO
}

void board_fire(bool fire)
{
	// TODO
}
