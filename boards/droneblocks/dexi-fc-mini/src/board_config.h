/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file board_config.h
 *
 * DroneBlocks DEXI-FC-Mini internal definitions.
 * Pin assignments come from the dexi-fc-mini netlist (U5 = STM32H743VITx LQFP100).
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/* LEDs: anode on the MCU pin, cathode through 10k to GND, so drive high = on. */
#define GPIO_LED_RED            /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_LED_GREEN          /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN11)
#define GPIO_LED_BLUE           /* PB15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* ADC
 *
 * PC4 ADC12_INP4: VBAT sense, R14 200k / R15 22k from VCC_24V
 * PC5 ADC12_INP8: ESC current sense (J6 pin 2)
 */
#define SYSTEM_ADC_BASE STM32_ADC1_BASE
#define ADC1_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PC4 */  GPIO_ADC12_INP4, \
	/* PC5 */  GPIO_ADC12_INP8

#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC4 */  ADC1_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC5 */  ADC1_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define BOARD_BATTERY1_V_DIV         (10.09f)   /* (200k + 22k) / 22k */
#define BOARD_BATTERY1_A_PER_V       (50.0f)    /* dexi-4in1: 20 mV/A */

/* PWM: TIM1 CH1-CH4 on J6 (PE9, PE11, PE13, PE14) */
#define DIRECT_PWM_OUTPUT_CHANNELS   4
#define DIRECT_INPUT_TIMER_CHANNELS  4

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* USB OTG FS
 *
 * The board has no VBUS sense pin (PA9 is unconnected), so
 * board_read_VBUS_state() in init.c always reports USB as present.
 */

/* High-resolution timer: TIM2 has no pins on this board */
#define HRT_TIMER               2  /* use timer2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* RC serial port: UART5 (RX PB12 through the U19 XOR inverter, TX PB13 on J7 pin 3) */
#define RC_SERIAL_PORT          "/dev/ttyS2"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* PD14 SBUS_INV drives U19 (SN74LVC1G86): high inverts the J7 RC line into UART5_RX.
 * R38 pulls it low, so the line is not inverted at reset (CRSF / GHST work as-is). */
#define GPIO_SBUS_INV           /* PD14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define RC_INVERT_INPUT(_invert_true)  px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true)

/* DPS368 SDO / interrupt (PB5): an input with pull-up so the I2C address stays 0x77 */
#define GPIO_BARO_DRDY          /* PB5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)

/* microSD power: PC3_C SD_PWR_EN to the RT9013 enable, pulled up by R39 (on at reset).
 * Left unconfigured by firmware on purpose (see the hardware notes in the PR). */
#define GPIO_SD_PWR_EN          /* PC3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)

/* SD card */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,            \
		GPIO_SBUS_INV,           \
		GPIO_BARO_DRDY,          \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define FLASH_BASED_PARAMS

#define BOARD_NUM_IO_TIMERS 1

__BEGIN_DECLS

#ifndef __ASSEMBLY__

int stm32_sdio_initialize(void);

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
