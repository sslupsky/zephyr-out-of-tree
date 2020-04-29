/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	struct device __attribute__((unused)) *muxa = device_get_binding(DT_ATMEL_SAM0_PINMUX_PINMUX_A_LABEL);
	struct device __attribute__((unused)) *muxb = device_get_binding(DT_ATMEL_SAM0_PINMUX_PINMUX_B_LABEL);

	ARG_UNUSED(dev);

#if DT_ATMEL_SAM0_UART_SERCOM_0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_4_BASE_ADDRESS
	/* SERCOM4 on TXD=PA12, RXD=PA15 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 15, PINMUX_FUNC_D);
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_5_BASE_ADDRESS
	/* SERCOM5 on TXD=PB22, RXD=PB23 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxb, 22, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 23, PINMUX_FUNC_D);
#endif


#if DT_ATMEL_SAM0_SPI_SERCOM_0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_1_BASE_ADDRESS
	/* SERCOM1 on SCK=PA17, MOSI=PA16, MISO=PA19 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 19, PINMUX_FUNC_C);
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_2_BASE_ADDRESS
	/* SERCOM4 on SCK=PA13, MOSI=PA12, MISO=15 */
	/* reference table 7-1 and 23.8.12 */
	// pinmux_pin_set(muxa, 13, PINMUX_FUNC_C);
	// pinmux_pin_set(muxa, 12, PINMUX_FUNC_C);
	// pinmux_pin_set(muxa, 15, PINMUX_FUNC_C);
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_4_BASE_ADDRESS
	/* SERCOM4 on SCK=PA13, MOSI=PA12, MISO=15 */
	pinmux_pin_set(muxa, 13, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 15, PINMUX_FUNC_D);
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if DT_ATMEL_SAM0_I2C_SERCOM_0_BASE_ADDRESS
	/* SERCOM0 on SDA=PA08, SCL=PA09 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 9, PINMUX_FUNC_C);
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_3_BASE_ADDRESS
	/* SERCOM3 on SDA=PA22, SCL=PB23 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
