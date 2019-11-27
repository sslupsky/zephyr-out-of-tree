/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa = device_get_binding(DT_ATMEL_SAM0_PINMUX_PINMUX_A_LABEL);
	struct device *muxb = device_get_binding(DT_ATMEL_SAM0_PINMUX_PINMUX_B_LABEL);

	ARG_UNUSED(dev);

#if DT_ATMEL_SAM0_UART_SERCOM_0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_1_BASE_ADDRESS
//  dummy sercom uart for console and shell via usb
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_UART_SERCOM_5_BASE_ADDRESS
#error Pin mapping is not configured
#endif


#if DT_ATMEL_SAM0_SPI_SERCOM_0_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_1_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_2_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_3_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_SPI_SERCOM_5_BASE_ADDRESS
#error Pin mapping is not configured
#endif

#if DT_ATMEL_SAM0_I2C_SERCOM_0_BASE_ADDRESS
	/* SERCOM0 on SDA=PA08, SCL=PA09 */
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
	/* SERCOM0 on SDA=PA22, SCL=PA23 */
	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_4_BASE_ADDRESS
#error Pin mapping is not configured
#endif
#if DT_ATMEL_SAM0_I2C_SERCOM_5_BASE_ADDRESS
	/* SERCOM0 on SDA=PB02, SCL=PB03 */
	pinmux_pin_set(muxb, 2, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 3, PINMUX_FUNC_D);
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
