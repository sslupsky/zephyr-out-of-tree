/*
 * Copyright (c) 2018 Google LLC.
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(struct device *dev)
{
	struct device *muxa = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
	struct device *muxb = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_b)));
	struct device *muxc = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_c)));
	struct device *muxd = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_d)));

	ARG_UNUSED(dev);
	ARG_UNUSED(muxa);
	ARG_UNUSED(muxb);
	ARG_UNUSED(muxc);
	ARG_UNUSED(muxd);

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
// 	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
// 	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
// #error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
//  zephyr console uart (also used for usb_cdc)
//  SERCOM4 is connected on ISB Rev 2036C.
	// pinmux_pin_set(muxb, 12, PINMUX_FUNC_C);
	// pinmux_pin_set(muxb, 13, PINMUX_FUNC_C);
// #error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
#error Pin mapping is not configured
#endif


#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	/* SERCOM0 on SDA=PA08, SCL=PA09 */
	pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 9, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 13, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	/* SERCOM5 on SDA=PB02, SCL=PB03 */
	pinmux_pin_set(muxb, 2, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 3, PINMUX_FUNC_D);
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_H);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_H);
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
