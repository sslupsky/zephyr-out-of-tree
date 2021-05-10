/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int board_pinmux_init(struct device *dev)
{
	struct device __attribute__((unused)) *muxa = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_a)));
	struct device __attribute__((unused)) *muxb = device_get_binding(DT_LABEL(DT_NODELABEL(pinmux_b)));

	ARG_UNUSED(dev);

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
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
	/* SERCOM4 on TXD=PA12, RXD=PA15 is FUNCTION "D" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 15, PINMUX_FUNC_D);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_uart) && (CONFIG_UART_SAM0 || CONFIG_OOT_UART_SAM0))
	/* SERCOM5 on TXD=PB22, RXD=PB23 is FUNCTION "D" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxb, 22, PINMUX_FUNC_D);
	pinmux_pin_set(muxb, 23, PINMUX_FUNC_D);
#endif


#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_spi) && CONFIG_SPI_SAM0)
	/* SERCOM1 on SCK=PA17, MOSI=PA16, MISO=PA19 is FUNCTION "C" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 17, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 16, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 19, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_spi) && CONFIG_SPI_SAM0)
	/* SERCOM2 on SCK=PA13, MOSI=PA12, MISO=15 is FUNCTION "C" */
	/* reference table 7-1 and 23.8.12 */
	// pinmux_pin_set(muxa, 13, PINMUX_FUNC_C);
	// pinmux_pin_set(muxa, 12, PINMUX_FUNC_C);
	// pinmux_pin_set(muxa, 15, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_spi) && CONFIG_SPI_SAM0)
	/* SERCOM4 on SCK=PA13, MOSI=PA12, MISO=15 is FUNCTION "D" */
	pinmux_pin_set(muxa, 13, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 12, PINMUX_FUNC_D);
	pinmux_pin_set(muxa, 15, PINMUX_FUNC_D);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_spi) && CONFIG_SPI_SAM0)
#error Pin mapping is not configured
#endif

#if (ATMEL_SAM0_DT_SERCOM_CHECK(0, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	/* SERCOM0 on SDA=PA08, SCL=PA09 is FUNCTION "C" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 8, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 9, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(1, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(2, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(3, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
	/* SERCOM3 on SDA=PA22, SCL=PB23 is FUNCTION "C" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 22, PINMUX_FUNC_C);
	pinmux_pin_set(muxa, 23, PINMUX_FUNC_C);
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(4, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif
#if (ATMEL_SAM0_DT_SERCOM_CHECK(5, atmel_sam0_i2c) && (CONFIG_I2C_SAM0 || CONFIG_OOT_I2C_SAM0))
#error Pin mapping is not configured
#endif

#ifdef CONFIG_USB_DC_SAM0
	/* USB DP on PA25, USB DM on PA24 is FUNCTION "G" */
	/* reference table 7-1 and 23.8.12 */
	pinmux_pin_set(muxa, 25, PINMUX_FUNC_G);
	pinmux_pin_set(muxa, 24, PINMUX_FUNC_G);
#endif

	return 0;
}

SYS_INIT(board_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
