# SPDX-License-Identifier: Apache-2.0

add_subdirectory_if_kconfig(flash)
add_subdirectory_if_kconfig(modem)
add_subdirectory_if_kconfig(sensor)
add_subdirectory_if_kconfig(adc)

add_subdirectory_ifdef(CONFIG_OOT_SAM0_RTC_TIMER timer)
add_subdirectory_ifdef(CONFIG_OOT_I2C_SAM0 i2c)
add_subdirectory_ifdef(CONFIG_OOT_UART_SAM0 serial)
