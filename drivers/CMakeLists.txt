# SPDX-License-Identifier: Apache-2.0

add_subdirectory_if_kconfig(flash)
add_subdirectory_if_kconfig(modem)
add_subdirectory_if_kconfig(sensor)
add_subdirectory_if_kconfig(adc)

add_subdirectory_ifdef(CONFIG_OOT_SAM0_RTC_TIMER timer)
