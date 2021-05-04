# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_OOT_SAM0_RTC_TIMER sam0_rtc_timer.c)
