# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SAM0_RTC_TIMER_OOT sam0_rtc_timer.c)