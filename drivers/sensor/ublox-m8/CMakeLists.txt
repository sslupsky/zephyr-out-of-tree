# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_UBLOX_M8 ublox_m8.c)
zephyr_library_sources_ifdef(CONFIG_UBLOX_M8_TRIGGER ublox_m8_trigger.c)
