# SPDX-License-Identifier: Apache-2.0

if(CONFIG_PINMUX_SAM0)
  zephyr_library()
  zephyr_library_sources(pinmux.c)
endif()

if(CONFIG_SYS_POWER_SLEEP_STATES)
  zephyr_library_sources(isb_power.c)
endif()
