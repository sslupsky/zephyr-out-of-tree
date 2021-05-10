# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_OOT_UART_SAM0 uart_sam0.c)
