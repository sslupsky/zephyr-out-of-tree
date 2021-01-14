# Copyright (c) 2020 Scanimetrics Inc.
# SPDX-License-Identifier: Apache-2.0

# bossac only works if running west on Linux (see bossac.py runner)
# include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)

board_runner_args(jlink "--device=atsamd51j20" "--speed=4000" "--offset=0x4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

# Warning:  Openocd overwrites the bootloader, so only use openocd if not using a bootloader
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
