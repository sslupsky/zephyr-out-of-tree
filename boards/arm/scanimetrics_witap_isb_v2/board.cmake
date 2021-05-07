# Copyright (c) 2020 Scanimetrics Inc.
# SPDX-License-Identifier: Apache-2.0

# added support for macOS in bossac runner (see bossac.py runner)
include(${ZEPHYR_BASE}/boards/common/bossac.board.cmake)

board_runner_args(jlink "--device=atsamd51j20" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

# Warning:  Openocd overwrites the bootloader, so only use openocd if not using a bootloader
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
