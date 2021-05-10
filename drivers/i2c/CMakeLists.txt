# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_OOT_I2C_SAM0 i2c_sam0.c)
