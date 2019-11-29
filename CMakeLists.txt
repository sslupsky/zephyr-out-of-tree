# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_ICM20649 icm20649.c)
zephyr_library_sources_ifdef(CONFIG_ICM20649_TRIGGER icm20649_trigger.c)
