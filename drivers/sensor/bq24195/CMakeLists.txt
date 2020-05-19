# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BQ24195 bq24195.c)
# zephyr_library_sources_ifdef(CONFIG_BQ24195_TRIGGER bq24195_trigger.c)
