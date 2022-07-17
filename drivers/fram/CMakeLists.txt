# SPDX-License-Identifier: Apache-2.0

# zephyr_library()

zephyr_library_sources_ifdef(CONFIG_USERSPACE fram_handlers.c)

zephyr_library_sources_ifdef(CONFIG_FRAM fram_mb85rs.c)
zephyr_library_sources_ifdef(CONFIG_FRAM_SHELL fram_shell.c)

