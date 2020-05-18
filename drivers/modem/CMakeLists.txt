# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_MODEM_MURATA_LORA murata-lora-cmwx1zzabz.c)

