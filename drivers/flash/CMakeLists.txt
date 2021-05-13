# SPDX-License-Identifier: Apache-2.0

# zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SPI_NAND spi_nand.c)
zephyr_library_sources_ifdef(CONFIG_SPI_NAND_SHELL spi_nand_shell.c)

