# Copyright (c) 2020, Scanimetrics
# SPDX-License-Identifier: Apache-2.0

# Suppress "node name for SPI buses should be 'spi'" warnings.
list(APPEND EXTRA_DTC_FLAGS "-Wno-spi_bus_bridge")

# Suppress "duplicate unit-address (also used in node /soc/gpio@41004400)" warnings.
#list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
