/**
 * @file
 * @brief GCLK Devicetree macro public API header file.
 */

/*
 * Copyright (c) 2020, Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DEVICETREE_GCLK_H_
#define ZEPHYR_INCLUDE_DEVICETREE_GCLK_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup devicetree-gclk Devicetree GCLK API
 * @ingroup devicetree
 * @{
 */

#define DT_INST_GCLK_REG_ADDR(inst)				\
	DT_REG_ADDR(DT_INST_PHANDLE_BY_NAME(inst, clocks, gclk))

#define DT_INST_GCLK_PROP(inst, prop)				\
	DT_PROP(DT_INST_PHANDLE_BY_NAME(inst, clocks, gclk), prop)

#define DT_INST_GCLK_FREQ_HZ(inst)				\
	DT_INST_GCLK_PROP(inst, clock_frequency)


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_INCLUDE_DEVICETREE_GCLK_H_ */
