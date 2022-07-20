/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-07
 * 
 * @copyright Copyright (c) 2020
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef WITAP_TYPES_H
#define WITAP_TYPES_H

#include <zephyr.h>
#include <device.h>

typedef struct {
    u32_t commissionTime;
    u32_t updateTime;
//     gps_t installLocation;
} CONFIG_FILE_t;


/* Forward declaration of "C" functions */
#ifdef __cplusplus
extern "C" {
#endif

	int spi_nand_read_parameter_page(struct device *);
	void spi_nand_get_registers(struct device *dev, u8_t *status, u8_t *ctrl, u8_t *lock);
	s64_t sam0_rtc_timer_boot_time(void);


#ifdef __cplusplus
}
#endif

#endif /* WITAP_TYPES_H */