/*
 * Copyright (c) 2022 Scanimetrics Inc.
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * Heavily based on drivers/flash.h which is:
 * Copyright (c) 2017 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for FRAM drivers
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FRAM_H_
#define ZEPHYR_INCLUDE_DRIVERS_FRAM_H_

/**
 * @brief FRAM Interface
 * @defgroup fram_interface FRAM Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/types.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*fram_api_read)(struct device *dev, off_t offset, void *data,
			       size_t len);
typedef int (*fram_api_write)(struct device *dev, off_t offset,
				const void *data, size_t len);
typedef size_t (*fram_api_size)(struct device *dev);

__subsystem struct fram_driver_api {
	fram_api_read read;
	fram_api_write write;
	fram_api_size size;
};

/**
 *  @brief Read data from FRAM
 *
 *  @param dev FRAM device
 *  @param offset Address offset to read from.
 *  @param data Buffer to store read data.
 *  @param len Number of bytes to read.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int fram_read(struct device *dev, off_t offset, void *data,
			  size_t len);

static inline int z_impl_fram_read(struct device *dev, off_t offset,
				     void *data, size_t len)
{
	const struct fram_driver_api *api =
		(const struct fram_driver_api *)dev->driver_api;

	return api->read(dev, offset, data, len);
}

/**
 *  @brief Write data to FRAM
 *
 *  @param dev FRAM device
 *  @param offset Address offset to write data to.
 *  @param data Buffer with data to write.
 *  @param len Number of bytes to write.
 *
 *  @return 0 on success, negative errno code on failure.
 */
__syscall int fram_write(struct device *dev, off_t offset, const void *data,
			   size_t len);

static inline int z_impl_fram_write(struct device *dev, off_t offset,
				      const void *data, size_t len)
{
	const struct fram_driver_api *api =
		(const struct fram_driver_api *)dev->driver_api;

	return api->write(dev, offset, data, len);
}

/**
 *  @brief Get the size of the FRAM in bytes
 *
 *  @param dev FRAM device.
 *
 *  @return FRAM size in bytes.
 */
__syscall size_t fram_get_size(struct device *dev);

static inline size_t z_impl_fram_get_size(struct device *dev)
{
	const struct fram_driver_api *api =
		(const struct fram_driver_api *)dev->driver_api;

	return api->size(dev);
}

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#include <syscalls/fram.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_FRAM_H_ */
