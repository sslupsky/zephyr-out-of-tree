/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/fram.h>

static inline int z_vrfy_fram_read(struct device *dev, off_t offset,
				     void *data, size_t len)
{
	Z_OOPS(Z_SYSCALL_DRIVER_EEPROM(dev, read));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(data, len));
	return z_impl_fram_read((struct device *)dev, offset, (void *)data,
				  len);
}
#include <syscalls/fram_read_mrsh.c>

static inline int z_vrfy_fram_write(struct device *dev, off_t offset,
				      const void *data, size_t len)
{
	Z_OOPS(Z_SYSCALL_DRIVER_EEPROM(dev, write));
	Z_OOPS(Z_SYSCALL_MEMORY_READ(data, len));
	return z_impl_fram_write((struct device *)dev, offset,
				   (const void *)data, len);
}
#include <syscalls/fram_write_mrsh.c>

static inline size_t z_vrfy_fram_get_size(struct device *dev)
{
	Z_OOPS(Z_SYSCALL_DRIVER_EEPROM(dev, size));
	return z_impl_fram_get_size((struct device *)dev);
}
#include <syscalls/fram_get_size_mrsh.c>

