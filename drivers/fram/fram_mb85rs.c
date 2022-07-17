/**
 * @file fram_mb85rs.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2019 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for Fujitsu MB85RS SPI FRAMs.
 */

#include <drivers/fram.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_FRAM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(fram_mb85rsx);

#include "fram_mb85rs.h"

static inline int fram_mb85rsx_write_protect(struct device *dev)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;

	if (!data->wp_gpio_dev) {
		return 0;
	}

	return gpio_pin_set(data->wp_gpio_dev, config->wp_gpio_pin, 1);
}

static inline int fram_mb85rsx_write_enable(struct device *dev)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;

	if (!data->wp_gpio_dev) {
		return 0;
	}

	return gpio_pin_set(data->wp_gpio_dev, config->wp_gpio_pin, 0);
}

static int fram_mb85rsx_read(struct device *dev, off_t offset, void *buf,
			    size_t len)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;
	int err;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	err = config->read_fn(dev, offset, buf, len);
	k_mutex_unlock(&data->lock);

	if (err) {
		LOG_ERR("failed to read FRAM (err %d)", err);
		return err;
	}

	return 0;
}

static size_t fram_mb85rsx_limit_write_count(struct device *dev, off_t offset,
					    size_t len)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	size_t count = len;
	off_t page_boundary;

	/* We can at most write one page at a time */
	if (count > config->pagesize) {
		count = config->pagesize;
	}

	/* Writes can not cross a page boundary */
	page_boundary = ROUND_UP(offset + 1, config->pagesize);
	if (offset + count > page_boundary) {
		count = page_boundary - offset;
	}

	return count;
}

static int fram_mb85rsx_write(struct device *dev, off_t offset, const void *buf,
			     size_t len)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;
	const uint8_t *pbuf = buf;
	int ret;

	if (config->readonly) {
		LOG_WRN("attempt to write to read-only device");
		return -EACCES;
	}

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = fram_mb85rsx_write_enable(dev);
	if (ret) {
		LOG_ERR("failed to write-enable FRAM (err %d)", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	while (len) {
		ret = config->write_fn(dev, offset, pbuf, len);
		if (ret < 0) {
			LOG_ERR("failed to write to FRAM (err %d)", ret);
			fram_mb85rsx_write_protect(dev);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		pbuf += ret;
		offset += ret;
		len -= ret;
	}

	ret = fram_mb85rsx_write_protect(dev);
	if (ret) {
		LOG_ERR("failed to write-protect FRAM (err %d)", ret);
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t fram_mb85rsx_size(struct device *dev)
{
	const struct fram_mb85rsx_config *config = dev->config_info;

	return config->size;
}

struct fram_mb85rsx_device_id fram_mb85rsx_id(struct device *dev)
{
	struct fram_mb85rsx_data *data = dev->driver_data;

	return data->id;
}

#ifdef CONFIG_FRAM_MB85RS
static int fram_mb85rs_rdsr(struct device *dev, uint8_t *status)
{
	struct fram_mb85rsx_data *data = dev->driver_data;
	uint8_t rdsr[2] = { FRAM_MB85RS_CMD_RDSR, 0 };
	uint8_t sr[2];
	int err;
	const struct spi_buf tx_buf = {
		.buf = rdsr,
		.len = sizeof(rdsr),
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_buf = {
		.buf = sr,
		.len = sizeof(sr),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1,
	};

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, &rx);
	if (!err) {
		*status = sr[1];
	}

	return err;
}

static int fram_mb85rs_wait_for_idle(struct device *dev)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	int64_t timeout;
	uint8_t status;
	int err;

	timeout = k_uptime_get() + config->timeout;
	do {
		err = fram_mb85rs_rdsr(dev, &status);
		if (err) {
			LOG_ERR("Could not read status register (err %d)", err);
			return err;
		}

		if (!(status & FRAM_MB85RS_STATUS_WIP)) {
			return 0;
		}
		k_sleep(K_MSEC(1));
	} while (timeout > k_uptime_get());

	return -EBUSY;
}

static int fram_mb85rs_read(struct device *dev, off_t offset, void *buf,
			    size_t len)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t cmd[4] = { FRAM_MB85RS_CMD_READ, 0, 0, 0 };
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_buf = {
		.buf = cmd,
		.len = cmd_len,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf rx_bufs[2] = {
		{
			.buf = NULL,
			.len = cmd_len,
		},
		{
			.buf = buf,
			.len = len,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to read past device boundary");
		return -EINVAL;
	}

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 24:
		*paddr++ = offset >> 16;
		/* Fallthrough */
	case 16:
		*paddr++ = offset >> 8;
		/* Fallthrough */
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = fram_mb85rs_wait_for_idle(dev);
	if (err) {
		LOG_ERR("FRAM idle wait failed (err %d)", err);
		k_mutex_unlock(&data->lock);
		return err;
	}

	return spi_transceive(data->bus_dev, &data->spi_cfg, &tx, &rx);
}

static int fram_mb85rs_wren(struct device *dev)
{
	struct fram_mb85rsx_data *data = dev->driver_data;
	uint8_t cmd = FRAM_MB85RS_CMD_WREN;
	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write(data->bus_dev, &data->spi_cfg, &tx);
}

static int fram_mb85rs_write(struct device *dev, off_t offset,
			     const void *buf, size_t len)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;
	int count = fram_mb85rsx_limit_write_count(dev, offset, len);
	uint8_t cmd[4] = { FRAM_MB85RS_CMD_WRITE, 0, 0, 0 };
	size_t cmd_len = 1 + config->addr_width / 8;
	uint8_t *paddr;
	int err;
	const struct spi_buf tx_bufs[2] = {
		{
			.buf = cmd,
			.len = cmd_len,
		},
		{
			.buf = (void *)buf,
			.len = count,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	paddr = &cmd[1];
	switch (config->addr_width) {
	case 24:
		*paddr++ = offset >> 16;
		/* Fallthrough */
	case 16:
		*paddr++ = offset >> 8;
		/* Fallthrough */
	case 8:
		*paddr++ = offset;
		break;
	default:
		__ASSERT(0, "invalid address width");
	}

	err = fram_mb85rs_wait_for_idle(dev);
	if (err) {
		LOG_ERR("FRAM idle wait failed (err %d)", err);
		return err;
	}

	err = fram_mb85rs_wren(dev);
	if (err) {
		LOG_ERR("failed to disable write protection (err %d)", err);
		return err;
	}

	err = spi_transceive(data->bus_dev, &data->spi_cfg, &tx, NULL);
	if (err) {
		return err;
	}

	return count;
}
#endif /* CONFIG_FRAM_MB85RS */

static int fram_mb85rsx_init(struct device *dev)
{
	const struct fram_mb85rsx_config *config = dev->config_info;
	struct fram_mb85rsx_data *data = dev->driver_data;
	int err;

	k_mutex_init(&data->lock);

	data->bus_dev = device_get_binding(config->bus_dev_name);
	if (!data->bus_dev) {
		LOG_ERR("could not get parent bus device");
		return -EINVAL;
	}

#ifdef CONFIG_FRAM_MB85RS
	data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
		SPI_WORD_SET(8);
	data->spi_cfg.frequency = config->max_freq;
	data->spi_cfg.slave = config->bus_addr;

	if (config->spi_cs_dev_name) {
		data->spi_cs.gpio_dev =
			device_get_binding(config->spi_cs_dev_name);
		if (!data->spi_cs.gpio_dev) {
			LOG_ERR("could not get SPI CS GPIO device");
			return -EINVAL;
		}

		data->spi_cs.gpio_pin = config->spi_cs_pin;
		data->spi_cfg.cs = &data->spi_cs;
	}
#endif /* CONFIG_FRAM_MB85RS */

	if (config->wp_gpio_name) {
		data->wp_gpio_dev = device_get_binding(config->wp_gpio_name);
		if (!data->wp_gpio_dev) {
			LOG_ERR("could not get WP GPIO device");
			return -EINVAL;
		}

		err = gpio_pin_configure(data->wp_gpio_dev, config->wp_gpio_pin,
					 GPIO_OUTPUT_ACTIVE | config->wp_gpio_flags);
		if (err) {
			LOG_ERR("failed to configure WP GPIO pin (err %d)",
				err);
			return err;
		}
	}

	return 0;
}

static const struct fram_driver_api fram_mb85rsx_api = {
	.read = fram_mb85rsx_read,
	.write = fram_mb85rsx_write,
	.size = fram_mb85rsx_size,
	.id = fram_mb85rsx_id,
};

#define ASSERT_MB85RSX_ADDR_W_VALID(w)			\
	BUILD_ASSERT(w == 8U || w == 16U || w == 24U,	\
		     "Unsupported address width")

#define INST_DT_MB85RSX(inst, t) DT_INST(inst, fujitsu_mb85rs)

#define FRAM_MB85RSX_DEVICE(n, t) \
	ASSERT_MB85RSX_ADDR_W_VALID(DT_PROP(INST_DT_MB85RSX(n, t), \
					    address_width)); \
	static const struct fram_mb85rsx_config fram_mb85rsx_config_##n = { \
		.bus_dev_name = DT_BUS_LABEL(INST_DT_MB85RSX(n, t)), \
		.bus_addr = DT_REG_ADDR(INST_DT_MB85RSX(n, t)), \
		.max_freq = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MB85RSX(n, t), \
					 spi_max_frequency), \
			DT_PROP(INST_DT_MB85RSX(n, t), spi_max_frequency)), \
		.spi_cs_dev_name = UTIL_AND( \
			DT_SPI_DEV_HAS_CS_GPIOS(INST_DT_MB85RSX(n, t)), \
			DT_SPI_DEV_CS_GPIOS_LABEL(INST_DT_MB85RSX(n, t))),	\
		.spi_cs_pin = UTIL_AND( \
			DT_SPI_DEV_HAS_CS_GPIOS(INST_DT_MB85RSX(n, t)), \
			DT_SPI_DEV_CS_GPIOS_PIN(INST_DT_MB85RSX(n, t))), \
		.wp_gpio_pin = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MB85RSX(n, t), wp_gpios), \
			DT_GPIO_PIN(INST_DT_MB85RSX(n, t), wp_gpios)), \
		.wp_gpio_flags = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MB85RSX(n, t), wp_gpios), \
			DT_GPIO_FLAGS(INST_DT_MB85RSX(n, t), wp_gpios)), \
		.wp_gpio_name = UTIL_AND( \
			DT_NODE_HAS_PROP(INST_DT_MB85RSX(n, t), wp_gpios), \
			DT_GPIO_LABEL(INST_DT_MB85RSX(n, t), wp_gpios)), \
		.size = DT_PROP(INST_DT_MB85RSX(n, t), size), \
		.pagesize = DT_PROP(INST_DT_MB85RSX(n, t), pagesize), \
		.addr_width = DT_PROP(INST_DT_MB85RSX(n, t), address_width), \
		.readonly = DT_PROP(INST_DT_MB85RSX(n, t), read_only), \
		.timeout = DT_PROP(INST_DT_MB85RSX(n, t), timeout), \
		.read_fn = fram_mb85rsx_read, \
		.write_fn = fram_mb85rsx_write, \
	}; \
	static struct fram_mb85rsx_data fram_mb85rsx_data_##n; \
	DEVICE_AND_API_INIT(fram_mb85rsx_##n, \
			    DT_LABEL(INST_DT_MB85RSX(n, t)), \
			    &fram_mb85rsx_init, &fram_mb85rsx_data_##n, \
			    &fram_mb85rsx_config_##n, POST_KERNEL, \
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			    &fram_mb85rsx_api)

#define FRAM_MB85RS_DEVICE(n) FRAM_MB85RSX_DEVICE(n, 25)

#define CALL_WITH_ARG(arg, expr) expr(arg);

#define INST_DT_MB85RSX_FOREACH(t, inst_expr) \
	UTIL_LISTIFY(DT_NUM_INST_STATUS_OKAY(fujitsu_mb85rs),	\
		     CALL_WITH_ARG, inst_expr)

#ifdef CONFIG_FRAM_MB85RS
INST_DT_MB85RSX_FOREACH(25, FRAM_MB85RS_DEVICE);
#endif
