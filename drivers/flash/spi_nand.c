/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * This driver is heavily inspired from the spi_flash_w25qxxdv.c SPI NAND driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT jedec_spi_nand

#include <errno.h>
#include <drivers/flash.h>
#include <drivers/spi.h>
#include <init.h>
#include <string.h>
#include <logging/log.h>

#include "spi_nand.h"
#include "flash_priv.h"

LOG_MODULE_REGISTER(spi_nand, CONFIG_FLASH_LOG_LEVEL);

/* Device Power Management Notes
 *
 * These flash devices have several modes during operation:
 * * When CSn is asserted (during a SPI operation) the device is
 *   active.
 * * When CSn is deasserted the device enters a standby mode.
 * * Some devices support a Deep Power-Down mode which reduces current
 *   to as little as 0.1% of standby.
 *
 * The power reduction from DPD is sufficent to warrant allowing its
 * use even in cases where Zephyr's device power management is not
 * available.  This is selected through the SPI_NAND_IDLE_IN_DPD
 * Kconfig option.
 *
 * When mapped to the Zephyr Device Power Management states:
 * * DEVICE_PM_ACTIVE_STATE covers both active and standby modes;
 * * DEVICE_PM_LOW_POWER_STATE, DEVICE_PM_SUSPEND_STATE, and
 *   DEVICE_PM_OFF_STATE all correspond to deep-power-down mode.
 */

#define SPI_NAND_MAX_ADDR_WIDTH 4

#ifndef NSEC_PER_MSEC
#define NSEC_PER_MSEC (NSEC_PER_USEC * USEC_PER_MSEC)
#endif

#if DT_INST_NODE_HAS_PROP(0, t_enter_dpd)
#define T_DP_MS ceiling_fraction(DT_INST_PROP(0, t_enter_dpd), NSEC_PER_MSEC)
#else /* T_ENTER_DPD */
#define T_DP_MS 0
#endif /* T_ENTER_DPD */
#if DT_INST_NODE_HAS_PROP(0, t_exit_dpd)
#define T_RES1_MS ceiling_fraction(DT_INST_PROP(0, t_exit_dpd), NSEC_PER_MSEC)
#endif /* T_EXIT_DPD */
#if DT_INST_NODE_HAS_PROP(0, dpd_wakeup_sequence)
#define T_DPDD_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_0), NSEC_PER_MSEC)
#define T_CRDP_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_1), NSEC_PER_MSEC)
#define T_RDP_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_2), NSEC_PER_MSEC)
#else /* DPD_WAKEUP_SEQUENCE */
#define T_DPDD_MS 0
#endif /* DPD_WAKEUP_SEQUENCE */

/**
 * struct spi_nand_data - Structure for defining the SPI NAND access
 * @spi: The SPI device
 * @spi_cfg: The SPI configuration
 * @cs_ctrl: The GPIO pin used to emulate the SPI CS if required
 * @sem: The semaphore to access to the flash
 */
struct spi_nand_data {
	struct device *spi;
	struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	/* Low 32-bits of uptime counter at which device last entered
	 * deep power-down.
	 */
	u32_t ts_enter_dpd;
#endif
	struct k_sem sem;
};

/* Capture the time at which the device entered deep power-down. */
static inline void record_entered_dpd(const struct device *const dev)
{
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	struct spi_nand_data *const driver_data = dev->driver_data;

	driver_data->ts_enter_dpd = k_uptime_get_32();
#endif
}

/* Check the current time against the time DPD was entered and delay
 * until it's ok to initiate the DPD exit process.
 */
static inline void delay_until_exit_dpd_ok(const struct device *const dev)
{
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	struct spi_nand_data *const driver_data = dev->driver_data;
	s32_t since = (s32_t)(k_uptime_get_32() - driver_data->ts_enter_dpd);

	/* If the time is negative the 32-bit counter has wrapped,
	 * which is certainly long enough no further delay is
	 * required.  Otherwise we have to check whether it's been
	 * long enough taking into account necessary delays for
	 * entering and exiting DPD.
	 */
	if (since >= 0) {
		/* Subtract time required for DPD to be reached */
		since -= T_DP_MS;

		/* Subtract time required in DPD before exit */
		since -= T_DPDD_MS;

		/* If the adjusted time is negative we have to wait
		 * until it reaches zero before we can proceed.
		 */
		if (since < 0) {
			k_sleep(K_MSEC((u32_t)-since));
		}
	}
#endif /* DT_INST_NODE_HAS_PROP(0, has_dpd) */
}

/*
 * @brief Send an SPI command
 *
 * @param dev Device struct
 * @param opcode The command to send
 * @param is_addressed A flag to define if the command is addressed
 * @param addr The address to send
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @param is_write A flag to define if it's a read or a write command
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_access(const struct device *const dev,
			  u8_t opcode, bool is_addressed, off_t addr,
			  void *data, size_t length, bool is_write)
{
	struct spi_nand_data *const driver_data = dev->driver_data;

	u8_t buf[4] = {
		opcode,
		(addr & 0xFF0000) >> 16,
		(addr & 0xFF00) >> 8,
		(addr & 0xFF),
	};

	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = (is_addressed) ? 4 : 1,
		},
		{
			.buf = data,
			.len = length
		}
	};
	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (length) ? 2 : 1
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2
	};

	if (is_write) {
		return spi_write(driver_data->spi,
			&driver_data->spi_cfg, &tx_set);
	}

	return spi_transceive(driver_data->spi,
		&driver_data->spi_cfg, &tx_set, &rx_set);
}

#define spi_nand_cmd_read(dev, opcode, dest, length) \
	spi_nand_access(dev, opcode, false, 0, dest, length, false)
#define spi_nand_cmd_addr_read(dev, opcode, addr, dest, length) \
	spi_nand_access(dev, opcode, true, addr, dest, length, false)
#define spi_nand_cmd_write(dev, opcode) \
	spi_nand_access(dev, opcode, false, 0, NULL, 0, true)
#define spi_nand_cmd_addr_write(dev, opcode, addr, src, length) \
	spi_nand_access(dev, opcode, true, addr, (void *)src, length, true)

static int enter_dpd(const struct device *const dev)
{
	int ret = 0;

	if (IS_ENABLED(DT_INST_PROP(0, has_dpd))) {
		ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_DPD);
		if (ret == 0) {
			record_entered_dpd(dev);
		}
	}
	return ret;
}

static int exit_dpd(const struct device *const dev)
{
	int ret = 0;

	if (IS_ENABLED(DT_INST_PROP(0, has_dpd))) {
		delay_until_exit_dpd_ok(dev);

#if DT_INST_NODE_HAS_PROP(0, dpd_wakeup_sequence)
		/* Assert CSn and wait for tCRDP.
		 *
		 * Unfortunately the SPI API doesn't allow us to
		 * control CSn so fake it by writing a known-supported
		 * single-byte command, hoping that'll hold the assert
		 * long enough.  This is highly likely, since the
		 * duration is usually less than two SPI clock cycles.
		 */
		ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_RDID);

		/* Deassert CSn and wait for tRDP */
		k_sleep(K_MSEC(T_RDP_MS));
#else /* DPD_WAKEUP_SEQUENCE */
		ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_RDPD);

		if (ret == 0) {
#if DT_INST_NODE_HAS_PROP(0, t_exit_dpd)
			k_sleep(K_MSEC(T_RES1_MS));
#endif /* T_EXIT_DPD */
		}
#endif /* DPD_WAKEUP_SEQUENCE */
	}
	return ret;
}

/* Everything necessary to acquire owning access to the device.
 *
 * This means taking the lock and, if necessary, waking the device
 * from deep power-down mode.
 */
static void acquire_device(struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nand_data *const driver_data = dev->driver_data;

		k_sem_take(&driver_data->sem, K_FOREVER);
	}

	if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)) {
		exit_dpd(dev);
	}
}

/* Everything necessary to release access to the device.
 *
 * This means (optionally) putting the device into deep power-down
 * mode, and releasing the lock.
 */
static void release_device(struct device *dev)
{
	if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)) {
		enter_dpd(dev);
	}

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nand_data *const driver_data = dev->driver_data;

		k_sem_give(&driver_data->sem);
	}
}

/**
 * @brief Retrieve the Flash JEDEC ID and compare it with the one expected
 *
 * @param dev The device structure
 * @param flash_id The flash info structure which contains the expected JEDEC ID
 * @return 0 on success, negative errno code otherwise
 */
static inline int spi_nand_read_id(struct device *dev,
				  const struct spi_nand_config *const flash_id)
{
	u8_t buf[SPI_NAND_MAX_ID_LEN];

	if (spi_nand_cmd_read(dev, SPI_NAND_CMD_RDID, buf,
	    SPI_NAND_MAX_ID_LEN) != 0) {
		return -EIO;
	}

	if (memcmp(flash_id->id, buf, SPI_NAND_MAX_ID_LEN) != 0) {
		return -ENODEV;
	}

	return 0;
}

/**
 * @brief Wait until the flash is ready
 *
 * @param dev The device structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_wait_until_ready(struct device *dev)
{
	int ret;
	u8_t reg;

	do {
		ret = spi_nand_cmd_read(dev, SPI_NAND_CMD_RDSR, &reg, 1);
	} while (!ret && (reg & SPI_NAND_WIP_BIT));

	return ret;
}

static int spi_nand_read(struct device *dev, off_t addr, void *dest,
			size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > params->size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	spi_nand_wait_until_ready(dev);

	ret = spi_nand_cmd_addr_read(dev, SPI_NAND_CMD_READ, addr, dest, size);

	release_device(dev);
	return ret;
}

static int spi_nand_write(struct device *dev, off_t addr, const void *src,
			 size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret = 0;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	while (size > 0) {
		size_t to_write = size;

		/* Don't write more than a page. */
		if (to_write >= SPI_NAND_PAGE_SIZE) {
			to_write = SPI_NAND_PAGE_SIZE;
		}

		/* Don't write across a page boundary */
		if (((addr + to_write - 1U) / SPI_NAND_PAGE_SIZE)
		    != (addr / SPI_NAND_PAGE_SIZE)) {
			to_write = SPI_NAND_PAGE_SIZE - (addr % SPI_NAND_PAGE_SIZE);
		}

		spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
		ret = spi_nand_cmd_addr_write(dev, SPI_NAND_CMD_PP, addr,
					     src, to_write);
		if (ret != 0) {
			goto out;
		}

		size -= to_write;
		src = (const u8_t *)src + to_write;
		addr += to_write;

		spi_nand_wait_until_ready(dev);
	}

out:
	release_device(dev);
	return ret;
}

static int spi_nand_erase(struct device *dev, off_t addr, size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret = 0;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	acquire_device(dev);

	while (size) {
		/* write enable */
		spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);

		if (size == params->size) {
			/* chip erase */
			spi_nand_cmd_write(dev, SPI_NAND_CMD_CE);
			size -= params->size;
		} else if ((size >= SPI_NAND_BLOCK_SIZE)
			   && SPI_NAND_IS_BLOCK_ALIGNED(addr)) {
			/* 64 KiB block erase */
			spi_nand_cmd_addr_write(dev, SPI_NAND_CMD_BE, addr,
			NULL, 0);
			addr += SPI_NAND_BLOCK_SIZE;
			size -= SPI_NAND_BLOCK_SIZE;
		} else if ((size >= SPI_NAND_BLOCK32_SIZE)
			   && SPI_NAND_IS_BLOCK32_ALIGNED(addr)) {
			/* 32 KiB block erase */
			spi_nand_cmd_addr_write(dev, SPI_NAND_CMD_BE_32K, addr,
					       NULL, 0);
			addr += SPI_NAND_BLOCK32_SIZE;
			size -= SPI_NAND_BLOCK32_SIZE;
		} else if ((size >= SPI_NAND_SECTOR_SIZE)
			   && SPI_NAND_IS_SECTOR_ALIGNED(addr)) {
			/* sector erase */
			spi_nand_cmd_addr_write(dev, SPI_NAND_CMD_SE, addr,
					       NULL, 0);
			addr += SPI_NAND_SECTOR_SIZE;
			size -= SPI_NAND_SECTOR_SIZE;
		} else {
			/* minimal erase size is at least a sector size */
			LOG_DBG("unsupported at 0x%lx size %zu", (long)addr,
				size);
			ret = -EINVAL;
			goto out;
		}

		spi_nand_wait_until_ready(dev);
	}

out:
	release_device(dev);

	return ret;
}

static int spi_nand_write_protection_set(struct device *dev, bool write_protect)
{
	int ret;

	acquire_device(dev);

	spi_nand_wait_until_ready(dev);

	ret = spi_nand_cmd_write(dev, (write_protect) ?
	      SPI_NAND_CMD_WRDI : SPI_NAND_CMD_WREN);

	if (IS_ENABLED(DT_INST_PROP(0, requires_ulbpr))
	    && (ret == 0)
	    && !write_protect) {
		ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_ULBPR);
	}

	release_device(dev);

	return ret;
}

/**
 * @brief Configure the flash
 *
 * @param dev The flash device structure
 * @param info The flash info structure
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_configure(struct device *dev)
{
	struct spi_nand_data *data = dev->driver_data;
	const struct spi_nand_config *params = dev->config->config_info;

	data->spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!data->spi) {
		return -EINVAL;
	}

	data->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	data->spi_cfg.operation = SPI_WORD_SET(8);
	data->spi_cfg.slave = DT_INST_REG_ADDR(0);

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	if (!data->cs_ctrl.gpio_dev) {
		return -ENODEV;
	}

	data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	data->cs_ctrl.delay = CONFIG_SPI_NAND_CS_WAIT_DELAY;

	data->spi_cfg.cs = &data->cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */

	/* Might be in DPD if system restarted without power cycle. */
	exit_dpd(dev);

	/* now the spi bus is configured, we can verify the flash id */
	if (spi_nand_read_id(dev, params) != 0) {
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)
	    && (enter_dpd(dev) != 0)) {
		return -ENODEV;
	}

	return 0;
}

/**
 * @brief Initialize and configure the flash
 *
 * @param name The flash name
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_init(struct device *dev)
{
	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nand_data *const driver_data = dev->driver_data;

		k_sem_init(&driver_data->sem, 1, UINT_MAX);
	}

	return spi_nand_configure(dev);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

/* instance 0 size in bytes */
#define INST_0_BYTES (DT_INST_PROP(0, size) / 8)

BUILD_ASSERT(SPI_NAND_IS_SECTOR_ALIGNED(CONFIG_SPI_NAND_FLASH_LAYOUT_PAGE_SIZE),
	     "SPI_NAND_FLASH_LAYOUT_PAGE_SIZE must be multiple of 4096");

/* instance 0 page count */
#define LAYOUT_PAGES_COUNT (INST_0_BYTES / CONFIG_SPI_NAND_FLASH_LAYOUT_PAGE_SIZE)

BUILD_ASSERT((CONFIG_SPI_NAND_FLASH_LAYOUT_PAGE_SIZE * LAYOUT_PAGES_COUNT)
	     == INST_0_BYTES,
	     "SPI_NAND_FLASH_LAYOUT_PAGE_SIZE incompatible with flash size");

static const struct flash_pages_layout dev_layout = {
	.pages_count = LAYOUT_PAGES_COUNT,
	.pages_size = CONFIG_SPI_NAND_FLASH_LAYOUT_PAGE_SIZE,
};
#undef LAYOUT_PAGES_COUNT

static void spi_nand_pages_layout(struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api spi_nand_api = {
	.read = spi_nand_read,
	.write = spi_nand_write,
	.erase = spi_nand_erase,
	.write_protection = spi_nand_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = spi_nand_pages_layout,
#endif
	.write_block_size = 1,
};

static const struct spi_nand_config flash_id = {
	.id = DT_INST_PROP(0, jedec_id),
#if DT_INST_NODE_HAS_PROP(0, has_be32k)
	.has_be32k = true,
#endif /* DT_INST_NODE_HAS_PROP(0, has_be32k) */
	.size = DT_INST_PROP(0, size) / 8,
};

static struct spi_nand_data spi_nand_memory_data;

DEVICE_AND_API_INIT(spi_flash_memory, DT_INST_LABEL(0),
		    &spi_nand_init, &spi_nand_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_SPI_NAND_INIT_PRIORITY,
		    &spi_nand_api);
