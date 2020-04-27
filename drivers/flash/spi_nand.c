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
#include <sys/util.h>

#include <stdlib.h>

#include "spi_nand.h"
// #include "flash_priv.h"

// LOG_MODULE_REGISTER(spi_nand, CONFIG_FLASH_LOG_LEVEL);
LOG_MODULE_REGISTER(spi_nand, LOG_LEVEL_DBG);

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

static u32_t _chip_page = SPI_NAND_INVALID_PAGE;
static bool _page_sync_required = false;
static bool _write_protect = true;

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

/**
 * @brief Send an SPI command
 *
 * @param dev Device struct
 * @param opcode The command to send
 * @param addr_size The size of the address to send (bytes)
 * @param addr The address to send
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @param is_write A flag to define if it's a read or a write command
 * @param wait The number of wait states between the tx and rx
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_access(const struct device *const dev,
			  u8_t opcode, size_t addr_size, off_t addr,
			  void *data, size_t length, bool is_write, u8_t wait)
{
	struct spi_nand_data *const driver_data = dev->driver_data;

	/* opcode (1), address (3), wait (2) */
	u8_t buf[SPI_NAND_HEADER_SIZE_MAX];
	
	buf[0] = opcode;
	switch (addr_size) {
	case 0:
		break;
	case 1:
		buf[SPI_NAND_OPCODE_LEN] = (addr & 0xFF);
		break;
	case 2:
		buf[SPI_NAND_OPCODE_LEN] = (addr & 0xFF00) >> 8;
		buf[1 + SPI_NAND_OPCODE_LEN] = (addr & 0xFF);
		break;
	case 3:
		buf[SPI_NAND_OPCODE_LEN] = (addr & 0xFF0000) >> 16;
		buf[1 + SPI_NAND_OPCODE_LEN] = (addr & 0xFF00) >> 8;
		buf[2 + SPI_NAND_OPCODE_LEN] = (addr & 0xFF);
		break;
	default:
		return -EINVAL;
	}

	switch (wait) {
	case 0:
		break;
	case 1:
		buf[addr_size + SPI_NAND_OPCODE_LEN] = 0;
		break;
	case 2:
		buf[addr_size + SPI_NAND_OPCODE_LEN] = 0;
		buf[addr_size + SPI_NAND_OPCODE_LEN] = 0;
		break;
	default:
		return -EINVAL;
	}

	/* transaction length = opcode + address + wait */
	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = SPI_NAND_OPCODE_LEN + addr_size + wait,
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

#define spi_nand_cmd_read(dev, opcode, dest, length, wait) \
	spi_nand_access(dev, opcode, 0, 0, dest, length, false, wait)
#define spi_nand_cmd_addr_read(dev, opcode, addr_size, addr, dest, length, wait) \
	spi_nand_access(dev, opcode, addr_size, addr, dest, length, false, wait)
#define spi_nand_cmd_write(dev, opcode) \
	spi_nand_access(dev, opcode, 0, 0, NULL, 0, true, 0)
#define spi_nand_cmd_addr_write(dev, opcode, addr_size, addr, src, length) \
	spi_nand_access(dev, opcode, addr_size, addr, (void *)src, length, true, 0)

/* tc58cvg2s0 timing requires a "dummy" byte after the column address */
/* so, we need to insert a wait state */
/* see datasheet 4.2.2 */
#define spi_nand_read_page_buf(dev, page_offset, dest, length) \
	spi_nand_access(dev, SPI_NAND_CMD_RDPB, SPI_NAND_COLUMN_ADDR_SIZE, page_offset, dest, length, false, 1)

#define spi_nand_write_page_buf(dev, row_addr, src, length) \
	spi_nand_access(dev, SPI_NAND_CMD_LDPBRDSPI, SPI_NAND_COLUMN_ADDR_SIZE, row_addr, (void *)src, length, true, 0)

#define spi_nand_cmd_read_flash_array(dev, row_addr) \
	spi_nand_access(dev, SPI_NAND_CMD_LDPBFCA, SPI_NAND_ROW_ADDR_SIZE, row_addr, NULL, 0, true, 0)

#define spi_nand_cmd_write_flash_array(dev, row_addr) \
	spi_nand_access(dev, SPI_NAND_CMD_WRPBFCA, SPI_NAND_ROW_ADDR_SIZE, row_addr, NULL, 0, true, 0)

#define spi_nand_erase_block(dev, row_addr) \
	spi_nand_access(dev, SPI_NAND_CMD_BLKERASE, SPI_NAND_ROW_ADDR_SIZE, row_addr, NULL, 0, true, 0)

#define spi_nand_read_status(dev, dest) \
	spi_nand_access(dev, SPI_NAND_CMD_RDFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_STATUS, dest, 1, false, 0)

#define spi_nand_write_status(dev, src) \
	spi_nand_access(dev, SPI_NAND_CMD_WRFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_STATUS, (void *)src, 1, true, 0)

#define spi_nand_read_lock(dev, dest) \
	spi_nand_access(dev, SPI_NAND_CMD_RDFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_LOCK, dest, 1, false, 0)

#define spi_nand_write_lock(dev, src) \
	spi_nand_access(dev, SPI_NAND_CMD_WRFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_LOCK, (void *)src, 1, true, 0)

#define spi_nand_read_ctrl(dev, dest) \
	spi_nand_access(dev, SPI_NAND_CMD_RDFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_CTRL, dest, 1, false, 0)

#define spi_nand_write_ctrl(dev, src) \
	spi_nand_access(dev, SPI_NAND_CMD_WRFR, SPI_NAND_FEATURE_ADDR_SIZE, SPI_NAND_FT_ADDR_CTRL, (void *)src, 1, true, 0)

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
	u8_t buf[SPI_NAND_ID_LEN];

	/* tc58cvg2s0 has a "dummy" byte cycle between command and rx */
	/* So insert wait */
	if (spi_nand_cmd_read(dev, SPI_NAND_CMD_RDID, buf, SPI_NAND_ID_LEN, 1)
	    < 0) {
		return -EIO;
	}

	/* jedec ID is only 2 bytes for tc58cvg2s0 */
	/* see datasheet 4.13 */
	if (memcmp(flash_id->id, buf, SPI_NAND_ID_LEN) != 0) {
		return -ENODEV;
	}

	return 0;
}

static int spi_nand_idre_set(struct device *dev, bool val)
{
	int ret;
	u8_t reg;

	ret = spi_nand_read_ctrl(dev, &reg);
	if (ret < 0) {
		goto done;
	}

	if ( val ) {
		reg |= SPI_NAND_CTRL_IDRE_BIT;
	} else {
		reg &= ~SPI_NAND_CTRL_IDRE_BIT;
	}

	ret = spi_nand_write_ctrl(dev, &reg);
done:
	return ret;
}

static int spi_nand_hse_set(struct device *dev, bool val)
{
	int ret;
	u8_t reg;

	ret = spi_nand_read_ctrl(dev, &reg);
	if (ret < 0) {
		goto done;
	}

	if ( val ) {
		reg |= SPI_NAND_CTRL_HSE_BIT;
	} else {
		reg &= ~SPI_NAND_CTRL_HSE_BIT;
	}

	ret = spi_nand_write_ctrl(dev, &reg);
done:
	return ret;
}

static int spi_nand_bl_set(struct device *dev, u8_t val)
{
	int ret;
	u8_t reg;

	ret = spi_nand_read_lock(dev, &reg);
	if (ret < 0) {
		goto done;
	}

	reg &= ~SPI_NAND_LOCK_BL_BIT;
	reg |= (val << SPI_NAND_LOCK_BL_POS) & SPI_NAND_LOCK_BL_BIT;

	ret = spi_nand_write_lock(dev, &reg);
done:
	return ret;
}

/**
 * @brief Wait until the flash is ready
 *
 * @param dev The device structure
 * @param timeout	timeout (microseconds)
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_wait_until_ready(struct device *dev, u32_t timeout)
{
	int ret;
	u8_t reg;
	u32_t t;

	t = k_cycle_get_32();
	do {
		ret = spi_nand_read_status(dev, &reg);
		if (ret < 0) {
			break;
		}
		if ((k_cycle_get_32() - t) > k_us_to_cyc_ceil32(timeout)) {
			LOG_ERR("nand timed out");
			ret = -ETIMEDOUT;
			break;
		}
	} while ((reg & SPI_NAND_STATUS_OIP_BIT));
	return ret;
}

/**
 * @brief program data buffer content into the physical memory page
 * 
 * @param row_addr 
 * @return int8_t 
 */
static int spi_nand_page_write(struct device *dev, u32_t row_addr)
{
	int ret;
	u8_t reg;
	
	LOG_DBG("page write: 0x%x", row_addr);

	if (_chip_page != row_addr) {
		LOG_DBG("invalid page");
		return -EINVAL;
	}
	// ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
	// if (ret < 0) {
	// 	 goto done;
	// }
	ret = spi_nand_read_status(dev, &reg);
	if (ret < 0) {
		goto done;
	}
	if (!(reg & SPI_NAND_STATUS_WEL_BIT)) {
		LOG_ERR("write enable error");
		ret = -EIO;
		goto done;
	}

	/* write the flash cell array */
	ret = spi_nand_cmd_write_flash_array(dev, row_addr);
	if (ret < 0) {
		 goto done;
	}

	ret = spi_nand_wait_until_ready(dev, SPI_NAND_PROG_TIMEOUT);
	if (ret < 0) {
		 goto done;
	}

	ret = spi_nand_read_status(dev, &reg);
	if (ret < 0) {
		 goto done;
	}

	if (reg &  SPI_NAND_STATUS_PROGF_BIT) {
		LOG_DBG("page write failed");
		ret = -EIO;
	} else {
		_chip_page = SPI_NAND_INVALID_PAGE;
		_page_sync_required = false;
	}
done:
	return ret;
}


/**
 * @brief Write the on chip buffer to the NAND array
 * 
 * @return int8_t 
 */
static int spi_nand_page_flush(struct device *dev)
{
	int ret;

	LOG_DBG("flush: 0x%x", _chip_page);
	if (_chip_page == SPI_NAND_INVALID_PAGE) {
		return 0;
	}

	if (_write_protect) {
		/* write enable */
		spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
	}

	ret = spi_nand_page_write(dev, _chip_page);

	if (_write_protect) {
		/* restore write protect */
		spi_nand_cmd_write(dev, SPI_NAND_CMD_WRDI);
	}
	return ret;
}

/**
 * @brief Read a page from the NAND array into the on chip buffer
 * 
 * @param row_addr 
 * @return int8_t 
 */
static int spi_nand_read_cell_array(struct device *dev, u32_t row_addr)
{
	int ret, ecc;
	u8_t reg;

	ret = spi_nand_cmd_read_flash_array(dev, row_addr);
	if (ret < 0) {
		goto done;
	}

	ret = spi_nand_wait_until_ready(dev, SPI_NAND_READ_TIMEOUT);
	if (ret < 0) {
		 goto done;
	}

	ret = spi_nand_read_status(dev, &reg);
	if (ret < 0) {
		 goto done;
	}

	/* check ecc bits */
	ecc = ((reg &  SPI_NAND_STATUS_ECC_BIT) >> SPI_NAND_STATUS_ECC_POS);
	if (ecc > 1) {
		LOG_DBG("ecc error: %d", ecc);
		ret = -EIO;
	}
	_chip_page = row_addr;

	// u8_t buf[64];
	// spi_nand_read_page_buf(dev, 0, buf, sizeof(buf));
	// LOG_HEXDUMP_DBG(buf, sizeof(buf), "page buffer");
done:
    	return ret;
}

int spi_nand_read_parameter_page(struct device *dev)
{
	int ret;
	union {
		char str[21];
		u32_t i32;
		u16_t i16;
		char c;
	} params;

	ret = spi_nand_idre_set(dev, true);
	if (ret < 0) {
		goto done;
	}
	ret = spi_nand_read_cell_array(dev, 1);
	if (ret < 0) {
		goto cleanup;
	}
	spi_nand_read_page_buf(dev, 0, params.str, 16);
	params.str[16] = '\0';
	LOG_INF("Signature: %s", log_strdup(params.str));
	spi_nand_read_page_buf(dev, 32, params.str, 12);
	params.str[12] = '\0';
	LOG_DBG("Manufacturer: %s", log_strdup(params.str));
	spi_nand_read_page_buf(dev, 44, params.str, 20);
	params.str[20] = '\0';
	LOG_DBG("Device model: %s", log_strdup(params.str));
	spi_nand_read_page_buf(dev, 80, &params.i32, sizeof(params.i32));
	LOG_DBG("bytes per page: %d", params.i32);
	spi_nand_read_page_buf(dev, 92, &params.i32, sizeof(params.i32));
	LOG_DBG("pages per block: %d", params.i32);
	spi_nand_read_page_buf(dev, 96, &params.i32, sizeof(params.i32));
	LOG_DBG("blocks per unit: %d", params.i32);
	spi_nand_read_page_buf(dev, 105, &params.i16, sizeof(params.i16));
	LOG_DBG("block endurance: %d", params.i16);
	spi_nand_read_page_buf(dev, 110, &params.c, sizeof(params.c));
	LOG_DBG("programs per page: %d", params.c);

cleanup:
	_chip_page = SPI_NAND_INVALID_PAGE;
	ret = spi_nand_idre_set(dev, false);
	if (ret < 0) {
		goto done;
	}
done:
	return ret;
}

/*  TODO:  THIS NEEDS TO BE TESTED  */
static int spi_nand_get_back_blocks(struct device *dev, u32_t Bad_Block_Table[])
{
	u32_t block;
	u8_t data;
	int ret;

	block = 0;
	for (block = 0; block < 2048; block++) {
		ret = spi_nand_read_cell_array(dev, block * 64);                                 ///<  Load nand page into chip buffer
		spi_nand_read_page_buf(dev, 0, &data, sizeof(data));

		//Check bad block
		if (data == 0) {
			Bad_Block_Table[block / (sizeof(Bad_Block_Table[0]) * 8)] |= 1 << (block % (sizeof(Bad_Block_Table[0]) * 8));
		}
	}
	return 0;
}

static int spi_nand_read(struct device *dev, off_t addr, void *dest,
			size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret = 0;
	u32_t row_addr = addr / SPI_NAND_PAGE_SIZE;
	u32_t page_offset = page_offset_of(addr);
	u32_t rd_bytes = MIN(SPI_NAND_PAGE_SIZE - page_offset, size);
	u32_t remain = size;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((addr + size) > params->size)) {
		return -EINVAL;
	}

	acquire_device(dev);

	while (remain) {
		if (_chip_page != row_addr) {
			if (_page_sync_required) {
				/* write the page buffer to the flash cell array */
				ret = spi_nand_page_flush(dev);
				if (ret < 0) {
					goto done;
				}
			}
			/* load the page into the page buffer from flash */
			ret = spi_nand_read_cell_array(dev, row_addr);
			if (ret < 0) {
				goto done;
			}
		}
		ret = spi_nand_read_page_buf(dev, page_offset, dest, rd_bytes);
		if (ret < 0) {
			goto done;
		}
		dest = (u8_t *)dest + rd_bytes;
		remain -= rd_bytes;
		row_addr++;
		page_offset = 0;
		rd_bytes = MIN(addr + size - (row_addr * SPI_NAND_PAGE_SIZE), SPI_NAND_PAGE_SIZE);
	}
done:
	release_device(dev);
	return ret;
}

static int spi_nand_write(struct device *dev, off_t addr, const void *src,
			 size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret = 0;
	u32_t row_addr = addr / SPI_NAND_PAGE_SIZE;
	u32_t page_offset = page_offset_of(addr);
	u32_t wr_bytes = MIN(SPI_NAND_PAGE_SIZE - page_offset, size);
	u32_t remain = size;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -EINVAL;
	}

	acquire_device(dev);
	ret = spi_nand_hse_set(dev, false);

	while (remain > 0) {
		// Fill page buffer
		if (_chip_page != row_addr) {
			/* write the page buffer to the flash cell array */
			ret = spi_nand_page_flush(dev);
			if (ret < 0) {
				goto cleanup;
			}
			/* load the page into the page buffer from flash */
			ret = spi_nand_read_cell_array(dev, row_addr);
			if (ret < 0) {
				goto cleanup;
			}
		}

		ret = spi_nand_write_page_buf(dev, page_offset, src, wr_bytes);
		_page_sync_required = true;
		if (ret < 0) {
			goto cleanup;
		}

		src = (const u8_t *)src + wr_bytes;
		remain -= wr_bytes;
		row_addr++;
		page_offset = 0;
		wr_bytes = MIN(addr + size - (row_addr * SPI_NAND_PAGE_SIZE), SPI_NAND_PAGE_SIZE);
	}

cleanup:
	ret = spi_nand_hse_set(dev, true);
	release_device(dev);
	return ret;
}

static int spi_nand_erase(struct device *dev, off_t addr, size_t size)
{
	const struct spi_nand_config *params = dev->config->config_info;
	int ret = 0;
	u8_t reg;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	acquire_device(dev);

	/* write enable */
	// ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
	// if (ret < 0) {
	// 	goto out;
	// }
	ret = spi_nand_read_status(dev, &reg);
	if (ret < 0) {
		goto out;
	}
	if (!(reg & SPI_NAND_STATUS_WEL_BIT)) {
		LOG_ERR("write enable error");
		ret = -EIO;
		goto out;
	}

	while (size) {
		if ((size >= SPI_NAND_BLOCK_SIZE)
			  && SPI_NAND_IS_BLOCK_ALIGNED(addr)) {
			/* 256 KiB block erase */
			spi_nand_erase_block(dev, addr / SPI_NAND_PAGE_SIZE);
			/* wait for OIP */
			spi_nand_wait_until_ready(dev, SPI_NAND_ERASE_TIMEOUT);
			/* check ERS_F */
			ret = spi_nand_read_status(dev, &reg);
			if (ret < 0) {
				goto out;
			}
			if (reg & SPI_NAND_STATUS_ERASEF_BIT) {
				LOG_ERR("erase error: 0x%08x", addr);
				// ret = -EIO;
				// goto out;
			}
			addr += SPI_NAND_BLOCK_SIZE;
			size -= SPI_NAND_BLOCK_SIZE;
		} else {
			/* minimal erase size is at least a block size */
			LOG_DBG("unsupported at 0x%lx size %zu", (long)addr,
				size);
			ret = -EINVAL;
			goto out;
		}
	}

out:
	/* write disable */
	// spi_nand_cmd_write(dev, SPI_NAND_CMD_WRDI);
	release_device(dev);

	return ret;
}

static int spi_nand_sync(struct device *dev)
{
	int ret;

	acquire_device(dev);
	
	ret = spi_nand_page_flush(dev);

	release_device(dev);

	return ret;
}

static int spi_nand_write_protection_set(struct device *dev, bool write_protect)
{
	int ret;

	acquire_device(dev);

	ret = spi_nand_wait_until_ready(dev, SPI_NAND_ERASE_TIMEOUT);
	if (ret < 0) {
		goto done;
	}

	ret = spi_nand_cmd_write(dev, (write_protect) ?
	      SPI_NAND_CMD_WRDI : SPI_NAND_CMD_WREN);
	if (ret == 0) {
		_write_protect = write_protect;
	}

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
	ret = spi_nand_wait_until_ready(dev, SPI_NAND_ERASE_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("device timed out");
		return -ETIMEDOUT;
	}

	/* now the spi bus is configured, we can verify the flash id */
	if (spi_nand_read_id(dev, params) != 0) {
		return -ENODEV;
	}

	LOG_DBG("device id ok");

	if (spi_nand_read_parameter_page(dev) < 0) {
		return -ENODEV;
	};

	/* all flash blocks are locked at power on */
	/* so unlock them */
	/* see datasheet 4.10 */
	if (spi_nand_bl_set(dev, 0) < 0) {
		return -ENODEV;
	}
	
	if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)
	    && (enter_dpd(dev) != 0)) {
		return -ENODEV;
	}

	_write_protect = true;

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
	.sync = spi_nand_sync,
	.write_protection = spi_nand_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = spi_nand_pages_layout,
#endif
	.write_block_size = SPI_NAND_PARTIAL_PAGE_SIZE,
};

static const struct spi_nand_config flash_id = {
	.id = DT_INST_PROP(0, jedec_id),
#if DT_INST_NODE_HAS_PROP(0, has_be32k)
	.has_be32k = true,
#endif /* DT_INST_NODE_HAS_PROP(0, has_be32k) */
	.size = DT_INST_PROP(0, size),
};

static struct spi_nand_data spi_nand_memory_data;

DEVICE_AND_API_INIT(spi_flash_memory, DT_INST_LABEL(0),
		    &spi_nand_init, &spi_nand_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_SPI_NAND_INIT_PRIORITY,
		    &spi_nand_api);
