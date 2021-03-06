/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * This driver is heavily inspired from the spi_flash_w25qxxdv.c SPI NAND driver.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file spi_nand.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
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

#define DT_DRV_COMPAT jedec_spi_nand

#include <errno.h>
#include <drivers/flash.h>
#include <drivers/spi.h>
#include <init.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include <sys/util.h>
#include <sys/byteorder.h>

#include <stdlib.h>

#include "spi_nand.h"
// #include "flash_priv.h"

LOG_MODULE_REGISTER(spi_nand, CONFIG_SPI_NAND_LOG_LEVEL);

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

static u32_t _chip_page = SPI_NAND_INVALID_PAGE;
static bool _page_sync_required = false;
static bool _write_protect = true;

static inline u32_t page_addr_of(u32_t addr)
{
	return addr & ~(SPI_NAND_PAGE_SIZE - 1U);
}

static inline u32_t page_offset_of(u32_t addr)
{
	return addr & (SPI_NAND_PAGE_SIZE - 1U);
}

static inline bool is_page_aligned(uint32_t addr)
{
	return (((addr) & (SPI_NAND_PAGE_SIZE - 1U)) == 0);
}

static inline bool is_sector_aligned(uint32_t addr)
{
	return (((addr) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0);
}

static inline bool is_block_aligned(uint32_t addr)
{
	return (((addr) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0);
}

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
		buf[1 + addr_size + SPI_NAND_OPCODE_LEN] = 0;
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


static int spi_nand_reg_read(struct device *dev, u8_t reg, u8_t *val)
{
	int ret;

	ret = spi_nand_access(dev, SPI_NAND_CMD_RDFR, SPI_NAND_FEATURE_ADDR_SIZE, reg, (void *)val, 1, false, 0);

	return ret;
}

static int spi_nand_reg_write(struct device *dev, u8_t reg, u8_t val)
{
	int ret;

	ret = spi_nand_access(dev, SPI_NAND_CMD_WRFR, SPI_NAND_FEATURE_ADDR_SIZE, reg, (void *)&val, 1, true, 0);

	return ret;
}

__attribute__((unused))
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

__attribute__((unused))
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

	// if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)) {
	// 	exit_dpd(dev);
	// }
}

/* Everything necessary to release access to the device.
 *
 * This means (optionally) putting the device into deep power-down
 * mode, and releasing the lock.
 */
static void release_device(struct device *dev)
{
	// if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)) {
	// 	enter_dpd(dev);
	// }

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
int spi_nand_read_id(struct device *dev,
				  const struct spi_nand_config *const flash_id)
{
	u8_t buf[SPI_NAND_JEDEC_ID_LEN];

	/* tc58cvg2s0 has a "dummy" byte cycle between command and rx */
	/* So insert wait */
	if (spi_nand_cmd_read(dev, SPI_NAND_CMD_RDID, buf, SPI_NAND_JEDEC_ID_LEN, 1)
	    < 0) {
		return -EIO;
	}

	/* jedec ID is only 2 bytes for tc58cvg2s0 */
	/* see datasheet 4.13 */
	if (memcmp(flash_id->id, buf, SPI_NAND_JEDEC_ID_LEN) != 0) {
		return -ENODEV;
	}

	LOG_HEXDUMP_DBG(buf, sizeof(buf), "Chip ID:");

	return 0;
}

/**
 * @brief Enable device data read mode
 * 
 * 	nand devices have a parameter page and may have other device specific data.
 * 	Kioxia TC58CVG2S0HRAIJ has a Unique ID and Parameter Page
 * 	Alliance AS5F34G04SND-08LIN has a Parameter Page and an OTP memory region
 * 
 * @param dev 
 * @param val 
 * @return int 
 */
static int spi_nand_device_data_mode_enable(struct device *dev, bool val)
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

__unused
static int spi_nand_high_speed_mode(struct device *dev, bool val)
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

static int spi_nand_block_lock(struct device *dev, u8_t val)
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
 * 	  Notes:
 * 	    It appears the maximum read, prog and erase times in the datasheet are not guaranteed.
 * 	    So, we cannot use them directly as timeouts.
 * 	    Therefore, the timeouts should be some multiple of the maximum shown in the datasheet
 * 	    so that we cannot get locked up.
 * 	    We have three timers that collect the cummulative time and count of the device access.
 * 	    These can be used to evaluate the average read, prog and erase times.
 *
 * @param dev The device structure
 * @param timeout	timeout (microseconds)
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_wait_until_ready(struct device *dev, u8_t *status_reg, enum SPI_NAND_TIMER_e timer)
{
	struct spi_nand_data *data = dev->driver_data;
	int ret;
	u32_t timeout, start_time, delta;

	switch (timer) {
		case SPI_NAND_TIMER_PAGE_READ_TIME:
			timeout = k_us_to_cyc_ceil32(SPI_NAND_MAX_PAGE_READ_TIME);
			break;
		case SPI_NAND_TIMER_PAGE_PROG_TIME:
			timeout = k_us_to_cyc_ceil32(SPI_NAND_MAX_PAGE_PROG_TIME);
			break;
		case SPI_NAND_TIMER_BLOCK_ERASE_TIME:
			timeout = k_us_to_cyc_ceil32(SPI_NAND_MAX_BLOCK_ERASE_TIME);
			break;
		case SPI_NAND_TIMER_RESET:
			timeout = k_us_to_cyc_ceil32(SPI_NAND_MAX_RESET_TIME);
			break;
		case SPI_NAND_TIMER_POWER_UP:
			timeout = k_us_to_cyc_ceil32(SPI_NAND_POWER_UP_WAIT);
			break;
		default:
			timeout = k_us_to_cyc_ceil32(1000);
			break;
	};

	start_time = k_cycle_get_32();
	while (1) {
		/*
		 * Capture the current time before reading OIP status so
		 * if we are interrupted by another thread, the consequence
		 * is the OIP status is captured at a later point in time.
		 * Therefore the calculated elapsed time will never be less
		 * than the actual time to read the OIP status.
		 * 
		 * This ensure that the read and program timings specified
		 * in the datasheet work properly.
		 */
		delta = k_cycle_get_32() - start_time;
		ret = spi_nand_read_status(dev, status_reg);
		if (ret < 0) {
			break;
		}
		/*  The order of the following checks is important  */
		/*  Check for an error condition first  */
		// if (*status_reg & SPI_NAND_STATUS_ERROR_BITS) {
		// 	break;
		// }
		if (*status_reg & SPI_NAND_STATUS_PROGF_BIT) {
			break;
		}
		if (*status_reg & SPI_NAND_STATUS_ERASEF_BIT) {
			break;
		}
		/*  Check if operation is complete last */
		if (!(*status_reg & SPI_NAND_STATUS_OIP_BIT)) {
			break;
		}
		if (delta > timeout) {
			LOG_INF("time out: REG=0x%02x, timeout=%dcycles, start time=%d, delta=%d", *status_reg, timeout, start_time, delta);
			ret = -ETIMEDOUT;
			break;
		}
		/*
		 *  spi_nand_wait_until_ready() wait duration can be 100's or
		 *  1000's of usec.  So, since the nand is locked we can allow other
		 *  threads to run while we wait
		 */
		k_yield();
	};
	if (timer < SPI_NAND_TIMER_NUM_TIMERS) {
		data->timer[timer] += delta;
		data->counter[timer]++;
	};
	return ret;
}

/**
 * @brief Reset the operation in progress
 * 
 * @param dev 
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nand_reset(struct device *dev)
{
	int ret;
	uint8_t reg;

	/* reset any on-going operation (erase or program) */
	ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_RESET);
	if (ret < 0) {
		LOG_ERR("nand reset error");
		return ret;
	}

	ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_RESET);
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
	
	if (_chip_page == SPI_NAND_INVALID_PAGE) {
		return 0;
	}

	if (_chip_page != row_addr) {
		LOG_DBG("invalid page");
		return -EINVAL;
	}

	LOG_DBG("page write: 0x%x", row_addr);

	if (_page_sync_required) {
		/* WREN must be set immediately before the cell array write command */
		spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
		/* write the flash cell array */
		ret = spi_nand_cmd_write_flash_array(dev, row_addr);
		if (ret < 0) {
			goto done;
		}

		ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_PAGE_PROG_TIME);
		if (ret < 0) {
			LOG_ERR("page program wait error %d", ret);
			goto done;
		}

		if (reg &  SPI_NAND_STATUS_PROGF_BIT) {
			LOG_ERR("page write failed");
			spi_nand_reset(dev);
			/*
			 *  Return EFAULT to indicate corrupt write
			 *  This error code is mapped to LFS_ERR_CORRUPT
			 *  by errno_to_lfs()
			 *  see littlefs_fs.c
			 */
			ret = -EFAULT;
		} else {
			// _chip_page = SPI_NAND_INVALID_PAGE;
			_page_sync_required = false;
		}
	} else {
		ret = 0;
	}
done:
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

	ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_PAGE_READ_TIME);
	if (ret < 0) {
		LOG_ERR("read cell array wait error %d", ret);
		goto done;
	}

	/* check ecc bits */
	ecc = ((reg &  SPI_NAND_STATUS_ECC_BIT) >> SPI_NAND_STATUS_ECC_POS);
	switch (ecc) {
	case 2:
		LOG_ERR("ecc: uncorrected bit flips (%d)", ecc);
		ret = -EIO;
		break;
	case 1:
	case 3:
		LOG_WRN("ecc: bit flips corrected (%d)", ecc);
		break;
	case 0:
	default:
		break;	

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
	struct spi_nand_data *data = dev->driver_data;
	int ret;
	uint8_t buf[4];
	char cbuf[MAX(sizeof(data->signature),MAX(sizeof(data->manufacturer),sizeof(data->model))) + 1];

	acquire_device(dev);

	/* write the page buffer to the flash cell array */
	ret = spi_nand_page_write(dev, _chip_page);
	if (ret < 0) {
		goto done;
	}

	spi_nand_device_data_mode_enable(dev, true);
	ret = spi_nand_read_cell_array(dev, SPI_NAND_PARAMETER_PAGE_ADDRESS);
	if (ret < 0) {
		goto done;
	}
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_SIGNATURE, data->signature, SPI_NAND_PARAMETER_SIGNATURE_LEN);
	snprintk(buf, MIN(sizeof(buf),sizeof(data->signature)+1), "%4s", data->signature);
	LOG_DBG("Signature: %s", log_strdup(cbuf));
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_DEVICE_MFG, data->manufacturer, SPI_NAND_PARAMETER_DEVICE_MFG_LEN);
	snprintk(buf, MIN(sizeof(buf),sizeof(data->manufacturer)+1), "%12s", data->manufacturer);
	LOG_DBG("Manufacturer: %s", log_strdup(cbuf));
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_DEVICE_MODEL, data->model, SPI_NAND_PARAMETER_DEVICE_MODEL_LEN);
	snprintk(buf, MIN(sizeof(buf),sizeof(data->model)+1), "%20s", data->model);
	LOG_DBG("Model: %s", log_strdup(cbuf));
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_JEDEC_ID, &data->jedec_id, SPI_NAND_PARAMETER_JEDEC_ID_LEN);
	LOG_DBG("JEDEC ID: %d", data->jedec_id);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_BITS_PER_CELL, &data->bits_per_cell, SPI_NAND_PARAMETER_BITS_PER_CELL_LEN);
	LOG_DBG("bits per cell: %d", data->bits_per_cell);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PROGRAMS_PER_PAGE, &data->programs_per_page, SPI_NAND_PARAMETER_PROGRAMS_PER_PAGE_LEN);
	LOG_DBG("programs per page: %d", data->programs_per_page);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_LUNS, &data->luns_per_device, SPI_NAND_PARAMETER_LUNS_LEN);
	LOG_DBG("LUNs per device: %d", data->luns_per_device);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PAGE_BYTES, buf, SPI_NAND_PARAMETER_PAGE_BYTES_LEN);
	data->page_size = sys_get_le32(buf);
	LOG_DBG("bytes per page: %d", data->page_size);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PARTIAL_PAGE_BYTES, buf, SPI_NAND_PARAMETER_PARTIAL_PAGE_BYTES_LEN);
	data->partial_page_size = sys_get_le32(buf);
	LOG_DBG("bytes per partial page: %d", data->partial_page_size);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PAGES_PER_BLOCK, buf, SPI_NAND_PARAMETER_PAGES_PER_BLOCK_LEN);
	data->pages_per_block = sys_get_le32(buf);
	LOG_DBG("pages per block: %d", data->pages_per_block);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_BLOCKS_PER_LUN, buf, SPI_NAND_PARAMETER_BLOCKS_PER_LUN_LEN);
	data->blocks_per_lun = sys_get_le32(buf);
	LOG_DBG("blocks per LUN: %d", data->blocks_per_lun);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_BLOCK_ENDURANCE, buf, SPI_NAND_PARAMETER_BLOCK_ENDURANCE_LEN);
	data->block_endurance = sys_get_le16(buf);
	LOG_DBG("block endurance: %d", data->block_endurance);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PAGE_PROGRAM_TIME, buf, SPI_NAND_PARAMETER_PAGE_PROGRAM_TIME_LEN);
	data->page_prog_time = sys_get_le16(buf);
	LOG_DBG("page program time: %d", data->page_prog_time);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_BLOCK_ERASE_TIME, buf, SPI_NAND_PARAMETER_BLOCK_ERASE_TIME_LEN);
	data->block_erase_time = sys_get_le16(buf);
	LOG_DBG("block erase time: %d", data->block_erase_time);
	spi_nand_read_page_buf(dev, SPI_NAND_PARAMETER_PAGE_READ_TIME, buf, SPI_NAND_PARAMETER_PAGE_READ_TIME_LEN);
	data->page_read_time = sys_get_le16(buf);
	LOG_DBG("page read time: %d", data->page_read_time);

done:
	_chip_page = SPI_NAND_INVALID_PAGE;
	spi_nand_device_data_mode_enable(dev, false);

	release_device(dev);
	return ret;
}

/*  TODO:  THIS NEEDS TO BE TESTED  */
__attribute__((unused))
static int spi_nand_get_bad_blocks(struct device *dev, u32_t Bad_Block_Table[])
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
	const struct spi_nand_config *params = dev->config_info;
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
			/* write the page buffer to the flash cell array */
			ret = spi_nand_page_write(dev, _chip_page);
			if (ret < 0) {
				goto done;
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
	const struct spi_nand_config *params = dev->config_info;
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
	// spi_nand_high_speed_mode(dev, false);

	while (remain) {
		// Fill page buffer
		if (_chip_page != row_addr) {
			/* write the page buffer to the flash cell array */
			ret = spi_nand_page_write(dev, _chip_page);
			if (ret < 0) {
				goto cleanup;
			}
			/* load the page into the page buffer from flash */
			ret = spi_nand_read_cell_array(dev, row_addr);
			if (ret < 0) {
				goto cleanup;
			}
		}

		/* WREN must be set immediately before the page buffer write command */
		if (!_write_protect) {
			spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
		}
		ret = spi_nand_write_page_buf(dev, page_offset, src, wr_bytes);
		if (ret < 0) {
			goto cleanup;
		}
		if (!_write_protect) {
			_page_sync_required = true;
		}

		src = (const u8_t *)src + wr_bytes;
		remain -= wr_bytes;
		row_addr++;
		page_offset = 0;
		wr_bytes = MIN(addr + size - (row_addr * SPI_NAND_PAGE_SIZE), SPI_NAND_PAGE_SIZE);
	}
	/* write the page buffer to the flash cell array */
	ret = spi_nand_page_write(dev, _chip_page);

cleanup:
	// spi_nand_high_speed_mode(dev, true);
	release_device(dev);
	return ret;
}

static int spi_nand_erase(struct device *dev, off_t addr, size_t size)
{
	const struct spi_nand_config *params = dev->config_info;
	int ret = 0;
	u8_t reg;

	/* should be between 0 and flash size */
	if ((addr < 0) || ((size + addr) > params->size)) {
		return -ENODEV;
	}

	acquire_device(dev);

	ret = spi_nand_read_status(dev, &reg);
	if (ret < 0) {
		goto out;
	}
	if (!(reg & SPI_NAND_STATUS_WEL_BIT)) {
		LOG_ERR("WEL=0, block erase aborted: 0x%lx", (long)addr);
		ret = -EROFS;
		goto out;
	}

	while (size) {
		if ((size >= SPI_NAND_BLOCK_SIZE)
			  && is_block_aligned(addr)) {
			/* 256 KiB block erase */
			/* WREN must be set immediately before the block erase command */
			if (!_write_protect) {
				spi_nand_cmd_write(dev, SPI_NAND_CMD_WREN);
			}
			ret = spi_nand_erase_block(dev, addr / SPI_NAND_PAGE_SIZE);
			if (ret < 0) {
				LOG_ERR("block erase error sending command");
			}
			/* wait for OIP */
			ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_BLOCK_ERASE_TIME);
			if (ret == -ETIMEDOUT) {
				LOG_ERR("block erase timed out");
				ret = -EFAULT;
				goto out;
			} else if (ret < 0) {
				LOG_ERR("error during block erase: %d", ret);
				ret = -EFAULT;
				goto out;
			}
			/* check ERS_F */
			if (reg & SPI_NAND_STATUS_ERASEF_BIT) {
				LOG_ERR("block erase failed at addr: 0x%08lx", (long)addr);
				spi_nand_reset(dev);
				/*
				*  Return EFAULT to indicate corrupt erase
				*  This error code is mapped to LFS_ERR_CORRUPT
				*  by errno_to_lfs()
				*  see littlefs_fs.c
				*/
				ret = -EFAULT;
				goto out;
			}
			addr += SPI_NAND_BLOCK_SIZE;
			size -= SPI_NAND_BLOCK_SIZE;
		} else {
			/* minimal erase size is at least a block size */
			LOG_ERR("unsupported block erase at 0x%lx size %zu", (long)addr,
				size);
			ret = -EINVAL;
			goto out;
		}
	}

out:
	release_device(dev);

	return ret;
}

static int spi_nand_sync(struct device *dev)
{
	int ret;

	acquire_device(dev);
	
	ret = spi_nand_page_write(dev, _chip_page);

	release_device(dev);

	return ret;
}

static int spi_nand_write_protection_set(struct device *dev, bool write_protect)
{
	int ret;
	uint8_t reg;

	acquire_device(dev);

	ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_BLOCK_ERASE_TIME);
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
done:
	release_device(dev);

	return ret;
}

void spi_nand_get_registers(struct device *dev, u8_t *status, u8_t *ctrl, u8_t *lock)
{
	spi_nand_read_status(dev, status);
	spi_nand_read_ctrl(dev, ctrl);
	spi_nand_read_lock(dev, lock);
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
	u8_t reg;

	struct spi_nand_data *data = dev->driver_data;
	const struct spi_nand_config *params = dev->config_info;
	int ret;

	data->spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!data->spi) {
		LOG_ERR("could not bind spi bus");
		return -EINVAL;
	}

	data->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	data->spi_cfg.operation = SPI_WORD_SET(8);
	data->spi_cfg.slave = DT_INST_REG_ADDR(0);
	switch (params->spi_mode) {
	case 0:
		break;
	case 1:
		data->spi_cfg.operation |= SPI_MODE_CPHA;
		break;
	case 2:
		data->spi_cfg.operation |= SPI_MODE_CPOL;
		break;
	case 3:
		data->spi_cfg.operation |= SPI_MODE_CPOL;
		data->spi_cfg.operation |= SPI_MODE_CPHA;
		break;
	default:
		LOG_ERR("invalid SPI mode");
		return -EINVAL;
	}

#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
	if (!data->cs_ctrl.gpio_dev) {
		LOG_ERR("could not bind cs gpio");
		return -ENODEV;
	}

	data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
	data->cs_ctrl.delay = CONFIG_SPI_NAND_CS_WAIT_DELAY;

	data->spi_cfg.cs = &data->cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */

	/* wait until device ready */
	ret = spi_nand_wait_until_ready(dev, &reg, SPI_NAND_TIMER_POWER_UP);
	if (ret < 0) {
		LOG_ERR("not ready after power up");
		return ret;
	}

	/* Might be in DPD if system restarted without power cycle. */
	// exit_dpd(dev);

	ret = spi_nand_reset(dev);
	if (ret < 0) {
		LOG_ERR("could not reset nand");
		return ret;
	}

	/* now the spi bus is configured, we can verify the flash id */
	ret = spi_nand_read_id(dev, params);
	if (ret != 0) {
		LOG_ERR("jedec id match failed");
		return -ENODEV;
	}

	LOG_DBG("jedec id match");

	ret = spi_nand_read_parameter_page(dev);
	if (ret < 0) {
		LOG_ERR("read parameter page failed");
		return -ENODEV;
	};

	/* all flash blocks are locked at power on */
	/* so unlock them */
	/* see datasheet 4.10 */
	ret = spi_nand_block_lock(dev, 0);
	if (ret < 0) {
		LOG_ERR("block unlock failed");
		return -ENODEV;
	}
	
	// if (IS_ENABLED(CONFIG_SPI_NAND_IDLE_IN_DPD)
	//     && (enter_dpd(dev) != 0)) {
	// 	LOG_ERR("enter dpd failed");
	// 	return -ENODEV;
	// }

	/* disable writes */
	ret = spi_nand_cmd_write(dev, SPI_NAND_CMD_WRDI);
	if (ret < 0) {
		LOG_ERR("write disable failed");
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
	int ret;

	if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		struct spi_nand_data *const driver_data = dev->driver_data;

		k_sem_init(&driver_data->sem, 1, UINT_MAX);
	}

	/* device cannot be accessed for an period tvsl after Vcc > Vcc(min) */
	while (k_uptime_ticks() < k_us_to_ticks_ceil32(SPI_NAND_TVSL)) {
		/* wait for chip to power up */
	}
	ret = spi_nand_configure(dev);
	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

BUILD_ASSERT(SPI_NAND_IS_SECTOR_ALIGNED(SPI_NAND_BLOCK_SIZE),
	     "LAYOUT_PAGE_SIZE must be multiple of sector size");

#define LAYOUT_PAGES_COUNT (SPI_NAND_SIZE / SPI_NAND_BLOCK_SIZE)

static const struct flash_pages_layout dev_layout = {
	.pages_count = LAYOUT_PAGES_COUNT,
	.pages_size = SPI_NAND_BLOCK_SIZE,
};
#undef LAYOUT_PAGES_COUNT

static const struct flash_parameters spi_nand_parameters = {
	.write_block_size = SPI_NAND_PAGE_SIZE,
	.erase_value = 0xff,
};

static void spi_nand_pages_layout(struct device *dev,
				 const struct flash_pages_layout **layout,
				 size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
spi_nand_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &spi_nand_parameters;
}

static const struct flash_driver_api spi_nand_api = {
	.read = spi_nand_read,
	.write = spi_nand_write,
	.erase = spi_nand_erase,
	.sync = spi_nand_sync,
	.write_protection = spi_nand_write_protection_set,
	.get_parameters = spi_nand_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = spi_nand_pages_layout,
#endif
};

static const struct spi_nand_config flash_id = {
	.id = DT_INST_PROP(0, jedec_id),
	.spi_mode = DT_INST_PROP(0, spi_transfer_mode),
	.size = DT_INST_PROP(0, size),
	.api = {
		.reg_read = spi_nand_reg_read,
		.reg_write = spi_nand_reg_write,
	},
};

static struct spi_nand_data spi_nand_memory_data;

DEVICE_AND_API_INIT(spi_flash_memory, DT_INST_LABEL(0),
		    &spi_nand_init, &spi_nand_memory_data,
		    &flash_id, POST_KERNEL, CONFIG_SPI_NAND_INIT_PRIORITY,
		    &spi_nand_api);
