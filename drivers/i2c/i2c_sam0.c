/*
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_i2c

#include <errno.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/i2c.h>
#include <drivers/dma.h>
#include <power/power.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_sam0, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

#ifndef SERCOM_I2CM_CTRLA_MODE_I2C_MASTER
#define SERCOM_I2CM_CTRLA_MODE_I2C_MASTER SERCOM_I2CM_CTRLA_MODE(5)
#endif

#ifndef SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE
#define SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE SERCOM_I2CS_CTRLA_MODE(4)
#endif

/* max i2c idle bus wait timeout is approx. 20us */
#define I2C_IDLE_TIMEOUT 20

struct i2c_sam0_dev_config {
	SercomI2cm *regs;
	uint32_t bitrate;
	uint16_t gclk_gen;
	uint32_t gclk_freq;
	uint16_t gclk_slow_id;
	uint16_t gclk_slow_gen;
#ifdef MCLK
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint16_t gclk_core_id;
#else
	uint32_t pm_apbcmask;
	uint16_t gclk_clkctrl_id;
#endif
	void (*irq_config_func)(struct device *dev);

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN
	char *dma_dev;
	uint8_t write_dma_request;
	uint8_t read_dma_request;
	uint8_t dma_channel;
#endif
};

struct i2c_sam0_msg {
	uint8_t *buffer;
	uint32_t size;
	uint32_t status;
};

struct i2c_sam0_dev_data {
	struct k_sem completion_sync;
	struct k_sem transfer_sync;
	struct i2c_sam0_msg msg;
	struct i2c_msg *msgs;
	u8_t num_msgs;

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN
	struct device *dma;
#endif

#ifdef CONFIG_I2C_SLAVE
	bool master_active;
	struct i2c_slave_config *slave_cfg;
	bool slave_attached;
#endif /* CONFIG_I2C_SLAVE */
};

#ifdef CONFIG_I2C_SLAVE
static void i2c_sam0_slave_event(struct device *dev);
#endif /* CONFIG_I2C_SLAVE */

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	((const struct i2c_sam0_dev_config *const)(dev)->config_info)
#define DEV_DATA(dev) \
	((struct i2c_sam0_dev_data *const)(dev)->driver_data)

static void wait_synchronization(SercomI2cm *regs)
{
#if defined(SERCOM_I2CM_SYNCBUSY_MASK)
	/* SYNCBUSY is a register */
	while ((regs->SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_MASK) != 0) {
	}
#elif defined(SERCOM_I2CM_STATUS_SYNCBUSY)
	/* SYNCBUSY is a bit */
	while ((regs->STATUS.reg & SERCOM_I2CM_STATUS_SYNCBUSY) != 0) {
	}
#else
#error Unsupported device
#endif
}

static bool i2c_sam0_terminate_on_error(struct device *dev)
{
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;

	if (!(i2c->STATUS.reg & (SERCOM_I2CM_STATUS_ARBLOST |
				 SERCOM_I2CM_STATUS_RXNACK |
#ifdef SERCOM_I2CM_STATUS_LENERR
				 SERCOM_I2CM_STATUS_LENERR |
#endif
#ifdef SERCOM_I2CM_STATUS_SEXTTOUT
				 SERCOM_I2CM_STATUS_SEXTTOUT |
#endif
#ifdef SERCOM_I2CM_STATUS_MEXTTOUT
				 SERCOM_I2CM_STATUS_MEXTTOUT |
#endif
				 SERCOM_I2CM_STATUS_LOWTOUT |
				 SERCOM_I2CM_STATUS_BUSERR))) {
		return false;
	}

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN
	if (data->dma && cfg->dma_channel != 0xFF) {
		dma_stop(data->dma, cfg->dma_channel);
	}
#endif

	data->msg.status = i2c->STATUS.reg;

	if (i2c->STATUS.reg & SERCOM_I2CM_STATUS_RXNACK) {
		i2c->CTRLB.bit.CMD = 3;
	}
	/*
	 * Clear all the flags that require an explicit clear
	 * (as opposed to being cleared by ADDR writes, etc)
	 */
	i2c->STATUS.reg = SERCOM_I2CM_STATUS_ARBLOST |
#ifdef SERCOM_I2CM_STATUS_LENERR
			  SERCOM_I2CM_STATUS_LENERR |
#endif
			  SERCOM_I2CM_STATUS_LOWTOUT |
			  SERCOM_I2CM_STATUS_BUSERR;

	i2c->INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR | SERCOM_I2CM_INTFLAG_MB
			 | SERCOM_I2CM_INTFLAG_SB;
	i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
	k_sem_give(&data->completion_sync);
	return true;
}

/*
 *  The i2c CMD bits can only be set when either the Slave on Bus
 *  interrupt flag (INTFLAG.SB) or Master on Bus interrupt flag
 *  (INTFLAG.MB) is '1'.  See 28.10.2
 *
 *  Writing or reading the data register when SBEN is enabled clears
 *  the SB and MB interrupt status bits.
 *
 *  So, the STOP command must be issued before the MB and SB interrupt
 *  flag bits are cleared; otherwise:
 *    1. the CMD bits are ignored,
 *    2. a STOP does not occur,
 *    3. the bus remains in the OWNER state,
 *    4. the clock (SCL) is held low.
 *  See 28.10.6
 */

static void i2c_sam0_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;

#ifdef CONFIG_I2C_SLAVE
	if (data->slave_attached && !data->master_active) {
		i2c_sam0_slave_event(dev);
		return;
	}
#endif /* CONFIG_I2C_SLAVE */

	int key = irq_lock();

	/* Get present interrupts */
	u32_t status = i2c->INTFLAG.reg;

	if (i2c_sam0_terminate_on_error(dev)) {
		irq_unlock(key);
		return;
	}

	if (status & SERCOM_I2CM_INTFLAG_MB) {
		if (data->msg.size == 0) {
			if (data->msgs->flags & I2C_MSG_STOP) {
				i2c->CTRLB.bit.CMD = 3;
			} else if (data->num_msgs > 1) {
				if ((data->msgs+1)->flags & I2C_MSG_RESTART) {
					/*
					 * if next message is flagged
					 * to restart, clear MB flag and
					 * do not send STOP.
					 */
					i2c->INTFLAG.reg &=
						~SERCOM_I2CM_INTFLAG_MB;
				}
			} else {
				/*  default:  send STOP  */
				i2c->CTRLB.bit.CMD = 3;
			}
			i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
			k_sem_give(&data->completion_sync);
			irq_unlock(key);
			return;
		}

		i2c->DATA.reg = *data->msg.buffer;
		data->msg.buffer++;
		data->msg.size--;
	}

	if (status & SERCOM_I2CM_INTFLAG_SB) {
		if (data->msg.size == 1) {
			/*
			* If this is the last byte, then prepare for an auto
			* NACK before doing the actual read.  This does not
			* require write synchronization.
			*/
			i2c->CTRLB.bit.ACKACT = 1;
			if (data->msgs->flags & I2C_MSG_STOP) {
				i2c->CTRLB.bit.CMD = 3;
			} else if (data->num_msgs > 1) {
				if ((data->msgs+1)->flags & I2C_MSG_RESTART) {
					/*
					 * if next message is flagged
					 * to restart, clear SB flag and
					 * do not send STOP.
					 */
					i2c->INTFLAG.reg &=
						~SERCOM_I2CM_INTFLAG_SB;
				}
			} else {
				/*  default:  send STOP  */
				i2c->CTRLB.bit.CMD = 3;
			}
		}

		*data->msg.buffer = i2c->DATA.reg;
		data->msg.buffer++;
		data->msg.size--;

		if (data->msg.size == 0) {
			i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
			k_sem_give(&data->completion_sync);
			irq_unlock(key);
			return;
		}
	}

	irq_unlock(key);
	return;
}

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN

static void i2c_sam0_dma_write_done(void *arg, uint32_t id, int error_code)
{
	struct device *dev = arg;
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;

	ARG_UNUSED(id);

	int key = irq_lock();

	if (i2c_sam0_terminate_on_error(dev)) {
		irq_unlock(key);
		return;
	}

	if (error_code < 0) {
		LOG_ERR("DMA write error on %s: %d", DEV_NAME(dev), error_code);
		i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
		irq_unlock(key);

		data->msg.status = error_code;

		k_sem_give(&data->completion_sync);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has written the whole message now, so just wait for the
	 * final I2C IRQ to indicate that it's finished transmitting.
	 */
	data->msg.size = 0;
	i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_MB;
}

static bool i2c_sam0_dma_write_start(struct device *dev)
{
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	int retval;

	if (!data->dma) {
		return false;
	}

	if (cfg->dma_channel == 0xFF) {
		return false;
	}

	if (data->msg.size <= 1) {
		/*
		 * Catch empty writes and skip DMA on single byte transfers.
		 */
		return false;
	}

	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_blk = { 0 };

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.callback_arg = dev;
	dma_cfg.dma_callback = i2c_sam0_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->write_dma_request;

	dma_blk.block_size = data->msg.size;
	dma_blk.source_address = (uint32_t)data->msg.buffer;
	dma_blk.dest_address = (uint32_t)(&(i2c->DATA.reg));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(data->dma, cfg->dma_channel, &dma_cfg);
	if (retval != 0) {
		LOG_ERR("Write DMA configure on %s failed: %d",
			DEV_NAME(dev), retval);
		return false;
	}

	retval = dma_start(data->dma, cfg->dma_channel);
	if (retval != 0) {
		LOG_ERR("Write DMA start on %s failed: %d",
			DEV_NAME(dev), retval);
		return false;
	}

	return true;
}

static void i2c_sam0_dma_read_done(void *arg, uint32_t id, int error_code)
{
	struct device *dev = arg;
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;

	ARG_UNUSED(id);

	int key = irq_lock();

	if (i2c_sam0_terminate_on_error(dev)) {
		irq_unlock(key);
		return;
	}

	if (error_code < 0) {
		LOG_ERR("DMA read error on %s: %d", DEV_NAME(dev), error_code);
		i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
		irq_unlock(key);

		data->msg.status = error_code;

		k_sem_give(&data->completion_sync);
		return;
	}

	irq_unlock(key);

	/*
	 * DMA has read all but the last byte now, so let the ISR handle
	 * that and the terminating NACK.
	 */
	data->msg.buffer += data->msg.size - 1;
	data->msg.size = 1;
	i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_SB;
}

static bool i2c_sam0_dma_read_start(struct device *dev)
{
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	int retval;

	if (!data->dma) {
		return false;
	}

	if (cfg->dma_channel == 0xFF) {
		return false;
	}

	if (data->msg.size <= 2) {
		/*
		 * The last byte is always handled by the I2C ISR so
		 * just skip a two length read as well.
		 */
		return false;
	}

	struct dma_config dma_cfg = { 0 };
	struct dma_block_config dma_blk = { 0 };

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.callback_arg = dev;
	dma_cfg.dma_callback = i2c_sam0_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = cfg->read_dma_request;

	dma_blk.block_size = data->msg.size - 1;
	dma_blk.dest_address = (uint32_t)data->msg.buffer;
	dma_blk.source_address = (uint32_t)(&(i2c->DATA.reg));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	retval = dma_config(data->dma, cfg->dma_channel, &dma_cfg);
	if (retval != 0) {
		LOG_ERR("Read DMA configure on %s failed: %d",
			DEV_NAME(dev), retval);
		return false;
	}

	retval = dma_start(data->dma, cfg->dma_channel);
	if (retval != 0) {
		LOG_ERR("Read DMA start on %s failed: %d",
			DEV_NAME(dev), retval);
		return false;
	}

	return true;
}

#endif

static int i2c_sam0_transfer(struct device *dev, struct i2c_msg *msgs,
			     uint8_t num_msgs, uint16_t addr)
{
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	u32_t addr_reg;
	int ret = 0;
	u32_t i;

	if (!num_msgs) {
		return 0;
	}

#ifdef CONFIG_I2C_SLAVE
	if (data->slave_attached) {
		LOG_ERR("Driver is registered as slave");
		return -EBUSY;
	}
#endif /* CONFIG_I2C_SLAVE */

	i = 0;
	wait_synchronization(i2c);
	while (i2c->STATUS.bit.BUSSTATE != 1) {
		if (i > I2C_IDLE_TIMEOUT) {
			LOG_WRN("Bus is not idle");
			return -EBUSY;
		}
		k_busy_wait(1);
		i++;
	}

	if (k_sem_take(&data->transfer_sync, K_MSEC(1000)) == -EAGAIN) {
		LOG_WRN("Failed to acquire sam0 i2c device lock");
		return -EAGAIN;
	}

#ifdef CONFIG_I2C_SLAVE
	data->master_active = true;
#endif /* CONFIG_I2C_SLAVE */

	for (; num_msgs > 0;) {
		if (!msgs->len) {
			if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
				ret = -EINVAL;
				break;
			}
		}

		i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;
		i2c->INTFLAG.reg = SERCOM_I2CM_INTFLAG_MASK;

		i2c->STATUS.reg = SERCOM_I2CM_STATUS_ARBLOST |
#ifdef SERCOM_I2CM_STATUS_LENERR
				  SERCOM_I2CM_STATUS_LENERR |
#endif
				  SERCOM_I2CM_STATUS_LOWTOUT |
				  SERCOM_I2CM_STATUS_BUSERR;
		wait_synchronization(i2c);

		data->msg.buffer = msgs->buf;
		data->msg.size = msgs->len;
		data->msg.status = 0;
		data->msgs = msgs;
		data->num_msgs = num_msgs;

		addr_reg = addr << 1U;
		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			addr_reg |= 1U;

			/* Set to auto ACK */
			i2c->CTRLB.bit.ACKACT = 0;
			wait_synchronization(i2c);
		}

		if (msgs->flags & I2C_MSG_ADDR_10_BITS) {
#ifdef SERCOM_I2CM_ADDR_TENBITEN
			addr_reg |= SERCOM_I2CM_ADDR_TENBITEN;
#else
			ret = -ENOTSUP;
			break;
#endif
		}

		int key = irq_lock();

		/*
		 * Writing the address starts the transaction, issuing
		 * a start/repeated start as required.
		 */
		i2c->ADDR.reg = addr_reg;

		/* enable MB interrupt */
		i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_MB;

#ifdef SERCOM_I2CM_INTENSET_ERROR
		i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_ERROR;
#endif

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN
		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			if (!i2c_sam0_dma_read_start(dev)) {
				i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_SB;
			}
		} else {
			i2c_sam0_dma_write_start(dev);
		}
#else
		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			i2c->INTENSET.reg = SERCOM_I2CM_INTENSET_SB;
		}
#endif

		irq_unlock(key);

		/*
		 * Wait for the ISR to handle bus transaction
		 *
		 * The following wait is a busy wait that prevents the kernel
		 * from idling and therefore prevents system power management
		 * until the bus transaction is complete.
		 */
		while (k_sem_take(&data->completion_sync, K_NO_WAIT)
			== -EBUSY) {
			/*  busy wait for semaphore  */
		}

		if (data->msg.status) {
			if (data->msg.status & SERCOM_I2CM_STATUS_ARBLOST) {
				LOG_DBG("Arbitration lost on %s",
					DEV_NAME(dev));
				ret = -EAGAIN;
				break;
			} else if (data->msg.status
				   & SERCOM_I2CM_STATUS_RXNACK) {
				LOG_DBG("RXNACK on %s", DEV_NAME(dev));
				ret = -EAGAIN;
				break;
			}

			LOG_ERR("Transaction error on %s: %08X",
				DEV_NAME(dev), data->msg.status);
			ret = -EIO;
			break;
		}

		num_msgs--;
		msgs++;
	}

#ifdef CONFIG_I2C_SLAVE
	data->master_active = false;
#endif /* CONFIG_I2C_SLAVE */

	k_sem_give(&data->transfer_sync);
	return ret;
}

static int i2c_sam0_set_apply_bitrate(struct device *dev, uint32_t config)
{
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	uint32_t baud;
	uint32_t baud_low;
	uint32_t baud_high;

	uint32_t CTRLA = i2c->CTRLA.reg;

#ifdef SERCOM_I2CM_CTRLA_SPEED_Msk
	CTRLA &= ~SERCOM_I2CM_CTRLA_SPEED_Msk;
#endif
	CTRLA &= ~SERCOM_I2CM_CTRLA_SDAHOLD_Msk;

	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
#ifdef SERCOM_I2CM_CTRLA_SPEED
		CTRLA |= SERCOM_I2CM_CTRLA_SPEED(0);
#endif
		CTRLA |= SERCOM_I2CM_CTRLA_SDAHOLD(0x0);
		i2c->CTRLA.reg = CTRLA;
		wait_synchronization(i2c);

		/* 5 is the nominal 100ns rise time from the app notes */
		baud = (cfg->gclk_freq / 100000U) / 2U;
		if (baud > 255U) {
			return -ERANGE;
		}

		LOG_DBG("Setting %s to standard mode with divisor %u",
			DEV_NAME(dev), baud);

		i2c->BAUD.reg = SERCOM_I2CM_BAUD_BAUD(baud);
		break;

	case I2C_SPEED_FAST:
		CTRLA |= SERCOM_I2CM_CTRLA_SDAHOLD(0x0);
		i2c->CTRLA.reg = CTRLA;
		wait_synchronization(i2c);

		/* 5 is the nominal 100ns rise time from the app notes */
		baud = (cfg->gclk_freq / 400000U) / 2U;
		if (baud > 255U) {
			return -ERANGE;
		}

		LOG_DBG("Setting %s to fast mode with divisor %u",
			DEV_NAME(dev), baud);

		i2c->BAUD.reg = SERCOM_I2CM_BAUD_BAUD(baud);
		break;

	case I2C_SPEED_FAST_PLUS:
#ifdef SERCOM_I2CM_CTRLA_SPEED
		CTRLA |= SERCOM_I2CM_CTRLA_SPEED(1);
#endif
		CTRLA |= SERCOM_I2CM_CTRLA_SDAHOLD(0x2);
		i2c->CTRLA.reg = CTRLA;
		wait_synchronization(i2c);

		/* 5 is the nominal 100ns rise time from the app notes */
		baud = (cfg->gclk_freq / 1000000U);

		/* 2:1 low:high ratio */
		baud_high = baud;
		baud_high /= 3U;
		baud_high = MAX(MIN(baud_high, 255U), 1U);
		baud_low = baud - baud_high;
		if (baud_low < 1U && baud_high > 1U) {
			--baud_high;
			++baud_low;
		}

		if (baud_low < 1U || baud_low > 255U) {
			return -ERANGE;
		}

		LOG_DBG("Setting %s to fast mode plus with divisors %u/%u",
			DEV_NAME(dev), baud_high, baud_low);

		i2c->BAUD.reg = SERCOM_I2CM_BAUD_BAUD(baud_high) |
				SERCOM_I2CM_BAUD_BAUDLOW(baud_low);
		break;

	case I2C_SPEED_HIGH:
#ifdef SERCOM_I2CM_CTRLA_SPEED
		CTRLA |= SERCOM_I2CM_CTRLA_SPEED(2);
#endif
		CTRLA |= SERCOM_I2CM_CTRLA_SDAHOLD(0x2);
		i2c->CTRLA.reg = CTRLA;
		wait_synchronization(i2c);

		baud = (cfg->gclk_freq / 3400000U);

		/* 2:1 low:high ratio */
		baud_high = baud;
		baud_high /= 3U;
		baud_high = MAX(MIN(baud_high, 255U), 1U);
		baud_low = baud - baud_high;
		if (baud_low < 1U && baud_high > 1U) {
			--baud_high;
			++baud_low;
		}

		if (baud_low < 1U || baud_low > 255U) {
			return -ERANGE;
		}

#ifdef SERCOM_I2CM_BAUD_HSBAUD
		LOG_DBG("Setting %s to high speed with divisors %u/%u",
			DEV_NAME(dev), baud_high, baud_low);

		/*
		 * 48 is just from the app notes, but the datasheet says
		 * it's ignored
		 */
		i2c->BAUD.reg = SERCOM_I2CM_BAUD_HSBAUD(baud_high) |
				SERCOM_I2CM_BAUD_HSBAUDLOW(baud_low) |
				SERCOM_I2CM_BAUD_BAUD(48) |
				SERCOM_I2CM_BAUD_BAUDLOW(48);
#else
		return -ENOTSUP;
#endif
		break;

	default:
		return -ENOTSUP;
	}

	wait_synchronization(i2c);
	return 0;
}

static int i2c_sam0_configure(struct device *dev, uint32_t config)
{
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	int retval;

	if (!(config & I2C_MODE_MASTER)) {
		/* configure for slave mode */
		i2c->CTRLA.bit.ENABLE = 0;
		wait_synchronization(i2c);

		/* Disable all I2C interrupts */
		i2c->INTENCLR.reg = SERCOM_I2CS_INTENCLR_MASK;

		/* I2C mode, enable timeouts */
		i2c->CTRLA.reg = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE
#ifdef SERCOM_I2CS_CTRLA_LOWTOUTEN
			| SERCOM_I2CS_CTRLA_LOWTOUTEN
#endif
			| SERCOM_I2CS_CTRLA_RUNSTDBY;
		wait_synchronization(i2c);

		/*
		 * Enable smart mode (auto ACK data received)
		 * and auto ack an address match
		 */
		i2c->CTRLB.reg = SERCOM_I2CS_CTRLB_SMEN;
		wait_synchronization(i2c);

		i2c->CTRLA.bit.ENABLE = 1;
		wait_synchronization(i2c);
	}

	if (config & I2C_MODE_MASTER) {
		/* configure for master mode */
		i2c->CTRLA.bit.ENABLE = 0;
		wait_synchronization(i2c);

		/* Disable all I2C interrupts */
		i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;

		/* I2C mode, enable timeouts */
		i2c->CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER
#ifdef SERCOM_I2CM_CTRLA_LOWTOUTEN
			| SERCOM_I2CM_CTRLA_LOWTOUTEN
#endif
			| SERCOM_I2CM_CTRLA_INACTOUT(0x3);
		wait_synchronization(i2c);

		/* Enable smart mode (auto ACK) */
		i2c->CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
		wait_synchronization(i2c);

		i2c->CTRLA.bit.ENABLE = 1;
		wait_synchronization(i2c);
	}

	if (config & I2C_SPEED_MASK) {
		i2c->CTRLA.bit.ENABLE = 0;
		wait_synchronization(i2c);

		retval = i2c_sam0_set_apply_bitrate(dev, config);

		i2c->CTRLA.bit.ENABLE = 1;
		wait_synchronization(i2c);

		if (retval != 0) {
			return retval;
		}
	}

	return 0;
}

static int i2c_sam0_initialize(struct device *dev)
{
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	const struct i2c_sam0_dev_config *const cfg = DEV_CFG(dev);
	SercomI2cm *i2c = cfg->regs;
	int retval;

#ifdef MCLK
	/* Enable the GCLK */
	GCLK->PCHCTRL[cfg->gclk_core_id].reg = GCLK_PCHCTRL_GEN(cfg->gclk_gen)
					     | GCLK_PCHCTRL_CHEN;
	while (!(GCLK->PCHCTRL[cfg->gclk_core_id].reg & GCLK_PCHCTRL_CHEN)) {
	}

	/*
	 * The GCLK_SERCOM_SLOW clock is used to time the time-outs
	 * and must be configured to use a 32KHz oscillator.
	 * See 14.8.4, Table 14-9 and 36.5.3
	 */
	GCLK->PCHCTRL[cfg->gclk_slow_id].reg =
		GCLK_PCHCTRL_GEN(cfg->gclk_slow_gen) | GCLK_PCHCTRL_CHEN;
	while (!(GCLK->PCHCTRL[cfg->gclk_slow_id].reg & GCLK_PCHCTRL_CHEN)) {
	}

	/* Enable SERCOM clock in MCLK */
	*cfg->mclk |= cfg->mclk_mask;
#else
	/* Enable the GCLK */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(cfg->gclk_clkctrl_id)
			  | GCLK_CLKCTRL_GEN(cfg->gclk_gen)
			  | GCLK_CLKCTRL_CLKEN;

	/*
	 * The GCLK_SERCOM_SLOW clock is used to time the time-outs
	 * and must be configured to use a 32KHz oscillator.  See 28.6.3.1
	 * GCLK4 is configured to 32768 Hz
	 */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(cfg->gclk_slow_id)
			  | GCLK_CLKCTRL_GEN(cfg->gclk_slow_gen)
			  | GCLK_CLKCTRL_CLKEN;

	/* Enable SERCOM clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;
#endif
	/* Disable all I2C interrupts */
	i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;

	/* I2C mode, enable timeouts */
	i2c->CTRLA.reg = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER |
#ifdef SERCOM_I2CM_CTRLA_LOWTOUTEN
			 SERCOM_I2CM_CTRLA_LOWTOUTEN |
#endif
			 SERCOM_I2CM_CTRLA_INACTOUT(0x3);
	wait_synchronization(i2c);

	/* Enable smart mode (auto ACK) */
	i2c->CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
	wait_synchronization(i2c);

	retval = i2c_sam0_set_apply_bitrate(dev,
			    i2c_map_dt_bitrate(cfg->bitrate));
	if (retval != 0) {
		return retval;
	}

	k_sem_init(&data->completion_sync, 0, 1);
	k_sem_init(&data->transfer_sync, 1, 1);

	cfg->irq_config_func(dev);

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN

	data->dma = device_get_binding(cfg->dma_dev);

#endif

	i2c->DBGCTRL.bit.DBGSTOP = 1;

	i2c->CTRLA.bit.ENABLE = 1;
	wait_synchronization(i2c);

	/* Force bus idle */
	i2c->STATUS.bit.BUSSTATE = 1;
	wait_synchronization(i2c);

#ifdef CONFIG_I2C_SLAVE
	data->master_active = false;
	data->slave_attached = false;
#endif /* CONFIG_I2C_SLAVE */

	return 0;
}

#ifdef CONFIG_I2C_SLAVE
static void i2c_sam0_slave_event(struct device *dev)
{
	const struct i2c_sam0_dev_config *cfg = DEV_CFG(dev);
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	SercomI2cs *i2c = (SercomI2cs *)cfg->regs;
	const struct i2c_slave_callbacks *slave_cb =
		data->slave_cfg->callbacks;

	/* Get present interrupts */
	u32_t interrupt_status = i2c->INTFLAG.reg;

	if (interrupt_status & SERCOM_I2CS_INTFLAG_ERROR) {
		/* error */
		if (i2c->STATUS.bit.LOWTOUT) {
			LOG_ERR("slave SCL low timeout");
		}

		if (i2c->STATUS.bit.SEXTTOUT) {
			LOG_ERR("slave SCL low extend timeout");
		}

		if (i2c->STATUS.bit.COLL || i2c->STATUS.bit.BUSERR) {
			/*  Bus error  */
			i2c->STATUS.bit.COLL = 1;
			i2c->STATUS.bit.BUSERR = 1;
			LOG_ERR("slave bus error");
		}
		i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_ERROR;
		i2c->INTENCLR.reg = SERCOM_I2CS_INTENCLR_ERROR
				  | SERCOM_I2CS_INTENCLR_DRDY
				  | SERCOM_I2CS_INTENCLR_PREC;
		slave_cb->stop(data->slave_cfg);
		return;
	}

	if (interrupt_status & SERCOM_I2CS_INTFLAG_PREC) {
		/* stop */
		i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
		i2c->INTENCLR.reg = SERCOM_I2CS_INTENCLR_ERROR
				  | SERCOM_I2CS_INTENCLR_DRDY
				  | SERCOM_I2CS_INTENCLR_PREC;
		slave_cb->stop(data->slave_cfg);
	}

	if (interrupt_status & SERCOM_I2CS_INTFLAG_AMATCH) {
		/* address match */
		i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;

		if (!i2c->STATUS.bit.DIR) {
			/*  Master requested write  */
			slave_cb->write_requested(data->slave_cfg);
		} else {
			/*  Master requested read  */
			u8_t val;

			slave_cb->read_requested(data->slave_cfg, &val);
			while (!(i2c->INTFLAG.reg & SERCOM_I2CS_INTFLAG_DRDY)) {
				/* wait for data ready */
			}
			i2c->DATA.reg = val;
		}

		i2c->INTENSET.reg = SERCOM_I2CS_INTENSET_ERROR
				  | SERCOM_I2CS_INTENSET_DRDY
				  | SERCOM_I2CS_INTENSET_PREC;
		i2c->CTRLB.bit.ACKACT = 0;
	}

	if (interrupt_status & SERCOM_I2CS_INTFLAG_DRDY) {
		/* data ready */
		if (!i2c->STATUS.bit.DIR) {
			/*  Master requested write  */
			u8_t val = i2c->DATA.reg;

			if (slave_cb->write_received(data->slave_cfg, val)) {
				/*
				 * If write_received() returns !=0, then
				 * handle exception and send NACK to master
				 */
			}
		} else {
			/*  Master requested read  */
			u8_t val;

			if (!i2c->STATUS.bit.RXNACK) {
				slave_cb->read_processed(data->slave_cfg, &val);
				i2c->DATA.reg = val;
			} else {
				/*
				 * Master terminated the bus transaction.
				 * Clear DRDY flag
				 */
				i2c->INTFLAG.reg = SERCOM_I2CS_INTFLAG_DRDY;
			}
		}
		return;
	}

}

/* Attach and start I2C as slave */
int i2c_sam0_slave_register(struct device *dev,
			    struct i2c_slave_config *config)
{
	const struct i2c_sam0_dev_config *cfg = DEV_CFG(dev);
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	SercomI2cs *i2c = (SercomI2cs *)cfg->regs;
	u32_t bitrate_cfg;
	int ret;

	if (!config) {
		return -EINVAL;
	}

	if (data->slave_attached) {
		return -EBUSY;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate) & ~I2C_MODE_MASTER;

	ret = i2c_sam0_configure(dev, bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

	i2c->CTRLA.bit.ENABLE = 0;
	wait_synchronization((SercomI2cm *)i2c);
	i2c->ADDR.reg = (config->address << 1);
	i2c->CTRLA.bit.ENABLE = 1;
	wait_synchronization((SercomI2cm *)i2c);
	data->slave_cfg = config;
	data->slave_attached = true;

	/* Enable Address Match interrupt */
	i2c->INTENSET.reg = SERCOM_I2CS_INTENSET_AMATCH;

	LOG_DBG("i2c: slave registered");

	return 0;
}

int i2c_sam0_slave_unregister(struct device *dev,
			       struct i2c_slave_config *config)
{
	const struct i2c_sam0_dev_config *cfg = DEV_CFG(dev);
	struct i2c_sam0_dev_data *data = DEV_DATA(dev);
	SercomI2cs *i2c = (SercomI2cs *)cfg->regs;
	int ret;

	if (!data->slave_attached) {
		return -EINVAL;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	i2c->CTRLA.bit.ENABLE = 0;
	wait_synchronization((SercomI2cm *)i2c);

	/* Disable all I2C interrupts */
	i2c->INTENCLR.reg = SERCOM_I2CM_INTENCLR_MASK;

	/* Clear all pending I2C interrupts */
	i2c->INTFLAG.reg = i2c->INTFLAG.reg;

	data->slave_attached = false;

	u32_t bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate) | I2C_MODE_MASTER;

	ret = i2c_sam0_configure(dev, bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failure initializing");
		return ret;
	}

	LOG_DBG("i2c: slave unregistered");

	return 0;
}

#endif /* CONFIG_I2C_SLAVE */


static const struct i2c_driver_api i2c_sam0_driver_api = {
	.configure = i2c_sam0_configure,
	.transfer = i2c_sam0_transfer,
#ifdef CONFIG_I2C_SLAVE
	.slave_register = i2c_sam0_slave_register,
	.slave_unregister = i2c_sam0_slave_unregister,
#endif /* CONFIG_I2C_SLAVE */
};

#ifdef CONFIG_I2C_SAM0_DMA_DRIVEN
#define I2C_SAM0_DMA_CHANNELS(n)					\
	.dma_dev = ATMEL_SAM0_DT_INST_DMA_NAME(n, tx),			\
	.write_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, tx),	\
	.read_dma_request = ATMEL_SAM0_DT_INST_DMA_TRIGSRC(n, rx),	\
	.dma_channel = ATMEL_SAM0_DT_INST_DMA_CHANNEL(n, rx),
#else
#define I2C_SAM0_DMA_CHANNELS(n)
#endif

#define SAM0_I2C_IRQ_CONNECT(n, m)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, m, irq),		\
			    DT_INST_IRQ_BY_IDX(n, m, priority),		\
			    i2c_sam0_isr,				\
			    DEVICE_GET(i2c_sam0_##n), 0);		\
		irq_enable(DT_INST_IRQ_BY_IDX(n, m, irq));		\
	} while (0)

#if DT_INST_IRQ_HAS_IDX(0, 3)
#define I2C_SAM0_IRQ_HANDLER(n)						\
static void i2c_sam0_irq_config_##n(struct device *dev)			\
{									\
	SAM0_I2C_IRQ_CONNECT(n, 0);					\
	SAM0_I2C_IRQ_CONNECT(n, 1);					\
	SAM0_I2C_IRQ_CONNECT(n, 2);					\
	SAM0_I2C_IRQ_CONNECT(n, 3);					\
}
#else
#define I2C_SAM0_IRQ_HANDLER(n)						\
static void i2c_sam0_irq_config_##n(struct device *dev)			\
{									\
	SAM0_I2C_IRQ_CONNECT(n, 0);					\
}
#endif

#ifdef MCLK
#define I2C_SAM0_CONFIG(n)						\
static const struct i2c_sam0_dev_config i2c_sam0_dev_config_##n = {	\
	.regs = (SercomI2cm *)DT_INST_REG_ADDR(n),			\
	.bitrate = DT_INST_PROP(n, clock_frequency),			\
	.mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(n),	\
	.mclk_mask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, bit)),	\
	.gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, periph_ch),\
	.gclk_gen = ATMEL_SAM0_DT_INST_GCLK_REG_ADDR(n),		\
	.gclk_freq = ATMEL_SAM0_DT_INST_GCLK_FREQ_HZ(n),		\
	.gclk_slow_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk_slow, periph_ch),\
	.gclk_slow_gen = ATMEL_SAM0_DT_INST_GCLK_SLOW_REG_ADDR(n),	\
	.irq_config_func = &i2c_sam0_irq_config_##n,			\
	I2C_SAM0_DMA_CHANNELS(n)					\
}
#else /* !MCLK */
#define I2C_SAM0_CONFIG(n)						\
static const struct i2c_sam0_dev_config i2c_sam0_dev_config_##n = {	\
	.regs = (SercomI2cm *)DT_INST_REG_ADDR(n),			\
	.bitrate = DT_INST_PROP(n, clock_frequency),			\
	.pm_apbcmask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(n, pm, bit)),	\
	.gclk_clkctrl_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk, clkctrl_id),\
	.gclk_gen = ATMEL_SAM0_DT_INST_GCLK_REG_ADDR(n),		\
	.gclk_freq = ATMEL_SAM0_DT_INST_GCLK_FREQ_HZ(n),		\
	.gclk_slow_id = DT_INST_CLOCKS_CELL_BY_NAME(n, gclk_slow, clkctrl_id),\
	.gclk_slow_gen = ATMEL_SAM0_DT_INST_GCLK_SLOW_REG_ADDR(n),	\
	.irq_config_func = &i2c_sam0_irq_config_##n,			\
	I2C_SAM0_DMA_CHANNELS(n)					\
}
#endif

#define I2C_SAM0_DEVICE(n)						\
	static void i2c_sam0_irq_config_##n(struct device *dev);	\
	I2C_SAM0_CONFIG(n);						\
	static struct i2c_sam0_dev_data i2c_sam0_dev_data_##n;		\
	DEVICE_AND_API_INIT(i2c_sam0_##n,				\
			    DT_INST_LABEL(n),				\
			    &i2c_sam0_initialize,			\
			    &i2c_sam0_dev_data_##n,			\
			    &i2c_sam0_dev_config_##n, POST_KERNEL,	\
			    CONFIG_I2C_INIT_PRIORITY,			\
			    &i2c_sam0_driver_api);			\
	I2C_SAM0_IRQ_HANDLER(n)

DT_INST_FOREACH_STATUS_OKAY(I2C_SAM0_DEVICE)
