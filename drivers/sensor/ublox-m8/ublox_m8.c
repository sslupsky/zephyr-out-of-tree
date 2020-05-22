/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gnss.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>
#include <devicetree.h>

#include "ublox_m8.h"

#define LOG_LEVEL CONFIG_GNSS_LOG_LEVEL
LOG_MODULE_REGISTER(UBLOX_M8);


//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void ubx_checksum(struct ubx_packet *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (u16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

__attribute__((unused))
static int ublox_m8_reg_read(struct device *dev, u8_t reg, u8_t *val, int size)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr, reg, (u8_t *)val,
			     size);
	return ret;
}

__attribute__((unused))
static int ublox_m8_message_read(struct device *dev, u8_t *val, int size)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	u8_t rx_buf[2];
	u16_t msg_len;

	/* get the message length */
	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			     UBLOX_M8_REGISTER_LEN, rx_buf, 2);
	if (ret < 0) {
		LOG_DBG("addr error");
		goto done;
	}
	msg_len = sys_get_be16(rx_buf);

	/* read the first byte */
	ret = i2c_read(drv_data->i2c, rx_buf, 1, cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("read error");
		goto done;
	}

	if (rx_buf[0] != 0xFF) {
		/*
		 * The first byte is valid.
		 * So store it in the return buffer and read
		 * the rest of the message
		 */
		*val++ = rx_buf[0];
		ret = i2c_read(drv_data->i2c, val, msg_len-1, cfg->i2c_addr);
		if (ret < 0) {
			LOG_DBG("read error");
			goto done;
		}
	} else {
		LOG_DBG("no data available");
	}
done:
	return ret;
}

__attribute__((unused))
static int ublox_m8_message_write(struct device *dev, u8_t *buf, int size)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	ret = i2c_write(drv_data->i2c, buf, size,
			cfg->i2c_addr);
	return ret;
}

/**
 * @brief Check the Device ID
 *
 * @param[in]   dev     Pointer to the device structure
 *
 * @retval 0 On success
 * @retval -EIO Otherwise
 */
static inline int ublox_m8_device_id_check(struct device *dev)
{
	u8_t value;
	int ret;

	ret = 0;
	value = 0;
	// ret = ublox_m8_reg_read(dev, UBLOX_M8_REGISTER_PMIC_VENDOR, &value, sizeof(value));
	if (ret < 0) {
		LOG_ERR("%s: Failed to get Device ID register",
			DT_INST_0_UBLOX_M8_LABEL);
		return ret;
	}

	if (value != UBLOX_M8_CHIPID) {
		LOG_ERR("%s: Failed to match the device IDs",
			DT_INST_0_UBLOX_M8_LABEL);
		return -EINVAL;
	}

	return 0;
}

static int ublox_m8_sample_fetch(struct device *dev, enum gnss_channel chan)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	ARG_UNUSED(drv_data);

	__ASSERT_NO_MSG(chan == GNSS_CHAN_ALL ||
			chan == GNSS_CHAN_TIME ||
			chan == GNSS_CHAN_POSITION ||
			chan == GNSS_CHAN_VELOCITY);

	switch (chan) {
	case GNSS_CHAN_ALL:
		break;

	case GNSS_CHAN_TIME:
		break;

	case GNSS_CHAN_POSITION:
		break;

	case GNSS_CHAN_VELOCITY:
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_channel_get(struct device *dev, enum gnss_channel chan,
			      struct gnss_pvt *val)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	ARG_UNUSED(drv_data);

	switch ((int) chan) {
	case GNSS_CHAN_ALL:
		break;

	case GNSS_CHAN_TIME:
		break;

	case GNSS_CHAN_POSITION:
		break;

	case GNSS_CHAN_VELOCITY:
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_attr_set(struct device *dev, enum gnss_channel chan,
			     enum gnss_attribute attr,
			     const struct gnss_pvt *val)
{
	// struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	switch ((int) attr) {
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static const struct gnss_driver_api ublox_m8_driver_api = {
	.attr_set = ublox_m8_attr_set,
	.sample_fetch = ublox_m8_sample_fetch,
	.channel_get = ublox_m8_channel_get,
#if CONFIG_UBLOX_M8_TRIGGER
	.trigger_set = ublox_m8_trigger_set,
#endif
};

static int ublox_m8_chip_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	while (k_uptime_ticks() < k_us_to_cyc_ceil32(UBLOX_M8_STARTUP_TIME_USEC)) {
		/* wait for chip to power up */
	}

	drv_data->i2c = device_get_binding(DT_INST_0_UBLOX_M8_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_INST_0_UBLOX_M8_BUS_NAME);
		return -EINVAL;
	}

	ret = ublox_m8_device_id_check(dev);
	if (ret < 0) {
		return -EIO;
	}

	LOG_DBG("module detected");

	/* setup extint gpio */
	drv_data->extint_gpio = device_get_binding(cfg->extint_gpio_name);
	if (drv_data->extint_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->extint_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->extint_gpio, cfg->extint_gpio_pin,
			   cfg->extint_gpio_flags |
			   GPIO_OUTPUT);

	/* setup reset gpio */
	drv_data->reset_gpio = device_get_binding(cfg->reset_gpio_name);
	if (drv_data->reset_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->reset_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->reset_gpio, cfg->reset_gpio_pin,
			   cfg->reset_gpio_flags |
			   GPIO_OUTPUT);

	/* setup safeboot gpio */
	drv_data->safeboot_gpio = device_get_binding(cfg->safeboot_gpio_name);
	if (drv_data->safeboot_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->safeboot_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->safeboot_gpio, cfg->safeboot_gpio_pin,
			   cfg->safeboot_gpio_flags |
			   GPIO_OUTPUT);

	/* setup rxd gpio */
	drv_data->rxd_gpio = device_get_binding(cfg->rxd_gpio_name);
	if (drv_data->rxd_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->rxd_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->rxd_gpio, cfg->rxd_gpio_pin,
			   cfg->rxd_gpio_flags |
			   GPIO_PULL_UP);

	/* setup txd gpio */
	// drv_data->txd_gpio = device_get_binding(cfg->txd_gpio_name);
	// if (drv_data->txd_gpio == NULL) {
	// 	LOG_ERR("Failed to get pointer to %s device",
	// 		    cfg->txd_gpio_name);
	// 	return -EINVAL;
	// }

	// gpio_pin_configure(drv_data->txd_gpio, cfg->txd_gpio_pin,
	// 		   cfg->txd_gpio_flags |
	// 		   GPIO_OUTPUT);

	// ublox_m8_apply_defaults(dev);
	// ublox_m8_get_registers(dev);

	return 0;
}

int ublox_m8_init(struct device *dev)
{
	int ret;

	ret = ublox_m8_chip_init(dev);
	if (ret == 0) {
#ifdef CONFIG_UBLOX_M8_TRIGGER
		ret = ublox_m8_init_interrupt(dev);
		if (ret < 0) {
			LOG_DBG("Failed to initialize interrupts.");
		}
#endif
	}

	return ret;
}

static struct ublox_m8_data ublox_m8_drv_data;

static const struct ublox_m8_dev_config ublox_m8_config = {
	.i2c_name = DT_INST_0_UBLOX_M8_BUS_NAME,
	.i2c_addr = DT_INST_0_UBLOX_M8_BASE_ADDRESS,
	.txready_gpio_name = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_CONTROLLER,
	.txready_gpio_pin = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_PIN,
	.txready_gpio_flags = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_FLAGS,
	.reset_gpio_name = DT_INST_0_UBLOX_M8_RESET_GPIOS_CONTROLLER,
	.reset_gpio_pin = DT_INST_0_UBLOX_M8_RESET_GPIOS_PIN,
	.reset_gpio_flags = DT_INST_0_UBLOX_M8_RESET_GPIOS_FLAGS,
	.timepulse_gpio_name = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_CONTROLLER,
	.timepulse_gpio_pin = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_PIN,
	.timepulse_gpio_flags = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_FLAGS,
	.safeboot_gpio_name = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_CONTROLLER,
	.safeboot_gpio_pin = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_PIN,
	.safeboot_gpio_flags = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_FLAGS,
	.extint_gpio_name = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_CONTROLLER,
	.extint_gpio_pin = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_PIN,
	.extint_gpio_flags = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_FLAGS,
};

DEVICE_AND_API_INIT(ublox_m8, DT_INST_0_UBLOX_M8_LABEL, ublox_m8_init,
		    &ublox_m8_drv_data, &ublox_m8_config, POST_KERNEL,
		    CONFIG_GNSS_INIT_PRIORITY, &ublox_m8_driver_api);
