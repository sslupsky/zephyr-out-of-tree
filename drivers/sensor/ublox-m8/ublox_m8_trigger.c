/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-20
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
#include <sys/util.h>
#include <kernel.h>
#include <drivers/gnss.h>
#include <logging/log.h>
#include <devicetree.h>
#include <sys/byteorder.h>

#include "ublox_m8.h"

LOG_MODULE_DECLARE(UBLOX_M8, CONFIG_GNSS_LOG_LEVEL);

const struct gnss_trigger txready_trigger = {
	.type = GNSS_TRIG_DATA_READY,
	.chan = GNSS_CHAN_ALL,
};

const struct gnss_trigger poll_trigger = {
	.type = GNSS_TRIG_TIMER,
	.chan = GNSS_CHAN_ALL,
};

/**
 * @brief 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_message_get(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	u16_t msg_len;
	size_t bytes_read, bytes_put;

	k_mutex_lock(&drv_data->msg_get_mtx, K_FOREVER);

	/* get the message length */
	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			     UBLOX_M8_REGISTER_LEN, drv_data->rx_buf, 2);
	if (ret < 0) {
		LOG_DBG("addr error");
		goto done;
	}
	/* ddc port message length is received big endian */
	msg_len = sys_get_be16(drv_data->rx_buf);
	if (msg_len == 0) {
		ret = -EAGAIN;
		goto done;
	}

	if (msg_len > 4095) {
		LOG_WRN("large msg buffer");
	}

	/* read the first byte */
	ret = i2c_read(drv_data->i2c, drv_data->rx_buf, 1, cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("read error");
		ret = -EIO;
		goto done;
	}
	if (drv_data->rx_buf[0] == 0xFF) {
		LOG_DBG("buffer not ready (0xFF)");
		ret = -EAGAIN;
		goto done;
	}

	ret = k_pipe_put(&drv_data->rx_pipe, drv_data->rx_buf, 1, &bytes_put, 1, drv_data->timeout);
	if (ret < 0) {
		if (ret == -EAGAIN) {
			LOG_DBG("pipe timeout");
		} else {
			LOG_DBG("pipe error");
		}
		goto done;
	}

	msg_len--;
	while (msg_len > 0) {
		bytes_read = MIN(msg_len, sizeof(drv_data->rx_buf));
		ret = i2c_read(drv_data->i2c, drv_data->rx_buf, bytes_read, cfg->i2c_addr);
		if (ret < 0) {
			LOG_DBG("read error");
			break;
		}
		ret = k_pipe_put(&drv_data->rx_pipe, drv_data->rx_buf, bytes_read, &bytes_put, bytes_read, drv_data->timeout);
		if (ret < 0) {
			if (ret == -EAGAIN) {
				LOG_DBG("pipe timeout");
			} else {
				LOG_DBG("pipe error");
			}
			break;;
		}
		msg_len -= bytes_read;
	}
done:
	k_mutex_unlock(&drv_data->msg_get_mtx);
	return ret;
}

static void ublox_m8_txready_handler(struct device *dev, struct gnss_trigger *trigger)
{
	int ret;

	ret = ublox_m8_message_get(dev);
	if (ret < 0) {
	}
	return;
}

static void ublox_m8_poll_handler(struct device *dev, struct gnss_trigger *trigger)
{
	int ret;

	ret = ublox_m8_message_get(dev);
	if (ret < 0) {
	}
	return;
}

static inline void ublox_m8_setup_int(struct device *dev,
			     bool enable)
{
	struct ublox_m8_data *data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	unsigned int flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure(data->txready_gpio,
				     cfg->txready_gpio_pin,
				     flags);
}

int ublox_m8_trigger_set(struct device *dev,
			const struct gnss_trigger *trig,
			gnss_trigger_handler_t handler)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	switch (trig->type) {
	case GNSS_TRIG_DATA_READY:
		ublox_m8_setup_int(dev, false);
		drv_data->data_ready_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->data_ready_trigger = *trig;
		ublox_m8_setup_int(dev, true);
		break;
	
	case GNSS_TRIG_TIMER:
		drv_data->poll_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->poll_trigger = *trig;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static void ublox_m8_gpio_callback(struct device *dev,
				   struct gpio_callback *cb, u32_t pins)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(cb, struct ublox_m8_data, txready_gpio_cb);

	ARG_UNUSED(pins);

	ublox_m8_setup_int(drv_data->dev, false);

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->trigger_gpio_sem);
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->trigger_work);
#endif
}

static void ublox_m8_trigger_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct ublox_m8_data *drv_data = dev->driver_data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	ublox_m8_setup_int(dev, true);
}

#ifdef CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD
static void ublox_m8_trigger_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	LOG_DBG("trigger thread started");

	while (1) {
		k_sem_take(&drv_data->trigger_gpio_sem, K_FOREVER);
		ublox_m8_trigger_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD
static void ublox_m8_trigger_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, trigger_work);

	ublox_m8_trigger_thread_cb(drv_data->dev);
}
#endif

int ublox_m8_init_interrupt(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	/* setup data ready gpio interrupt */
	drv_data->txready_gpio = device_get_binding(cfg->txready_gpio_name);
	if (drv_data->txready_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->txready_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
			   cfg->txready_gpio_flags |
			   GPIO_INPUT | GPIO_PULL_UP);

	/* read the state of the pin */
	ret = gpio_pin_get(drv_data->txready_gpio, cfg->txready_gpio_pin);
	LOG_INF("txReady: %d", ret);
	// if (!ret) {
	// 	/*
	// 	 * logical 0 indicates chip is driving pin low
	// 	 * so chip is present.
	// 	 * Turn off the pull-up.
	// 	 */
	// 	gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
	// 			cfg->txready_gpio_flags |
	// 			GPIO_INPUT);
	// } else {
	// 	/*
	// 	 * logical 1 indicates pin is being pulled up my mcu
	// 	 * so the chip is not present.
	// 	 * Disable input.
	// 	 */
	// 	gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
	// 			GPIO_DISCONNECTED);
	// }

	gpio_init_callback(&drv_data->txready_gpio_cb,
			   ublox_m8_gpio_callback,
			   BIT(cfg->txready_gpio_pin));

	if (gpio_add_callback(drv_data->txready_gpio, &drv_data->txready_gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->trigger_gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->trigger_thread, drv_data->trigger_thread_stack,
			CONFIG_UBLOX_M8_TRIGGER_THREAD_STACK_SIZE,
			(k_thread_entry_t)ublox_m8_trigger_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_UBLOX_M8_TRIGGER_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&drv_data->trigger_thread, CONFIG_UBLOX_M8_TRIGGER_THREAD_NAME);

#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	drv_data->trigger_work.handler = ublox_m8_trigger_work_cb;
	drv_data->dev = dev;
#endif

	// ublox_m8_trigger_set(dev, &txready_trigger, ublox_m8_txready_handler);
	ublox_m8_trigger_set(dev, &poll_trigger, ublox_m8_poll_handler);

	LOG_DBG("Data ready interrupt initialized.");

	return 0;
}
