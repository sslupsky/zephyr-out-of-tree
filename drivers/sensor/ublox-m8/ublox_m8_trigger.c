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
#include "ublox_m8.h"

LOG_MODULE_DECLARE(UBLOX_M8, CONFIG_GNSS_LOG_LEVEL);

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

	if (trig->type != GNSS_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	ublox_m8_setup_int(dev, false);

	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	drv_data->data_ready_trigger = *trig;

	ublox_m8_setup_int(dev, true);

	return 0;
}

static void ublox_m8_gpio_callback(struct device *dev,
				   struct gpio_callback *cb, u32_t pins)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(cb, struct ublox_m8_data, txready_gpio_cb);

	ARG_UNUSED(pins);

	ublox_m8_setup_int(dev, false);

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->txready_gpio_sem);
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void ublox_m8_thread_cb(void *arg)
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
static void ublox_m8_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->txready_gpio_sem, K_FOREVER);
		ublox_m8_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD
static void ublox_m8_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, work);

	ublox_m8_thread_cb(drv_data->dev);
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
	if (!ret) {
		/*
		 * logical 0 indicates chip is driving pin low
		 * so chip is present.
		 * Turn off the pull-up.
		 */
		gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
				cfg->txready_gpio_flags |
				GPIO_INPUT);
	} else {
		/*
		 * logical 1 indicates pin is being pulled up my mcu
		 * so the chip is not present.
		 * Disable input.
		 */
		gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
				GPIO_DISCONNECTED);
	}

	gpio_init_callback(&drv_data->txready_gpio_cb,
			   ublox_m8_gpio_callback,
			   BIT(cfg->txready_gpio_pin));

	if (gpio_add_callback(drv_data->txready_gpio, &drv_data->txready_gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

	/* enable data ready interrupt */
	// ret = ublox_m8_set_txready(dev);
	// if (ret < 0) {
	// 	return -EIO;
	// }


#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->txready_gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_UBLOX_M8_THREAD_STACK_SIZE,
			(k_thread_entry_t)ublox_m8_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_UBLOX_M8_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&drv_data->thread, CONFIG_UBLOX_M8_THREAD_NAME);

#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = ublox_m8_work_cb;
	drv_data->dev = dev;
#endif

	ublox_m8_setup_int(dev, true);

	LOG_DBG("Data ready interrupt initialized.");

	return 0;
}
