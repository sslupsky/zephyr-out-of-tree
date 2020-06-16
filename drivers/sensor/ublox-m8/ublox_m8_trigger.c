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

static inline void ublox_m8_setup_txready_int(struct device *dev,
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

static inline void ublox_m8_setup_timepulse_int(struct device *dev,
			     bool enable)
{
	struct ublox_m8_data *data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	unsigned int flags = enable
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure(data->timepulse_gpio,
				     cfg->timepulse_gpio_pin,
				     flags);
}

int ublox_m8_trigger_set(struct device *dev,
			const struct gnss_trigger *trig,
			gnss_trigger_handler_t handler)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	switch (trig->type) {
	case GNSS_TRIG_DATA_READY:
		ublox_m8_setup_txready_int(dev, false);
		drv_data->txready_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->txready_trigger = *trig;
		ublox_m8_setup_txready_int(dev, true);
		break;
	
	case GNSS_TRIG_POLL:
		drv_data->poll_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->poll_trigger = *trig;
		break;

	case GNSS_TRIG_PVT:
		drv_data->pvt_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->pvt_trigger = *trig;
		break;

	case GNSS_TRIG_TIMEPULSE:
		ublox_m8_setup_timepulse_int(dev, false);
		drv_data->timepulse_handler = handler;
		if (handler == NULL) {
			return 0;
		}
		drv_data->timepulse_trigger = *trig;
		ublox_m8_setup_timepulse_int(dev, false);
		break;

	case GNSS_TRIG_GEOFENCE:
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void ublox_m8_txready_gpio_callback(struct device *dev,
				   struct gpio_callback *cb, u32_t pins)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(cb, struct ublox_m8_data, txready_gpio_cb);

	ARG_UNUSED(pins);

	// ublox_m8_setup_txready_int(drv_data->dev, false);

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	// k_sem_give(&drv_data->txready_gpio_sem);
	k_poll_signal_raise(&drv_data->txready_signal, 0);
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->trigger_txready_work);
#endif
}

static void ublox_m8_timepulse_gpio_callback(struct device *dev,
				   struct gpio_callback *cb, u32_t pins)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(cb, struct ublox_m8_data, timepulse_gpio_cb);

	ARG_UNUSED(pins);

	// ublox_m8_setup_timepulse_int(drv_data->dev, false);

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	// k_sem_give(&drv_data->timepulse_gpio_sem);
	k_poll_signal_raise(&drv_data->timepulse_signal, 0);
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->trigger_timepulse_work);
#endif
}

static void ublox_m8_trigger_txready_cb(void *arg)
{
	struct device *dev = arg;
	struct ublox_m8_data *drv_data = dev->driver_data;

	k_sem_give(&drv_data->msg_sem);
	if (drv_data->txready_handler != NULL) {
		drv_data->txready_handler(dev, &drv_data->txready_trigger);
	}

	// ublox_m8_setup_txready_int(dev, true);
}

static void ublox_m8_trigger_timepulse_cb(void *arg)
{
	struct device *dev = arg;
	struct ublox_m8_data *drv_data = dev->driver_data;

	if (drv_data->timepulse_handler != NULL) {
		drv_data->timepulse_handler(dev, &drv_data->timepulse_trigger);
	}

	// ublox_m8_setup_timepulse_int(dev, true);
}

static void ublox_m8_trigger_pvt_cb(void *arg, int timeout)
{
	struct device *dev = arg;
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	s64_t t;

	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
	// drv_data->pvt_ready = false;

	if (!timeout) {
		timeout = UBLOX_M8_PVT_TIMEOUT_MSEC;
	}

	t = k_uptime_get();
	do {
		k_sleep(K_MSEC(500));
		k_sem_give(&drv_data->msg_sem);
		k_yield();
		/* check if fix ok and psm state is tracking */
		if (drv_data->pvt_ready) {
			break;
		}
	} while (k_uptime_get() - t < timeout);

	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);

	if (drv_data->pvt_handler != NULL) {
		drv_data->pvt_handler(dev, &drv_data->pvt_trigger);
	}
}

#ifdef CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD
static void ublox_m8_trigger_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	LOG_DBG("trigger thread started");

	while (1) {
		// k_sem_take(&drv_data->txready_gpio_sem, K_FOREVER);
		k_poll(drv_data->events, 3, K_FOREVER);
		if (drv_data->events[0].state == K_POLL_STATE_SIGNALED) {
			ublox_m8_trigger_txready_cb(dev);
			drv_data->events[0].signal->signaled = 0;
			drv_data->events[0].state = K_POLL_STATE_NOT_READY;
		}
		if (drv_data->events[1].state == K_POLL_STATE_SIGNALED) {
			ublox_m8_trigger_timepulse_cb(dev);
			drv_data->events[1].signal->signaled = 0;
			drv_data->events[1].state = K_POLL_STATE_NOT_READY;
		}
		if (drv_data->events[2].state == K_POLL_STATE_SIGNALED) {
			ublox_m8_trigger_pvt_cb(dev, drv_data->events[2].signal->result);
			drv_data->events[2].signal->signaled = 0;
			drv_data->events[2].state = K_POLL_STATE_NOT_READY;
		}
	}
}
#endif

#ifdef CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD
static void ublox_m8_trigger_txready_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, trigger_txready_work);

	ublox_m8_trigger_txready_cb(drv_data->dev);
}

static void ublox_m8_trigger_timepulse_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, trigger_timepulse_work);

	ublox_m8_trigger_timepulse_cb(drv_data->dev);
}
#endif

int ublox_m8_init_interrupt(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	/* setup data txReady gpio interrupt */
	drv_data->txready_gpio = device_get_binding(cfg->txready_gpio_name);
	if (drv_data->txready_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->txready_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->txready_gpio, cfg->txready_gpio_pin,
			   cfg->txready_gpio_flags |
			   GPIO_INPUT);

	/* read the state of the pin */
	ret = gpio_pin_get(drv_data->txready_gpio, cfg->txready_gpio_pin);
	LOG_DBG("txReady: %d", ret);
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
			   ublox_m8_txready_gpio_callback,
			   BIT(cfg->txready_gpio_pin));

	if (gpio_add_callback(drv_data->txready_gpio, &drv_data->txready_gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

	/* setup data timepulse gpio interrupt */
	drv_data->timepulse_gpio = device_get_binding(cfg->timepulse_gpio_name);
	if (drv_data->timepulse_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->timepulse_gpio_name);
		return -EINVAL;
	}

	/* configure pull-up to check if module is connected */
	gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
			   cfg->timepulse_gpio_flags |
			   GPIO_INPUT | GPIO_PULL_UP);

	/* read the state of the pin */
	ret = gpio_pin_get(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin);
	LOG_DBG("timepulse: %d", ret);
	if (!ret) {
		/*
		 * logical 0 indicates chip is driving pin low
		 * so chip is present.
		 * Turn off the pull-up.
		 */
		gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
				cfg->timepulse_gpio_flags |
				GPIO_INPUT);
	} else {
		/*
		 * logical 1 indicates pin is being pulled up my mcu
		 * so the chip is not present.
		 * Disable input.
		 */
		gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
				GPIO_DISCONNECTED);
	}

	gpio_init_callback(&drv_data->timepulse_gpio_cb,
			   ublox_m8_timepulse_gpio_callback,
			   BIT(cfg->timepulse_gpio_pin));

	if (gpio_add_callback(drv_data->timepulse_gpio, &drv_data->timepulse_gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->txready_gpio_sem, 0, UINT_MAX);

	k_poll_signal_init(&drv_data->txready_signal);
	k_poll_signal_init(&drv_data->timepulse_signal);
	k_poll_signal_init(&drv_data->pvt_signal);

	k_poll_event_init(&drv_data->events[0],
			  K_POLL_TYPE_SIGNAL,
			  K_POLL_MODE_NOTIFY_ONLY,
			  &drv_data->txready_signal);
	k_poll_event_init(&drv_data->events[1],
			  K_POLL_TYPE_SIGNAL,
			  K_POLL_MODE_NOTIFY_ONLY,
			  &drv_data->timepulse_signal);
	k_poll_event_init(&drv_data->events[2],
			  K_POLL_TYPE_SIGNAL,
			  K_POLL_MODE_NOTIFY_ONLY,
			  &drv_data->pvt_signal);

	k_thread_create(&drv_data->trigger_thread, drv_data->trigger_thread_stack,
			CONFIG_UBLOX_M8_TRIGGER_THREAD_STACK_SIZE,
			(k_thread_entry_t)ublox_m8_trigger_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_UBLOX_M8_TRIGGER_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&drv_data->trigger_thread, CONFIG_UBLOX_M8_TRIGGER_THREAD_NAME);

#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	drv_data->trigger_txready_work.handler = ublox_m8_trigger_txready_work_cb;
	drv_data->trigger_timepulse_work.handler = ublox_m8_trigger_timepulse_work_cb;
	drv_data->dev = dev;
#endif

	LOG_DBG("Data ready interrupt initialized.");

	return 0;
}
