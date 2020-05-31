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
#include <drivers/pmic.h>
#include <logging/log.h>
#include "bq24195.h"

LOG_MODULE_DECLARE(BQ24195, CONFIG_PMIC_LOG_LEVEL);

int bq24195_trigger_set(struct device *dev,
			const struct pmic_trigger *trig,
			pmic_trigger_handler_t handler)
{
	struct bq24195_data *drv_data = dev->driver_data;

	if (trig->type != PMIC_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	gpio_pin_disable_callback(drv_data->gpio, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN);

	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	drv_data->data_ready_trigger = *trig;

	gpio_pin_enable_callback(drv_data->gpio, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN);

	return 0;
}

static void bq24195_gpio_callback(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	struct bq24195_data *drv_data =
		CONTAINER_OF(cb, struct bq24195_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_disable_callback(dev, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN);

#if defined(CONFIG_BQ24195_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_BQ24195_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void bq24195_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct bq24195_data *drv_data = dev->driver_data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	gpio_pin_enable_callback(drv_data->gpio, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN);
}

#ifdef CONFIG_BQ24195_TRIGGER_OWN_THREAD
static void bq24195_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct bq24195_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		bq24195_thread_cb(dev);
	}
}
#endif

#ifdef CONFIG_BQ24195_TRIGGER_GLOBAL_THREAD
static void bq24195_work_cb(struct k_work *work)
{
	struct bq24195_data *drv_data =
		CONTAINER_OF(work, struct bq24195_data, work);

	bq24195_thread_cb(drv_data->dev);
}
#endif

int bq24195_init_interrupt(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(DT_INST_0_TI_BQ24195_IRQ_GPIOS_CONTROLLER);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    DT_INST_0_TI_BQ24195_IRQ_GPIOS_CONTROLLER);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->gpio, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN,
			   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			   GPIO_INT_ACTIVE_HIGH | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&drv_data->gpio_cb,
			   bq24195_gpio_callback,
			   BIT(DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

	/* enable data ready interrupt */
	if (i2c_reg_update_byte(drv_data->i2c, drv_data->i2c_slave_addr,
			       BQ24195_REG_INT_ENABLE_1, BQ24195_RAW_DATA_0_RDY_EN, BQ24195_RAW_DATA_0_RDY_EN) < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return -EIO;
	}

#if defined(CONFIG_BQ24195_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_BQ24195_THREAD_STACK_SIZE,
			(k_thread_entry_t)bq24195_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_BQ24195_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_BQ24195_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = bq24195_work_cb;
	drv_data->dev = dev;
#endif

	gpio_pin_enable_callback(drv_data->gpio, DT_INST_0_TI_BQ24195_IRQ_GPIOS_PIN);

	LOG_DBG("Data ready interrupt initialized.");

	return 0;
}
