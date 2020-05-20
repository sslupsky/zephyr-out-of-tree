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
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>

#include "bq24195.h"

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(BQ24195);

#ifdef CONFIGURE_PMIC
#endif

#define PMIC_STACK_SIZE		512

K_THREAD_STACK_DEFINE(pmic_stack, PMIC_STACK_SIZE);

struct k_thread pmic_thread;

static int bq24195_reg_read(struct device *dev, u8_t reg, u8_t *val, int size)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config->config_info;
	int ret;

	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr, reg, (u8_t *)val,
			     size);
	return ret;
}

static int bq24195_reg_write(struct device *dev, u8_t reg, u8_t val)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config->config_info;
	int ret;

	u8_t tx_buf[2] = { reg, val };
	ret = i2c_write(drv_data->i2c, tx_buf, sizeof(tx_buf),
			cfg->i2c_addr);
	return ret;
}

static int bq24195_reg_update(struct device *dev, u8_t reg,
			      u8_t mask, u8_t val)
{
	u8_t old_val;
	u8_t new_val;
	int ret;

	ret = bq24195_reg_read(dev, reg, &old_val, sizeof(old_val));
	if (ret < 0) {
		return ret;
	}

	new_val = old_val & ~mask;
	new_val |= val & mask;

	ret = bq24195_reg_write(dev, reg, new_val);
	return ret;
}


static int bq24195_disable_watchdog(struct device *dev)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_TERMINATION_CONTROL, 0b00110000, 0);
	return ret;
}

static int bq24195_set_input_voltage_limit(struct device *dev, u8_t voltage)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b01111000, voltage << 3);
	return ret;
}

static int bq24195_set_input_current_limit(struct device *dev, u8_t current)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b00000111, current);
	return ret;
}

static int bq24195_set_charge_current(struct device *dev, u8_t current)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_CURRENT_CONTROL, 0b11111100, current << 2);
	return ret;
}

static int bq24195_set_charge_voltage(struct device *dev, u8_t voltage)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_VOLTAGE_CONTROL, 0b11111100, voltage << 2);
	return ret;
}

static int bq24195_disable_charge(struct device *dev)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_POWERON_CONFIG, 0b00110000, 0);
	return ret;
}

static int apply_pmic_defaults(struct device *dev)
{
	int ret;

	ret = bq24195_disable_watchdog(dev);
	if (ret < 0) {
		return ret;
	}

	//disableDPDM();
	ret = bq24195_disable_charge(dev);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_input_voltage_limit(dev, INPUT_VOLTAGE_LIMIT_4360);	// default
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_input_current_limit(dev, CURRENT_LIMIT_2000);		// 2A
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_charge_current(dev, CHARGE_CURRENT_512);		// 512mA
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_charge_voltage(dev, CHARGE_VOLTAGE_4112);		// 4.112V termination voltage
	return ret;
}


static int configure_pmic(struct device *dev)
{
	int ret;

	ret = apply_pmic_defaults(dev);
	return ret;
}


static bool is_battery_present(struct device *dev)
{
	int ret;
	BQ24195_SYSTEM_STATUS_t value;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_SYSTEM_STATUS, (u8_t *)&value.reg, sizeof(value.reg));
	return (value.bit.DPM_STAT != 0);
}


static int bq24195_set_charge_mode(struct device *dev, u8_t mode)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_POWERON_CONFIG, 0b00110000, mode << 4);
	return ret;
}


/**
 *  @brief  Configure the PMIC HiZ mode
 *  @param  enable
 *          enable to disable
 *  @return value from selected register
 */
static int bq24195_set_hiz(struct device *dev, bool enable)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b10000000, enable << 7);
	return ret;
}


/**
 *  @brief  Disable the PMIC
 *  @param  none
 *  @return value from selected register
 */
static int bq24195_disable(struct device *dev)
{
	int ret;

	ret = bq24195_disable_charge(dev);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_disable_watchdog(dev);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_hiz(dev, true);
	return ret;
}


/**
 *  @brief  Reset / Tickle the PMIC watchdog timer
 *  @param  none
 *  @return false if register read failed
 */
static int bq24195_reset_watchdog(struct device *dev)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_POWERON_CONFIG, 0b01000000, 1 << 6);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_POWERON_CONFIG, 0b01000000, 1 << 6);
	return ret;
}

static int bq24195_disable_batfet(struct device *dev, bool disable) {
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_MISC_OPERATION_CONTROL, 0b00100000, disable ? 1 << 5 : 0);
	return ret;
}

static bool isPowerGood(struct device *dev) {
	int ret;
	BQ24195_SYSTEM_STATUS_t value;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_SYSTEM_STATUS, &value.reg, sizeof(value.reg));
	return (value.bit.PG_STAT == 1);
}


static int bq24195_get_system_status(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_SYSTEM_STATUS, &drv_data->pmic.reg.SystemStatus.reg, sizeof(drv_data->pmic.reg.SystemStatus.reg));
	return ret;
}

static int bq24195_get_fault(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	/*
	 * Note: To read the current fault status, you must read the fault register twice
	 * See 8.3.4.2 and 8.3.6.5.2
	 */
	ret = bq24195_reg_read(dev, BQ24195_REGISTER_FAULT, &drv_data->pmic.reg.Fault.reg, sizeof(drv_data->pmic.reg.Fault.reg));
	ret = bq24195_reg_read(dev, BQ24195_REGISTER_FAULT, &drv_data->pmic.CurrentFault.reg, sizeof(drv_data->pmic.CurrentFault.reg));
	return ret;
}

static int bq24195_get_registers(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;
	// u8_t value;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, drv_data->pmic.raw, sizeof(drv_data->pmic.raw));
	if (ret < 0) {
		LOG_WRN("could not read registers");
	}

	/* read the fault register again to obtain the current fault status */
	ret = bq24195_get_fault(dev);
	if (ret < 0) {
		LOG_WRN("could not read registers");
	}

	return ret;
}

static void pmic_process(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	enum pmic_state pmic_state;
	enum pmic_state next_state;
	BQ24195_SYSTEM_STATUS_t status;
	int ret;

	LOG_DBG("starting");
	pmic_state = PMIC_STATE_start;
	next_state = pmic_state;

	while (true) {
		ret = bq24195_get_system_status(dev);
		if (ret < 0) {
			LOG_DBG("could not read pmic register");
		}

		switch (pmic_state) {
		case PMIC_STATE_start:
			if (drv_data->pmic.reg.SystemStatus.bit.PG_STAT) {
				next_state = PMIC_STATE_connected;
			} else {
				next_state = PMIC_STATE_disconnected;
			}
			break;

		case PMIC_STATE_idle:
			bq24195_reset_watchdog(dev);
			if (drv_data->pmic.reg.SystemStatus.bit.PG_STAT) {
				next_state = PMIC_STATE_connected;
			}
			break;

		case PMIC_STATE_connected:
			LOG_INF("connect");
			if (drv_data->pmic.reg.SystemStatus.bit.VSYS_STAT) {
				LOG_DBG("battery not present");
				bq24195_set_charge_mode(dev, 0);
				next_state = PMIC_STATE_batteryNotPresent;
			} else {
				LOG_DBG("charging");
				bq24195_set_charge_mode(dev, 1);
				next_state = PMIC_STATE_charging;
			}
			break;

		case PMIC_STATE_disconnected:
			LOG_INF("disconnect");
			bq24195_set_charge_mode(dev, 0);
			next_state = PMIC_STATE_idle;
			break;

		case PMIC_STATE_charging:
			bq24195_reset_watchdog(dev);
			if (drv_data->pmic.reg.SystemStatus.bit.VSYS_STAT) {
				LOG_DBG("battery not present");
				bq24195_set_charge_mode(dev, 0);
				next_state = PMIC_STATE_batteryNotPresent;
			}
			if (!drv_data->pmic.reg.SystemStatus.bit.PG_STAT) {
				next_state = PMIC_STATE_disconnected;
			}
			break;

		case PMIC_STATE_batteryNotPresent:
			bq24195_reset_watchdog(dev);
			if (!drv_data->pmic.reg.SystemStatus.bit.VSYS_STAT) {
				LOG_DBG("charging");
				bq24195_set_charge_mode(dev, 1);
				next_state = PMIC_STATE_charging;
			}
			if (!drv_data->pmic.reg.SystemStatus.bit.PG_STAT) {
				next_state = PMIC_STATE_disconnected;
			}
			break;

		case PMIC_STATE_deviceNotPresent:
		default:
			break;
		}
		if (next_state != pmic_state) {
			/* TODO: callback */
			pmic_state = next_state;
		} else {
			k_sleep(K_SECONDS(30));
		}
	}
}

/**
 * @brief Check the Device ID
 *
 * @param[in]   dev     Pointer to the device structure
 *
 * @retval 0 On success
 * @retval -EIO Otherwise
 */
static inline int bq24195_device_id_check(struct device *dev)
{
	BQ24195_VENDOR_STATUS_t value;
	int ret;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_PMIC_VENDOR, &value.reg, sizeof(value.reg));
	if (ret < 0) {
		LOG_ERR("%s: Failed to get Device ID register",
			DT_INST_0_TI_BQ24195_LABEL);
		return ret;
	}

	if (value.reg != BQ24195_CHIPID) {
		LOG_ERR("%s: Failed to match the device IDs",
			DT_INST_0_TI_BQ24195_LABEL);
		return -EINVAL;
	}

	return 0;
}

static int bq24195_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bq24195_data *drv_data = dev->driver_data;
	BQ24195_SYSTEM_STATUS_t status;
	BQ24195_FAULT_t fault;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_PMIC_STATUS ||
			chan == SENSOR_CHAN_PMIC_FAULT);

	switch ((int) chan) {
	case SENSOR_CHAN_ALL:
		ret = bq24195_get_registers(dev);
		LOG_HEXDUMP_DBG(drv_data->pmic.raw, sizeof(drv_data->pmic.raw), "pmic registers");
		break;

	case SENSOR_CHAN_PMIC_STATUS:
		ret = bq24195_get_system_status(dev);
		break;

	case SENSOR_CHAN_PMIC_FAULT:
		ret = bq24195_get_fault(dev);
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_channel_get(struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret = 0;

	switch ((int) chan) {
	case SENSOR_CHAN_ALL:
		LOG_HEXDUMP_DBG(drv_data->pmic.raw, sizeof(drv_data->pmic.raw), "pmic registers");
		break;

	case SENSOR_CHAN_PMIC_STATUS:
		val->val1 = drv_data->pmic.reg.SystemStatus.reg;
		break;

	case SENSOR_CHAN_PMIC_FAULT:
		val->val1 = drv_data->pmic.reg.Fault.reg;
		val->val2 = drv_data->pmic.CurrentFault.reg;
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_attr_set(struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr,
			     const struct sensor_value *val)
{
	// struct bq24195_data *drv_data = dev->driver_data;
	int ret = 0;

	switch ((int) attr) {
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static const struct sensor_driver_api bq24195_driver_api = {
	.attr_set = bq24195_attr_set,
	.sample_fetch = bq24195_sample_fetch,
	.channel_get = bq24195_channel_get,
	.trigger_set = NULL,
};

static int bq24195_chip_init(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	while (k_uptime_ticks() < k_us_to_cyc_ceil32(BQ24195_STARTUP_TIME_USEC)) {
		/* wait for chip to power up */
	}

	drv_data->i2c = device_get_binding(DT_INST_0_TI_BQ24195_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_INST_0_TI_BQ24195_BUS_NAME);
		return -EINVAL;
	}

	ret = bq24195_device_id_check(dev);
	if (ret < 0) {
		return -EIO;
	}

	LOG_DBG("BQ24195 pmic detected");

	apply_pmic_defaults(dev);
	bq24195_get_registers(dev);

	return 0;
}

int bq24195_init(struct device *dev)
{
	k_tid_t thread;
	int ret;

	ret = bq24195_chip_init(dev);
	if (ret == 0) {
		thread = k_thread_create(&pmic_thread, pmic_stack,
				K_THREAD_STACK_SIZEOF(pmic_stack),
				(k_thread_entry_t) pmic_process,
				dev, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

		k_thread_name_set(thread, "pmic");
	}

	return ret;
}

static struct bq24195_data bq24195_drv_data;

static const struct bq24195_dev_config bq24195_config = {
	.i2c_addr = DT_INST_0_TI_BQ24195_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(bq24195, DT_INST_0_TI_BQ24195_LABEL, bq24195_init,
		    &bq24195_drv_data, &bq24195_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &bq24195_driver_api);
