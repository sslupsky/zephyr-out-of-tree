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

#define DT_DRV_COMPAT ti_bq24195

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>

#include <drivers/pmic.h>
#include "bq24195.h"

#define LOG_LEVEL CONFIG_BQ24195_LOG_LEVEL
LOG_MODULE_REGISTER(BQ24195);

#ifdef CONFIGURE_PMIC
#endif

#define PMIC_STACK_SIZE		512

K_THREAD_STACK_DEFINE(pmic_stack, PMIC_STACK_SIZE);

struct k_thread pmic_thread;

#define DEFAULT_PMIC_WAIT				1000

#define DEFAULT_CHARGE_CYCLE_COUNT_THRESHOLD		3
#define DEFAULT_CHARGE_DISABLED_TIMEOUT			120000


static int bq24195_reg_read(struct device *dev, u8_t reg, u8_t *val, int size)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config_info;
	int ret;

	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr, reg, (u8_t *)val,
			     size);
	return ret;
}

static int bq24195_reg_write(struct device *dev, u8_t reg, u8_t val)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config_info;
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

u8_t mapInputCurrentLimit(u16_t *value)
{
	const u16_t inputCurrentLimits[] = {
		100, 150, 500, 900, 1200, 1500, 2000, 3000
	};
	u16_t v;
	u8_t idx = 0;
	u16_t prev = inputCurrentLimits[0];

	// Find closest matching current input limit value <= 'value'
	for(u8_t i = 0; i < ARRAY_SIZE(inputCurrentLimits); i++) {
		v = inputCurrentLimits[i];
		if (v > *value) {
			*value = prev;
			return idx;
		}
		if (i) {
			idx++;
		}
		prev = v;
	}

	*value = prev;
	return idx;
}

u8_t mapInputVoltageLimit(u16_t *value)
{
	u16_t baseValue = 3880;
	u8_t idx;

	// Find closest matching voltage input limit value within [3880, 5080] >= 'value'
	volatile u16_t v = MIN(MAX(*value, (u16_t)baseValue), (u16_t)5080) - baseValue;
	v /= 80;
	idx = v;
	v *= 80;
	v += baseValue;
	*value = v;
	return idx;
}

u8_t mapChargeCurrent(u16_t *value)
{
	u16_t baseValue = 512;
	u8_t idx;

	// Find closest matching current value within [512, 4544] >= 'value'
	volatile u16_t v = MIN(MAX(*value, (u16_t)baseValue), (u16_t)4544) - baseValue;
	v = v >> 6;
	idx = v;
	v = v << 6;
	v += baseValue;
	*value = v;
	return idx;
}

u8_t mapChargeVoltage(u16_t *value)
{
	u16_t baseValue = 3504;
	u8_t idx;

	// Find closest matching current value within [3504, 4400] >= 'value'
	volatile u16_t v = MIN(MAX(*value, (u16_t)baseValue), (u16_t)4400) - baseValue;
	v = v >> 4;
	idx = v;
	v = v << 4;
	v += baseValue;
	*value = v;
	return idx;
}

static int bq24195_set_input_voltage_limit_mV(struct device *dev, u16_t voltage)
{
	int ret;
	u8_t val;

	val = mapInputVoltageLimit(&voltage);
	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b01111000, val << 3);
	return ret;
}

static int bq24195_set_input_voltage_limit(struct device *dev, enum bq24195_input_voltage_limit voltage)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b01111000, voltage << 3);
	return ret;
}

static int bq24195_set_input_current_limit_mA(struct device *dev, u16_t current)
{
	int ret;
	u8_t val;

	val = mapInputCurrentLimit(&current);
	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b00000111, val);
	return ret;
}

static int bq24195_set_input_current_limit(struct device *dev, enum bq24195_input_current_limit current)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b00000111, current);
	return ret;
}

static int bq24195_set_charge_current_mA(struct device *dev, u16_t current)
{
	int ret;
	u8_t val;

	val = mapChargeCurrent(&current);
	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_CURRENT_CONTROL, 0b11111100, val << 2);
	return ret;
}

static int bq24195_set_charge_current(struct device *dev, enum bq24195_charge_current current)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_CURRENT_CONTROL, 0b11111100, current << 2);
	return ret;
}

static int bq24195_set_charge_voltage_mV(struct device *dev, u16_t voltage)
{
	int ret;
	u8_t val;

	val = mapChargeVoltage(&voltage);
	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_VOLTAGE_CONTROL, 0b11111100, val << 2);
	return ret;
}

static int bq24195_set_charge_voltage(struct device *dev, enum bq24195_charge_voltage voltage)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_CHARGE_VOLTAGE_CONTROL, 0b11111100, voltage << 2);
	return ret;
}

static int bq24195_set_charge_mode(struct device *dev, u8_t mode)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_POWERON_CONFIG, 0b00110000, mode << 4);
	return ret;
}

static int bq24195_get_system_status(struct device *dev, BQ24195_SYSTEM_STATUS_t *status)
{
	int ret;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_SYSTEM_STATUS, (u8_t *)status, sizeof(BQ24195_SYSTEM_STATUS_t));
	return ret;
}

static int bq24195_get_fault(struct device *dev, BQ24195_FAULT_DATA_t *fault)
{
	int ret;

	/*
	 * Note: To read the current fault status, you must read the fault register twice
	 * See 8.3.4.2 and 8.3.6.5.2
	 */
	/* FIXME: probably need a mutex here so both reads cannot be interrupted */
	ret = bq24195_reg_read(dev, BQ24195_REGISTER_FAULT, (u8_t *)&fault->latched_fault, sizeof(BQ24195_FAULT_t));
	ret = bq24195_reg_read(dev, BQ24195_REGISTER_FAULT, (u8_t *)&fault->fault, sizeof(BQ24195_FAULT_t));
	return ret;
}

static int poll_connect_status(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	BQ24195_SYSTEM_STATUS_t status;
	BQ24195_FAULT_DATA_t fault;
	int charge_count = 0;
	int state = 0;
	int next_state = 0;
	s64_t t;
	u32_t dt;
	int ret;

	ret = bq24195_get_fault(dev, &fault);
	if (ret < 0) {
		LOG_DBG("could not read pmic register");
		return -EIO;
	}

	t = k_uptime_get();
	do {
		if (charge_count) {
			k_sleep(K_MSEC(40));
		}
		ret = bq24195_get_system_status(dev, &status);
		if (ret < 0) {
			LOG_DBG("could not read pmic register");
			return -EIO;
		}
		/* check the charge status */
		switch (status.bit.CHRG_STAT) {
		case BQ24195_CHARGE_STATUS_NOT_CHARGING:
		case BQ24195_CHARGE_STATUS_CHARGE_DONE:
			next_state = 0;
			break;
		case BQ24195_CHARGE_STATUS_PRECHARGE:
		case BQ24195_CHARGE_STATUS_FAST_CHARGE:
			next_state = 1;
			break;
		}
		switch (state) {
		case 0:
			charge_count += next_state;
			break;
		case 1:
			break;
		}
		state = next_state;
		dt = k_uptime_get() - t;
	} while ( dt < 500 && charge_count < DEFAULT_CHARGE_CYCLE_COUNT_THRESHOLD);
	drv_data->battery_disconnected = charge_count < DEFAULT_CHARGE_CYCLE_COUNT_THRESHOLD ? 0 : 1;
	return drv_data->battery_disconnected;
}

/**
 *  @brief  Configure the PMIC HiZ mode
 *  @param  enable
 *          enable to disable
 *  @return value from selected register
 */
__unused
static int bq24195_set_hiz(struct device *dev, bool enable)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b10000000, enable << 7);
	return ret;
}


/**
 *  @brief  Enable PMIC Buck converter (disable HiZ)
 *  @return value from selected register
 */
static int bq24195_enable_buck(struct device *dev)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b10000000, 0);
	return ret;
}


/**
 *  @brief  Disable PMIC Buck converter (enable HiZ)
 *  @param  enable
 *          enable to disable
 *  @return value from selected register
 */
static int bq24195_disable_buck(struct device *dev)
{
	int ret;

	ret = bq24195_reg_update(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, 0b10000000, BIT(7));
	return ret;
}


static int bq24195_disable_charge(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("disable charging");
	ret = bq24195_set_charge_mode(dev, BQ24195_CHARGE_DISABLE);
	drv_data->charging_disabled_timestamp = k_uptime_get();
	return ret;
}

static int bq24195_enable_charge(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("enable charging");
	ret = bq24195_set_charge_mode(dev, BQ24195_CHARGE_BATTERY);
	if (poll_connect_status(dev)) {
		bq24195_disable_charge(dev);
	} else {
		drv_data->charging_disabled_timestamp = 0;
	}
	return ret;
}

static int bq24195_configure(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config_info;
	int ret;

	ret = bq24195_enable_buck(dev);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_disable_watchdog(dev);
	if (ret < 0) {
		return ret;
	}

	//disableDPDM();
	ret = bq24195_disable_charge(dev);
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_input_voltage_limit_mV(dev, drv_data->power_config.vin_min_voltage);		// min input voltage
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_input_current_limit_mA(dev, drv_data->power_config.vin_max_current);		// max input current
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_charge_current_mA(dev, drv_data->power_config.charge_current);		// charge current
	if (ret < 0) {
		return ret;
	}

	ret = bq24195_set_charge_voltage_mV(dev, drv_data->power_config.termination_voltage);		// termination voltage

	return ret;
}


static int apply_default_config(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	const struct bq24195_dev_config *cfg = dev->config_info;
	int ret;

	drv_data->power_config.vin_min_voltage = cfg->default_power_config.vin_min_voltage;
	drv_data->power_config.vin_max_current = cfg->default_power_config.vin_max_current;
	drv_data->power_config.charge_current = cfg->default_power_config.charge_current;
	drv_data->power_config.termination_voltage = cfg->default_power_config.termination_voltage;
	drv_data->power_config.charge_disabled_timeout = cfg->default_power_config.charge_disabled_timeout;

	ret = bq24195_configure(dev);

	return ret;
}


static bool is_battery_present(struct device *dev)
{
	int ret;
	BQ24195_SYSTEM_STATUS_t value;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_SYSTEM_STATUS, (u8_t *)&value.reg, sizeof(value.reg));
	return (value.bit.DPM_STAT != 0);
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

	ret = bq24195_disable_buck(dev);
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


static int bq24195_get_registers(struct device *dev, BQ24195_device_t *reg)
{
	int ret;

	ret = bq24195_reg_read(dev, BQ24195_REGISTER_INPUT_SOURCE_CONTROL, (u8_t *)reg, sizeof(BQ24195_device_t) - sizeof(BQ24195_FAULT_t));
	if (ret < 0) {
		LOG_WRN("could not read registers");
	}

	/* read the fault register again to obtain the current fault status */
	ret = bq24195_reg_read(dev, BQ24195_REGISTER_FAULT, (u8_t *)&reg->Fault, sizeof(BQ24195_FAULT_t));
	if (ret < 0) {
		LOG_WRN("could not read registers");
	}

	return ret;
}

static void pmic_update_battery_state(struct device *dev, enum pmic_battery_state present_state, enum pmic_battery_state next_state, bool lowBat)
{
	struct bq24195_data *drv_data = dev->driver_data;

	if (next_state == present_state) {
		/* no state change, nothing to do */
		return;
	}

	switch(present_state) {
	case PMIC_BATTERY_STATE_UNKNOWN:
		break;
	case PMIC_BATTERY_STATE_NOT_CHARGING:
		break;
	case PMIC_BATTERY_STATE_CHARGING:
		break;
	case PMIC_BATTERY_STATE_CHARGED:
		break;
	case PMIC_BATTERY_STATE_DISCHARGING:
		break;
	case PMIC_BATTERY_STATE_FAULT:
		break;
	case PMIC_BATTERY_STATE_DISCONNECTED: {
		bq24195_enable_charge(dev);
		break;
	}
	default:
		break;
	}

	switch(next_state) {
	case PMIC_BATTERY_STATE_UNKNOWN:
		break;
	case PMIC_BATTERY_STATE_NOT_CHARGING:
		break;
	case PMIC_BATTERY_STATE_CHARGING:
		poll_connect_status(dev);
		break;
	case PMIC_BATTERY_STATE_CHARGED:
		break;
	case PMIC_BATTERY_STATE_DISCHARGING:
		break;
	case PMIC_BATTERY_STATE_FAULT:
		break;
	case PMIC_BATTERY_STATE_DISCONNECTED:
		bq24195_disable_charge(dev);
		break;
	default:
		break;
	}

	static const char* states[] = {
		"UNKNOWN",
		"NOT_CHARGING",
		"CHARGING",
		"CHARGED",
		"DISCHARGING",
		"FAULT",
		"DISCONNECTED"
	};
	LOG_DBG("Battery state %s -> %s", states[present_state], states[next_state]);

	drv_data->battery_state = next_state;

	/* fire battery state callback */
	if (drv_data->battery_state_change_cb) {
		drv_data->battery_state_change_cb((u8_t) next_state);
	}

}

static void pmic_update_power_state(struct device *dev, enum pmic_power_state present_state, enum pmic_power_state next_state)
{
	struct bq24195_data *drv_data = dev->driver_data;

	if (next_state == present_state) {
		return;
	}

	switch (present_state) {
	case PMIC_POWER_STATE_UNKNOWN:
		break;
	case PMIC_POWER_STATE_USB_HOST:
		break;
	case PMIC_POWER_STATE_USB_ADAPTER:
		break;
	case PMIC_POWER_STATE_USB_OTG:
		break;
	case PMIC_POWER_STATE_BATTERY:
		break;
	default:
		break;
	}

	switch (next_state) {
	case PMIC_POWER_STATE_UNKNOWN:
		bq24195_enable_charge(dev);
		break;
	case PMIC_POWER_STATE_USB_HOST:
		bq24195_enable_charge(dev);
		break;
	case PMIC_POWER_STATE_USB_ADAPTER:
		bq24195_enable_charge(dev);
		break;
	case PMIC_POWER_STATE_USB_OTG:
		break;
	case PMIC_POWER_STATE_BATTERY:
		bq24195_disable_charge(dev);
		break;
	default:
		break;
	}

	static const char* states[] = {
		"UNKNOWN",
		"VIN",
		"BATTERY",
		"USB_HOST",
		"USB_ADAPTER",
		"USB_OTG",
		"SOLAR",
		"THERMO",
		"PRIMARY_BATTERY",
		"BACKUP",
	};
	LOG_DBG("Power state %s -> %s", states[present_state], states[next_state]);

	drv_data->power_state = next_state;

	/* fire power state callback */
	if (drv_data->power_state_change_cb) {
		drv_data->power_state_change_cb((u8_t) next_state);
	}
}

static void pmic_update_device_state(struct device *dev, enum pmic_device_state present_state, enum pmic_device_state next_state)
{
	struct bq24195_data *drv_data = dev->driver_data;

	if (next_state == present_state) {
		return;
	}
	
	switch (present_state) {
	case PMIC_DEVICE_STATE_UNKNOWN:
	case PMIC_DEVICE_STATE_DISABLED:
	case PMIC_DEVICE_STATE_UNINITIALIZED:
	case PMIC_DEVICE_STATE_INITIALIZED:
	case PMIC_DEVICE_STATE_ACTIVE:
	case PMIC_DEVICE_STATE_HOT:
	case PMIC_DEVICE_STATE_FAULT:
	case PMIC_DEVICE_STATE_DEVICE_NOT_PRESENT:
	default:
		break;
	}

	switch (next_state) {
	case PMIC_DEVICE_STATE_UNKNOWN:
	case PMIC_DEVICE_STATE_DISABLED:
	case PMIC_DEVICE_STATE_UNINITIALIZED:
	case PMIC_DEVICE_STATE_INITIALIZED:
	case PMIC_DEVICE_STATE_ACTIVE:
	case PMIC_DEVICE_STATE_HOT:
	case PMIC_DEVICE_STATE_FAULT:
	case PMIC_DEVICE_STATE_DEVICE_NOT_PRESENT:
	default:
		break;
	}

	drv_data->device_state = next_state;

	/* fire device state callback */
	if (drv_data->device_state_change_cb) {
		drv_data->device_state_change_cb((u8_t) next_state);
	}
}

static void handle_update(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	BQ24195_SYSTEM_STATUS_t status;
	BQ24195_FAULT_DATA_t fault;
	enum pmic_battery_state next_battery_state;
	enum pmic_power_state next_power_state;
	enum pmic_device_state next_device_state;
	int ret;

	ret = bq24195_get_fault(dev, &fault);
	if (ret < 0) {
		LOG_DBG("could not read pmic register");
	}
	ret = bq24195_get_system_status(dev, &status);
	if (ret < 0) {
		LOG_DBG("could not read pmic register");
	}

	/* check the watchdog */
	if (fault.fault.bit.WATCHDOG_FAULT) {
		bq24195_configure(dev);
		bq24195_enable_charge(dev);
	}
	next_battery_state = PMIC_BATTERY_STATE_UNKNOWN;

	/* check the charge status */
	switch (status.bit.CHRG_STAT) {
	case BQ24195_CHARGE_STATUS_NOT_CHARGING:
		next_battery_state = PMIC_BATTERY_STATE_NOT_CHARGING;
		if (fault.fault.bit.BAT_FAULT) {
			next_battery_state = PMIC_BATTERY_STATE_FAULT;
		} else if (!status.bit.PG_STAT) {
			next_battery_state = PMIC_BATTERY_STATE_DISCHARGING;
		}
		break;
	case BQ24195_CHARGE_STATUS_PRECHARGE:
	case BQ24195_CHARGE_STATUS_FAST_CHARGE:
		next_battery_state = PMIC_BATTERY_STATE_CHARGING;
		break;
	case BQ24195_CHARGE_STATUS_CHARGE_DONE:
		next_battery_state = PMIC_BATTERY_STATE_CHARGED;
		break;
	}

	/* check if we should enable charging */
	if (drv_data->battery_state == PMIC_BATTERY_STATE_DISCONNECTED &&
	   (drv_data->charging_disabled_timestamp) &&
	   ((k_uptime_get() - drv_data->charging_disabled_timestamp) >= drv_data->power_config.charge_disabled_timeout)) {
		// Re-enable charging, do not run DPDM detection
		LOG_DBG("re-enabling charging");
		bq24195_enable_charge(dev);
	}

	if (drv_data->battery_disconnected) {
		next_battery_state = PMIC_BATTERY_STATE_DISCONNECTED;
	}

	/* check if we should stay in disconnected state */
	if (drv_data->battery_state == PMIC_BATTERY_STATE_DISCONNECTED && next_battery_state == PMIC_BATTERY_STATE_NOT_CHARGING &&
		drv_data->charging_disabled_timestamp) {
		// charging has been disabled, stay in disconnected state
		next_battery_state = PMIC_BATTERY_STATE_DISCONNECTED;
	}

	/* FIXME: get low battery status */
	bool lowBat = false;
	pmic_update_battery_state(dev, drv_data->battery_state, next_battery_state, lowBat);

	next_power_state = PMIC_POWER_STATE_UNKNOWN;

	if (status.bit.PG_STAT) {
		switch (status.bit.VBUS_STAT) {
		case BQ24195_VBUS_STATUS_UNKNOWN:
			next_power_state = PMIC_POWER_STATE_UNKNOWN;
			break;
		case BQ24195_VBUS_STATUS_USB_HOST:
			break;
		case BQ24195_VBUS_STATUS_ADAPTER:
			next_power_state = PMIC_POWER_STATE_USB_ADAPTER;
			break;
		case BQ24195_VBUS_STATUS_USB_OTG:
			next_power_state = PMIC_POWER_STATE_USB_OTG;
			break;
		}
	} else {
		if (drv_data->battery_state == PMIC_BATTERY_STATE_DISCHARGING) {
			next_power_state = PMIC_POWER_STATE_BATTERY;
		} else {
			next_power_state = PMIC_POWER_STATE_UNKNOWN;
		}
	}

	pmic_update_power_state(dev, drv_data->power_state, next_power_state);

	/* FIXME: add device state checks - faults */
	switch (fault.fault.bit.NTC_FAULT) {
	case BQ24195_FAULT_NTC_COLD:
		next_device_state = PMIC_DEVICE_STATE_COLD;
		break;
	case BQ24195_FAULT_NTC_HOT:
		next_device_state = PMIC_DEVICE_STATE_HOT;
		break;
	case BQ24195_FAULT_NTC_NORMAL:
		break;
	}

	switch (fault.fault.bit.CHRG_FAULT) {
	case BQ24195_FAULT_CHARGE_NORMAL:
		break;
	case BQ24195_FAULT_CHARGE_INPUT:
	case BQ24195_FAULT_CHARGE_THERMAL:
	case BQ24195_FAULT_CHARGE_SAFETY_TIMER:
		break;
	}
	
	if (lowBat) {
		/* handle low battery indication */
	}
}

/* update() should be triggered from a usb state change or a pm state change */
static int bq24195_update(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;

	k_poll_signal_raise(&drv_data->update_signal, 0);
	return 0;
}

static void pmic_process(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;

	LOG_DBG("starting pmic process thread");
	drv_data->device_state = PMIC_DEVICE_STATE_ACTIVE;
	bq24195_enable_charge(dev);

	while (true) {
		/* FIXME:  Should we use k_poll and signals here so we can send signal to (re)load a config? */
		/* FIXME:  Use settings to store and retrieve power configs? */
		k_poll(drv_data->events, ARRAY_SIZE(drv_data->events), K_MSEC(DEFAULT_PMIC_WAIT));
		if (drv_data->events[0].state == K_POLL_STATE_SIGNALED) {
			/* update */
			drv_data->events[0].signal->signaled = 0;
			drv_data->events[0].state = K_POLL_STATE_NOT_READY;
			switch (drv_data->events[0].signal->result) {
			case BQ24195_SIGNAL_UPDATE_IMMEDIATE:
				break;
			case BQ24195_SIGNAL_UPDATE_DELAYED:
				k_sleep(K_MSEC(100));
				break;
			}
			handle_update(dev);
		} else {
			/* update on poll timeout */
			handle_update(dev);
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
			DT_INST_LABEL(0));
		return ret;
	}

	if (value.reg != BQ24195_CHIPID) {
		LOG_ERR("%s: Failed to match the device IDs",
			DT_INST_LABEL(0));
		return -EINVAL;
	}

	return 0;
}

static int bq24195_sample_fetch(struct device *dev, enum pmic_channel chan)
{
	int ret;

	switch (chan) {
	case PMIC_CHAN_ALL:
	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_channel_get(struct device *dev, enum pmic_channel chan,
			      struct pmic_value *val)
{
	int ret = 0;

	switch (chan) {
	case PMIC_CHAN_ALL:
	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_attr_get(struct device *dev, enum pmic_channel chan,
			     enum pmic_attribute attr,
			     const void *val)
{
	int ret = 0;

	switch (attr) {
	case PMIC_ATTR_INFO_ID: {
		ret = bq24195_reg_read(dev, BQ24195_REGISTER_PMIC_VENDOR, (u8_t *)val, sizeof(BQ24195_VENDOR_STATUS_t));
		if (ret < 0) {
			LOG_ERR("Failed to get Device ID register");
		}
		break;
	}
	case PMIC_ATTR_STATUS: {
		ret = bq24195_get_system_status(dev, (BQ24195_SYSTEM_STATUS_t *)val);
		if (ret < 0) {
			LOG_ERR("Failed to get Device ID register");
			return ret;
		}
		/* FIXME: need to define PMIC status and translate */
		break;
	}
	case PMIC_ATTR_FAULT: {
		BQ24195_FAULT_DATA_t fault;

		ret = bq24195_get_fault(dev, &fault);
		if (ret < 0) {
			LOG_ERR("Failed to get Device ID register");
			return ret;
		}
		*(BQ24195_FAULT_DATA_t *)val = fault;
		/* FIXME: need to define PMIC fault and translate */
		break;
	}
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_attr_set(struct device *dev, enum pmic_channel chan,
			     enum pmic_attribute attr,
			     const void *val)
{
	int ret = 0;

	switch (attr) {
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int bq24195_register_battery_state_callback(struct device *dev, pmic_battery_state_callback_t callback)
{
	struct bq24195_data *drv_data = dev->driver_data;

	drv_data->battery_state_change_cb = callback;
	return 0;
}

static int bq24195_register_power_state_callback(struct device *dev, pmic_power_state_callback_t callback)
{
	struct bq24195_data *drv_data = dev->driver_data;

	drv_data->power_state_change_cb = callback;
	return 0;
}

static int bq24195_register_device_state_callback(struct device *dev, pmic_device_state_callback_t callback)
{
	struct bq24195_data *drv_data = dev->driver_data;

	drv_data->device_state_change_cb = callback;
	return 0;
}

static const struct pmic_driver_api pmic_driver_api = {
	.attr_get = bq24195_attr_get,
	.attr_set = bq24195_attr_set,
	.sample_fetch = bq24195_sample_fetch,
	.channel_get = bq24195_channel_get,
	.trigger_set = NULL,
	.update = bq24195_update,
	.register_battery_state_callback = bq24195_register_battery_state_callback,
	.register_power_state_callback = bq24195_register_power_state_callback,
	.register_device_state_callback = bq24195_register_device_state_callback,
};

static int bq24195_chip_init(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	int ret;

	while (k_uptime_ticks() < k_us_to_ticks_ceil32(BQ24195_STARTUP_TIME_USEC)) {
		/* wait for chip to power up */
	}

	drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
	if (drv_data->i2c == NULL) {
		drv_data->device_state = PMIC_DEVICE_STATE_DEVICE_NOT_PRESENT;
		LOG_ERR("Failed to get pointer to %s device",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

	ret = bq24195_device_id_check(dev);
	if (ret < 0) {
		drv_data->device_state = PMIC_DEVICE_STATE_DEVICE_NOT_PRESENT;
		return -EIO;
	}

	LOG_DBG("BQ24195 pmic detected");

	load_default_config(dev);
	ret = bq24195_configure(dev);

	return ret;
}

int bq24195_init(struct device *dev)
{
	struct bq24195_data *drv_data = dev->driver_data;
	k_tid_t thread;
	int ret;

	drv_data->device_state = PMIC_DEVICE_STATE_UNINITIALIZED;
	drv_data->battery_state = PMIC_BATTERY_STATE_UNKNOWN;
	drv_data->power_state = PMIC_POWER_STATE_UNKNOWN;
	ret = bq24195_chip_init(dev);
	if (ret == 0) {
		drv_data->battery_state_change_cb = NULL;
		drv_data->power_state_change_cb = NULL;
		drv_data->device_state_change_cb = NULL;
		k_poll_signal_init(&drv_data->update_signal);
		k_poll_event_init(&drv_data->events[0],
				K_POLL_TYPE_SIGNAL,
				K_POLL_MODE_NOTIFY_ONLY,
				&drv_data->update_signal);
		thread = k_thread_create(&pmic_thread, pmic_stack,
				K_THREAD_STACK_SIZEOF(pmic_stack),
				(k_thread_entry_t) pmic_process,
				dev, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

		k_thread_name_set(thread, "pmic");
		drv_data->device_state = PMIC_DEVICE_STATE_INITIALIZED;
		bq24195_update(dev);
	}

	return ret;
}

static struct bq24195_data bq24195_drv_data;

static const struct bq24195_dev_config bq24195_config = {
	.i2c_addr = DT_INST_REG_ADDR(0),
	.api = {
		.reg_read = bq24195_reg_read,
		.reg_write = bq24195_reg_write,
	},
	.default_power_config = {
		.flags = 0,
		.version = 0,
		.size = sizeof(struct pmic_power_config),
		.vin_min_voltage = DEFAULT_INPUT_VOLTAGE_LIMIT,
		.vin_max_current = DEFAULT_INPUT_CURRENT_LIMIT,
		.charge_current = DEFAULT_CHARGE_CURRENT,
		.termination_voltage = DEFAULT_TERMINATION_VOLTAGE,
		.charge_disabled_timeout = DEFAULT_CHARGE_DISABLED_TIMEOUT,
	},
};

DEVICE_AND_API_INIT(bq24195, DT_INST_LABEL(0), bq24195_init,
		    &bq24195_drv_data, &bq24195_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &pmic_driver_api);
