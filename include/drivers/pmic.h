/**
 * @file pmic.h
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

#ifndef ZEPHYR_INCLUDE_DRIVERS_PMIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_PMIC_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
// #include <dt-bindings/gpio/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Representation of a pmic readout value.
 *
 * The value is represented as having an integer and a fractional part,
 * and can be obtained using the formula val1 + val2 * 10^(-6). Negative
 * values also adhere to the above formula, but may need special attention.
 * Here are some examples of the value representation:
 *
 *      0.5: val1 =  0, val2 =  500000
 *     -0.5: val1 =  0, val2 = -500000
 *     -1.0: val1 = -1, val2 =  0
 *     -1.5: val1 = -1, val2 = -500000
 */
struct pmic_value {
	/** Integer part of the value. */
	s32_t val1;
	/** Fractional part of the value (in one-millionth parts). */
	s32_t val2;
};

/* driver sensor channels */
enum pmic_channel {
	PMIC_CHAN_NONE,
	/** All channels. */
	PMIC_CHAN_ALL,

	/**
	 * Number of all common sensor channels.
	 */
	PMIC_CHAN_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	PMIC_CHAN_PRIV_START = PMIC_CHAN_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor channel type.
	 */
	PMIC_CHAN_MAX = INT16_MAX,
};

/* driver sensor attributes */
enum pmic_attribute {
	PMIC_ATTR_STATUS,
	PMIC_ATTR_FAULT,
	PMIC_ATTR_FAULT_COUNT,
	PMIC_ATTR_LATCH,
	PMIC_ATTR_INFO_ID,
	PMIC_ATTR_INFO_VERSION,

	/**
	 * Number of all common sensor attributes.
	 */
	PMIC_ATTR_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	PMIC_ATTR_PRIV_START = PMIC_ATTR_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	PMIC_ATTR_MAX = INT16_MAX,
};

/**
 * @brief Sensor trigger types.
 */
enum pmic_trigger_type {
	/**
	 * Timer-based trigger, useful when the sensor does not have an
	 * interrupt line.
	 */
	PMIC_TRIG_TIMER,
	/** Trigger fires whenever new data is ready. */
	PMIC_TRIG_DATA_READY,
	/**
	 * Trigger fires when the selected channel varies significantly.
	 * This includes any-motion detection when the channel is
	 * acceleration or gyro. If detection is based on slope between
	 * successive channel readings, the slope threshold is configured
	 * via the @ref PMIC_ATTR_SLOPE_TH and @ref PMIC_ATTR_SLOPE_DUR
	 * attributes.
	 */
	PMIC_TRIG_DELTA,
	/**
	 * Trigger fires when channel reading transitions configured
	 * thresholds.  The thresholds are configured via the @ref
	 * PMIC_ATTR_LOWER_THRESH and @ref PMIC_ATTR_UPPER_THRESH
	 * attributes.
	 */
	PMIC_TRIG_THRESHOLD,

	/**
	 * Number of all common sensor triggers.
	 */
	PMIC_TRIG_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	PMIC_TRIG_PRIV_START = PMIC_TRIG_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor trigger type.
	 */
	PMIC_TRIG_MAX = INT16_MAX,
};

/**
 * @brief Sensor trigger spec.
 */
struct pmic_trigger {
	/** Trigger type. */
	enum pmic_trigger_type type;
	/** Channel the trigger is set on. */
	enum pmic_channel chan;
};

enum pmic_state {
	PMIC_STATE_IDLE,
	PMIC_STATE_START,
	PMIC_STATE_CONNECTED,
	PMIC_STATE_DISCONNECTED,
	PMIC_STATE_CHARGING,
	PMIC_STATE_BATTERY_NOT_PRESENT,
	PMIC_STATE_DEVICE_NOT_PRESENT,
	PMIC_STATE_DISABLED,
};

enum pmic_device_state {
	PMIC_DEVICE_STATE_UNKNOWN = 0,
	PMIC_DEVICE_STATE_DISABLED,
	PMIC_DEVICE_STATE_UNINITIALIZED,
	PMIC_DEVICE_STATE_INITIALIZED,
	PMIC_DEVICE_STATE_ACTIVE,
	PMIC_DEVICE_STATE_HOT,
	PMIC_DEVICE_STATE_COLD,
	PMIC_DEVICE_STATE_FAULT,
	PMIC_DEVICE_STATE_DEVICE_NOT_PRESENT,
};

enum pmic_power_state {
	PMIC_POWER_STATE_UNKNOWN = 0,
	PMIC_POWER_STATE_VIN,
	PMIC_POWER_STATE_BATTERY,
	PMIC_POWER_STATE_USB_HOST,
	PMIC_POWER_STATE_USB_ADAPTER,
	PMIC_POWER_STATE_USB_OTG,
	PMIC_POWER_STATE_SOLAR,
	PMIC_POWER_STATE_THERMO,
	PMIC_POWER_STATE_PRIMARY_BATTERY,
	PMIC_POWER_STATE_BACKUP,
};

enum pmic_battery_state {
	PMIC_BATTERY_STATE_UNKNOWN = 0,
	PMIC_BATTERY_STATE_NOT_CHARGING,
	PMIC_BATTERY_STATE_CHARGING,
	PMIC_BATTERY_STATE_CHARGED,
	PMIC_BATTERY_STATE_DISCHARGING,
	PMIC_BATTERY_STATE_FAULT,
	PMIC_BATTERY_STATE_DISCONNECTED,
};

enum pmic_charge_state {
	PMIC_CHARGE_STATE_UNKNOWN = 0,
	PMIC_CHARGE_STATE_NOT_CHARGING,
	PMIC_CHARGE_STATE_CHARGING,
	PMIC_CHARGE_STATE_CHARGED,
	PMIC_CHARGE_STATE_DISCHARGING,
	PMIC_CHARGE_STATE_FAULT,
	PMIC_CHARGE_STATE_DISCONNECTED,
};

struct pmic_power_config {
	u16_t flags;
	u8_t version;
	u8_t size;
	u16_t vin_min_voltage;
	u16_t vin_max_current;
	u16_t charge_current;
	u16_t termination_voltage;
};

/**
 * @typedef pmic_trigger_handler_t
 * @brief Callback API upon firing of a trigger
 *
 * @param "struct device *dev" Pointer to the sensor device
 * @param "struct pmic_trigger *trigger" The trigger
 */
typedef void (*pmic_trigger_handler_t)(struct device *dev,
					struct pmic_trigger *trigger);

/**
 * @typedef pmic_trigger_set_t
 * @brief Callback API for setting a sensor's trigger and handler
 *
 * See pmic_trigger_set() for argument description
 */
typedef int (*pmic_trigger_set_t)(struct device *dev,
				    const struct pmic_trigger *trig,
				    pmic_trigger_handler_t handler);

/**
 * @typedef pmic_attr_get_t
 * @brief Callback API upon getting a sensor's attributes
 *
 * See pmic_attr_get() for argument description
 */
typedef int (*pmic_attr_get_t)(struct device *dev,
				 enum pmic_channel chan,
				 enum pmic_attribute attr,
				 const void *val);

/**
 * @typedef pmic_attr_set_t
 * @brief Callback API upon setting a sensor's attributes
 *
 * See pmic_attr_set() for argument description
 */
typedef int (*pmic_attr_set_t)(struct device *dev,
				 enum pmic_channel chan,
				 enum pmic_attribute attr,
				 const void *val);
/**
 * @typedef pmic_sample_fetch_t
 * @brief Callback API for fetching data from a sensor
 *
 * See pmic_sample_fetch() for argument description
 */
typedef int (*pmic_sample_fetch_t)(struct device *dev,
				     enum pmic_channel chan);
/**
 * @typedef pmic_channel_get_t
 * @brief Callback API for getting a reading from a sensor
 *
 * See pmic_channel_get() for argument description
 */
typedef int (*pmic_channel_get_t)(struct device *dev,
				    enum pmic_channel chan,
				    struct pmic_value *val);

/**
 * @typedef pmic_update_t
 * @brief Callback API for getting a reading from a sensor
 *
 * See pmic_update() for argument description
 */
typedef int (*pmic_update_t)(struct device *dev);

__subsystem struct pmic_driver_api {
	pmic_attr_get_t attr_get;
	pmic_attr_set_t attr_set;
	pmic_trigger_set_t trigger_set;
	pmic_sample_fetch_t sample_fetch;
	pmic_channel_get_t channel_get;
	pmic_update_t update;
};

/**
 * @brief Get an attribute for a sensor
 *
 * @param dev Pointer to the sensor device
 * @param chan The channel the attribute belongs to, if any.  Some
 * attributes may only be set for all channels of a device, depending on
 * device capabilities.
 * @param attr The attribute to set
 * @param val The value to set the attribute to
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_attr_get(struct device *dev,
			      enum pmic_channel chan,
			      enum pmic_attribute attr,
			      const void *val);

static inline int z_impl_pmic_attr_get(struct device *dev,
					enum pmic_channel chan,
					enum pmic_attribute attr,
					const void *val)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	if (api->attr_get == NULL) {
		return -ENOTSUP;
	}

	return api->attr_get(dev, chan, attr, val);
}

/**
 * @brief Set an attribute for a sensor
 *
 * @param dev Pointer to the pmic device
 * @param chan The channel the attribute belongs to, if any.  Some
 * attributes may only be set for all channels of a device, depending on
 * device capabilities.
 * @param attr The attribute to set
 * @param val The value to set the attribute to
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_attr_set(struct device *dev,
			      enum pmic_channel chan,
			      enum pmic_attribute attr,
			      const void *val);

static inline int z_impl_pmic_attr_set(struct device *dev,
					enum pmic_channel chan,
					enum pmic_attribute attr,
					const void *val)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	if (api->attr_set == NULL) {
		return -ENOTSUP;
	}

	return api->attr_set(dev, chan, attr, val);
}

/**
 * @brief Activate a sensor's trigger and set the trigger handler
 *
 * The handler will be called from a thread, so I2C or SPI operations are
 * safe.  However, the thread's stack is limited and defined by the
 * driver.  It is currently up to the caller to ensure that the handler
 * does not overflow the stack.
 *
 * This API is not permitted for user threads.
 *
 * @param dev Pointer to the pmic device
 * @param trig The trigger to activate
 * @param handler The function that should be called when the trigger
 * fires
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int pmic_trigger_set(struct device *dev,
				     struct pmic_trigger *trig,
				     pmic_trigger_handler_t handler)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	if (api->trigger_set == NULL) {
		return -ENOTSUP;
	}

	return api->trigger_set(dev, trig, handler);
}

/**
 * @brief Fetch a sample from the sensor and store it in an internal
 * driver buffer
 *
 * Read all of a sensor's active channels and, if necessary, perform any
 * additional operations necessary to make the values useful.  The user
 * may then get individual channel values by calling @ref
 * pmic_channel_get.
 *
 * Since the function communicates with the pmic device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * @param dev Pointer to the pmic device
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_sample_fetch(struct device *dev);

static inline int z_impl_pmic_sample_fetch(struct device *dev)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	return api->sample_fetch(dev, PMIC_CHAN_ALL);
}

/**
 * @brief Fetch a sample from the sensor and store it in an internal
 * driver buffer
 *
 * Read and compute compensation for one type of sensor data (magnetometer,
 * accelerometer, etc). The user may then get individual channel values by
 * calling @ref pmic_channel_get.
 *
 * This is mostly implemented by multi function devices enabling reading at
 * different sampling rates.
 *
 * Since the function communicates with the pmic device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * @param dev Pointer to the pmic device
 * @param type The channel that needs updated
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_sample_fetch_chan(struct device *dev,
				       enum pmic_channel type);

static inline int z_impl_pmic_sample_fetch_chan(struct device *dev,
						 enum pmic_channel type)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	return api->sample_fetch(dev, type);
}

/**
 * @brief Get a reading from a pmic device
 *
 * Return a useful value for a particular channel, from the driver's
 * internal data.  Before calling this function, a sample must be
 * obtained by calling @ref pmic_sample_fetch or
 * @ref pmic_sample_fetch_chan. It is guaranteed that two subsequent
 * calls of this function for the same channels will yield the same
 * value, if @ref pmic_sample_fetch or @ref pmic_sample_fetch_chan
 * has not been called in the meantime.
 *
 * For vectorial data samples you can request all axes in just one call
 * by passing the specific channel with _XYZ suffix. The sample will be
 * returned at val[0], val[1] and val[2] (X, Y and Z in that order).
 *
 * @param dev Pointer to the pmic device
 * @param chan The channel to read
 * @param val Where to store the value
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_channel_get(struct device *dev,
				 enum pmic_channel chan,
				 struct pmic_value *val);

static inline int z_impl_pmic_channel_get(struct device *dev,
					   enum pmic_channel chan,
					   struct pmic_value *val)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	return api->channel_get(dev, chan, val);
}

/**
 * @brief Update the pmic state machine
 *
 * @param dev Pointer to the pmic device
 * @param chan The channel the attribute belongs to, if any.  Some
 * attributes may only be set for all channels of a device, depending on
 * device capabilities.
 * @param attr The attribute to set
 * @param val The value to set the attribute to
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int pmic_update(struct device *dev);

static inline int z_impl_pmic_update(struct device *dev)
{
	const struct pmic_driver_api *api =
		(const struct pmic_driver_api *)dev->driver_api;

	if (api->update == NULL) {
		return -ENOTSUP;
	}

	return api->update(dev);
}

#ifdef __cplusplus
}
#endif

#include <syscalls/pmic.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_PMIC_H_ */
