/**
 * @file gnss.h
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

#ifndef ZEPHYR_INCLUDE_DRIVERS_GNSS_H_
#define ZEPHYR_INCLUDE_DRIVERS_GNSS_H_

#include <sys/__assert.h>
#include <sys/slist.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <device.h>
// #include <dt-bindings/gpio/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct gnss_time {
	u32_t timeOfWeek;		 // ms
	u16_t gpsYear;
	u8_t gpsMonth;
	u8_t gpsDay;
	u8_t gpsHour;
	u8_t gpsMinute;
	u8_t gpsSecond;
	u8_t valid;
	u32_t accuracy;
	s32_t gpsNanosecond;
};

struct gnss_position {
	u8_t fixType;		 //Tells us when we have a solution aka lock
	u8_t fixflags;
	u8_t flags2;
	u8_t SIV;			 //Number of satellites used in position solution
	s32_t longitude;		 //Degrees * 10^-7 (more accurate than floats)
	s32_t latitude;		 //Degrees * 10^-7 (more accurate than floats)
	s32_t altitude;		 //Number of mm above ellipsoid
	s32_t altitudeMSL;	 //Number of mm above Mean Sea Level
	u32_t horizontalAccuracy; // mm * 10^-1 (i.e. 0.1mm)
	u32_t verticalAccuracy;	 // mm * 10^-1 (i.e. 0.1mm)
};

struct gnss_velocity {
	s32_t north;	 //mm/s
	s32_t east;	 //mm/s
	s32_t down;	 //mm/s
	s32_t ground_speed;	 //mm/s
	s32_t heading_of_motion; //degrees * 10^-5
	u32_t speed_accuracy;
	u32_t heading_accuracy;
	u16_t pDOP;			 //Positional dilution of precision
};

struct gnss_pvt {
	//The major datums we want to globally store
	struct gnss_time time;
	struct gnss_position position;
	struct gnss_velocity velocity;
};

struct gnss_firmware_version {
	u8_t versionLow;		 //Loaded from getProtocolVersion().
	u8_t versionHigh;
};

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
struct geo_fence_state {
	u8_t status;	   // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
	u8_t numFences; // Number of geofences
	u8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
	u8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
};

// Struct to hold the current geofence parameters
struct geo_fence_params {
	u8_t numFences; // Number of active geofences
	s32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
	s32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
	u32_t rads[4];  // Radii of geofences (in m * 10^-2)
};


/* driver sensor channels */
enum gnss_channel {
	GNSS_CHAN_TIME,
	GNSS_CHAN_POSITION,
	GNSS_CHAN_VELOCITY,
	
	/** All channels. */
	GNSS_CHAN_ALL,

	/**
	 * Number of all common sensor channels.
	 */
	GNSS_CHAN_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	GNSS_CHAN_PRIV_START = GNSS_CHAN_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor channel type.
	 */
	GNSS_CHAN_MAX = INT16_MAX,
};

/* driver sensor attributes */
enum gnss_attribute {
	/**
	 * Sensor sampling frequency, i.e. how many times a second the
	 * sensor takes a measurement.
	 */
	GNSS_ATTR_SAMPLING_FREQUENCY,

	/**
	 * Number of all common sensor attributes.
	 */
	GNSS_ATTR_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	GNSS_ATTR_PRIV_START = GNSS_ATTR_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	GNSS_ATTR_MAX = INT16_MAX,
};

/**
 * @brief Sensor trigger types.
 */
enum gnss_trigger_type {
	/**
	 * Timer-based trigger, useful when the sensor does not have an
	 * interrupt line.
	 */
	GNSS_TRIG_TIMER,
	/** Trigger fires whenever new data is ready. */
	GNSS_TRIG_DATA_READY,
	/** Trigger fires when a geofence event is detected. */
	GNSS_TRIG_GEOFENCE,
	/**
	 * Trigger fires when channel reading transitions configured
	 * thresholds.  The thresholds are configured via the @ref
	 * GNSS_ATTR_LOWER_THRESH and @ref GNSS_ATTR_UPPER_THRESH
	 * attributes.
	 */
	GNSS_TRIG_THRESHOLD,

	/**
	 * Number of all common sensor triggers.
	 */
	GNSS_TRIG_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	GNSS_TRIG_PRIV_START = GNSS_TRIG_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor trigger type.
	 */
	GNSS_TRIG_MAX = INT16_MAX,
};

/**
 * @brief Sensor trigger spec.
 */
struct gnss_trigger {
	/** Trigger type. */
	enum gnss_trigger_type type;
	/** Channel the trigger is set on. */
	enum gnss_channel chan;
};

enum gnss_state {
	GNSS_STATE_idle,
	GNSS_STATE_start,
	GNSS_STATE_connected,
	GNSS_STATE_disconnected,
	GNSS_STATE_charging,
	GNSS_STATE_batteryNotPresent,
	GNSS_STATE_deviceNotPresent,
	GNSS_STATE_disabled,
};


//Depending on the sentence type the processor will load characters into different arrays
enum gnss_sentence_type {
	NONE = 0,
	NMEA,
	UBX,
	RTCM
};

//Controls which port we look to for incoming bytes
enum gnss_module_comm_type {
	COMM_TYPE_I2C = 0,
	COMM_TYPE_SERIAL,
	COMM_TYPE_SPI,
};

enum gnss_model // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
{
	GNSS_MODEL_PORTABLE = 0, //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
	// 1 is not defined
	GNSS_MODEL_STATIONARY = 2, //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
	GNSS_MODEL_PEDESTRIAN,	  //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
	GNSS_MODEL_AUTOMOTIVE,	  //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
	GNSS_MODEL_SEA,			  //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
	GNSS_MODEL_AIRBORNE1g,	  //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
	GNSS_MODEL_AIRBORNE2g,	  //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
	GNSS_MODEL_AIRBORNE4g,	  //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
	GNSS_MODEL_WRIST,		  // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
	GNSS_MODEL_BIKE,			  // Supported in protocol versions 19.2
};

/**
 * @typedef gnss_trigger_handler_t
 * @brief Callback API upon firing of a trigger
 *
 * @param "struct device *dev" Pointer to the sensor device
 * @param "struct gnss_trigger *trigger" The trigger
 */
typedef void (*gnss_trigger_handler_t)(struct device *dev,
					struct gnss_trigger *trigger);

/**
 * @typedef gnss_trigger_set_t
 * @brief Callback API for setting a sensor's trigger and handler
 *
 * See gnss_trigger_set() for argument description
 */
typedef int (*gnss_trigger_set_t)(struct device *dev,
				    const struct gnss_trigger *trig,
				    gnss_trigger_handler_t handler);

/**
 * @typedef gnss_attr_set_t
 * @brief Callback API upon setting a sensor's attributes
 *
 * See gnss_attr_set() for argument description
 */
typedef int (*gnss_attr_set_t)(struct device *dev,
				 enum gnss_channel chan,
				 enum gnss_attribute attr,
				 const struct gnss_pvt *val);
/**
 * @typedef gnss_sample_fetch_t
 * @brief Callback API for fetching data from a sensor
 *
 * See gnss_sample_fetch() for argument description
 */
typedef int (*gnss_sample_fetch_t)(struct device *dev,
				     enum gnss_channel chan);
/**
 * @typedef gnss_channel_get_t
 * @brief Callback API for getting a reading from a sensor
 *
 * See gnss_channel_get() for argument description
 */
typedef int (*gnss_channel_get_t)(struct device *dev,
				    enum gnss_channel chan,
				    struct gnss_pvt *val);

__subsystem struct gnss_driver_api {
	gnss_attr_set_t attr_set;
	gnss_trigger_set_t trigger_set;
	gnss_sample_fetch_t sample_fetch;
	gnss_channel_get_t channel_get;
};

/**
 * @brief Set an attribute for a sensor
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
__syscall int gnss_attr_set(struct device *dev,
			      enum gnss_channel chan,
			      enum gnss_attribute attr,
			      const struct gnss_pvt *val);

static inline int z_impl_gnss_attr_set(struct device *dev,
					enum gnss_channel chan,
					enum gnss_attribute attr,
					const struct gnss_pvt *val)
{
	const struct gnss_driver_api *api =
		(const struct gnss_driver_api *)dev->driver_api;

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
 * @param dev Pointer to the sensor device
 * @param trig The trigger to activate
 * @param handler The function that should be called when the trigger
 * fires
 *
 * @return 0 if successful, negative errno code if failure.
 */
static inline int gnss_trigger_set(struct device *dev,
				     struct gnss_trigger *trig,
				     gnss_trigger_handler_t handler)
{
	const struct gnss_driver_api *api =
		(const struct gnss_driver_api *)dev->driver_api;

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
 * gnss_channel_get.
 *
 * Since the function communicates with the sensor device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * @param dev Pointer to the sensor device
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int gnss_sample_fetch(struct device *dev);

static inline int z_impl_gnss_sample_fetch(struct device *dev)
{
	const struct gnss_driver_api *api =
		(const struct gnss_driver_api *)dev->driver_api;

	return api->sample_fetch(dev, GNSS_CHAN_ALL);
}

/**
 * @brief Fetch a sample from the sensor and store it in an internal
 * driver buffer
 *
 * Read and compute compensation for one type of sensor data (magnetometer,
 * accelerometer, etc). The user may then get individual channel values by
 * calling @ref gnss_channel_get.
 *
 * This is mostly implemented by multi function devices enabling reading at
 * different sampling rates.
 *
 * Since the function communicates with the sensor device, it is unsafe
 * to call it in an ISR if the device is connected via I2C or SPI.
 *
 * @param dev Pointer to the sensor device
 * @param type The channel that needs updated
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int gnss_sample_fetch_chan(struct device *dev,
				       enum gnss_channel type);

static inline int z_impl_gnss_sample_fetch_chan(struct device *dev,
						 enum gnss_channel type)
{
	const struct gnss_driver_api *api =
		(const struct gnss_driver_api *)dev->driver_api;

	return api->sample_fetch(dev, type);
}

/**
 * @brief Get a reading from a sensor device
 *
 * Return a useful value for a particular channel, from the driver's
 * internal data.  Before calling this function, a sample must be
 * obtained by calling @ref gnss_sample_fetch or
 * @ref gnss_sample_fetch_chan. It is guaranteed that two subsequent
 * calls of this function for the same channels will yield the same
 * value, if @ref gnss_sample_fetch or @ref gnss_sample_fetch_chan
 * has not been called in the meantime.
 *
 * For vectorial data samples you can request all axes in just one call
 * by passing the specific channel with _XYZ suffix. The sample will be
 * returned at val[0], val[1] and val[2] (X, Y and Z in that order).
 *
 * @param dev Pointer to the sensor device
 * @param chan The channel to read
 * @param val Where to store the value
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int gnss_channel_get(struct device *dev,
				 enum gnss_channel chan,
				 struct gnss_pvt *val);

static inline int z_impl_gnss_channel_get(struct device *dev,
					   enum gnss_channel chan,
					   struct gnss_pvt *val)
{
	const struct gnss_driver_api *api =
		(const struct gnss_driver_api *)dev->driver_api;

	return api->channel_get(dev, chan, val);
}


#ifdef __cplusplus
}
#endif

#include <syscalls/gnss.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_GNSS_H_ */
