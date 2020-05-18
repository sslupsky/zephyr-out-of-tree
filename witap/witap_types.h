/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef WITAP_TYPES_H
#define WITAP_TYPES_H

#include <zephyr.h>
#include <usb/usb_device.h>

/* forward declaration of global scope variables */
extern volatile enum usb_dc_status_code usbState;
extern volatile bool application_boot;
extern volatile bool force_sleep;

enum witap_pm_devices {
	WITAP_PM_I2C,
	WITAP_PM_SPI,
	WITAP_PM_LORA_UART,
	WITAP_PM_USB,
	WITAP_PM_BOOT,
	WITAP_PM_FORCE_SLEEP,
};

typedef struct {
    u32_t commissionTime;
    u32_t updateTime;
//     gps_t installLocation;
} CONFIG_FILE_t;

/**
 * @brief Boot status descriptor
 * 
 */
struct BOOT_STATUS {
    u64_t time {};
    u32_t count {};
    u8_t  cause {};
} __attribute__((packed));


enum WITAP_SD_POWER_e {
    SD_PWR_ON,
    SD_PWR_OFF
};

struct isb_config_reg {
	union {
		u32_t reg;		
		struct {
			u32_t reset : 1;
			u32_t enable : 1;
			u32_t sensor1_enable : 1;
			u32_t sensor2_enable : 1;
			u32_t ths_enable : 1;
			u32_t bps_enable : 1;
			u32_t als_enable : 1;
			u32_t imu_enable : 1;
			u32_t gps_enable : 1;
		} bit;
	};
};

struct dock1310_config_reg {
	union {
		u32_t reg;		
		struct {
			u32_t reset : 1;
			u32_t enable : 1;
			u32_t sensor1_enable : 1;
			u32_t sensor2_enable : 1;
			u32_t ts_enable : 1;
			u32_t ads1115_enable : 1;
		} bit;
	};
};

struct isb_status_flags {
	u32_t enabled : 1;
};

struct isb_config_data {
	char name[16];
	u16_t version;
} __attribute__((packed));

struct isb_device_data {
	isb_config_reg config;
	struct BOOT_STATUS boot;
	u64_t uptime;
	u64_t sleep_time;
	u16_t battery;
} __attribute__((packed));

struct isb_sensor_data {
	s16_t ths_temperature;
	u16_t ths_humidity;
	s16_t bps_temperature;
	u16_t bps_pressure;
	u32_t ambient_light;
	s16_t accelX;
	s16_t accelY;
	s16_t accelZ;
	s16_t gyroX;
	s16_t gyroY;
	s16_t gyroZ;
	s16_t sensor1;
	s16_t sensor2;
	s32_t altitude;
	s32_t latitude;
	s32_t longitude;
	u8_t satelites;
	u8_t dop;
	u32_t gnss_time;
} __attribute__((packed));

struct witap_isb {
	u8_t idx;
	struct isb_config_data config;
	struct isb_device_data device;
	struct isb_sensor_data sensor;
} __attribute__((packed));

struct dock1310_config_data {
	char name[16];
	u16_t version;
};

struct dock1310_device_data {
	dock1310_config_reg config;
	struct BOOT_STATUS boot;
	u64_t uptime;
	u64_t sleep_time;
	u16_t battery;
};

struct dock1310_sensor_data {
	s16_t ths_temperature;
	s16_t sensor1;
	s16_t sensor2;
	s16_t ads1115_0;
	s16_t ads1115_1;
	s16_t ads1115_2;
	s16_t ads1115_3;
};

struct witap_dock1310 {
	struct dock1310_config_data config;
	struct dock1310_device_data device;
	struct dock1310_sensor_data sensor;
};

/* Forward declaration of "C" functions */
#ifdef __cplusplus
extern "C" {
#endif

/* none */

#ifdef __cplusplus
}
#endif

#endif /* WITAP_TYPES_H */