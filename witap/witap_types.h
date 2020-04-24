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
extern bool application_boot;

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

struct isb_config {
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

struct isb_status_flags {
	u32_t enabled : 1;
};

struct isb_device {
	u8_t idx;
	isb_config config;
	char name[16];
	u16_t version;
	struct BOOT_STATUS boot;
	s16_t ambient_temp;
	u16_t humidity;
	s32_t barometric_press;
	s32_t ambient_light;
	s16_t accelX;
	s16_t accelY;
	s16_t accelZ;
	s16_t gyroX;
	s16_t gyroY;
	s16_t gyroZ;
	s16_t magX;
	s16_t magY;
	s16_t magZ;
	s16_t sensor1;
	s16_t sensor2;
	s32_t altitude;
	s32_t latitude;
	s32_t longitude;
	u8_t satelites;
	u8_t dop;
	u32_t gnss_time;
} __attribute__((packed));

/* Forward declaration of "C" functions */
#ifdef __cplusplus
extern "C" {
#endif

/* none */

#ifdef __cplusplus
}
#endif

#endif /* WITAP_TYPES_H */