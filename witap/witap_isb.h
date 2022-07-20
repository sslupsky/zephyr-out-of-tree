/**
 * @file witap_isb.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

/*
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef _WITAP_ISB_H_
#define _WITAP_ISB_H_

#include <zephyr/types.h>

struct witap_isb_config_reg {
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

struct witap_isb_status_flags {
	u32_t enabled : 1;
};

struct witap_isb_sensor_ready_flags {
	u32_t ths_temperature : 1;
	u32_t ths_humidity : 1;
	u32_t bps_temperature : 1;
	u32_t bps_pressure : 1;
	u32_t ambient_light : 1;
	u32_t accelX : 1;
	u32_t accelY : 1;
	u32_t accelZ : 1;
	u32_t gyroX : 1;
	u32_t gyroY : 1;
	u32_t gyroZ : 1;
	u32_t sensor1 : 1;
	u32_t sensor2 : 1;
	u32_t altitude : 1;
	u32_t latitude : 1;
	u32_t longitude : 1;
	u32_t satelites : 1;
	u32_t dop : 1;
	u32_t gnss_time : 1;
	u32_t gnss_fixOK : 1;
} __packed;

struct witap_isb_config_data {
	char name[16];
	u16_t version;
} __attribute__((packed));

struct witap_isb_device_data {
	witap_isb_config_reg config;
	struct boot_status boot;
	u64_t uptime;
	u64_t sleep_time;
	u16_t battery;
} __attribute__((packed));

struct witap_isb_sensor_data {
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
	struct witap_isb_sensor_ready_flags ready;
} __attribute__((packed));

struct witap_isb {
	u8_t idx;
	struct witap_isb_config_data config;
	struct witap_isb_device_data device;
	struct witap_isb_sensor_data sensor;
} __attribute__((packed));


#endif  /*  _WITAP_ISB_H_  */