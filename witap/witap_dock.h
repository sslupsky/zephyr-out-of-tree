/**
 * @file witap_dock.h
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


#ifndef _WITAP_DOCK_H_
#define _WITAP_DOCK_H_

#include <zephyr/types.h>


enum DOCK_SD_POWER_e {
    SD_PWR_ON,
    SD_PWR_OFF
};

struct witap_mkr_dock_config_reg {
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

struct witap_mkr_dock_config_data {
	char name[16];
	u16_t version;
};

struct witap_mkr_dock_sensor_data {
	s16_t ths_temperature;
	s16_t sensor1;
	s16_t sensor2;
	s16_t ads1115_0;
	s16_t ads1115_1;
	s16_t ads1115_2;
	s16_t ads1115_3;
	uint16_t battery;
};

struct witap_mkr_dock {
	struct witap_mkr_dock_config_data config;
	struct witap_mkr_dock_sensor_data sensor;
};

#endif /*  _WITAP_DOCK_H_  */
