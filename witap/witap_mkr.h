/**
 * @file witap_mkr.h
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


#ifndef _WITAP_MKR_H_
#define _WITAP_MKR_H_

#include <zephyr/types.h>

#define WITAP_STATUS_POR_DEFAULT {	\
	.current_time = 0,		\
	.uptime = 0,			\
	.sleep_time = 0,		\
	.boot = {			\
		.time = 0,		\
		.count = 0,		\
		.cause = 0,		\
	},				\
}


/**
 * @brief Boot status descriptor
 * 
 */
struct witap_mkr_boot_status {
	time_t	 time;
	uint32_t count;
	u8_t	 cause;
} __attribute__((packed));


struct witap_mkr_status {
	time_t  current_time;
	time_t  uptime;
	time_t  sleep_time;
	struct witap_mkr_boot_status boot;
};


#endif  /* _WITAP_MKR_H  */
