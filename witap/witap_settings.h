/**
 * @file witap_settings.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
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


#ifndef __WITAP_SETTINGS_H__
#define __WITAP_SETTINGS_H__

#include <stdio.h>
#include <getopt.h>

#include <zephyr.h>
#include <device.h>
#include <version.h>
#include <stdlib.h>
#include <soc.h>
#include <devicetree.h>

#include <settings/settings.h>

#include <logging/log.h>

struct dock_config_settings {
	u16_t sample_isb : 1;
	u16_t sample_dock : 1;
	u16_t sample_gnss : 1;
	u16_t sample_status : 1;
	u16_t sample : 1;
	u16_t log_data : 1;
	u16_t log_console : 1;
	u16_t reserved_1 : 2;
	u16_t network : 1;
	u16_t reserved_2 : 6;
};

#define DOCK_CONFIG_SETTINGS_SIZE sizeof(struct dock_config_settings)

struct dock_settings {
	u16_t heartbeat_period;				// (sec)
	u16_t heartbeat_led_period;			// (sec)
	u16_t keypress_period;				// (sec)
	u32_t sample_period;				// (sec)
	u32_t isb_sample_period;			// (sec)
	u32_t gnss_sample_period;			// (sec)
	u32_t dock_sample_period;			// (sec)
	u32_t status_period;				// (sec)
	u32_t reboot_period;				// (sec)
	u32_t log_file_sync_period;			// (sec)
};

#define DOCK_SETTINGS_SIZE sizeof(struct dock_settings)

#define SLEEP_TIME_SETTING_NAME			"system/sleep_period"
#define HEARTBEAT_PERIOD_SETTING_NAME		"system/heartbeat_period"
#define HEARTBEAT_LED_PERIOD_SETTING_NAME	"system/heartbeat_led_period"
#define SAMPLE_PERIOD_SETTING_NAME		"system/sample_period"
#define ISB_SAMPLE_PERIOD_SETTING_NAME		"isb/sample_period"
#define GNSS_SAMPLE_PERIOD_SETTING_NAME		"gnss/sample_period"
#define DOCK_SAMPLE_PERIOD_SETTING_NAME		"dock/sample_period"
#define WITAP_STATUS_PERIOD_SETTING_NAME	"status/sample_period"
#define WAIT_FOR_KEYPRESS_PERIOD_SETTING_NAME	"system/keypress_period"
#define LOG_SYNC_TIMEOUT_SETTING_NAME		"log/file/sync_period"
#define LOG_DATA_FILE_SETTING_NAME		"log/file/name"
#define REBOOT_PERIOD_SETTING_NAME		"system/reboot_period"
#define NETWORK_ENABLE_SETTING_NAME		"network/enable"
#define NETWORK_APP_EUI_SETTING_NAME		"network/app_eui"
#define NETWORK_APP_KEY_SETTING_NAME		"network/app_key"
#define NETWORK_IPV4_SETTING_NAME		"network/ipv4/address"
#define NETWORK_IPV4_SUBNET_MASK_SETTING_NAME	"network/ipv4/subnet_mask"
#define NETWORK_IPV4_ROUTER_SETTING_NAME	"network/ipv4/router"
#define NETWORK_IPV4_DNS_SETTING_NAME		"network/ipv4/dns"
#define NETWORK_IPV6_SETTING_NAME		"network/ipv6/address"


#ifdef __cplusplus
extern "C" {
#endif

/*  prototypes  */

#ifdef __cplusplus
}
#endif

#endif /* __WITAP_SETTINGS_H__ */