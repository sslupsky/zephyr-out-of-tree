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

struct witap_app_key {
	uint16_t net_idx;
	uint16_t app_idx;
	bool  updated;
	struct witap_app_keys {
		uint8_t id;
		uint8_t val[16];
	} keys[2];
};

/* witap.flags */
enum {
	WITAP_VALID,           /* We have been provisioned */
	WITAP_SUSPENDED,       /* Network is temporarily suspended */
	WITAP_IVU_IN_PROGRESS, /* IV Update in Progress */
	WITAP_IVU_INITIATOR,   /* IV Update initiated by us */
	WITAP_IVU_TEST,        /* IV Update test mode */
	WITAP_IVU_PENDING,     /* Update blocked by SDU in progress */

	/* pending storage actions, must reside within first 32 flags */
	WITAP_RPL_PENDING,
	WITAP_KEYS_PENDING,
	WITAP_NET_PENDING,
	WITAP_IV_PENDING,
	WITAP_SEQ_PENDING,
	WITAP_HB_PUB_PENDING,
	WITAP_CFG_PENDING,
	WITAP_MOD_PENDING,
	WITAP_VA_PENDING,

	/* Don't touch - intentionally last */
	WITAP_FLAG_COUNT,
};

#define CONFIG_WITAP_APP_KEY_COUNT	1
#define WITAP_KEY_UNUSED		0xffff

struct witap_net {
	uint32_t iv_index; /* Current IV Index */
	uint32_t seq;      /* Next outgoing sequence number (24 bits) */

	ATOMIC_DEFINE(flags, WITAP_FLAG_COUNT);

	/* Local network interface */
	struct k_work local_work;
	sys_slist_t local_queue;

	/* Number of hours in current IV Update state */
	uint8_t  ivu_duration;

	/* Timer to track duration in current IV Update state */
	struct k_delayed_work ivu_timer;

	uint8_t dev_key[16];

	struct witap_app_key app_keys[CONFIG_WITAP_APP_KEY_COUNT];

};

struct system_config_settings_struct {
	u32_t sample_isb : 1;
	u32_t sample_dock : 1;
	u32_t sample_gnss : 1;
	u32_t sample_status : 1;
	u32_t reserved_sample : 3;
	u32_t sample : 1;
	u32_t log_data : 1;
	u32_t log_console : 1;
	u32_t reserved_log : 5;
	u32_t log_enable : 1;
	u32_t network_send : 1;
	u32_t reserved_network : 7;
	u32_t reserved : 8;
};

union system_config_settings_t {
	struct system_config_settings_struct bit;
	uint32_t reg;
};

#define SYSTEM_CONFIG_SETTINGS_SIZE sizeof(struct system_config_settings)

struct system_settings_t {
	u32_t sample_period;				// (sec)
	u32_t status_period;				// (sec)
	u32_t reboot_period;				// (sec)
	u16_t sleep_period;
	u16_t heartbeat_period;				// (sec)
	u16_t heartbeat_led_period;			// (sec)
	u16_t keypress_period;				// (sec)
	union system_config_settings_t config;
};

struct dock_settings_t {
	u32_t adc_sample_period;
	u32_t temp_sample_period;
	u32_t battery_sample_period;	
};

struct log_settings_t {
	u32_t file_sync_period;			// (sec)
};

struct network_settings_t {
};

struct witap_isb_settings_t {
	u32_t sample_period;			// (sec)
};

struct gnss_settings_t {
	u32_t sample_period;			// (sec)
};


#define DOCK_SETTINGS_SIZE sizeof(struct system_settings)

#define SYSTEM_SETTING_NAME			"system"
#define SAMPLE_PERIOD_SETTING_NAME		"system/sample_period"
#define STATUS_PERIOD_SETTING_NAME		"system/status_period"
#define REBOOT_PERIOD_SETTING_NAME		"system/reboot_period"
#define SLEEP_PERIOD_SETTING_NAME		"system/sleep_period"
#define HEARTBEAT_PERIOD_SETTING_NAME		"system/heartbeat_period"
#define HEARTBEAT_LED_PERIOD_SETTING_NAME	"system/heartbeat_led_period"
#define WAIT_FOR_KEYPRESS_PERIOD_SETTING_NAME	"system/keypress_period"
#define SYSTEM_CONFIG_SETTING_NAME		"system/config"

#define DOCK_SETTING_NAME			"dock"
#define DOCK_ADC_SAMPLE_PERIOD_SETTING_NAME	"dock/adc/sample_period"
#define DOCK_TEMP_SAMPLE_PERIOD_SETTING_NAME	"dock/temp/sample_period"
#define DOCK_BATTERY_SAMPLE_PERIOD_SETTING_NAME "dock/battery/sample_period"

#define ISB_SETTING_NAME			"isb"
#define ISB_SAMPLE_PERIOD_SETTING_NAME		"isb/sample_period"

#define GNSS_SETTING_NAME			"gnss"
#define GNSS_SAMPLE_PERIOD_SETTING_NAME		"gnss/sample_period"

#define LOG_SETTING_NAME			"log"
#define LOG_SYNC_TIMEOUT_SETTING_NAME		"log/file/sync_period"
#define LOG_DATA_FILE_SETTING_NAME		"log/file/name"

#define NETWORK_SETTING_NAME			"network"
#define NETWORK_ENABLE_SETTING_NAME		"network/enable"
#define NETWORK_APP_EUI_SETTING_NAME		"network/app_eui"
#define NETWORK_APP_KEY_SETTING_NAME		"network/app_key"
#define NETWORK_IPV4_SETTING_NAME		"network/ipv4/address"
#define NETWORK_IPV4_SUBNET_MASK_SETTING_NAME	"network/ipv4/subnet_mask"
#define NETWORK_IPV4_ROUTER_SETTING_NAME	"network/ipv4/router"
#define NETWORK_IPV4_DNS_SETTING_NAME		"network/ipv4/dns"
#define NETWORK_IPV6_SETTING_NAME		"network/ipv6/address"
#define NETWORK_SEND_SETTING_NAME		"network/send"

#define LORA_SETTING_NAME			"lora"

/* 1000 msec = 1 sec */
#define SLEEP_PERIOD_DEFAULT			5		// (sec)
#define HEARTBEAT_PERIOD_DEFAULT		1		// (sec)
#define HEARTBEAT_LED_PERIOD_DEFAULT		10		// (sec)
#define SAMPLE_PERIOD_DEFAULT			120		// (sec)
#define ISB_SAMPLE_PERIOD_DEFAULT		300
#define GNSS_SAMPLE_PERIOD_DEFAULT		3600
#define DOCK_SAMPLE_PERIOD_DEFAULT		120
#define WITAP_STATUS_PERIOD_DEFAULT		3600		// (sec)
#define WAIT_FOR_KEYPRESS_PERIOD_DEFAULT	30		// (sec)
#define WITAP_LOG_SYNC_TIMEOUT_DEFAULT		1200		// (sec) Time between file sync's
#define REBOOT_PERIOD_DEFAULT			14400		// (sec) The amount of time to wait for reboot


#define SYSTEM_CONFIG_DEFAULT { 	\
	.bit = {			\
		.sample_isb = true,	\
		.sample_dock = true,	\
		.sample_gnss = false,	\
		.sample_status = true,	\
		.sample = true,		\
		.log_data = true,	\
		.log_console = false,	\
		.network_send = false,	\
	},				\
}

#define SYSTEM_SETTINGS_DEFAULT {				\
	.sample_period = SAMPLE_PERIOD_DEFAULT,			\
	.status_period = WITAP_STATUS_PERIOD_DEFAULT,		\
	.reboot_period = REBOOT_PERIOD_DEFAULT,			\
	.sleep_period = SLEEP_PERIOD_DEFAULT,			\
	.heartbeat_period = HEARTBEAT_PERIOD_DEFAULT,		\
	.heartbeat_led_period = HEARTBEAT_LED_PERIOD_DEFAULT,	\
	.keypress_period = WAIT_FOR_KEYPRESS_PERIOD_DEFAULT,	\
	.config = SYSTEM_CONFIG_DEFAULT,			\
}

#define DOCK_SETTINGS_DEFAULT {					\
	.adc_sample_period = DOCK_SAMPLE_PERIOD_DEFAULT,	\
	.temp_sample_period = 0,				\
	.battery_sample_period = 0,				\
}

#define ISB_SETTINGS_DEFAULT {					\
	.sample_period = ISB_SAMPLE_PERIOD_DEFAULT,		\
}

#define GNSS_SETTINGS_DEFAULT {					\
	.sample_period = GNSS_SAMPLE_PERIOD_DEFAULT,		\
}

#define LOG_SETTINGS_DEFAULT {					\
	.file_sync_period = WITAP_LOG_SYNC_TIMEOUT_DEFAULT,	\
}

#ifdef __cplusplus
extern "C" {
#endif

/*  prototypes  */

#ifdef __cplusplus
}
#endif

#endif /* __WITAP_SETTINGS_H__ */