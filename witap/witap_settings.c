/**
 * @file witap_settings.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-18
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
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 *  references:
 *    subsys/bluetooth/mesh/settings.c
 *    subsys/bluetooth/mesh/settings.h
 *    subsys/bluetooth/mesh/net.h
 */

#include <zephyr.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/util.h>
#include <sys/byteorder.h>

#include <settings/settings.h>
#include <logging/log.h>

#include "witap_settings.h"

LOG_MODULE_DECLARE(witap, LOG_LEVEL_INF);

#define CONFIG_WITAP_STORE_TIMEOUT 2

static struct k_delayed_work pending_store;

struct system_settings_t settings = SYSTEM_SETTINGS_DEFAULT;

struct dock_settings_t dock_settings = DOCK_SETTINGS_DEFAULT;

struct witap_isb_settings_t witap_isb_settings = ISB_SETTINGS_DEFAULT;

struct gnss_settings_t gnss_settings = GNSS_SETTINGS_DEFAULT;

struct log_settings_t log_settings = LOG_SETTINGS_DEFAULT;

struct witap_net witap = {
	.local_queue = SYS_SLIST_STATIC_INIT(&witap.local_queue),
	.app_keys = {
		[0 ... (CONFIG_WITAP_APP_KEY_COUNT - 1)] = {
			.net_idx = WITAP_KEY_UNUSED,
		}
	},
};

/**
 * @brief read a setting from persistent storage
 * 
 * @param read_cb 
 * @param cb_arg 
 * @param out 
 * @param read_len 
 * @return int 
 */
static inline int settings_x_set(settings_read_cb read_cb, void *cb_arg, void *out,
			     size_t read_len)
{
	ssize_t len;

	len = read_cb(cb_arg, out, read_len);
	if (len < 0) {
		LOG_ERR("Failed to read value (err %zd)", len);
		return len;
	}

	LOG_HEXDUMP_DBG(out, len, "val");

	if ((size_t) len != read_len) {
		LOG_ERR("Unexpected value length (%zd != %zu)", len, read_len);
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief copy system sleep setting from persistent storage to setting ram
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int system_sleep_time_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	uint8_t buf[sizeof(settings.sleep_period)];
	int err;

	if (len_rd == 0) {
		LOG_DBG("val (null)");

		settings.sleep_period = SLEEP_PERIOD_DEFAULT;
		return 0;
	}

	err = settings_x_set(read_cb, cb_arg, buf, sizeof(buf));
	if (err) {
		LOG_ERR("Failed to set \'net\'");
		return err;
	}

	settings.sleep_period = sys_get_le32(buf);

	LOG_DBG("sleep period: %d s", settings.sleep_period);

	return 0;
}

/**
 * @brief copy config setting from persistent storage to setting ram
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int system_config_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	uint8_t buf[sizeof(settings.config)];
	int err;

	if (len_rd == 0) {
		LOG_DBG("val (null)");

		settings.config = (union system_config_settings_t) SYSTEM_CONFIG_DEFAULT;
		return 0;
	}

	err = settings_x_set(read_cb, cb_arg, buf, sizeof(buf));
	if (err) {
		LOG_ERR("Failed to set \'net\'");
		return err;
	}

	settings.config.reg = sys_get_le32(buf);

	LOG_DBG("sleep period: %d s", settings.sleep_period);

	return 0;
}

/**
 * @brief copy system settings from persistent storage to setting ram
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int system_settings_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	struct system_settings_t buf;
	int err;

	if (len_rd == 0) {
		LOG_DBG("val (null)");

		settings = (struct system_settings_t) SYSTEM_SETTINGS_DEFAULT;
		return 0;
	}

	err = settings_x_set(read_cb, cb_arg, &buf, sizeof(buf));
	if (err) {
		LOG_ERR("Failed to set \'net\'");
		return err;
	}

	settings = buf;

	LOG_HEXDUMP_DBG(&settings, sizeof(settings), "system settings");

	return 0;
}

/**
 * @brief copy dock adc setting from persistent storage to setting ram
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int dock_adc_sample_period_set(const char *name, size_t len_rd, settings_read_cb read_cb,
		   void *cb_arg)
{
	uint8_t buf[sizeof(dock_settings.adc_sample_period)];
	int err;

	if (len_rd == 0) {
		LOG_DBG("val (null)");

		dock_settings.adc_sample_period = DOCK_SAMPLE_PERIOD_DEFAULT;
		return 0;
	}

	err = settings_x_set(read_cb, cb_arg, buf, sizeof(buf));
	if (err) {
		LOG_ERR("Failed to set \'net\'");
		return err;
	}

	dock_settings.adc_sample_period = sys_get_le32(buf);

	LOG_DBG("dock adc sample period: %d s", dock_settings.adc_sample_period);

	return 0;
}

const struct settings_map_t {
	const char *name;
	int (*func)(const char *name, size_t len_rd,
		    settings_read_cb read_cb, void *cb_arg);
} settings_map[] = {
	{ SLEEP_PERIOD_SETTING_NAME, system_sleep_time_set },
	{ SYSTEM_CONFIG_SETTING_NAME, system_config_set },
	{ DOCK_ADC_SAMPLE_PERIOD_SETTING_NAME, dock_adc_sample_period_set },
/*
	{ SAMPLE_PERIOD_SETTING_NAME, <<func>> },
	{ STATUS_PERIOD_SETTING_NAME, <<func>> },
	{ REBOOT_PERIOD_SETTING_NAME, <<func>> },
	{ SLEEP_PERIOD_SETTING_NAME, <<func>> },
	{ HEARTBEAT_PERIOD_SETTING_NAME, <<func>> },
	{ HEARTBEAT_LED_PERIOD_SETTING_NAME, <<func>> },
	{ WAIT_FOR_KEYPRESS_PERIOD_SETTING_NAME, <<func>> },
	{ SYSTEM_CONFIG_SETTING_NAME, <<func>> },

	{ DOCK_ADC_SAMPLE_PERIOD_SETTING_NAME, <<func>> },
	{ DOCK_TEMP_SAMPLE_PERIOD_SETTING_NAME, <<func>> },
	{ DOCK_BATTERY_SAMPLE_PERIOD_SETTING_NAME, <<func>> },

	{ ISB_SAMPLE_PERIOD_SETTING_NAME, <<func>> },
	{ GNSS_SAMPLE_PERIOD_SETTING_NAME, <<func>> },

	{ LOG_SYNC_TIMEOUT_SETTING_NAME, <<func>> },
	{ LOG_DATA_FILE_SETTING_NAME, <<func>> },

	{ NETWORK_ENABLE_SETTING_NAME, <<func>> },
	{ NETWORK_APP_EUI_SETTING_NAME, <<func>> },
	{ NETWORK_APP_KEY_SETTING_NAME, <<func>> },
	{ NETWORK_IPV4_SETTING_NAME, <<func>> },
	{ NETWORK_IPV4_SUBNET_MASK_SETTING_NAME, <<func>> },
	{ NETWORK_IPV4_ROUTER_SETTING_NAME, <<func>> },
	{ NETWORK_IPV4_DNS_SETTING_NAME, <<func>> },
	{ NETWORK_IPV6_SETTING_NAME, <<func>> },
*/
};


static int settings_set(const char *name, size_t len_rd,
		    settings_read_cb read_cb, void *cb_arg)
{
	int i, len;
	const char *next;

	if (!name) {
		LOG_ERR("Insufficient number of arguments");
		return -EINVAL;
	}

	len = settings_name_next(name, &next);

	for (i = 0; i < ARRAY_SIZE(settings_map); i++) {
		if (!strncmp(settings_map[i].name, name, len)) {
			return settings_map[i].func(next, len_rd, read_cb, cb_arg);
		}
	}

	LOG_WRN("No matching handler for key %s", log_strdup(name));

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(system, "system", NULL, settings_set, NULL,
			       NULL);

/* Pending flags that use K_NO_WAIT as the storage timeout */
#define NO_WAIT_PENDING_BITS (BIT(WITAP_NET_PENDING) |           \
			      BIT(WITAP_IV_PENDING) |            \
			      BIT(WITAP_SEQ_PENDING))

/* Pending flags that use CONFIG_WITAP_STORE_TIMEOUT */
#define GENERIC_PENDING_BITS (BIT(WITAP_KEYS_PENDING) |          \
			      BIT(WITAP_HB_PUB_PENDING) |        \
			      BIT(WITAP_CFG_PENDING) |           \
			      BIT(WITAP_MOD_PENDING))

static void schedule_store(int flag)
{
	int32_t timeout_ms, remaining;

	atomic_set_bit(witap.flags, flag);

	if (atomic_get(witap.flags) & NO_WAIT_PENDING_BITS) {
		timeout_ms = 0;
	} else {
		timeout_ms = CONFIG_WITAP_STORE_TIMEOUT * MSEC_PER_SEC;
	}

	remaining = k_delayed_work_remaining_get(&pending_store);
	if ((remaining > 0) && remaining < timeout_ms) {
		LOG_DBG("Not rescheduling due to existing earlier deadline");
		return;
	}

	LOG_DBG("Waiting %d seconds", timeout_ms / MSEC_PER_SEC);

	k_delayed_work_submit(&pending_store, K_MSEC(timeout_ms));
}

/**
 * @brief store dock adc sample period to persistent storage
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int store_pending_dock_config(void)
{
	int err;

	err = settings_save_one(DOCK_SETTING_NAME, &dock_settings, sizeof(dock_settings));

	if (err) {
		LOG_ERR("Failed to write setting: %d", err);
	}

	return err;
}

/**
 * @brief store system config to persistent storage
 * 
 * @param name 
 * @param len_rd 
 * @param read_cb 
 * @param cb_arg 
 * @return int 
 */
static int store_pending_system_config(void)
{
	int err;

	err = settings_save_one(SYSTEM_SETTING_NAME, &settings, sizeof(settings));

	if (err) {
		LOG_ERR("Failed to write setting: %d", err);
	}

	return err;
}

static void store_pending(struct k_work *work)
{
	LOG_DBG("");

	// if (atomic_test_and_clear_bit(witap.flags, WITAP_KEYS_PENDING)) {
	// 	store_pending_keys();
	// }

	// if (atomic_test_and_clear_bit(witap.flags, WITAP_NET_PENDING)) {
	// 	if (atomic_test_bit(witap.flags, WITAP_VALID)) {
	// 		store_pending_net();
	// 	} else {
	// 		clear_net();
	// 	}
	// }

	if (atomic_test_and_clear_bit(witap.flags, WITAP_CFG_PENDING)) {
		if (atomic_test_bit(witap.flags, WITAP_VALID)) {
			store_pending_system_config();
		} else {
			// clear_cfg();
		}
	}

	// if (atomic_test_and_clear_bit(witap.flags, WITAP_IV_PENDING)) {
	// 	if (atomic_test_bit(witap.flags, WITAP_VALID)) {
	// 		store_pending_iv();
	// 	} else {
	// 		clear_iv();
	// 	}
	// }

	// if (atomic_test_and_clear_bit(witap.flags, WITAP_MOD_PENDING)) {
	// 	witap_model_foreach(store_pending_mod, NULL);
	// }

	// if (IS_ENABLED(CONFIG_WITAP_ISB)) {
	// 	if (atomic_test_and_clear_bit(witap_isb.flags,
	// 				      WITAP_ISB_SUBNET_PENDING)) {
	// 		if (atomic_test_bit(witap_isb.flags,
	// 				    WITAP_ISB_VALID)) {
	// 			store_pending_cdb();
	// 		} else {
	// 			clear_cdb();
	// 		}
	// 	}

	// 	if (atomic_test_and_clear_bit(witap_isb.flags,
	// 				      WITAP_ISB_NODES_PENDING)) {
	// 		store_pending_cdb_nodes();
	// 	}

	// 	if (atomic_test_and_clear_bit(witap_isb.flags,
	// 				      WITAP_ISB_KEYS_PENDING)) {
	// 		store_pending_cdb_keys();
	// 	}
	// }
}

void witap_settings_init(void)
{
	k_delayed_work_init(&pending_store, store_pending);
}
