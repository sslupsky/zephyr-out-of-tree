/**
 * @file sam0_rtc.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * 
 * @version 0.1
 * @date 2021-05-09
 * 
 * @copyright Copyright (c) 2021
 * SPDX-License-Identifier: Apache-2.0
 * 
 * 
   _____                 _                _        _          
  / ____|               (_)              | |      (_)         
 | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
  \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
  ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
                                                              
 */

/*  TODO:  The compatible needs to change since it is the same as 
 *         the compatible used for the sam0_rtc_timer driver
 */
#define DT_DRV_COMPAT atmel_sam0_rtc

static volatile int64_t rtc_boot_time;
static volatile int64_t uptime;

static void rtc_reset(void) {
	uptime = k_uptime_get();
}

int64_t rtc_boot_time(void) {
	return rtc_boot_time;
}

int __weak rtc_device_ctrl(struct device *device, uint32_t ctrl_command,
			       void *context, device_pm_cb cb, void *arg)
{
	return -ENOTSUP;
}

SYS_DEVICE_DEFINE("rtc_timer", rtc_driver_init, rtc_device_ctrl,
		PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
