/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-26
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


#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

#include <device.h>
#include <devicetree.h>
#include <settings/settings.h>

#include <shell/shell.h>
#include "ublox_m8.h"

#define GPS_DEVICE_NAME		DT_PROP(DT_INST(0,ublox_m8),label)
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

const struct shell *ctx_shell;

static void gnss_ready(int err)
{
	if (err) {
		shell_error(ctx_shell, "GNSS init failed (err %d)", err);
		return;
	}

	shell_print(ctx_shell, "GNSS initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
}

static int cmd_init(const struct shell *shell, size_t argc, char *argv[])
{
	int ret;

	ctx_shell = shell;

	/* TODO:  add gnss_enable() to gnss api ? */
	// ret = gnss_enable(gnss_ready);
	ret = 0;
	if (ret) {
		shell_error(shell, "GNSS init failed (err %d)", ret);
	}

	return ret;
}

static int cmd_info(const struct shell *shell, size_t argc, char *argv[])
{
	return 0;
}

static int cmd_poll(const struct shell *shell)
{
	int ret;
	struct device *dev;
	struct ublox_m8_data *drv_data;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
	} else {
		drv_data = dev->driver_data;
		drv_data->poll_handler(dev, &drv_data->poll_trigger);
	}

	return 0;
}

#define HELP_NONE "[none]"

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_cmds,
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 1, 0),
	SHELL_CMD_ARG(info, NULL, HELP_NONE, cmd_info, 1, 2),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_poll),
	SHELL_SUBCMD_SET_END
);

static int cmd_gnss(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

SHELL_CMD_REGISTER(gnss, &gnss_cmds, "GNSS shell commands", cmd_gnss);
