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
#include <time.h>

#include <device.h>
#include <devicetree.h>
#include <settings/settings.h>

#include <shell/shell.h>
#include "ublox_m8.h"

#define GPS_DEVICE_NAME		DT_PROP(DT_INST(0,ublox_m8),label)
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

const struct shell *ctx_shell;

__unused
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
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret;

	ctx_shell = shell;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	/* TODO:  add gnss_enable() to gnss api ? */
	// ret = gnss_enable(gnss_ready);

	// ublox_m8_trigger_set(dev, &txready_trigger, ublox_m8_txready_handler);

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

static int cmd_id(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;
	u64_t id;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	/* send header with empty payload  */
	// gnss_channel_get(dev, GNSS_CHAN_ID, &id);
	id = sys_be64_to_cpu(drv_data->device_id.be_word);

	shell_print(shell, "unique id: %llx", id);
	return ret;
}

static int cmd_pvt(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;
	struct gnss_pvt pvt;
	struct tm time;
	char buf[20];

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	/* send header with empty payload  */
	gnss_sample_fetch(dev);
	gnss_channel_get(dev, GNSS_CHAN_ALL, &pvt);
	shell_print(shell, "GNSS Fix Status:  gnssFixOK: %d, fixType: %d, SIV: %d, pDOP: %d",
			drv_data->pvt.position.fixflags & BIT(0),
			drv_data->pvt.position.fixType,
			drv_data->pvt.position.SIV,
			drv_data->pvt.velocity.pDOP);
	shell_print(shell, "     Position:    Lat: %d, Lon: %d, Alt: %d",
			drv_data->pvt.position.latitude,
			drv_data->pvt.position.longitude,
			drv_data->pvt.position.altitude);
	shell_print(shell, "     Speed:       %d",
			drv_data->pvt.velocity.ground_speed);
	// shell_print(shell, "     Time:        %02d:%02d:%02d",
	// 		drv_data->pvt.time.gpsHour,
	// 		drv_data->pvt.time.gpsMinute,
	// 		drv_data->pvt.time.gpsSecond);

	time.tm_year = drv_data->pvt.time.gpsYear;
	time.tm_mon = drv_data->pvt.time.gpsMonth;
	time.tm_hour = drv_data->pvt.time.gpsHour;
	time.tm_min = drv_data->pvt.time.gpsMinute;
	time.tm_sec = drv_data->pvt.time.gpsSecond;
	strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", time);
	shell_print(shell, "     Time:        %s", buf);

	return ret;
}

static int cmd_version(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	shell_print(shell, "software version: %s", drv_data->sw_version);
	shell_print(shell, "hardware version: %s", drv_data->hw_version);

	return ret;
}

static int cmd_sleep(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_sample_fetch_chan(dev, GNSS_CHAN_SLEEP);

	return ret;
}

static int cmd_ddc(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_sample_fetch_chan(dev, GNSS_CHAN_DDC);

	return ret;
}

__unused
static int cmd_skeleton(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	return ret;
}

#define HELP_NONE "[none]"

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_cmds,
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 1, 0),
	SHELL_CMD_ARG(info, NULL, HELP_NONE, cmd_info, 1, 2),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_poll),
	SHELL_CMD(pvt, NULL, HELP_NONE, cmd_pvt),
	SHELL_CMD(id, NULL, HELP_NONE, cmd_id),
	SHELL_CMD(version, NULL, HELP_NONE, cmd_version),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_sleep),
	SHELL_CMD(ddc, NULL, HELP_NONE, cmd_ddc),
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
