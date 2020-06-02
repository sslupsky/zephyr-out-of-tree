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

#include <getopt.h>

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

static int cmd_poll(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}
	
	drv_data = dev->driver_data;

	k_sem_give(&drv_data->msg_sem);

	return 0;
}

/***********************/

static int cmd_nav_pvt(const struct shell *shell)
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

	time.tm_year = drv_data->pvt.time.gpsYear - 1900;
	time.tm_mon = drv_data->pvt.time.gpsMonth - 1;
	time.tm_mday = drv_data->pvt.time.gpsDay;
	time.tm_hour = drv_data->pvt.time.gpsHour;
	time.tm_min = drv_data->pvt.time.gpsMinute;
	time.tm_sec = drv_data->pvt.time.gpsSecond;
	strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &time);
	shell_print(shell, "     Time:        %s", buf);

	return ret;
}

static int cmd_nav_status(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_STATUS, NULL);

	return ret;
}

static int cmd_nav_pvt_rate(const struct shell *shell, size_t argc, char **argv)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	u8_t rate;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;
	if (argc == 2) {
		rate = MIN(MAX(strtol(argv[1], NULL, 0), 1), 3600);
		gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_RATE_PVT, &rate);
	} else {
		gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_RATE_PVT, &rate);
	}

	shell_print(shell, "nav pvt rate: %d", rate);

	return ret;
}

/***********************/

static int cmd_info_version(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;
	u8_t hw_ver[30] = "";
	u8_t sw_ver[30] = "";
	u8_t proto_ver[30] = "";

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_VERSION_HW, hw_ver);
	shell_print(shell, "hardware version: %s", hw_ver);
	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_VERSION_SW, sw_ver);
	shell_print(shell, "software version: %s", sw_ver);
	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_VERSION_PROTO, proto_ver);
	shell_print(shell, "protocol version: %s", proto_ver);

	return ret;
}

static int cmd_info_id(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_ID, &id);

	shell_print(shell, "unique id: %llx", id);
	return ret;
}

/***********************/

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

	gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_SLEEP, NULL);

	return ret;
}

/***********************/

static int cmd_port_ddc(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_prt_ddc port;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_PORT_I2C, &port);

	shell_print(shell, "txReady: %x", port.txReady);
	shell_print(shell, "mode: %x", port.mode);
	shell_print(shell, "In Proto Mask: %x", port.inProtoMask);
	shell_print(shell, "Out Proto Mask: %x", port.outProtoMask);
	shell_print(shell, "flags: %x", port.flags);

	return ret;
}

static int cmd_port_uart(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_prt_uart port;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_PORT_UART, &port);

	shell_print(shell, "txReady: %x", port.txReady);
	shell_print(shell, "mode: %x", port.mode);
	shell_print(shell, "Baud rate: %d", port.baudRate);
	shell_print(shell, "In Proto Mask: %x", port.inProtoMask);
	shell_print(shell, "Out Proto Mask: %x", port.outProtoMask);
	shell_print(shell, "flags: %x", port.flags);

	return ret;
}

/***********************/

static int cmd_rate(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	u16_t val;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_MEASUREMENT_RATE, &val);

	shell_print(shell, "Measurement rate: %d", val);
	return ret;
}

/***********************/

static int cmd_debug_idle_rate(const struct shell *shell, size_t argc, char **argv)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	int ret = 0;
	extern int debug_idle_rate;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	debug_idle_rate = MIN(MAX(strtol(argv[1], NULL, 0), 1), 3600);
	return ret;
}

/***********************/

static int cmd_gnss_info(const struct shell *shell, size_t argc, char *argv[])
{
	return 0;
}

static int cmd_gnss_port(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_gnss_nav(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_gnss_debug(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

/***********************/

static int cmd_gnss(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}


/***********************/

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

__unused
static int cmd_getopt_skeleton(const struct shell *shell, size_t argc, char **argv)
{
	int ch, ret;
	int c_option, n_option;
	bool q_option = false;
	int longoption1, longoption2;

	static struct option long_options[] = {
		{"longOption1",	required_argument,	0,	0},
		{"longOption2",	no_argument,		0,	0},
		{0,		0,			0,	0},
	};

	ARG_UNUSED(long_options);
	ARG_UNUSED(longoption1);
	ARG_UNUSED(longoption2);

	/* reset getopt() with optind=0 so that successive calls
	 * to getopts() parses the args.  Otherwise getops()
	 * remembers the last parse and returns -1 when
	 * tail is run more than once.
	 * see linux man page
	 * http://man7.org/linux/man-pages/man3/getopt.3.html
	 */
	optind = 0;

	while ((ch = getopt(argc, argv, "c:n:q")) != -1 ) {
		switch (ch) {
		case 'c':
			c_option = strtol(optarg, NULL, 0);
			break;
		case 'n':
			n_option = strtol(optarg, NULL, 0);
			break;
		case 'q':
			q_option = true;
			break;
		case '?':
		default:
			return -ENOTSUP;
		}
	}
	argc -= optind;
	argv += optind;

	return ret;
}

#define HELP_NONE "[none]"

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_info_cmds,
	SHELL_CMD(id, NULL, HELP_NONE, cmd_info_id),
	SHELL_CMD(version, NULL, HELP_NONE, cmd_info_version),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_port_cmds,
	SHELL_CMD(ddc, NULL, HELP_NONE, cmd_port_ddc),
	SHELL_CMD(uart, NULL, HELP_NONE, cmd_port_uart),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_nav_cmds,
	SHELL_CMD(status, NULL, HELP_NONE, cmd_nav_status),
	SHELL_CMD(pvt, NULL, HELP_NONE, cmd_nav_pvt),
	SHELL_CMD_ARG(pvtrate, NULL, HELP_NONE, cmd_nav_pvt_rate, 0, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_debug_cmds,
	SHELL_CMD_ARG(idlerate, NULL, HELP_NONE, cmd_debug_idle_rate, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_cmds,
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 1, 0),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_poll),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_sleep),
	SHELL_CMD(rate, NULL, HELP_NONE, cmd_rate),
	SHELL_CMD(info, &gnss_info_cmds, HELP_NONE, cmd_gnss_info),
	SHELL_CMD(port, &gnss_port_cmds, HELP_NONE, cmd_gnss_port),
	SHELL_CMD(nav, &gnss_nav_cmds, HELP_NONE, cmd_gnss_nav),
	SHELL_CMD(debug, &gnss_debug_cmds, HELP_NONE, cmd_gnss_debug),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gnss, &gnss_cmds, "GNSS shell commands", cmd_gnss);
