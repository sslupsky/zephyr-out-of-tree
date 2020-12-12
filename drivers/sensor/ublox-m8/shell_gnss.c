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
			drv_data->pvt.position.fix_status.gnssFixOK,
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
	shell_print(shell, "Fix: %svalid, TOW: %svalid, Week Number: %svalid",
		    drv_data->nav_status.fix_status.gpsFixOk ? "" : "not ",
		    drv_data->nav_status.fix_status.towSet ? "" : "not ",
		    drv_data->nav_status.fix_status.wknSet ? "" : "not ");
	shell_print(shell, "Fix: %d", drv_data->nav_status.fix);
	shell_print(shell, "Time of week: %d", drv_data->nav_status.tow);
	shell_print(shell, "Time to first fix: %d", drv_data->nav_status.ttff);
	return ret;
}

static int cmd_nav_meas_rate(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MEASUREMENT_RATE, &val);

	shell_print(shell, "Measurement rate: %d", val);
	return ret;
}

static int cmd_nav_nav_rate(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_SOLUTION_RATE, &val);

	shell_print(shell, "Measurement rate: %d", val);
	return ret;
}

static int cmd_nav_pvt_msg_rate(const struct shell *shell, size_t argc, char **argv)
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
		shell_print(shell, "setting rate to: %d", rate);
		gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_PVT_RATE, &rate);
	} else {
		gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_PVT_RATE, &rate);
	}

	shell_print(shell, "nav pvt message rate: %d", rate);

	return ret;
}

static int cmd_nav_sol_msg_rate(const struct shell *shell, size_t argc, char **argv)
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
		gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_SOL_RATE, &rate);
	} else {
		gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_SOL_RATE, &rate);
	}

	shell_print(shell, "nav solution message rate: %d", rate);
	return ret;
}

static int cmd_nav_status_msg_rate(const struct shell *shell, size_t argc, char **argv)
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
		shell_print(shell, "setting rate to: %d", rate);
		gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_STATUS_RATE, &rate);
	} else {
		gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_MSG_STATUS_RATE, &rate);
	}

	shell_print(shell, "nav status message rate: %d", rate);

	return ret;
}

static int cmd_nav_settings(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_nav5 val;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_NAV_SETTINGS, &val);

	shell_print(shell, "Navigation engine settings");
	shell_print(shell, "    mask:              %x", val.mask);
	shell_print(shell, "    dynMode1:          %d", val.dynMode1);
	shell_print(shell, "    fixMode:           %d", val.fixMode);
	shell_print(shell, "    fixedAlt:          %d", val.fixedAlt);
	shell_print(shell, "    fixedAltVar:       %d", val.fixedAltVar);
	shell_print(shell, "    minElev:           %d", val.minElev);
	shell_print(shell, "    drLimit:           %d", val.drLimit);
	shell_print(shell, "    pDop:              %d", val.pDop);
	shell_print(shell, "    tDop:              %d", val.tDop);
	shell_print(shell, "    pAcc:              %d", val.pAcc);
	shell_print(shell, "    tAcc:              %d", val.tAcc);
	shell_print(shell, "    staticHoldThresh:  %d", val.staticHoldThresh);
	shell_print(shell, "    dgnssTimeout:      %d", val.dgnssTimeout);
	shell_print(shell, "    cnoThreshNumSVs:   %d", val.cnoThreshNumSVs);
	shell_print(shell, "    cnoThresh:         %d", val.cnoThresh);
	shell_print(shell, "    staticHoldMaxDist: %d", val.staticHoldMaxDist);
	shell_print(shell, "    utcStandard:       %d", val.utcStandard);

	return ret;
}

static int cmd_nav_getfix(const struct shell *shell)
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
	k_poll_signal_raise(&drv_data->pvt_signal, 180000);

	return ret;
}

/***********************/

static int cmd_info_version(const struct shell *shell, size_t argc, char **argv)
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

	if (argc == 2) {
		if (strncmp(argv[1], "hw", 2) == 0) {
			gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_VERSION_HW, hw_ver);
			shell_print(shell, "hardware version: %s", hw_ver);
		}
		if (strncmp(argv[1], "sw", 2) == 0) {
			gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_VERSION_SW, sw_ver);
			shell_print(shell, "software version: %s", sw_ver);
		}
		if (strncmp(argv[1], "proto", 5) == 0) {
			gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_VERSION_PROTO, proto_ver);
			shell_print(shell, "protocol version: %s", proto_ver);
		}
	}

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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_ID, &id);

	shell_print(shell, "unique id: %llx", id);
	return ret;
}

static int cmd_info_hw(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_HW_STATUS, NULL);

	shell_print(shell, "Hardware status:");
	shell_print(shell, "        noise:    %d", sys_get_le16(&drv_data->ubx_get_buf[16]));
	shell_print(shell, "        pin sel:  %08x", sys_get_le32(&drv_data->ubx_get_buf[0]));
	shell_print(shell, "        pin bank: %08x", sys_get_le32(&drv_data->ubx_get_buf[4]));
	shell_print(shell, "        pin dir:  %08x", sys_get_le32(&drv_data->ubx_get_buf[8]));
	shell_print(shell, "        pin val:  %08x", sys_get_le32(&drv_data->ubx_get_buf[12]));
	shell_print(shell, "        pullH:    %08x", sys_get_le32(&drv_data->ubx_get_buf[52]));
	shell_print(shell, "        pullL:    %08x", sys_get_le32(&drv_data->ubx_get_buf[56]));
	shell_print(shell, "        vp mask:  %08x", sys_get_le32(&drv_data->ubx_get_buf[24]));
	shell_hexdump(shell, &drv_data->ubx_get_buf[28], 17);

	return ret;
}

static int cmd_info_hw2(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_HW2_STATUS, NULL);

	return ret;
}

static int cmd_info_io(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_IO_STATUS, NULL);

	shell_print(shell, "I/O system status:");
	for (u8_t i=0; i<6; i++) {
		shell_print(shell, "    Port %d", i);
		shell_print(shell, "        rx bytes:          %d", sys_get_le32(&drv_data->ubx_get_buf[0+i*20]));
		shell_print(shell, "        tx bytes:          %d", sys_get_le32(&drv_data->ubx_get_buf[4+i*20]));
		shell_print(shell, "        parity errors:     %d", sys_get_le16(&drv_data->ubx_get_buf[8+i*20]));
		shell_print(shell, "        framing errors:    %d", sys_get_le16(&drv_data->ubx_get_buf[10+i*20]));
		shell_print(shell, "        overrun errors:    %d", sys_get_le16(&drv_data->ubx_get_buf[12+i*20]));
		shell_print(shell, "        break conditions:  %d", sys_get_le16(&drv_data->ubx_get_buf[14+i*20]));
		shell_print(shell, "");
	}

	return ret;
}

static int cmd_info_gnss(const struct shell *shell)
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

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_INFO_GNSS_STATUS, NULL);

	shell_print(shell, "GNSS selection");
	shell_print(shell, "  Supported");
	shell_print(shell, "        GPS %ssupported", drv_data->ubx_get_buf[1] & BIT(0) ? "" : "not ");
	shell_print(shell, "        Glonass %ssupported", drv_data->ubx_get_buf[1] & BIT(1) ? "" : "not ");
	shell_print(shell, "        Beidou %ssupported", drv_data->ubx_get_buf[1] & BIT(2) ? "" : "not ");
	shell_print(shell, "        Galileo %ssupported", drv_data->ubx_get_buf[1] & BIT(3) ? "" : "not ");
	shell_print(shell, "  Default");
	shell_print(shell, "        GPS %senabled", drv_data->ubx_get_buf[2] & BIT(0) ? "" : "not ");
	shell_print(shell, "        Glonass %senabled", drv_data->ubx_get_buf[2] & BIT(1) ? "" : "not ");
	shell_print(shell, "        Beidou %senabled", drv_data->ubx_get_buf[2] & BIT(2) ? "" : "not ");
	shell_print(shell, "        Galileo %senabled", drv_data->ubx_get_buf[2] & BIT(3) ? "" : "not ");
	shell_print(shell, "  Enabled");
	shell_print(shell, "        GPS %senabled", drv_data->ubx_get_buf[3] & BIT(0) ? "" : "not ");
	shell_print(shell, "        Glonass %senabled", drv_data->ubx_get_buf[3] & BIT(1) ? "" : "not ");
	shell_print(shell, "        Beidou %senabled", drv_data->ubx_get_buf[3] & BIT(2) ? "" : "not ");
	shell_print(shell, "        Galileo %senabled", drv_data->ubx_get_buf[3] & BIT(3) ? "" : "not ");
	shell_print(shell, "  Maximum concurrent gnss: %d", drv_data->ubx_get_buf[4]);
	return ret;
}

/***********************/

static int cmd_pm_sleep(const struct shell *shell)
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

	gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_PM_SLEEP, NULL);

	return ret;
}

static int cmd_pm_status(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_pm2 *pm;
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;
	pm = (struct ubx_payload_cfg_pm2 *)(drv_data->ubx_get_buf);

	gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_PM_STATUS, NULL);

	shell_print(shell, "Power management configuration");
	shell_print(shell, "    Max Startup State Duration:  %d", pm->maxStartupStateDur);
	shell_print(shell, "    update period:               %d", pm->updatePeriod);
	shell_print(shell, "    search period:               %d", pm->searchPeriod);
	shell_print(shell, "    grid offset:                 %d", pm->gridOffset);
	shell_print(shell, "    onTime:                      %d", pm->onTime);
	shell_print(shell, "    min acqusition time:         %d", pm->minAcqTime);
	shell_print(shell, "    extint inactivity:           %d", pm->extintInactivityMs);
	shell_print(shell, "    flags:");
	shell_print(shell, "        extintSel:               %d", pm->flags.extintSel);
	shell_print(shell, "        extintWake:              %d", pm->flags.extintWake);
	shell_print(shell, "        extintBackup:            %d", pm->flags.extintBackup);
	shell_print(shell, "        limitPeakCurr:           %d", pm->flags.limitPeakCurr);
	shell_print(shell, "        waitTimeFix:             %d", pm->flags.waitTimeFix);
	shell_print(shell, "        updateRTC:               %d", pm->flags.updateRTC);
	shell_print(shell, "        updateEPH:               %d", pm->flags.updateEPH);
	shell_print(shell, "        doNotEnterOff:           %d", pm->flags.doNotEnterOff);
	shell_print(shell, "        mode:                    %d", pm->flags.mode);

	return ret;
}

static int cmd_pm_mode(const struct shell *shell, size_t argc, char **argv)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_pms mode = {
		.version = 0,
		.powerSetupValue = 2,
	};
	int ret = 0;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}

	drv_data = dev->driver_data;

	if (argc == 3) {
		mode.period = MIN(MAX(strtol(argv[1], NULL, 0), 1), 3600);
		mode.onTime = MIN(MAX(strtol(argv[2], NULL, 0), 1), 3600);
		shell_print(shell, "period %d, onTime %d", mode.period, mode.onTime);
		gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_PM_MODE, &mode);
	} else {
		gnss_attr_get(dev, GNSS_CHAN_NONE, GNSS_ATTR_PM_MODE, NULL);
	}
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

static int cmd_gnss_poll(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}
	
	drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config_info;

	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
	k_sleep(K_MSEC(500));
	k_sem_give(&drv_data->msg_sem);
	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);

	return 0;
}

static int cmd_gnss_save(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_cfg payload = {
		.saveMask.ioPort = 1,
		.saveMask.msgConf = 1,
		.saveMask.infMsg = 1,
		.saveMask.navConf = 1,
		.saveMask.rxmConf = 1,
		.deviceMask.devBBR = 1,
	};

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}
	
	drv_data = dev->driver_data;

	gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_SAVE, &payload);

	return 0;
}

static int cmd_gnss_load(const struct shell *shell)
{
	struct device *dev;
	struct ublox_m8_data *drv_data;
	struct ubx_payload_cfg_cfg payload = {
		.loadMask.ioPort = 1,
		.loadMask.msgConf = 1,
		.loadMask.infMsg = 1,
		.loadMask.navConf = 1,
		.loadMask.rxmConf = 1,
		.deviceMask.devBBR = 1,
	};

	dev = device_get_binding(GPS_DEVICE_NAME);
	if (!dev) {
		shell_error(shell, "device not found: %s", GPS_DEVICE_NAME);
		return -EIO;
	}
	
	drv_data = dev->driver_data;

	gnss_attr_set(dev, GNSS_CHAN_NONE, GNSS_ATTR_SAVE, &payload);

	return 0;
}

static int cmd_gnss_info(const struct shell *shell, size_t argc, char *argv[])
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_gnss_pm(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
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
	SHELL_CMD(gnss, NULL, HELP_NONE, cmd_info_gnss),
	SHELL_CMD(hw, NULL, HELP_NONE, cmd_info_hw),
	SHELL_CMD(hw2, NULL, HELP_NONE, cmd_info_hw2),
	SHELL_CMD(id, NULL, HELP_NONE, cmd_info_id),
	SHELL_CMD(io, NULL, HELP_NONE, cmd_info_io),
	SHELL_CMD_ARG(version, NULL, HELP_NONE, cmd_info_version, 0, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_port_cmds,
	SHELL_CMD(ddc, NULL, HELP_NONE, cmd_port_ddc),
	SHELL_CMD(uart, NULL, HELP_NONE, cmd_port_uart),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_pm_cmds,
	SHELL_CMD(mode, NULL, HELP_NONE, cmd_pm_mode),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_pm_sleep),
	SHELL_CMD(status, NULL, HELP_NONE, cmd_pm_status),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_nav_cmds,
	SHELL_CMD(getfix, NULL, HELP_NONE, cmd_nav_getfix),
	SHELL_CMD(measRate, NULL, HELP_NONE, cmd_nav_meas_rate),
	SHELL_CMD(navRate, NULL, HELP_NONE, cmd_nav_nav_rate),
	SHELL_CMD(pvt, NULL, HELP_NONE, cmd_nav_pvt),
	SHELL_CMD_ARG(pvtRate, NULL, HELP_NONE, cmd_nav_pvt_msg_rate, 0, 1),
	SHELL_CMD(settings, NULL, HELP_NONE, cmd_nav_settings),
	SHELL_CMD(solRate, NULL, HELP_NONE, cmd_nav_sol_msg_rate),
	SHELL_CMD(status, NULL, HELP_NONE, cmd_nav_status),
	SHELL_CMD_ARG(statusRate, NULL, HELP_NONE, cmd_nav_status_msg_rate, 0, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_debug_cmds,
	SHELL_CMD_ARG(idlerate, NULL, HELP_NONE, cmd_debug_idle_rate, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(gnss_cmds,
	SHELL_CMD(debug, &gnss_debug_cmds, HELP_NONE, cmd_gnss_debug),
	SHELL_CMD(info, &gnss_info_cmds, HELP_NONE, cmd_gnss_info),
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 1, 0),
	SHELL_CMD(load, NULL, HELP_NONE, cmd_gnss_load),
	SHELL_CMD(pm, &gnss_pm_cmds, HELP_NONE, cmd_gnss_pm),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_gnss_poll),
	SHELL_CMD(port, &gnss_port_cmds, HELP_NONE, cmd_gnss_port),
	SHELL_CMD(nav, &gnss_nav_cmds, HELP_NONE, cmd_gnss_nav),
	SHELL_CMD(save, NULL, HELP_NONE, cmd_gnss_save),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(gnss, &gnss_cmds, "GNSS shell commands", cmd_gnss);
