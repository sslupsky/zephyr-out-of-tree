/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-18
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

#define DT_DRV_COMPAT ublox_m8

#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gnss.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>
#include <devicetree.h>
// #include <net/buf.h>

#include "ublox_m8.h"

#define LOG_LEVEL CONFIG_GNSS_LOG_LEVEL
LOG_MODULE_REGISTER(UBLOX_M8, CONFIG_GNSS_LOG_LEVEL);

FOR_EACH(UBX_HEADER_CFG_DEFINE, cfg, msg, nav5, pm2, prt, pms, pwr, rate, rst, rxm);
FOR_EACH(UBX_HEADER_CFG_DEFINE, tp5);
FOR_EACH(UBX_HEADER_NAV_DEFINE, pvt, status, geofence, sat, timeutc);
FOR_EACH(UBX_HEADER_SEC_DEFINE, uniqid);
FOR_EACH(UBX_HEADER_ACK_DEFINE, ack, nak);
FOR_EACH(UBX_HEADER_RXM_DEFINE, pmreq);
FOR_EACH(UBX_HEADER_MON_DEFINE, gnss, hw2, hw, io, ver);

const struct ubx_frame_status ubx_frame_status_init = {
	.response_request = UBX_RESPONSE_NONE,
	.response_received = UBX_RESPONSE_NONE,
	.checksum_valid = false,
	.error = false,
};

const union ubx_cfg_prt_txready ubx_txready_config = {
	.bit.en = 1,
	.bit.pol = 0,
	.bit.pin = 6,
	.bit.thres = 1,
};

const union ubx_cfg_prt_output_protocol ubx_ddc_protocol = {
	.bit.ubx = 1,
};

/*
 * u-center configuration:
 * These settings were determined using "messsage view" of the
 * u-center application.
 * 
 * UART1 port: none in, none out, baud 460800, 8 N 1, 
 * B5 62 06 00 14 00
 * 01 00 00 00 D0 08 00 00   00 08 07 00 00 00 00 00
 * 00 00 00 00
 * 02 72
 * 
 * DDC port:  ubx in, ubx out, slave=42, txready, pio=6, thres 8
 * B5 62 06 00 14 00
 * 00 00 99 00 84 00 00 00   00 00 00 00 01 00 01 00
 * 00 00 00 00
 * 39 58
 * 
 * DDC port:  ubx in, ubx out, slave=42, txready, pio=6, thres 8, extended timeout
 * B5 62 06 00 14 00
 * 00 00 99 00 84 00 00 00   00 00 00 00 01 00 01 00
 * 02 00 00 00
 * 3B 60
 * 
 * Save configuration:  All settings saved to BBR
 * B5 62 06 09 0D 00
 * 00 00 00 00 FF FF 00 00   00 00 00 00 01
 * 1B A9
 * 
 * PM2 configuration:  extint0, extintWake, extintBackup, updateRTC, updateEPH, mode ON/OFF
 * B5 62 06 3B 30 00
 * 02 06 00 00 60 98 40 01   00 00 00 00 00 00 00 00
 * 00 00 00 00 02 00 00 00   2C 01 00 00 4F C1 03 00
 * 87 02 00 00 FF 00 00 00   64 40 01 00 00 00 00 00
 * 21 10
 * 
 * RXM configuration:  power save mode
 * B5 62 06 11 02 00
 * 08 01
 * 22 92
 * 
 * RXM pmreq:  v2, duration 10000ms, backup, wake extint0
 * B5 62 02 41 10 00
 * 00 00 00 00 10 27 00 00   02 00 00 00 20 00 00 00
 * AC 18
 */

// const u8_t ucenter_cfg_prt_uart1[UBX_PAYLOAD_CFG_PRT_SIZE] =	{0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// const u8_t ucenter_cfg_prt_ddc[UBX_PAYLOAD_CFG_PRT_SIZE] =	{0x00, 0x00, 0x99, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00};

// const u8_t ucenter_cfg_cfg_bbr[UBX_PAYLOAD_CFG_CFG_SIZE] =	{0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
// const u8_t ucenter_cfg_cfg_bbr[UBX_PAYLOAD_CFG_CFG_SIZE] =	{0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// const u8_t ucenter_cfg_pm2[UBX_PAYLOAD_CFG_PM2_SIZE] = 		{0x02, 0x06, 0x00, 0x00, 0x60, 0x98, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
// 								 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
// 								 0x64, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
// const u8_t ucenter_cfg_pm2[UBX_PAYLOAD_CFG_PM2_SIZE] = 		{0x02, 0x00, 0x00, 0x00, 0x60, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
// 								 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
// 								 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


/*
 * UBX_CFG_PRT
 * ucenter configuration:
 * portID: DDC, inProtoMask: ubx in, outProtoMask: ubx out, mode: slaveAddr=42, txReady: pio=6, thres 1 (8 bytes), flags: extendedTxTimeout
 * B5 62 06 00 14 00
 * 00 00 99 00 84 00 00 00   00 00 00 00 01 00 01 00
 * 02 00 00 00
 * 3B 60
 */
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_ddc = {
	.ddc.portID = 0,
	.ddc.txReady.en = 1,
	.ddc.txReady.pol = 0,
	.ddc.txReady.pin = 6,
	.ddc.txReady.thres = 1,
	.ddc.mode.slaveAddr = 0x42,
	.ddc.inProtoMask.inUbx = 1,
	.ddc.outProtoMask.outUbx = 1,
	.ddc.flags.extendedTxTimeout = 1,
};

#define UBX_PAYLOAD_CFG_PRT_POLL_DDC_SIZE 1
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_poll_ddc = {
	.ddc.portID = 0,
};

/*
 * UBX_CFG_PRT
 * ucenter configuration:
 * portID: UART1, inProtoMask: none, outProtoMask: none, baud: 460800, mode: 8 N 1 
 * B5 62 06 00 14 00
 * 01 00 00 00 D0 08 00 00   00 08 07 00 00 00 00 00
 * 00 00 00 00
 * 02 72
 */
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_uart1 = {
	.uart.portID = 1,
	.uart.txReady.en = 0,
	.uart.mode.charLen = 3,
	.uart.mode.parity = 4,
	.uart.mode.nStopBits = 0,
	.uart.baudRate = 460800,
};

#define UBX_PAYLOAD_CFG_PRT_POLL_UART1_SIZE 1
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_poll_uart1 = {
	.uart.portID = 1,
};

/*
 * UBX_CFG_PM2
 * ucenter configuration:
 * extint0, extintWake, extintBackup, updateRTC, updateEPH, mode ON/OFF
 * B5 62 06 3B 30 00
 * 02 06 00 00 60 98 40 01   00 00 00 00 00 00 00 00
 * 00 00 00 00 02 00 00 00   2C 01 00 00 4F C1 03 00
 * 87 02 00 00 FF 00 00 00   64 40 01 00 00 00 00 00
 * 21 10
 */
const struct ubx_payload_cfg_pm2 ubx_payload_cfg_pm2 = {
	.version = 2,
	.flags.extintSel = 0,
	.flags.extintWake = 1,			// extint wakes the receiver (so we can read the registers)
	.flags.extintBackup = 1,		// extint puts receiver to sleep/backup
	.flags.extintInactive = 0,
	.flags.updateRTC = 1,			// wakes every 5 minutes to update the rtc
	.flags.updateEPH = 1,			// wakes every 30 minutes to update the ephemerides
	.flags.mode = 0,			// PSMOO mode
	.maxStartupStateDur = 0,		// 120 second max startup state duration
	.updatePeriod = 0,			// 1 hour update period
	.searchPeriod = 0,			// 5 minute search period
	.gridOffset = 0,			// 
	.onTime = 0,
	.minAcqTime = 0,			// receiver determines the acquisition time
};

const struct ubx_payload_cfg_pms ubx_payload_cfg_pms = {
	.version = 0,
	.powerSetupValue = 2,
	.period = 10,
	.onTime = 2,
};

/*
 * UBX_CFG_CFG
 * ucenter configuration:
 * clearMask: <none>, saveMask: <all>, loadMask: <none>
 * B5 62 06 09 0D 00
 * 00 00 00 00 FF FF 00 00   00 00 00 00 01
 * 1B A9
 */
const struct ubx_payload_cfg_cfg ubx_payload_cfg_cfg_save = {
	.saveMask.ioPort = 1,
	.saveMask.msgConf = 1,
	.saveMask.infMsg = 1,
	.saveMask.navConf = 1,
	.saveMask.rxmConf = 1,
	.deviceMask.devBBR = 1,
};

/*
 * UBX_CFG_RXM
 * ucenter configuration:
 * lpMode: power save mode
 * B5 62 06 11 02 00
 * 08 01
 * 22 92
 */
const struct ubx_payload_cfg_rxm ubx_payload_cfg_rxm = {
	.lpMode = 1,
};

/*
 * UBX_RXM_PMREQ
 * ucenter configuration
 * version: 0, duration: 10000ms, flags: backup, wakeupSources: extint0
 * B5 62 02 41 10 00
 * 00 00 00 00 10 27 00 00   02 00 00 00 20 00 00 00
 * AC 18
 */
const struct ubx_payload_rxm_pmreq ubx_payload_rxm_pmreq = {
	.version = 0,
	.duration = sys_cpu_to_le32(10000),
	.flags.backup = 1,
	.wakeupSources.extint0 = 1,
};

const struct ubx_payload_cfg_tp5 ubx_payload_cfg_tp5 = {
	.tpIdx = 0,
};

const struct ubx_payload_cfg_nav5 ubx_payload_cfg_nav5 = {
	.mask.dyn = 1,
	.mask.staticHoldMask = 1,
	.dynMode1 = 2,			// stationary model
	.staticHoldThresh = 255,	// 255 cm/s
	.staticHoldMaxDist = 200,	// 200m
};

const struct ubx_payload_cfg_rst ubx_payload_cfg_rst = {
	.navBbrMask = {},
	.resetMode = 2,
};

int debug_idle_rate = 60;

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
static void ubx_frame_checksum(struct ubx_frame *frame)
{
	frame->checksum.A = 0;
	frame->checksum.B = 0;

	frame->checksum.A += frame->header.class;
	frame->checksum.B += frame->checksum.A;

	frame->checksum.A += frame->header.id;
	frame->checksum.B += frame->checksum.A;

	frame->checksum.A += (frame->header.len[0]);
	frame->checksum.B += frame->checksum.A;

	frame->checksum.A += (frame->header.len[1]);
	frame->checksum.B += frame->checksum.A;

	for (uint16_t i = 0; i < frame->len; i++)
	{
		frame->checksum.A += frame->payload[i];
		frame->checksum.B += frame->checksum.A;
	}
}

__unused
static void ubx_rolling_checksum(u8_t incoming, bool reset)
{
	static u8_t checksumA;
	static u8_t checksumB;

	if (reset) {
		checksumA = 0;
		checksumB = 0;
	}
	checksumA += incoming;
	checksumB += checksumA;
}

static int ubx_validate_frame_checksum(struct ubx_frame *frame)
{
	u8_t checksumA = 0;
	u8_t checksumB = 0;
	u8_t len = sys_get_le16(frame->header.len);

	checksumA += frame->header.class;
	checksumB += checksumA;

	checksumA += frame->header.id;
	checksumB += checksumA;

	checksumA += (frame->header.len[0]);
	checksumB += checksumA;

	checksumA += (frame->header.len[1]);
	checksumB += checksumA;

	for (uint16_t i = 0; i < len; i++)
	{
		checksumA += frame->payload[i];
		checksumB += checksumA;
	}
	if (checksumA == frame->checksum.A && checksumB == frame->checksum.B) {
		frame->status.checksum_valid = true;
	} else {
		LOG_DBG("checksum failed: A:(%02x,%02x), B:(%02x,%02x)",
			checksumA, frame->checksum.A,
			checksumB, frame->checksum.B);
	}
	return frame->status.checksum_valid;
}

static int ublox_m8_flush_msg_buffer(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	u8_t rx_buf[64];
	u16_t len;

	len = 0;
	do {
		ret = i2c_read(drv_data->i2c, &rx_buf[len], 1,
			       cfg->i2c_addr);
		if (ret < 0) {
			break;
		}
		if (len == sizeof(rx_buf)-1) {
			LOG_HEXDUMP_DBG(rx_buf, len+1, "buffer flushed");
			len = 0;
		}
	} while (rx_buf[len++] != 0xFF);

	if (len) {
		LOG_HEXDUMP_DBG(rx_buf, len, "buffer flushed");
	}

	return ret;
}

static void update_pvt(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	/* FIXME: the gnss structs could be different than the ublox ubx structs. */
	memcpy(&drv_data->pvt.position, drv_data->frame.ubx.payload+20, sizeof(struct gnss_position));
	memcpy(&drv_data->pvt.velocity, drv_data->frame.ubx.payload+48, sizeof(struct gnss_velocity));
	memcpy(&drv_data->pvt.time, drv_data->frame.ubx.payload, sizeof(struct gnss_time));
	drv_data->pvt_ready = drv_data->pvt.position.fix_status.gnssFixOK && (drv_data->pvt.position.fix_status.psmState == UBX_PSM_STATE_TRACKING);
	switch (drv_data->pvt.position.fix_status.psmState) {
	case UBX_PSM_STATE_ACQUISITION:
		drv_data->tracking_state = GNSS_TRACKING_STATE_ACQUISITION;
		break;
	case UBX_PSM_STATE_TRACKING:
	case UBX_PSM_STATE_POT:
		drv_data->tracking_state = GNSS_TRACKING_STATE_TRACKING;
		break;
	case UBX_PSM_STATE_INACTIVE:
		drv_data->tracking_state = GNSS_TRACKING_STATE_INACTIVE;
		break;
	case UBX_PSM_STATE_ENABLED:
		drv_data->tracking_state = GNSS_TRACKING_STATE_ENABLED;
		break;
	default:
		drv_data->tracking_state = GNSS_TRACKING_STATE_DISABLED;
	}
}

static void update_status(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	drv_data->nav_status.tow = sys_get_le32((const u8_t *)&((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->iTOW);
	drv_data->nav_status.ttff = sys_get_le32((const u8_t *)&((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->ttff);
	drv_data->nav_status.uptime = sys_get_le32((const u8_t *)&((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->msss);
	drv_data->nav_status.fix = ((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->gpsFix;
	drv_data->nav_status.fix_status.gpsFixOk = ((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->flags.gpsFixOk;
	drv_data->nav_status.fix_status.towSet = ((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->flags.towSet;
	drv_data->nav_status.fix_status.wknSet = ((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->flags.wknSet;
	drv_data->nav_status.fix_status.diffSoln = ((struct ubx_payload_nav_status *)drv_data->frame.ubx.payload)->flags.diffSoln;
}

static void update_response_buffer(struct device *dev, void *buf, size_t len)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	memcpy(buf, drv_data->frame.ubx.payload, len);
}

static void set_ubx_response(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	struct ublox_frame *frame = &drv_data->frame;
	int ret = 0;

	switch (frame->ubx.rx_header.class) {
	case ubx_class_ack:
		switch (frame->ubx.rx_header.id) {
		case ubx_ack_id_ack:
			LOG_DBG("ubx %s, class: %d, id: %d", drv_data->frame.ubx.header.id ? "ack" : "nak",
				drv_data->frame.ubx.payload[0], drv_data->frame.ubx.payload[1]);
			frame->ubx.status.response_received = UBX_RESPONSE_ACK;
			update_response_buffer(dev, &drv_data->ubx_ack_buf, sizeof(struct ubx_payload_ack));
			k_sem_give(&drv_data->ubx_ack_sem);
			break;
		case ubx_ack_id_nak:
			LOG_DBG("ubx %s, class: %d, id: %d", drv_data->frame.ubx.header.id ? "ack" : "nak",
				drv_data->frame.ubx.payload[0], drv_data->frame.ubx.payload[1]);
			frame->ubx.status.response_received = UBX_RESPONSE_NAK;
			ret = -EINVAL;
			update_response_buffer(dev, &drv_data->ubx_ack_buf, sizeof(struct ubx_payload_ack));
			k_sem_give(&drv_data->ubx_ack_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported ack class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_cfg:
		switch (frame->ubx.rx_header.id) {
		case ubx_cfg_id_cfg:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_cfg));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_msg:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_msg));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_nav5:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_nav5));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_pm2:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_pm2));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_pms:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_pms));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_prt:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(union ubx_payload_cfg_prt));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_rate:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_rate));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_rxm:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_rxm));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_tp5:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_tp5));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported cfg class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_mon:
		switch (frame->ubx.rx_header.id) {
		case ubx_mon_id_ver:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_mon_ver));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_mon_id_hw:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_mon_hw));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_mon_id_hw2:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_mon_hw2));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_mon_id_io:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_mon_io));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_mon_id_gnss:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_mon_gnss));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported mon class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_nav:
		switch (frame->ubx.rx_header.id) {
		case ubx_nav_id_pvt:
			frame->ubx.status.response_received = UBX_RESPONSE_POLL;
			update_pvt(dev);
			k_sem_give(&drv_data->ubx_poll_sem);
			break;
		case ubx_nav_id_status:
			frame->ubx.status.response_received = UBX_RESPONSE_POLL;
			update_status(dev);
			k_sem_give(&drv_data->ubx_poll_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported nav class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_rxm:
		switch (frame->ubx.rx_header.id) {
		case ubx_rxm_id_pmreq:
			frame->ubx.status.response_received = UBX_RESPONSE_OUTPUT;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported rxm class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_sec:
		switch (frame->ubx.rx_header.id) {
		case ubx_sec_id_uniqid:
			frame->ubx.status.response_received = UBX_RESPONSE_OUTPUT;
			update_response_buffer(dev, drv_data->ubx_get_buf, sizeof(struct ubx_payload_sec_uniqid));
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported sec class id: %d",
				frame->ubx.header.id);
			break;
		}
		break;
	default:
		ret = -EPROTO;
		LOG_DBG("unsupported class %d",
			frame->ubx.header.class);
		break;
	}
	drv_data->last_error = ret;
}

static void ublox_m8_msg_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	size_t len;
	u32_t idle_count = 0;
	u8_t rx_buf;
	bool eol;

	ARG_UNUSED(unused);

	LOG_DBG("msg thread started");

	/* process message */
	while (1) {
		switch (drv_data->sentence_state) {
		case GNSS_SENTENCE_STATE_IDLE:
			len = 1;
			// gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
			// k_sleep(K_MSEC(100));
			/* read the first byte */
			ret = i2c_read(drv_data->i2c, &rx_buf, 1, cfg->i2c_addr);
			// if (ret < 0) {
			// 	LOG_DBG("read error");
			// 	break;
			// }
			if (ret < 0 || rx_buf == 0xFF) {
				if (idle_count % debug_idle_rate == 0) {
					LOG_DBG("idle (%d)", idle_count);
				}
				// gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);
				k_sem_take(&drv_data->msg_sem, K_HOURS(1));
				/* 
				 * Note:  When txReady fires, the duration does not appear to be
				 * related to the number of bytes remaining.  Rather, the duration
				 * appears to have a long (fixed?) pulse width.
				 * So we must wait a period of time after txReady fires for its
				 * state to reset so that we avoid an i2c bus storm that
				 * can last up to 80ms.
				 * 
				 * When the module sleeps (UBX_RXM_PMREQ), txReady appears to go to
				 * its active state.  This causes an i2c bus storm (and i2c bus
				 * errors because the slave does not respond) during the
				 * entire sleep cycle.
				 * 
				 * FIXME:  It seems like the interrupt is level sensitive Rather
				 * than edge triggered.  So, need to investigate this.
				 * 
				 * If the module does not detect activity, it will
				 * shut down the i2c port.  The module detects activity on the
				 * data (addr=0xFF) register.  I am not certain activity on the
				 * length register(s) (addr=0xFD and 0xFE) is recognized.
				 * 
				 * I do not believe txReady fires when there is "data available in the
				 * data buffer" such as would occur when the device responds to
				 * a host message such as a command, get/set, output
				 * or poll request.
				 * 
				 * Rather, txReady fires only when a navigation solution is ready.  
				 * No data is actually in the data buffer yet.  To get the data you
				 * must poll the device by sending a UBX-NAV-PVT message.
				 * 
				 * Thus, for the default nav configuration, txReady will fire once per
				 * second to indicate when the NAV solution has been computed.  The data buffer
				 * remains empty until the host issues a UBX-NAV-PVT poll request message.
				 * 
				 * wait for the data
				 */
				k_sleep(K_MSEC(100));
				idle_count++;
				break;
			}

			if (rx_buf == ubx_sync_1) {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_UBX;
			} else if (rx_buf == '$') {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_NMEA;
			} else if (rx_buf == 0xD3) {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_RTCM;
			} else {
				LOG_WRN("sentence not recognized: %d '%c'",
					rx_buf, rx_buf);
				ublox_m8_flush_msg_buffer(dev);
			}
			break;
		case GNSS_SENTENCE_STATE_UBX:
			k_sem_take(&drv_data->ubx_frame_sem, K_FOREVER);
			drv_data->frame.ubx.rx_header.sync1 = rx_buf;
			/* get the rest of the header */
			ret = i2c_read(drv_data->i2c, &drv_data->frame.ubx.rx_header.raw_data[1], sizeof(union ubx_header)-1, cfg->i2c_addr);
			if (ret < 0) {
				LOG_DBG("read ubx header error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				k_sem_give(&drv_data->ubx_frame_sem);
				break;
			}
			/* get length of payload */
			len = sys_get_le16(drv_data->frame.ubx.rx_header.len);
			/* copy header to incoming frame */
			drv_data->frame.ubx.header = drv_data->frame.ubx.rx_header;
			/* get payload */
			ret = i2c_read(drv_data->i2c, (u8_t *)drv_data->frame.ubx.payload, len, cfg->i2c_addr);
			if (ret < 0) {
				LOG_DBG("read ubx payload error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				k_sem_give(&drv_data->ubx_frame_sem);
				break;
			}
			/* get checksum */
			ret = i2c_read(drv_data->i2c, (u8_t *)&drv_data->frame.ubx.checksum, sizeof(struct ubx_checksum), cfg->i2c_addr);
			if (ret < 0) {
				LOG_DBG("read ubx checksum error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				k_sem_give(&drv_data->ubx_frame_sem);
				break;
			}
			drv_data->frame.ubx.len = sys_get_le16(drv_data->frame.ubx.header.len);
			if (ubx_validate_frame_checksum(&drv_data->frame.ubx)) {
				/* Everything is ok */
				drv_data->last_error = 0;
				set_ubx_response(dev);
			} else {
				LOG_WRN("ubx checksum failed");
			}
			LOG_HEXDUMP_DBG(&drv_data->frame.ubx.header, UBX_FRAME_HEADER_SIZE, "ubx header");
			LOG_HEXDUMP_DBG(drv_data->frame.ubx.payload, len, "ubx payload");
			LOG_HEXDUMP_DBG(&drv_data->frame.ubx.checksum, UBX_FRAME_CHECKSUM_SIZE, "ubx checksum");
			drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
			k_sem_give(&drv_data->ubx_frame_sem);
			break;
		case GNSS_SENTENCE_STATE_NMEA:
			k_sem_take(&drv_data->ubx_frame_sem, K_FOREVER);
			drv_data->frame.nmea[0] = rx_buf;
			drv_data->frame.nmea[1] = 0;
			len = 1;
			eol = false;
			/* nmea messages terminate with \r\n */
			while (!eol) {
				ret = i2c_read(drv_data->i2c, &drv_data->frame.nmea[len++], 1, cfg->i2c_addr);
				if (ret < 0) {
					LOG_DBG("read error");
					break;
				}
				if (len >= sizeof(drv_data->frame.nmea)) {
					LOG_DBG("nmea msg too large");
					ret = -ENOMEM;
					ublox_m8_flush_msg_buffer(dev);
					break;
				}
				/* FIXME:  what do we do if no data?  ie: char is 0xFF? */
				if (drv_data->frame.nmea[len-1] == 0xFF) {
					break;
				}
				/* check for eol */
				if (drv_data->frame.nmea[len-2] == '\r' && drv_data->frame.nmea[len-1] == '\n') {
					eol = true;
				}
			}
			if (ret == 0) {
				drv_data->frame.nmea[len-2] = 0;
				LOG_DBG("nmea: %s", log_strdup(drv_data->frame.nmea));
			}
			drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
			k_sem_give(&drv_data->ubx_frame_sem);
			break;
		case GNSS_SENTENCE_STATE_RTCM:
		case GNSS_SENTENCE_STATE_RTCM3:
		default:
			k_sem_take(&drv_data->ubx_frame_sem, K_FOREVER);
			LOG_WRN("unsupported msg");
			ublox_m8_flush_msg_buffer(dev);
			drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
			k_sem_give(&drv_data->ubx_frame_sem);
			break;
		}
	}
}

static void ublox_m8_msg_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, msg_work);

	ARG_UNUSED(drv_data);
}

/**
 * @brief used to read an i2c register from the device
 *        since registers 0 to FC (252) are not used
 * 	  this function is not used at the moment.
 * 
 * @param dev pointer to device
 * @param reg address of i2c device register to read
 * @param val pointer to buffer to store read result
 * @param size number of bytes to read
 * @return * int 0 indicates success
 * 		<0 indicates error
 */
__attribute__((unused))
static int ublox_m8_register_read(struct device *dev, u8_t reg, u8_t *val, int size)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr, reg, (u8_t *)val,
			     size);
	return ret;
}

static int ublox_m8_message_send(struct device *dev, struct ubx_frame *frame, k_timeout_t timeout)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	/* ubx length is sent little endian */
	sys_put_le16(frame->len, frame->header.len);
	ubx_frame_checksum(frame);

	/*
	 * write the header and payload
	 * 
	 * Note:
	 * header and payload must be written at the same time
	 * to avoid ddc protocol error when the payload is only one byte
	 * that is, minimum write size is 2 bytes, see 11.5.2
	 */
	ret = i2c_write(drv_data->i2c, (u8_t *)&frame->header, sizeof(frame->header) + frame->len,
			cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("write error");
		k_sem_give(&drv_data->ubx_frame_sem);
		goto done;
	}
	/* write the checksum */
	ret = i2c_write(drv_data->i2c, (u8_t *)&frame->checksum, sizeof(frame->checksum),
			cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("write error");
		k_sem_give(&drv_data->ubx_frame_sem);
		goto done;
	}

	k_sem_give(&drv_data->ubx_frame_sem);

	if (K_TIMEOUT_EQ(timeout,K_NO_WAIT)) {
		k_sem_give(&drv_data->msg_sem);
		ret = 0;
		return ret;
	}

	/* wait for response */
	switch (frame->status.response_request) {
	case UBX_RESPONSE_ACK:
	case UBX_RESPONSE_NAK:
		k_sem_reset(&drv_data->ubx_ack_sem);
		k_sleep(K_MSEC(100));
		k_sem_give(&drv_data->msg_sem);
		ret = k_sem_take(&drv_data->ubx_ack_sem, timeout);
		break;
	case UBX_RESPONSE_GET:
	case UBX_RESPONSE_OUTPUT:
		k_sem_reset(&drv_data->ubx_get_sem);
		k_sleep(timeout);
		k_sem_give(&drv_data->msg_sem);
		ret = k_sem_take(&drv_data->ubx_get_sem, timeout);
		break;
	case UBX_RESPONSE_POLL:
		k_sem_reset(&drv_data->ubx_poll_sem);
		k_sleep(timeout);
		k_sem_give(&drv_data->msg_sem);
		ret = k_sem_take(&drv_data->ubx_poll_sem, timeout);
		break;
	case UBX_RESPONSE_NONE:
		ret = 0;
		break;
	default:
		ret = -ENOTSUP;
		LOG_DBG("unsupported response type");
		break;
	}

	if (ret == 0) {
		/* FIXME:  what about nmea and rtcm? */
		LOG_DBG("response: %d, error: %d",
			drv_data->frame.ubx.status.response_received,
			drv_data->last_error);
		ret = drv_data->last_error;
	} else if (ret == -EAGAIN) {
		LOG_DBG("response time out");
		ret = -ETIMEDOUT;
	}

done:
	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_ubx_msg_set(struct device *dev, const union ubx_header *header,
				const void *payload, size_t payload_len,
				enum ubx_message type,
				enum ubx_response response,
				k_timeout_t timeout)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	struct ubx_frame frame;
	int ret;

	k_sem_take(&drv_data->ubx_frame_sem, K_FOREVER);
	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
	k_sleep(K_MSEC(100));

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_init, UBX_FRAME_STATUS_SIZE);
	frame.status.response_request = response;

	/* prepare frame header */
	memcpy(&frame.header, header, sizeof(frame.header));

	/* prepare the payload */
	memcpy(frame.payload, payload, payload_len);
	frame.len = payload_len;

	frame.type = type;

	ret = ublox_m8_message_send(dev, &frame, timeout);
	if (ret < 0) {
		LOG_ERR("msg set failed");
	}

	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);
	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_ubx_msg_get(struct device *dev, const union ubx_header *header,
				const void *payload, size_t payload_len,
				enum ubx_message type,
				enum ubx_response response,
				k_timeout_t timeout)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	struct ubx_frame frame;
	int ret;

	k_sem_take(&drv_data->ubx_frame_sem, K_FOREVER);
	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
	k_sleep(K_MSEC(100));

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_init, UBX_FRAME_STATUS_SIZE);
	frame.status.response_request = response;

	/* prepare frame header */
	memcpy(&frame.header, header, sizeof(frame.header));

	/* prepare the payload */
	memcpy(frame.payload, payload, payload_len);
	frame.len = payload_len;

	frame.type = type;
	
	ret = ublox_m8_message_send(dev, &frame, timeout);
	if (ret < 0) {
		LOG_ERR("msg get failed");
	}

	gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);
	return ret;
}

/**
 * @brief Get the Device ID
 *
 * @param[in]   dev     Pointer to the device structure
 *
 * @retval 0 On success
 * @retval -EIO I2C error
 *         -ETIMEDOUT if module did not respond
 */
static int ublox_m8_get_device_id(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	LOG_DBG("");
	/* send empty payload to poll device id */
	ublox_m8_ubx_msg_get(dev, &ubx_header_sec_uniqid, NULL, 0, UBX_MESSAGE_OUTPUT, UBX_RESPONSE_OUTPUT, drv_data->timeout);

	drv_data->device_id.be_word = 0;
	memcpy(drv_data->device_id.id, drv_data->ubx_get_buf+4, 5);
	LOG_INF("unique id: 0x%02x%08x",
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) >> 32),
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) & (BIT64(32)-1)));

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to get unique id",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_uart_port(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_uart1, sizeof(ubx_payload_cfg_prt_uart1), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure UART port",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_ddc_port(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_ddc, sizeof(ubx_payload_cfg_prt_ddc), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure DDC port",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_pm2(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_pm2, &ubx_payload_cfg_pm2, sizeof(ubx_payload_cfg_pm2), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure PM2",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_rxm(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_rxm, &ubx_payload_cfg_rxm, UBX_PAYLOAD_CFG_RXM_SIZE, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure RXM",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_nav_msg_rate(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	u8_t payload[3];
	int ret;

	LOG_DBG("");
	payload[0] = ubx_class_nav;
	payload[1] = ubx_nav_id_pvt;
	payload[2] = 1;
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_msg, payload, 3, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure nav msg rate",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_nav_settings(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_nav5, &ubx_payload_cfg_nav5, sizeof(struct ubx_payload_cfg_nav5), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure nav settings",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_configure_save_bbr(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_cfg, &ubx_payload_cfg_cfg_save, sizeof(ubx_payload_cfg_cfg_save), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to save to BBR",
			DT_LABEL(DT_INST(0,DT_DRV_COMPAT)));
	}

	return ret;
}

const struct gnss_trigger poll_trigger = {
	.type = GNSS_TRIG_POLL,
	.chan = GNSS_CHAN_ALL,
};

static int ublox_m8_sample_fetch(struct device *dev, enum gnss_channel chan)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	ARG_UNUSED(drv_data);

	switch (chan) {
	case GNSS_CHAN_ALL:
		k_poll_signal_raise(&drv_data->pvt_signal, 0);
		break;

	case GNSS_CHAN_TIME:
		break;

	case GNSS_CHAN_POSITION:
		break;

	case GNSS_CHAN_VELOCITY:
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_channel_get(struct device *dev, enum gnss_channel chan,
				void *val)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	ARG_UNUSED(drv_data);

	switch ((int) chan) {
	case GNSS_CHAN_ALL:
		// memcpy(val, &drv_data->pvt, sizeof(struct gnss_pvt));
		*(struct gnss_pvt *)val = drv_data->pvt;
		drv_data->pvt_ready = false;
		break;

	case GNSS_CHAN_TIME:
		*(struct gnss_time *)val = drv_data->pvt.time;
		break;

	case GNSS_CHAN_POSITION:
		*(struct gnss_position *)val = drv_data->pvt.position;
		break;

	case GNSS_CHAN_VELOCITY:
		*(struct gnss_velocity *)val = drv_data->pvt.velocity;
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_attr_get(struct device *dev, enum gnss_channel chan,
			     enum gnss_attribute attr,
			     void *val)
{
	static const u8_t protver[] = "PROTVER=";
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;
	u8_t payload[3];

	switch ((int) attr) {
	case GNSS_ATTR_INFO_ID:
		ublox_m8_ubx_msg_get(dev, &ubx_header_sec_uniqid, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			*(u64_t *)val = sys_get_be40(&drv_data->ubx_get_buf[4]);
		}
		break;

	case GNSS_ATTR_INFO_VERSION_HW:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_ver, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			strncpy((u8_t *)val, drv_data->ubx_get_buf+30, 10);
		}
		break;

	case GNSS_ATTR_INFO_VERSION_SW:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_ver, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			strncpy((u8_t *)val, drv_data->ubx_get_buf, 30);
		}
		break;

	case GNSS_ATTR_INFO_VERSION_PROTO:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_ver, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			for (u8_t i=0; i<4; i++) {
				int c = strncmp(protver, &drv_data->ubx_get_buf[40+i*30], sizeof(protver)-1);
				if (c == 0) {
					strncpy((u8_t *)val, &drv_data->ubx_get_buf[40+i*30], 30);
					break;
				}
			}
		}
		break;

	case GNSS_ATTR_INFO_HW_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_hw, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_INFO_HW2_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_hw2, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_INFO_IO_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_io, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_INFO_GNSS_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_gnss, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	/* FIXME:  this is really ublox specific data.  Can this be generalized or should be moved to a ublox api? */
	case GNSS_ATTR_PORT_I2C:
		payload[0] = ubx_port_ddc;
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_prt, payload, 1, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			memcpy(val, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_prt_ddc));
		}
		break;

	/* FIXME:  this is really ublox specific data.  Can this be generalized or should be moved to a ublox api? */
	case GNSS_ATTR_PORT_UART:
		payload[0] = ubx_port_uart1;
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_prt, payload, 1, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			memcpy(val, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_prt_uart));
		}
		break;

	case GNSS_ATTR_NAV_MEASUREMENT_RATE:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_rate, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			*(u16_t *)val = sys_get_le16(&drv_data->ubx_get_buf[0]);
		}
		break;

	case GNSS_ATTR_NAV_SOLUTION_RATE:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_rate, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			*(u16_t *)val = sys_get_le16(&drv_data->ubx_get_buf[2]);
		}
		break;

	case GNSS_ATTR_NAV_TIMEREF:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_rate, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			*(u16_t *)val = sys_get_le16(&drv_data->ubx_get_buf[4]);
		}
		break;

	case GNSS_ATTR_NAV_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_nav_status, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_POLL, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_NAV_SETTINGS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_nav5, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss settings */
			memcpy(val, drv_data->ubx_get_buf, sizeof(struct ubx_payload_cfg_nav5));
		}
		break;

	case GNSS_ATTR_NAV_MSG_PVT_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_pvt;
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_msg, payload, 2, UBX_MESSAGE_GET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			*(u8_t *)val = drv_data->ubx_get_buf[2]; 
		}
		break;

	case GNSS_ATTR_NAV_MSG_STATUS_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_status;
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_msg, payload, 2, UBX_MESSAGE_GET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			*(u8_t *)val = drv_data->ubx_get_buf[2]; 
		}
		break;

	case GNSS_ATTR_NAV_MSG_SOL_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_sol;
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_msg, payload, 2, UBX_MESSAGE_GET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			*(u8_t *)val = drv_data->ubx_get_buf[2]; 
		}
		break;

	case GNSS_ATTR_PM_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_pm2, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_PM_MODE:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_pms, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_TIMEPULSE_STATUS:
		ublox_m8_ubx_msg_get(dev, &ubx_header_cfg_tp5, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_attr_set(struct device *dev, enum gnss_channel chan,
			     enum gnss_attribute attr,
			     void *val)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret = 0;
	u8_t payload[3];

	switch ((int) attr) {
	case GNSS_ATTR_NAV_MSG_PVT_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_pvt;
		if (!val) {
			LOG_DBG("invalid pointer");
			ret = -EINVAL;
			break;
		}
		payload[2] = *(u8_t *)val;
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_msg, payload, 3, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
		break;

	case GNSS_ATTR_NAV_MSG_STATUS_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_status;
		if (!val) {
			LOG_DBG("invalid pointer");
			ret = -EINVAL;
			break;
		}
		payload[2] = *(u8_t *)val;
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_msg, payload, 3, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
		break;

	case GNSS_ATTR_NAV_MSG_SOL_RATE:
		payload[0] = ubx_class_nav;
		payload[1] = ubx_nav_id_sol;
		if (!val) {
			LOG_DBG("invalid pointer");
			ret = -EINVAL;
			break;
		}
		payload[2] = *(u8_t *)val;
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_msg, payload, 3, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
		break;

	case GNSS_ATTR_PM_SLEEP:
		// ublox_m8_ubx_msg_set(dev, &ubx_header_rxm_pmreq, &ubx_payload_rxm_pmreq, sizeof(ubx_payload_rxm_pmreq), UBX_MESSAGE_COMMAND, UBX_RESPONSE_OUTPUT, K_NO_WAIT);
		ret = gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 0);
		k_sleep(K_SECONDS(10));
		ret = gpio_pin_set(drv_data->extint_gpio, cfg->extint_gpio_pin, 1);
		break;

	case GNSS_ATTR_PM_MODE:
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_pms, val, sizeof(struct ubx_payload_cfg_pms), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
		if (val) {
			/*  FIXME:  need to define gnss status */
		}
		break;

	case GNSS_ATTR_SAVE:
		if (!val) {
			LOG_DBG("invalid pointer");
			ret = -EINVAL;
			break;
		}
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_cfg, val, sizeof(struct ubx_payload_cfg_cfg), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_connect_status(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	/*
	 * Note: if module times out it suspends the port
	 * and the bus transaction is not acked.
	 */
	ret = i2c_write(drv_data->i2c, NULL, 0, cfg->i2c_addr);
	return ret;
}

static void ublox_m8_reset(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;

	/* assert reset pin */
	drv_data->device_state = GNSS_DEVICE_STATE_RESET;
	gpio_pin_set(drv_data->reset_gpio, cfg->reset_gpio_pin, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set(drv_data->reset_gpio, cfg->reset_gpio_pin, 0);
	k_sleep(K_MSEC(100));
}

static int ublox_m8_pin_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret = 0;

	/*
	 * OUTPUT PINS
	 *
	 * setup EXTINT / PIO13 gpio
	 * 
	 * EXTINT is an external interrupt pin used for wake-up functions in
	 * Power Save Mode and for aiding.
	 *
	 * If EXTINT is not used for an external interrupt function, the pin
	 * can be used as a generic PIO (PIO13).
	 * 
	 * The PIO13 can be configured to function as an output pin for the
	 * TXD Ready feature to indicate that the receiver has data to
	 * transmit.
	 *
	 * The power control feature allows overriding the automatic
	 * active/inactive cycle of Power Save Mode.
	 *
	 * The state of the receiver can be controlled through the EXTINT
	 * pin. The receiver can also be forced OFF using EXTINT when
	 * Power Save Mode is not active.
	 *
	 * The EXTINT pin can be used to supply time or frequency aiding data
	 * to the receiver.
	 * 
	 * See Hardware Integration Manual 1.4.2 and 1.5.2
	 * 
	 * We use EXTINT for wake-up functions in Power Save Mode.
	 */
	drv_data->extint_gpio = device_get_binding(cfg->extint_gpio_name);
	if (drv_data->extint_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->extint_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->extint_gpio, cfg->extint_gpio_pin,
			   cfg->extint_gpio_flags |
			   GPIO_OUTPUT_ACTIVE);

	/*
	 * setup RESET gpio
	 * 
	 * Driving RESET_N low activates a hardware reset of the system.
	 * Use this pin only to reset the module. Do not use RESET_N to
	 * turn the module on and off, since the reset state increases
	 * power consumption.
	 * 
	 * See Hardware Integration Manual 1.5.1
	 */
	drv_data->reset_gpio = device_get_binding(cfg->reset_gpio_name);
	if (drv_data->reset_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->reset_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->reset_gpio, cfg->reset_gpio_pin,
			   cfg->reset_gpio_flags |
			   GPIO_OUTPUT_INACTIVE);

	/*
	 * setup SAFEBOOT gpio
	 * 
	 * The SAFEBOOT_N pin is for future service, updates and reconfiguration.
	 * 
	 * See Hardware Integration Manual 1.5.4
	 */
	drv_data->safeboot_gpio = device_get_binding(cfg->safeboot_gpio_name);
	if (drv_data->safeboot_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->safeboot_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->safeboot_gpio, cfg->safeboot_gpio_pin,
			   cfg->safeboot_gpio_flags |
			   GPIO_OUTPUT_INACTIVE);

	/*
	 * setup RXD gpio
	 * 
	 * UART RXD pin.
	 * 
	 * See Hardware Integration Manual 1.4.1
	 * 
	 * The UART is not used so this pin is disabled.
	 */
	drv_data->rxd_gpio = device_get_binding(cfg->rxd_gpio_name);
	if (drv_data->rxd_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->rxd_gpio_name);
		return -EINVAL;
	}

	gpio_pin_configure(drv_data->rxd_gpio, cfg->rxd_gpio_pin,
			   GPIO_DISCONNECTED);

	/*
	 * INPUT PINS
	 *
	 * setup TIMEPULSE gpio
	 * 
	 * A configurable time pulse signal. By default, the time pulse signal
	 * is configured to one pulse per second.
	 * 
	 * See Hardware Integration Manual 1.5.3
	 * 
	 * Timepulse gpio is initialized in ublox_m8_trigger.c
	 */

	/*
	 * setup TXD gpio
	 * 
	 * The TX_READY function can be mapped to TXD (PIO 06).
	 * The TX_READY function is disabled by default.
	 * 
	 * The UART is not used so TXD is not initialized.
	 * TX_READY is initialized in ublox_m8_trigger.c
	 * See Hardware Integration Manual 1.4.2
	 */
	// drv_data->txd_gpio = device_get_binding(cfg->txd_gpio_name);
	// if (drv_data->txd_gpio == NULL) {
	// 	LOG_ERR("Failed to get pointer to %s device",
	// 		    cfg->txd_gpio_name);
	// 	return -EINVAL;
	// }

	// gpio_pin_configure(drv_data->txd_gpio, cfg->txd_gpio_pin,
	// 		   cfg->txd_gpio_flags |
	// 		   GPIO_OUTPUT);

	return ret;
}

static int ublox_m8_configure(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	u8_t payload[3];
	int ret = 0;

	if (drv_data->device_state != GNSS_DEVICE_STATE_INITIALIZED) {
		LOG_ERR("device not initialized");
		return -EINVAL;
	}

#ifdef CONFIG_UBLOX_M8_TRIGGER
	ret = ublox_m8_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize ublox interrupts");
		return -EINVAL;
	}
#endif

	/* FIXME:  If the uart of ddc initialization are reversed,
	 * there is some sort of panic situation.
	 * 
	 * The problem might have something to do with incoming
	 * nmea messages being processed when the ddc port configuration
	 * is sent
	 */
	// ret = ublox_m8_configure_uart_port(dev);
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_uart1, sizeof(ubx_payload_cfg_prt_uart1), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to configure uart port");
		return ret;
	}

	// ret = ublox_m8_configure_ddc_port(dev);
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_ddc, sizeof(ubx_payload_cfg_prt_ddc), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to configure ddc port");
		return ret;
	}

	// ret = ublox_m8_configure_rxm(dev);
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_rxm, &ubx_payload_cfg_rxm, UBX_PAYLOAD_CFG_RXM_SIZE, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to configure low power mode");
		return ret;
	}

	// ret = ublox_m8_configure_pm2(dev);
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_pm2, &ubx_payload_cfg_pm2, sizeof(ubx_payload_cfg_pm2), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to configure power management");
		return ret;
	}

	// ret = ublox_m8_configure_nav_msg_rate(dev);
	payload[0] = ubx_class_nav;
	payload[1] = ubx_nav_id_pvt;
	payload[2] = 1;
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_msg, payload, 3, UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to set nav pvt message rate");
		return ret;
	}

	// ret = ublox_m8_configure_nav_settings(dev);
	// ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_nav5, &ubx_payload_cfg_nav5, sizeof(struct ubx_payload_cfg_nav5), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	// ret = drv_data->last_error;
	// if (ret < 0) {
	//	LOG_ERR("failed to configure navigation settings");
	// 	return ret;
	// }

	// ret = ublox_m8_configure_save_bbr(dev);
	ret = ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_cfg, &ubx_payload_cfg_cfg_save, sizeof(ubx_payload_cfg_cfg_save), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to save configuration settings");
		return ret;
	}

	// ret = ublox_m8_get_device_id(dev);
	ret = ublox_m8_ubx_msg_get(dev, &ubx_header_sec_uniqid, NULL, 0, UBX_MESSAGE_OUTPUT, UBX_RESPONSE_OUTPUT, drv_data->timeout);
	if (ret < 0) {
		LOG_ERR("failed to get the device id");
		return ret;
	}
	drv_data->device_id.be_word = 0;
	memcpy(drv_data->device_id.id, drv_data->ubx_get_buf+4, 5);
	LOG_INF("unique id: 0x%02x%08x",
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) >> 32),
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) & (BIT64(32)-1)));

	drv_data->device_state = GNSS_DEVICE_STATE_CONFIGURED;
	LOG_INF("ublox m8 configured");

	return 0;
}

static void ublox_m8_config_work_handler(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, config_work);
	struct device *dev = drv_data->dev;

	ublox_m8_configure(dev);
}

static int ublox_m8_enable(struct device *dev, gnss_status_callback_t handler)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	ARG_UNUSED(handler);

	ret = ublox_m8_configure(dev);
	if (ret == 0) {
		drv_data->tracking_state = GNSS_TRACKING_STATE_ACQUISITION;
		k_poll_signal_raise(&drv_data->pvt_signal, 180000);
	}

	return ret;
}

/* FIXME:  make sure that triggers and signals are disabled if device driver fails to initialize */
int ublox_m8_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	drv_data->dev = dev;
	drv_data->device_state = GNSS_DEVICE_STATE_UNINITIALIZED;
	while (k_uptime_ticks() < k_us_to_ticks_ceil32(UBLOX_M8_STARTUP_TIME_USEC)) {
		/* wait for chip to power up */
	}

	drv_data->i2c = device_get_binding(cfg->i2c_name);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->i2c_name);
		return -EINVAL;
	}

	ret = ublox_m8_pin_init(dev);
	if (ret < 0) {
		LOG_ERR("pin initialization failed");
		drv_data->device_state = GNSS_DEVICE_STATE_DISCONNECTED;
		return ret;
	}

	ublox_m8_reset(dev);
	
	ret = ublox_m8_connect_status(dev);
	if (ret < 0) {
		LOG_ERR("device did not ack i2c write");
		drv_data->device_state = GNSS_DEVICE_STATE_DISCONNECTED;
		return -ENODEV;
	}

	drv_data->timeout = K_MSEC(UBLOX_M8_MESSAGE_TIMEOUT_MSEC);
	drv_data->poll_status = 0;
	drv_data->pvt_ready = false;
	drv_data->tracking_state = GNSS_TRACKING_STATE_DISABLED;
	k_sem_init(&drv_data->msg_sem, 0, 1);
	k_sem_init(&drv_data->ubx_frame_sem, 1, 1);
	k_sem_init(&drv_data->ubx_get_sem, 0, 1);
	k_sem_init(&drv_data->ubx_ack_sem, 0, 1);
	k_sem_init(&drv_data->ubx_poll_sem, 0, 1);
	k_work_init(&drv_data->msg_work, ublox_m8_msg_work_cb);
	k_delayed_work_init(&drv_data->config_work, ublox_m8_config_work_handler);
	k_mutex_init(&drv_data->msg_get_mtx);
	k_mutex_init(&drv_data->msg_send_mtx);

	drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
	k_thread_create(&drv_data->msg_thread, drv_data->msg_thread_stack,
			CONFIG_UBLOX_M8_MSG_THREAD_STACK_SIZE,
			(k_thread_entry_t)ublox_m8_msg_thread, dev,
			0, NULL, K_PRIO_COOP(CONFIG_UBLOX_M8_MSG_THREAD_PRIORITY),
			0, K_NO_WAIT);
	k_thread_name_set(&drv_data->msg_thread, CONFIG_UBLOX_M8_MSG_THREAD_NAME);

	/* poll ddc interface and flush pending messages */
	k_sem_give(&drv_data->msg_sem);
	drv_data->device_state = GNSS_DEVICE_STATE_INITIALIZED;
	// k_delayed_work_submit(&drv_data->config_work, K_SECONDS(2));
	return 0;
}

static const struct gnss_driver_api ublox_m8_driver_api = {
	.sample_fetch = ublox_m8_sample_fetch,
	.channel_get = ublox_m8_channel_get,
	.attr_get = ublox_m8_attr_get,
	.attr_set = ublox_m8_attr_set,
	.enable = ublox_m8_enable,
#if CONFIG_UBLOX_M8_TRIGGER
	.trigger_set = ublox_m8_trigger_set,
#endif
};

static struct ublox_m8_data ublox_m8_drv_data;

static const struct ublox_m8_dev_config ublox_m8_config = {
	.i2c_name = DT_BUS_LABEL(DT_INST(0,DT_DRV_COMPAT)),
	.i2c_addr = DT_REG_ADDR(DT_INST(0,DT_DRV_COMPAT)),
	.txready_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),txready_gpios),
	.txready_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),txready_gpios),
	.txready_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),txready_gpios),
	.reset_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),reset_gpios),
	.reset_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),reset_gpios),
	.reset_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),reset_gpios),
	.timepulse_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),timepulse_gpios),
	.timepulse_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),timepulse_gpios),
	.timepulse_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),timepulse_gpios),
	.safeboot_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),safeboot_gpios),
	.safeboot_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),safeboot_gpios),
	.safeboot_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),safeboot_gpios),
	.extint_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),extint_gpios),
	.extint_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),extint_gpios),
	.extint_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),extint_gpios),
	.rxd_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),rxd_gpios),
	.rxd_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),rxd_gpios),
	.rxd_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),rxd_gpios),
	// .txd_gpio_name = DT_GPIO_LABEL(DT_INST(0,DT_DRV_COMPAT),txd_gpios),
	// .txd_gpio_pin = DT_GPIO_PIN(DT_INST(0,DT_DRV_COMPAT),txd_gpios),
	// .txd_gpio_flags = DT_GPIO_FLAGS(DT_INST(0,DT_DRV_COMPAT),txd_gpios),
};

DEVICE_AND_API_INIT(ublox_m8, DT_LABEL(DT_INST(0,DT_DRV_COMPAT)), ublox_m8_init,
		    &ublox_m8_drv_data, &ublox_m8_config, POST_KERNEL,
		    CONFIG_GNSS_INIT_PRIORITY, &ublox_m8_driver_api);
