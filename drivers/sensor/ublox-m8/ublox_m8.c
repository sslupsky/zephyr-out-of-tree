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

// NET_BUF_POOL_DEFINE(ublox_buf_pool, 8, 32, 0, NULL);

FOR_EACH(UBX_HEADER_CFG_DEFINE, cfg, pm2, prt, pms, pwr, rxm);
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
 * B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 08 07 00 00 00 00 00 00 00 00 00 02 72
 * 
 * DDC port:  ubx in, ubx out, slave=42, txready, pio=6, thres 8
 * B5 62 06 00 14 00 00 00 99 00 84 00 00 00 00 00 00 00 01 00 01 00 00 00 00 00 39 58
 * 
 * DDC port:  ubx in, ubx out, slave=42, txready, pio=6, thres 8, extended timeout
 * B5 62 06 00 14 00 00 00 99 00 84 00 00 00 00 00 00 00 01 00 01 00 02 00 00 00 3B 60
 * 
 * Save configuration:  All settings saved to BBR
 * B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 01 1B A9
 * 
 * PM2 configuration:  extint0, extintWake, extintBackup, updateRTC, updateEPH, mode ON/OFF
 * B5 62 06 3B 30 00 02 06 00 00 60 98 40 01 00 00 00 00 00 00 00 00 00 00 00 00 02 00 00 00 2C 01 00 00 4F
 * C1 03 00 87 02 00 00 FF 00 00 00 64 40 01 00 00 00 00 00 21 10
 * 
 * RXM configuration:  power save mode
 * B5 62 06 11 02 00 08 01 22 92
 * 
 * RXM pmreq:  v2, duration 10000ms, backup, wake extint0
 * B5 62 02 41 10 00 00 00 00 00 10 27 00 00 02 00 00 00 20 00 00 00 AC 18
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
 * B5 62 06 00 14 00 00 00 99 00 84 00 00 00 00 00 00 00 01 00 01 00 02 00 00 00 3B 60
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

#define UBX_PAYLOAD_CFG_PRT_DDC_POLL_SIZE 1
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_ddc_poll = {
	.ddc.portID = 0,
};

/*
 * UBX_CFG_PRT
 * ucenter configuration:
 * portID: UART1, inProtoMask: none, outProtoMask: none, baud: 460800, mode: 8 N 1 
 * B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 08 07 00 00 00 00 00 00 00 00 00 02 72
 */
const union ubx_payload_cfg_prt ubx_payload_cfg_prt_uart1 = {
	.uart.portID = 1,
	.uart.txReady.en = 0,
	.uart.mode.charLen = 3,
	.uart.mode.parity = 4,
	.uart.mode.nStopBits = 0,
	.uart.baudRate = 460800,
};

/*
 * UBX_CFG_PM2
 * ucenter configuration:
 * extint0, extintWake, extintBackup, updateRTC, updateEPH, mode ON/OFF
 * B5 62 06 3B 30 00 02 06 00 00 60 98 40 01 00 00 00 00 00 00 00 00 00 00 00 00 02 00 00 00 2C 01 00 00 4F
 * C1 03 00 87 02 00 00 FF 00 00 00 64 40 01 00 00 00 00 00 21 10
 */
const struct ubx_payload_cfg_pm2 ubx_payload_cfg_pm2 = {
	.version = 2,
	.flags.extintSel = 0,
	.flags.extintWake = 1,
	.flags.extintBackup = 1,
	.flags.extintInactive = 0,
	.flags.updateRTC = 1,
	.flags.updateEPH = 1,
	.flags.mode = 0,
};

/*
 * UBX_CFG_CFG
 * ucenter configuration:
 * clearMask: <none>, saveMask: <all>, loadMask: <none>
 * B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 01 1B A9
 */
const struct ubx_payload_cfg_cfg ubx_payload_cfg_cfg_save = {
	.saveMask.ioPort = 1,
	.saveMask.msgConf = 1,
	.saveMask.infMsg = 1,
	.saveMask.navConf = 1,
	.saveMask.rxmConf = 1,
};

/*
 * UBX_CFG_RXM
 * ucenter configuration:
 * lpMode: power save mode
 * B5 62 06 11 02 00 08 01 22 92
 */
const struct ubx_payload_cfg_rxm ubx_payload_cfg_rxm = {
	.lpMode = 1,
};

/*
 * UBX_RXM_PMREQ
 * ucenter configuration
 * version: 0, duration: 10000ms, flags: backup, wakeupSources: extint0
 * B5 62 02 41 10 00 00 00 00 00 10 27 00 00 02 00 00 00 20 00 00 00 AC 18
 */
const struct ubx_payload_rxm_pmreq ubx_payload_rxm_pmreq = {
	.version = 0,
	.duration = sys_cpu_to_le32(10000),
	.flags.backup = 1,
	.wakeupSources.extint0 = 1,
};

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

	checksumA += frame->header.class;
	checksumB += checksumA;

	checksumA += frame->header.id;
	checksumB += checksumA;

	checksumA += (frame->header.len[0]);
	checksumB += checksumA;

	checksumA += (frame->header.len[1]);
	checksumB += checksumA;

	for (uint16_t i = 0; i < frame->len; i++)
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

__unused
static int ublox_m8_flush_msg_buffer(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	u8_t rx_buf[16];
	u16_t msg_len, read_bytes;

	/* get the message length */
	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			     UBLOX_M8_REGISTER_LEN, rx_buf, 2);
	if (ret < 0) {
		LOG_DBG("addr error");
		return -EIO;
	}

	/* ddc port message length is received big endian */
	msg_len = sys_get_be16(rx_buf);
	if (msg_len == 0) {
		LOG_DBG("msg buffer empty");
		return 0;
	}

	while (msg_len > 0) {
		read_bytes = MIN(msg_len, sizeof(rx_buf));
		ret = i2c_read(drv_data->i2c, rx_buf, read_bytes,
			       cfg->i2c_addr);
		if (ret < 0) {
			break;
		}
		LOG_HEXDUMP_DBG(rx_buf, read_bytes, "buffer flush");
		msg_len -= read_bytes;
	}

	return ret;
}

static int ublox_m8_flush_msg_pipe(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	u8_t rx_buf[128];
	int ret = 0;
	size_t bytes_read;

	while (1) {
		ret = k_pipe_get(&drv_data->rx_pipe, rx_buf, sizeof(rx_buf), &bytes_read, 1, K_NO_WAIT);
		if (bytes_read) {
			LOG_HEXDUMP_DBG(rx_buf, bytes_read, "pipe flushed");
		}
		if (ret < 0 || bytes_read == 0) {
			break;
		}
	}
	return 0;
}

static void set_ubx_response(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	union ublox_frame *frame = &drv_data->frame;
	int ret = 0;

	switch (frame->ubx.header.class) {
	case ubx_class_ack:
		switch (frame->ubx.header.id) {
		case ubx_ack_id_ack:
			frame->ubx.status.response_received = UBX_RESPONSE_ACK;
			k_sem_give(&drv_data->ubx_ack_sem);
			break;
		case ubx_ack_id_nak:
			frame->ubx.status.response_received = UBX_RESPONSE_NAK;
			ret = -EINVAL;
			k_sem_give(&drv_data->ubx_ack_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_cfg:
		switch (frame->ubx.header.id) {
		case ubx_cfg_id_cfg:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		case ubx_cfg_id_pm2:
		case ubx_cfg_id_prt:
		case ubx_cfg_id_tp5:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_mon:
		switch (frame->ubx.header.id) {
		case ubx_mon_id_ver:
			frame->ubx.status.response_received = UBX_RESPONSE_GET;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_nav:
		switch (frame->ubx.header.id) {
		case ubx_nav_id_pvt:
			frame->ubx.status.response_received = UBX_RESPONSE_POLL;
			k_sem_give(&drv_data->ubx_poll_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_rxm:
		switch (frame->ubx.header.id) {
		case ubx_rxm_id_pmreq:
			frame->ubx.status.response_received = UBX_RESPONSE_OUTPUT;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	case ubx_class_sec:
		switch (frame->ubx.header.id) {
		case ubx_sec_id_uniqid:
			frame->ubx.status.response_received = UBX_RESPONSE_OUTPUT;
			k_sem_give(&drv_data->ubx_get_sem);
			break;
		default:
			ret = -EPROTO;
			LOG_DBG("unsupported response class %d, id %d",
				frame->ubx.header.class,
				frame->ubx.header.id);
			break;
		}
		break;
	default:
		ret = -EPROTO;
		LOG_DBG("unsupported response class %d",
			frame->ubx.header.class);
		break;
	}
	drv_data->last_error = ret;
}

static void update_pvt(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;

	memcpy(&drv_data->pvt.position, drv_data->frame.ubx.payload+20, sizeof(struct gnss_position));
	memcpy(&drv_data->pvt.velocity, drv_data->frame.ubx.payload+48, sizeof(struct gnss_velocity));
	memcpy(&drv_data->pvt.time, drv_data->frame.ubx.payload, sizeof(struct gnss_time));
}

static void ublox_m8_msg_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;
	size_t bytes_read, len = 0;
	union ublox_frame *frame = &drv_data->frame;

	ARG_UNUSED(unused);

	LOG_DBG("msg thread started");

	/* process message */
	while (1) {
		switch (drv_data->sentence_state) {
		case GNSS_SENTENCE_STATE_IDLE:
			len = 1;
			ret = k_pipe_get(&drv_data->rx_pipe, frame, 1,
					 &bytes_read, 1, K_FOREVER);
			if (ret < 0) {
				LOG_DBG("pipe error");
			}
			if (frame->ubx.header.sync1 == ubx_sync_1) {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_UBX;
			} else if (frame->nmea[0] == '$') {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_NMEA;
			} else if (frame->rtcm[0] == 0xD3) {
				drv_data->sentence_state = GNSS_SENTENCE_STATE_RTCM;
			} else {
				LOG_WRN("sentence not recognized: %d '%c'",
					frame->raw[0], frame->raw[0]);
				ublox_m8_flush_msg_pipe(dev);
			}
			break;
		case GNSS_SENTENCE_STATE_UBX:
			/* get the rest of the header */
			ret = k_pipe_get(&drv_data->rx_pipe, &frame->ubx.header.sync2,
				UBX_FRAME_HEADER_SIZE - len, &bytes_read,
				UBX_FRAME_HEADER_SIZE - len, K_MSEC(100));
			if (ret < 0 || bytes_read < UBX_FRAME_HEADER_SIZE - len ||
			    frame->ubx.header.sync2 != ubx_sync_2) {
				LOG_DBG("ubx header error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				break;
			}
			/* get length of payload */
			len = sys_get_le16(frame->ubx.header.len);
			/* get payload */
			ret = k_pipe_get(&drv_data->rx_pipe, frame->payload, len,
					 &bytes_read, len, K_MSEC(100));
			if (ret < 0 || bytes_read <  len) {
				LOG_DBG("ubx payload error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				break;
			}
			/* get checksum */
			ret = k_pipe_get(&drv_data->rx_pipe, &frame->ubx.checksum,
					 UBX_FRAME_CHECKSUM_SIZE, &bytes_read,
					 UBX_FRAME_CHECKSUM_SIZE, K_MSEC(100));
			if (ret < 0 || bytes_read < UBX_FRAME_CHECKSUM_SIZE) {
				LOG_DBG("ubx checksum error");
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				break;
			}
			frame->ubx.len = sys_get_le16(frame->ubx.header.len);
			if (ubx_validate_frame_checksum(&frame->ubx)) {
				/* Everything is ok */
				drv_data->last_error = 0;
				set_ubx_response(dev);
			}
			update_pvt(dev);
			LOG_HEXDUMP_DBG(&frame->ubx.header, UBX_FRAME_HEADER_SIZE, "ubx header");
			LOG_HEXDUMP_DBG(frame->payload, len, "ubx payload");
			LOG_HEXDUMP_DBG(&frame->ubx.checksum, UBX_FRAME_CHECKSUM_SIZE, "ubx checksum");
			drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
			break;
		case GNSS_SENTENCE_STATE_NMEA:
			ret = k_pipe_get(&drv_data->rx_pipe, frame->nmea+len, 1,
					 &bytes_read, 1, K_MSEC(100));
			if (bytes_read) {
				/* check for eol */
				if (frame->nmea[len-1] == '\r' && frame->nmea[len] == '\n') {
					frame->nmea[len-1] = 0;
					LOG_DBG("nmea: %s", log_strdup(frame->nmea));
					drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
					break;
				}
				len++;
			}
			if (ret < 0 || bytes_read == 0 || len == sizeof(frame->nmea)) {
				LOG_DBG("nmea msg error");
				ublox_m8_flush_msg_pipe(dev);
				drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
				break;
			}
			break;
		case GNSS_SENTENCE_STATE_RTCM:
		case GNSS_SENTENCE_STATE_RTCM3:
		default:
			LOG_WRN("unsupported msg");
			ublox_m8_flush_msg_pipe(dev);
			drv_data->sentence_state = GNSS_SENTENCE_STATE_IDLE;
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

	k_mutex_lock(&drv_data->msg_send_mtx, K_FOREVER);

	/* ubx length is sent little endian */
	sys_put_le16(frame->len, frame->header.len);
	ubx_frame_checksum(frame);

	/* write the header */
	ret = i2c_write(drv_data->i2c, (u8_t *)&frame->header, sizeof(frame->header),
			cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("write error");
		goto done;
	}
	/* check if there is a payload */
	if (frame->len) {
		/* write the payload */
		ret = i2c_write(drv_data->i2c, frame->payload, frame->len,
				cfg->i2c_addr);
		if (ret < 0) {
			LOG_DBG("write error");
			goto done;
		}
	}
	/* write the checksum */
	ret = i2c_write(drv_data->i2c, (u8_t *)&frame->checksum, sizeof(frame->checksum),
			cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("write error");
		goto done;
	}
	// frame->status.req_sent = true;

	if (K_TIMEOUT_EQ(timeout,K_NO_WAIT)) {
		ret = 0;
		goto done;
	}

	/* FIXME: this is temporary to test polling */
	// k_sleep(K_MSEC(100));
	// drv_data->poll_handler(dev, &drv_data->poll_trigger);

	/* wait for response */
	switch (frame->status.response_request) {
	case UBX_RESPONSE_ACK:
	case UBX_RESPONSE_NAK:
		k_sem_reset(&drv_data->ubx_ack_sem);
		ret = k_sem_take(&drv_data->ubx_ack_sem, timeout);
		break;
	case UBX_RESPONSE_GET:
	case UBX_RESPONSE_OUTPUT:
		k_sem_reset(&drv_data->ubx_get_sem);
		ret = k_sem_take(&drv_data->ubx_get_sem, timeout);
		break;
	case UBX_RESPONSE_POLL:
		k_sem_reset(&drv_data->ubx_poll_sem);
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
		LOG_DBG("response: %d", drv_data->frame.ubx.status.response_received);
		ret = drv_data->last_error;
	} else if (ret == -EAGAIN) {
		LOG_DBG("response time out");
		ret = -ETIMEDOUT;
	}

done:
	k_mutex_unlock(&drv_data->msg_send_mtx);
	return ret;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_ubx_msg_set(struct device *dev, const struct ubx_header *header,
				const void *payload, size_t payload_len,
				enum ubx_message type,
				enum ubx_response response,
				k_timeout_t timeout)
{
	struct ubx_frame frame;
	int ret;

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_init, UBX_FRAME_STATUS_SIZE);
	frame.status.response_request = response;

	/* prepare frame request */
	memcpy(&frame.header, header, sizeof(frame.header));

	/* prepare the payload */
	frame.payload = payload;
	frame.len = payload_len;

	frame.type = type;

	ret = ublox_m8_message_send(dev, &frame, timeout);
	if (ret < 0) {
		LOG_ERR("msg set failed");
		return ret;
	}

	return 0;
}

/**
 * @brief 
 * 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_ubx_msg_get(struct device *dev, const struct ubx_header *header,
				const void *payload, size_t payload_len,
				enum ubx_message type,
				enum ubx_response response,
				k_timeout_t timeout)
{
	struct ubx_frame frame;
	int ret;

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_init, UBX_FRAME_STATUS_SIZE);
	frame.status.response_request = response;

	/* prepare frame request */
	memcpy(&frame.header, header, sizeof(frame.header));

	/* prepare the payload */
	frame.payload = payload;
	frame.len = payload_len;

	frame.type = type;
	
	ret = ublox_m8_message_send(dev, &frame, timeout);
	if (ret < 0) {
		LOG_ERR("msg get failed");
		return ret;
	}

	return 0;
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
	memcpy(drv_data->device_id.id, drv_data->frame.ubx.payload+4, 5);
	LOG_INF("unique id: 0x%02x%08x",
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) >> 32),
		(u32_t) (sys_be64_to_cpu(drv_data->device_id.be_word) & (BIT64(32)-1)));

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to get unique id",
			DT_INST_0_UBLOX_M8_LABEL);
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
static int ublox_m8_get_version(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	LOG_DBG("");
	/* send empty payload to poll version */
	ublox_m8_ubx_msg_get(dev, &ubx_header_mon_ver, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);

	strncpy(drv_data->sw_version, drv_data->frame.ubx.payload, 30);
	strncpy(drv_data->hw_version, drv_data->frame.ubx.payload+30, 10);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to get version",
			DT_INST_0_UBLOX_M8_LABEL);
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
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_uart1, sizeof(ubx_payload_cfg_prt_uart1), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, K_NO_WAIT);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure UART port",
			DT_INST_0_UBLOX_M8_LABEL);
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
	ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_ddc, sizeof(ubx_payload_cfg_prt_ddc), UBX_MESSAGE_SET, UBX_RESPONSE_ACK, K_NO_WAIT);

	ret = drv_data->last_error;
	if (ret < 0) {
		LOG_ERR("%s: Failed to configure DDC port",
			DT_INST_0_UBLOX_M8_LABEL);
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
			DT_INST_0_UBLOX_M8_LABEL);
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
			DT_INST_0_UBLOX_M8_LABEL);
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
			DT_INST_0_UBLOX_M8_LABEL);
	}

	return ret;
}

const struct gnss_trigger txready_trigger = {
	.type = GNSS_TRIG_DATA_READY,
	.chan = GNSS_CHAN_ALL,
};

const struct gnss_trigger poll_trigger = {
	.type = GNSS_TRIG_POLL,
	.chan = GNSS_CHAN_ALL,
};

/**
 * @brief 
 * 
 * @param dev 
 * @return int 
 */
static int ublox_m8_message_get(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;
	u16_t msg_len;
	size_t bytes_read, bytes_put;

	k_mutex_lock(&drv_data->msg_get_mtx, K_FOREVER);

	/* get the message length */
	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			     UBLOX_M8_REGISTER_LEN, drv_data->rx_buf, 2);
	if (ret < 0) {
		LOG_DBG("addr error");
		goto done;
	}
	/* ddc port message length is received big endian */
	msg_len = sys_get_be16(drv_data->rx_buf);
	if (msg_len == 0) {
		ret = -EAGAIN;
		goto done;
	}

	if (msg_len > 4095) {
		LOG_WRN("large msg buffer");
	}

	/* read the first byte */
	ret = i2c_read(drv_data->i2c, drv_data->rx_buf, 1, cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("read error");
		ret = -EIO;
		goto done;
	}
	if (drv_data->rx_buf[0] == 0xFF) {
		LOG_DBG("buffer not ready (0xFF)");
		ret = -EAGAIN;
		goto done;
	}

	ret = k_pipe_put(&drv_data->rx_pipe, drv_data->rx_buf, 1, &bytes_put, 1, drv_data->timeout);
	if (ret < 0) {
		if (ret == -EAGAIN) {
			LOG_DBG("pipe timeout");
			ret = -ETIMEDOUT;
		} else {
			LOG_DBG("pipe error");
		}
		goto done;
	}

	msg_len--;
	while (msg_len > 0) {
		bytes_read = MIN(msg_len, sizeof(drv_data->rx_buf));
		ret = i2c_read(drv_data->i2c, drv_data->rx_buf, bytes_read, cfg->i2c_addr);
		if (ret < 0) {
			LOG_DBG("read error");
			break;
		}
		ret = k_pipe_put(&drv_data->rx_pipe, drv_data->rx_buf, bytes_read, &bytes_put, bytes_read, drv_data->timeout);
		if (ret < 0) {
			if (ret == -EAGAIN) {
				LOG_DBG("pipe timeout");
				ret = -ETIMEDOUT;
			} else {
				LOG_DBG("pipe error");
			}
			break;;
		}
		msg_len -= bytes_read;
	}
done:
	drv_data->poll_status = ret;
	k_mutex_unlock(&drv_data->msg_get_mtx);
	return ret;
}

static void ublox_m8_txready_handler(struct device *dev, struct gnss_trigger *trigger)
{
	int ret;

	ret = ublox_m8_message_get(dev);
	if (ret < 0) {
	}
	return;
}

static void ublox_m8_poll_handler(struct device *dev, struct gnss_trigger *trigger)
{
	int ret;

	ret = ublox_m8_message_get(dev);
	if (ret < 0) {
	}
	return;
}

static int ublox_m8_sample_fetch(struct device *dev, enum gnss_channel chan)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	ARG_UNUSED(drv_data);

	switch (chan) {
	case GNSS_CHAN_ALL:
		/* send header with empty payload  */
		ublox_m8_ubx_msg_get(dev, &ubx_header_nav_pvt, NULL, 0, UBX_MESSAGE_POLL, UBX_RESPONSE_POLL, drv_data->timeout);
		break;

	case GNSS_CHAN_TIME:
		break;

	case GNSS_CHAN_POSITION:
		break;

	case GNSS_CHAN_VELOCITY:
		break;

	case GNSS_CHAN_ID:
		ublox_m8_ubx_msg_get(dev, &ubx_header_sec_uniqid, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		break;

	case GNSS_CHAN_VERSION:
		ublox_m8_ubx_msg_get(dev, &ubx_header_mon_ver, NULL, 0, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		break;

	case GNSS_CHAN_SLEEP:
		ublox_m8_ubx_msg_set(dev, &ubx_header_rxm_pmreq, &ubx_payload_rxm_pmreq, sizeof(ubx_payload_rxm_pmreq), UBX_MESSAGE_COMMAND, UBX_RESPONSE_OUTPUT, drv_data->timeout);
		break;

	case GNSS_CHAN_DDC:
		ublox_m8_ubx_msg_set(dev, &ubx_header_cfg_prt, &ubx_payload_cfg_prt_ddc_poll, UBX_PAYLOAD_CFG_PRT_DDC_POLL_SIZE, UBX_MESSAGE_GET, UBX_RESPONSE_OUTPUT, drv_data->timeout);
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

	case GNSS_CHAN_ID:
		*(u64_t *)val = drv_data->device_id.be_word;
		break;

	default:
		LOG_WRN("sensor channel not supported");
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int ublox_m8_attr_set(struct device *dev, enum gnss_channel chan,
			     enum gnss_attribute attr,
			     const struct gnss_pvt *val)
{
	// struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	switch ((int) attr) {
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static const struct gnss_driver_api ublox_m8_driver_api = {
	.sample_fetch = ublox_m8_sample_fetch,
	.channel_get = ublox_m8_channel_get,
	.attr_set = ublox_m8_attr_set,
#if CONFIG_UBLOX_M8_TRIGGER
	.trigger_set = ublox_m8_trigger_set,
#endif
};

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

static int ublox_m8_pin_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

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
			   GPIO_OUTPUT_INACTIVE);

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
	 */
	drv_data->timepulse_gpio = device_get_binding(cfg->timepulse_gpio_name);
	if (drv_data->timepulse_gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->timepulse_gpio_name);
		return -EINVAL;
	}

	/* configure pull-up to check if module is connected */
	gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
			   cfg->timepulse_gpio_flags |
			   GPIO_INPUT | GPIO_PULL_UP);

	/* read the state of the pin */
	ret = gpio_pin_get(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin);
	if (!ret) {
		/*
		 * logical 0 indicates chip is driving pin low
		 * so chip is present.
		 * Turn off the pull-up.
		 */
		gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
				cfg->timepulse_gpio_flags |
				GPIO_INPUT);
	} else {
		/*
		 * logical 1 indicates pin is being pulled up my mcu
		 * so the chip is not present.
		 * Disable input.
		 */
		gpio_pin_configure(drv_data->timepulse_gpio, cfg->timepulse_gpio_pin,
				GPIO_DISCONNECTED);
	}

	/*
	 * setup TXD gpio
	 * 
	 * The TX_READY function can be mapped to TXD (PIO 06).
	 * The TX_READY function is disabled by default.
	 * 
	 * The UART is not used to TXD is not initialized.
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

	/* assert reset pin */
	drv_data->device_state = GNSS_DEVICE_STATE_RESET;
	gpio_pin_set(drv_data->reset_gpio, cfg->reset_gpio_pin, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set(drv_data->reset_gpio, cfg->reset_gpio_pin, 0);

	return 0;
}

static void ublox_m8_config_work_handler(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, config_work);
	struct device *dev = drv_data->dev;
	int ret;

	if (drv_data->device_state != GNSS_DEVICE_STATE_INITIALIZED) {
		LOG_ERR("device not initialized");
		return;
	}

	ret = ublox_m8_connect_status(dev);
	if (ret < 0) {
		LOG_ERR("device did not ack i2c write");
		drv_data->device_state = GNSS_DEVICE_STATE_DISCONNECTED;
		return;
	}

#ifdef CONFIG_UBLOX_M8_TRIGGER
	ret = ublox_m8_init_interrupt(dev);
	if (ret < 0) {
		LOG_DBG("Failed to initialize interrupts.");
	}
#endif

	ret = ublox_m8_configure_uart_port(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_configure_ddc_port(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_configure_rxm(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_configure_pm2(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_get_device_id(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_get_version(dev);
	if (ret < 0) {
		return;
	}

	ret = ublox_m8_configure_save_bbr(dev);
	if (ret < 0) {
		return;
	}

	ublox_m8_trigger_set(dev, &txready_trigger, ublox_m8_txready_handler);
	drv_data->device_state = GNSS_DEVICE_STATE_CONFIGURED;
	LOG_INF("ublox m8 configured");

}

int ublox_m8_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	drv_data->device_state = GNSS_DEVICE_STATE_UNINITIALIZED;
	while (k_uptime_ticks() < k_us_to_cyc_ceil32(UBLOX_M8_STARTUP_TIME_USEC)) {
		/* wait for chip to power up */
	}

	drv_data->i2c = device_get_binding(cfg->i2c_name);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->i2c_name);
		return -EINVAL;
	}
	drv_data->dev = dev;
	// drv_data->buf_pool = &ublox_buf_pool;
	drv_data->timeout = K_MSEC(UBLOX_M8_MESSAGE_TIMEOUT_MSEC);
	drv_data->frame.ubx.payload = drv_data->frame.payload;
	drv_data->poll_status = 0;
	k_pipe_init(&drv_data->rx_pipe, drv_data->rb_buf, sizeof(drv_data->rb_buf));
	k_sem_init(&drv_data->msg_sem, 0, 1);
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

	ret = ublox_m8_pin_init(dev);
	if (ret < 0) {
		return ret;
	}

	ublox_m8_trigger_set(dev, &poll_trigger, ublox_m8_poll_handler);

	drv_data->device_state = GNSS_DEVICE_STATE_INITIALIZED;
	k_delayed_work_submit(&drv_data->config_work, K_SECONDS(2));
	return 0;
}

static struct ublox_m8_data ublox_m8_drv_data;

static const struct ublox_m8_dev_config ublox_m8_config = {
	.i2c_name = DT_INST_0_UBLOX_M8_BUS_NAME,
	.i2c_addr = DT_INST_0_UBLOX_M8_BASE_ADDRESS,
	.txready_gpio_name = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_CONTROLLER,
	.txready_gpio_pin = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_PIN,
	.txready_gpio_flags = DT_INST_0_UBLOX_M8_TXREADY_GPIOS_FLAGS,
	.reset_gpio_name = DT_INST_0_UBLOX_M8_RESET_GPIOS_CONTROLLER,
	.reset_gpio_pin = DT_INST_0_UBLOX_M8_RESET_GPIOS_PIN,
	.reset_gpio_flags = DT_INST_0_UBLOX_M8_RESET_GPIOS_FLAGS,
	.timepulse_gpio_name = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_CONTROLLER,
	.timepulse_gpio_pin = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_PIN,
	.timepulse_gpio_flags = DT_INST_0_UBLOX_M8_TIMEPULSE_GPIOS_FLAGS,
	.safeboot_gpio_name = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_CONTROLLER,
	.safeboot_gpio_pin = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_PIN,
	.safeboot_gpio_flags = DT_INST_0_UBLOX_M8_SAFEBOOT_GPIOS_FLAGS,
	.extint_gpio_name = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_CONTROLLER,
	.extint_gpio_pin = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_PIN,
	.extint_gpio_flags = DT_INST_0_UBLOX_M8_EXTINT_GPIOS_FLAGS,
	.rxd_gpio_name = DT_INST_0_UBLOX_M8_RXD_GPIOS_CONTROLLER,
	.rxd_gpio_pin = DT_INST_0_UBLOX_M8_RXD_GPIOS_PIN,
	.rxd_gpio_flags = DT_INST_0_UBLOX_M8_RXD_GPIOS_FLAGS,
	// .txd_gpio_name = DT_INST_0_UBLOX_M8_TXD_GPIOS_CONTROLLER,
	// .txd_gpio_pin = DT_INST_0_UBLOX_M8_TXD_GPIOS_PIN,
	// .txd_gpio_flags = DT_INST_0_UBLOX_M8_TXD_GPIOS_FLAGS,
};

DEVICE_AND_API_INIT(ublox_m8, DT_INST_0_UBLOX_M8_LABEL, ublox_m8_init,
		    &ublox_m8_drv_data, &ublox_m8_config, POST_KERNEL,
		    CONFIG_GNSS_INIT_PRIORITY, &ublox_m8_driver_api);
