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
#include <net/buf.h>

#include "ublox_m8.h"

#define LOG_LEVEL CONFIG_GNSS_LOG_LEVEL
LOG_MODULE_REGISTER(UBLOX_M8);

NET_BUF_POOL_DEFINE(ublox_buf_pool, 8, 32, 0, NULL);

FOR_EACH(UBX_HEADER_CFG_DEFINE, cfg, pm2, prt, pms, pwr, rxm);
FOR_EACH(UBX_HEADER_NAV_DEFINE, pvt, status, geofence, sat, timeutc);
FOR_EACH(UBX_HEADER_SEC_DEFINE, uniqid);
FOR_EACH(UBX_HEADER_ACK_DEFINE, ack, nak);

const struct ubx_frame_status ubx_frame_status_request_init = {
	.req_sent = false,
	.resp_received = false,
	.ack_required = true,
	.ack_received = false,
	.ack = false,
	.checksum_valid = false,
};

const union ubx_cfg_prt_txready ubx_txready_config = {
	.bit.en = 1,
	.bit.pol = 0,
	.bit.pin = 6,
	.bit.thres = 1,
};

/*
 * UBX_CFG_CFG_REQ
 * Clear, save and load configurations
 */
// UBX_HEADER_DEFINE(cfg,cfg);

/*
 * UBX_CFG_PRT_PM2
 * Extended power management configuration
 */
// UBX_HEADER_DEFINE(cfg,pm2);

/*
 * UBX_CFG_PRT_REQ
 * Port configuration for I2C (DDC) port
 */
// UBX_HEADER_DEFINE(cfg,prt);

/*
 * UBX_NAV_PVT_REQ
 * Navigation position velocity time solution
 */
// UBX_HEADER_DEFINE(nav,pvt);

/*
 * UBX_SEC_UNIQID_REQ
 * Unique chip ID
 */
// UBX_HEADER_DEFINE(sec,uniqid);

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

//Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
//This is used when receiving messages from module
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

static void ubx_validate_frame_checksum(struct ubx_frame *frame)
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
	}
}

static int ublox_m8_parse_ubx(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	struct ubx_frame frame;
	int ret;

	ret = 0;
	return ret;
}

static void ublox_m8_msg_thread_cb(void *arg)
{
	struct device *dev = arg;
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	/* process message */
	ret = ublox_m8_parse_ubx(dev);
	if (ret < 0) {
		return;
	}
	/* Everything is ok */
	drv_data->last_error = 0;
	k_sem_give(&drv_data->response_sem);
	return;
}

static void ublox_m8_msg_thread(int dev_ptr, int unused)
{
	struct device *dev = INT_TO_POINTER(dev_ptr);
	struct ublox_m8_data *drv_data = dev->driver_data;

	ARG_UNUSED(unused);

	while (1) {
		k_sem_take(&drv_data->msg_sem, K_FOREVER);
		ublox_m8_msg_thread_cb(dev);
	}
}

static void ublox_m8_msg_work_cb(struct k_work *work)
{
	struct ublox_m8_data *drv_data =
		CONTAINER_OF(work, struct ublox_m8_data, msg_work);

	ublox_m8_msg_thread_cb(drv_data->dev);
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
static int ublox_m8_reg_read(struct device *dev, u8_t reg, u8_t *val, int size)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr, reg, (u8_t *)val,
			     size);
	return ret;
}

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
	u16_t read_bytes, bytes_put;
	u8_t *buf;

	/* get the message length */
	ret = i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			     UBLOX_M8_REGISTER_LEN, drv_data->rx_buf, 2);
	if (ret < 0) {
		LOG_DBG("addr error");
		goto done;
	}
	/* ddc port message length is received big endian */
	msg_len = sys_get_be16(drv_data->rx_buf);
	if (msg_len > UBX_MAX_FRAME_SIZE) {
		LOG_ERR("message size great than buffer size");
		ret = -ENOMEM;
		goto done;
	}
	if (msg_len == 0) {
		LOG_DBG("no message");
		ret = -EAGAIN;
		goto done;
	}

	/* read the first byte */
	buf = drv_data->rx_buf;
	ret = i2c_read(drv_data->i2c, buf, 1, cfg->i2c_addr);
	if (ret < 0) {
		LOG_DBG("read error");
		ret = -EIO;
		goto done;
	}
	if (drv_data->rx_buf[0] == 0xFF) {
		LOG_DBG("not ready");
		ret = -EAGAIN;
		goto done;
	}

	ring_buf_put(&drv_data->rx_rb, buf, 1);
	msg_len--;
	buf++;
	read_bytes = MIN(msg_len, sizeof(drv_data->rx_buf));
	while (msg_len > 0) {
		ret = i2c_read(drv_data->i2c, buf, read_bytes, cfg->i2c_addr);
		if (ret < 0) {
			LOG_DBG("read error");
			goto done;
		}
		bytes_put = ring_buf_put(&drv_data->rx_rb, buf, read_bytes);
		if (bytes_put < read_bytes) {
			LOG_WRN("ring buffer full");
			break;
		}
		msg_len -= read_bytes;
		buf += read_bytes;
		read_bytes = MIN(msg_len, sizeof(drv_data->rx_buf));
	}
	// k_sem_give(&drv_data->msg_sem);
	k_work_submit(&drv_data->msg_work);
done:
	return ret;
}

static int ublox_m8_message_send(struct device *dev, struct ubx_frame *frame)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

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
	frame->status.req_sent = true;

	if (drv_data->timeout == K_NO_WAIT) {
		ret = 0;
		goto done;
	}

	/* wait for response */
	k_sem_reset(&drv_data->response_sem);
	ret = k_sem_take(&drv_data->response_sem, drv_data->timeout);
	if (ret == 0) {
		ret = drv_data->last_error;
	} else if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
	}

done:
	return ret;
}

static void ublox_m8_txready_handler(struct device *dev, struct gnss_trigger *trigger)
{
	int ret;

	ret = ublox_m8_message_get(dev);
	if (ret < 0) {
		LOG_DBG("");
	}
	return;
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
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	struct ubx_frame frame;
	u8_t payload[UBX_SEC_UNIQID_PAYLOAD_SIZE] = {};
	int ret;

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_request_init, UBX_FRAME_STATUS_SIZE);
	/* prepare frame request */
	memcpy(&frame, &ubx_header_sec_uniqid, UBX_FRAME_HEADER_SIZE);
	frame.payload = payload;
	ret = ublox_m8_message_send(dev, &frame);
	if (ret < 0) {
		LOG_ERR("%s: Failed to get Device ID",
			DT_INST_0_UBLOX_M8_LABEL);
		return ret;
	}

	LOG_INF("unique id: 0x%02x0x%02x0x%02x0x%02x0x%02x", payload[4], payload[5], payload[6], payload[7], payload[8]);
	return 0;
}

static int ublox_m8_set_txready(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	struct ubx_frame frame;
	u8_t payload[UBX_SEC_UNIQID_PAYLOAD_SIZE] = {};
	int ret;

	/* initialize frame status */
	memcpy(&frame.status, &ubx_frame_status_request_init, UBX_FRAME_STATUS_SIZE);
	/* prepare frame request */
	memcpy(&frame, &ubx_header_cfg_prt, UBX_FRAME_HEADER_SIZE);
	frame.payload = payload;
	payload[0] = UBLOX_SAM_M8Q_TXREADY_PIO;
	sys_put_le16(ubx_txready_config.reg, payload+2);
	frame.len = 4;
	ret = ublox_m8_message_send(dev, &frame);
	if (ret < 0) {
		LOG_ERR("%s: Failed to get Device ID",
			DT_INST_0_UBLOX_M8_LABEL);
		return ret;
	}

	return 0;
}

static int ublox_m8_sample_fetch(struct device *dev, enum gnss_channel chan)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret;

	ARG_UNUSED(drv_data);

	__ASSERT_NO_MSG(chan == GNSS_CHAN_ALL ||
			chan == GNSS_CHAN_TIME ||
			chan == GNSS_CHAN_POSITION ||
			chan == GNSS_CHAN_VELOCITY);

	switch (chan) {
	case GNSS_CHAN_ALL:
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
			      struct gnss_pvt *val)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	int ret = 0;

	ARG_UNUSED(drv_data);

	switch ((int) chan) {
	case GNSS_CHAN_ALL:
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
	.attr_set = ublox_m8_attr_set,
	.sample_fetch = ublox_m8_sample_fetch,
	.channel_get = ublox_m8_channel_get,
#if CONFIG_UBLOX_M8_TRIGGER
	.trigger_set = ublox_m8_trigger_set,
#endif
};

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
			   GPIO_OUTPUT);

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
			   GPIO_OUTPUT);

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
			   GPIO_OUTPUT);

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

	// ublox_m8_apply_defaults(dev);
	// ublox_m8_get_registers(dev);

	return 0;
}

int ublox_m8_init(struct device *dev)
{
	struct ublox_m8_data *drv_data = dev->driver_data;
	const struct ublox_m8_dev_config *cfg = dev->config->config_info;
	int ret;

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
	drv_data->buf_pool = &ublox_buf_pool;
	drv_data->timeout = K_MSEC(UBLOX_M8_MESSAGE_TIMEOUT_MSEC);
	ring_buf_init(&drv_data->rx_rb, sizeof(drv_data->rx_rb_buf), drv_data->rx_rb_buf);
	k_sem_init(&drv_data->msg_sem, 0, 1);
	k_sem_init(&drv_data->response_sem, 0, 1);
	k_work_init(&drv_data->msg_work, ublox_m8_msg_work_cb);

	ret = ublox_m8_pin_init(dev);

	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			DT_INST_0_UBLOX_M8_BUS_NAME);
		return -EINVAL;
	}

#ifdef CONFIG_UBLOX_M8_TRIGGER
	ret = ublox_m8_init_interrupt(dev);
	if (ret < 0) {
		LOG_DBG("Failed to initialize interrupts.");
	} else {
		ublox_m8_trigger_set(dev, &drv_data->data_ready_trigger, ublox_m8_txready_handler);
	}

	ret = ublox_m8_set_txready(dev);
	if (ret < 0) {
		return -EIO;
	}
#endif

	ret = ublox_m8_get_device_id(dev);
	if (ret < 0) {
		return -EIO;
	}

	LOG_DBG("module detected");

	return ret;
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
