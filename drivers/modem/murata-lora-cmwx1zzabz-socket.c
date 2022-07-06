/*
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file murata-lora-cmwx1zzabz-socket.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * SPDX-License-Identifier: Apache-2.0
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */


#include <logging/log.h>
LOG_MODULE_REGISTER(modem_murata_lora_cmwx1zzabz, CONFIG_MODEM_LOG_LEVEL);

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <device.h>
#include <init.h>

#include <net/net_if.h>

#include "../../../zephyr/drivers/modem/modem_context.h"
#include "../../../zephyr/drivers/modem/modem_socket.h"
#include "../../../zephyr/drivers/modem/modem_cmd_handler.h"
#include "../../../zephyr/drivers/modem/modem_iface_uart.h"

#if !defined(CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO)
#define CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO ""
#endif

/* pin settings */
enum mdm_control_pins {
	MDM_POWER = 0,
	MDM_RESET,
#if defined(DT_INST_0_MURATA_LORA_MDM_VINT_GPIOS_CONTROLLER)
	MDM_VINT,
#endif
	MDM_BOOT0,
	MDM_IRQ,
};

static struct modem_pin modem_pins[] = {
#if defined(DT_INST_0_MURATA_LORA_MDM_POWER_GPIOS_CONTROLLER)
	/* MDM_POWER */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_POWER_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_POWER_GPIOS_PIN, GPIO_OUTPUT),
#endif

	/* MDM_RESET */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_PIN, GPIO_OUTPUT),

#if defined(DT_INST_0_MURATA_LORA_MDM_VINT_GPIOS_CONTROLLER)
	/* MDM_VINT */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_VINT_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_VINT_GPIOS_PIN, GPIO_INPUT),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_CONTROLLER)
	/* MDM_BOOT0 */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_PIN, GPIO_OUTPUT),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_CONTROLLER)
	/* MDM_IRQ */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_PIN, GPIO_INPUT),
#endif
};

#define MDM_UART_DEV_NAME		DT_INST_0_MURATA_LORA_BUS_NAME

#define MDM_POWER_ENABLE		1
#define MDM_POWER_DISABLE		0
#define MDM_RESET_NOT_ASSERTED		1
#define MDM_RESET_ASSERTED		0
#if defined(DT_INST_0_MURATA_LORA_MDM_VINT_GPIOS_CONTROLLER)
#define MDM_VINT_ENABLE			1
#define MDM_VINT_DISABLE		0
#endif
#define MDM_BOOT0_ENABLE		1
#define MDM_BOOT0_DISABLE		0
#define MDM_IRQ_NOT_ASSERTED		1
#define MDM_IRQ_ASSERTED		0

#define MDM_CMD_TIMEOUT			K_SECONDS(10)
#define MDM_REGISTRATION_TIMEOUT	K_SECONDS(180)
#define MDM_PROMPT_CMD_DELAY		K_MSEC(75)

#define MDM_MAX_DATA_LENGTH		1024
#define MDM_RECV_MAX_BUF		8
#define MDM_RECV_BUF_SIZE		128

#define MDM_MAX_SOCKETS			6
#define MDM_BASE_SOCKET_NUM		0

#define MDM_NETWORK_RETRY_COUNT		3
#define MDM_WAIT_FOR_RSSI_COUNT		10
#define MDM_WAIT_FOR_RSSI_DELAY		K_SECONDS(2)

#define BUF_ALLOC_TIMEOUT		K_SECONDS(1)

#define MDM_MANUFACTURER_LENGTH		10
#define MDM_MODEL_LENGTH		16
#define MDM_REVISION_LENGTH		64
#define MDM_DEVEUI_LENGTH			16

#define RSSI_TIMEOUT_SECS		30

NET_BUF_POOL_DEFINE(mdm_recv_pool, MDM_RECV_MAX_BUF, MDM_RECV_BUF_SIZE,
		    0, NULL);

/* RX thread structures */
K_THREAD_STACK_DEFINE(modem_rx_stack,
		      CONFIG_MODEM_MURATA_LORA_RX_STACK_SIZE);
struct k_thread modem_rx_thread;

/* RX thread work queue */
K_THREAD_STACK_DEFINE(modem_workq_stack,
		      CONFIG_MODEM_MURATA_LORA_RX_WORKQ_STACK_SIZE);
static struct k_work_q modem_workq;

/* socket read callback data */
struct socket_read_data {
	char *recv_buf;
	size_t recv_buf_len;
	struct sockaddr *recv_addr;
	u16_t recv_read_len;
};

/* driver data */
struct modem_data {
	struct net_if *net_iface;
	u8_t mac_addr[6];

	/* modem interface */
	struct modem_iface_uart_data iface_data;
	u8_t iface_isr_buf[MDM_RECV_BUF_SIZE];
	u8_t iface_rb_buf[MDM_MAX_DATA_LENGTH];

	/* modem cmds */
	struct modem_cmd_handler_data cmd_handler_data;
	u8_t cmd_read_buf[MDM_RECV_BUF_SIZE];
	u8_t cmd_match_buf[MDM_RECV_BUF_SIZE];

	/* socket data */
	struct modem_socket_config socket_config;
	struct modem_socket sockets[MDM_MAX_SOCKETS];

	/* RSSI work */
	struct k_delayed_work rssi_query_work;

	/* modem data */
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_deveui[MDM_DEVEUI_LENGTH];

	/* modem state */
	int ev_creg;
	int network_joined;

	/* response semaphore */
	struct k_sem sem_response;
};

static struct modem_data mdata;
static struct modem_context mctx;

/* helper macro to keep readability */
#define ATOI(s_, value_, desc_) modem_atoi(s_, value_, desc_, __func__)

/**
 * @brief  Convert string to long integer, but handle errors
 *
 * @param  s: string with representation of integer number
 * @param  err_value: on error return this value instead
 * @param  desc: name the string being converted
 * @param  func: function where this is called (typically __func__)
 *
 * @retval return integer conversion on success, or err_value on error
 */
static int modem_atoi(const char *s, const int err_value,
		      const char *desc, const char *func)
{
	int ret;
	char *endptr;

	ret = (int)strtol(s, &endptr, 10);
	if (!endptr || *endptr != '\0') {
		LOG_ERR("bad %s '%s' in %s", log_strdup(s), log_strdup(desc),
			log_strdup(func));
		return err_value;
	}

	return ret;
}

/* convert a hex-encoded buffer back into a binary buffer */
static int hex_to_binary(struct modem_cmd_handler_data *data,
			 u16_t data_length,
			 u8_t *bin_buf, u16_t bin_buf_len)
{
	int i;
	u8_t c = 0U, c2;

	/* make sure we have room for a NUL char at the end of bin_buf */
	if (data_length > bin_buf_len - 1) {
		return -ENOMEM;
	}

	for (i = 0; i < data_length * 2; i++) {
		if (!data->rx_buf) {
			return -ENOMEM;
		}

		c2 = *data->rx_buf->data;
		if (isdigit(c2)) {
			c += c2 - '0';
		} else if (isalpha(c2)) {
			c += c2 - (isupper(c2) ? 'A' - 10 : 'a' - 10);
		} else {
			return -EINVAL;
		}

		if (i % 2) {
			bin_buf[i / 2] = c;
			c = 0U;
		} else {
			c = c << 4;
		}

		/* pull data from buf and advance to the next frag if needed */
		net_buf_pull_u8(data->rx_buf);
		if (!data->rx_buf->len) {
			data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
		}
	}

	/* end with a NUL char */
	bin_buf[i / 2] = '\0';
	return 0;
}

/* send binary data via the +USO[ST/WR] commands */
static int send_socket_data(struct modem_socket *sock,
			    const struct sockaddr *dst_addr,
			    struct modem_cmd *handler_cmds,
			    size_t handler_cmds_len,
			    const char *buf, size_t buf_len, int timeout)
{
	int ret;
	char send_buf[sizeof("AT+USO**=#,!###.###.###.###!,#####,####\r\n")];
	u16_t dst_port = 0U;

	if (!sock) {
		return -EINVAL;
	}

	if (sock->ip_proto == IPPROTO_UDP) {
		ret = modem_context_get_addr_port(dst_addr, &dst_port);
		snprintk(send_buf, sizeof(send_buf),
			 "AT+USOST=%d,\"%s\",%u,%zu", sock->id,
			 modem_context_sprint_ip_addr(dst_addr),
			 dst_port, buf_len);
	} else {
		snprintk(send_buf, sizeof(send_buf), "AT+USOWR=%d,%zu",
			 sock->id, buf_len);
	}

	k_sem_take(&mdata.cmd_handler_data.sem_tx_lock, K_FOREVER);

	ret = modem_cmd_send_nolock(&mctx.iface, &mctx.cmd_handler,
				    NULL, 0U, send_buf, NULL, K_NO_WAIT);
	if (ret < 0) {
		goto exit;
	}

	/* set command handlers */
	ret = modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    handler_cmds, handler_cmds_len,
					    true);
	if (ret < 0) {
		goto exit;
	}

	/* slight pause per spec so that @ prompt is received */
	k_sleep(MDM_PROMPT_CMD_DELAY);
#if defined(CONFIG_MODEM_MURATA_LORA_CMWX1ZZABZ)
	/*
	 * HACK: Apparently, enabling HEX transmit mode also
	 * affects the BINARY send method.  We need to encode
	 * the "binary" data as HEX values here
	 * TODO: Something faster
	 */
	for (int i = 0; i < buf_len; i++) {
		snprintk(send_buf, sizeof(send_buf), "%02x", buf[i]);
		mctx.iface.write(&mctx.iface, send_buf, 2U);
	}
#else
	mctx.iface.write(&mctx.iface, buf, buf_len);
#endif

	if (timeout == K_NO_WAIT) {
		ret = 0;
		goto exit;
	}

	k_sem_reset(&mdata.sem_response);
	ret = k_sem_take(&mdata.sem_response, timeout);

	if (ret == 0) {
		ret = modem_cmd_handler_get_error(&mdata.cmd_handler_data);
	} else if (ret == -EAGAIN) {
		ret = -ETIMEDOUT;
	}

exit:
	/* unset handler commands and ignore any errors */
	(void)modem_cmd_handler_update_cmds(&mdata.cmd_handler_data,
					    NULL, 0U, false);
	k_sem_give(&mdata.cmd_handler_data.sem_tx_lock);

	return ret;
}

/*
 * Modem Response Command Handlers
 */

/* Handler: OK */
MODEM_CMD_DEFINE(on_cmd_ok)
{
	if (!len) {
		LOG_DBG("+OK response: no payload.");
		goto done;
	}

	/*
	 * make sure we still have buf data and next char in the buffer is an
	 * equal character.
	 */
	if (!data->rx_buf || *data->rx_buf->data != '=') {
		LOG_ERR("+OK response:  incorrect format.");
		goto done;
	}

	/* skip equal */
	len--;
	net_buf_pull_u8(data->rx_buf);
	if (!data->rx_buf->len) {
		data->rx_buf = net_buf_frag_del(NULL, data->rx_buf);
	}


done:
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: ERROR */
MODEM_CMD_DEFINE(on_cmd_error)
{
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: +CME Error: <err>[0] */
MODEM_CMD_DEFINE(on_cmd_exterror)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EIO);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_param_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EINVAL);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_busy_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EBUSY);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_overflow_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -E2BIG);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_network_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -ENETDOWN);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_rx_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -EBADMSG);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/* Handler: Error */
MODEM_CMD_DEFINE(on_cmd_unknown_error)
{
	/* TODO: map extended error codes to values */
	modem_cmd_handler_set_error(data, -ENOMSG);
	k_sem_give(&mdata.sem_response);
	return 0;
}

/*
 * Modem Info Command Handlers
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_manufacturer,
				    sizeof(mdata.mdm_manufacturer) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(mdata.mdm_manufacturer));
	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_model,
				    sizeof(mdata.mdm_model) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", log_strdup(mdata.mdm_model));
	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_revision,
				    sizeof(mdata.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", log_strdup(mdata.mdm_revision));
	return 0;
}

/* Handler: <DEVEUI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_deveui)
{
	size_t out_len;

	out_len = net_buf_linearize(mdata.mdm_deveui, sizeof(mdata.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	mdata.mdm_deveui[out_len] = '\0';
	LOG_INF("DEVEUI: %s", log_strdup(mdata.mdm_deveui));
	return 0;
}

/*
 * Handler: +CESQ: <rxlev>[0],<ber>[1],<rscp>[2],<ecn0>[3],<rsrq>[4],<rsrp>[5]
 */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_rssi_cesq)
{
	int rsrp;

	rsrp = ATOI(argv[5], 0, "rsrp");
	if (rsrp >= 0 && rsrp <= 97) {
		mctx.data_rssi = -140 + rsrp;
	} else {
		mctx.data_rssi = -1000;
	}

	LOG_INF("RSRP: %d", mctx.data_rssi);
	return 0;
}

/*
 * MODEM UNSOLICITED NOTIFICATION HANDLERS
 */


/* Handler: +EVENT=0,0 (Module reset event) */
MODEM_CMD_DEFINE(on_cmd_reset)
{
	LOG_DBG("Murata module reset");
	return 0;
}

/* Handler: +EVENT=1,1 (Network join event) */
MODEM_CMD_DEFINE(on_cmd_join)
{
	mdata.network_joined = true;
	LOG_DBG("Network join");
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_rx_data)
{
	return 0;
}

/* RX thread */
static void modem_rx(void)
{
	while (true) {
		/* wait for incoming data */
		k_sem_take(&mdata.iface_data.rx_sem, K_FOREVER);

		mctx.cmd_handler.process(&mctx.cmd_handler, &mctx.iface);

		/* give up time if we have a solid stream of data */
		k_yield();
	}
}

static int pin_init(void)
{
	LOG_INF("Setting Modem Pins");

	LOG_DBG("MDM_BOOT0_PIN -> NOT_ASSERTED");
	modem_pin_write(&mctx, MDM_BOOT0, MDM_BOOT0_DISABLE);

	LOG_DBG("MDM_RESET_PIN -> NOT_ASSERTED");
	modem_pin_write(&mctx, MDM_RESET, MDM_RESET_NOT_ASSERTED);
	k_sleep(1);

	LOG_DBG("MDM_RESET_PIN -> NOT_ASSERTED");
	modem_pin_write(&mctx, MDM_RESET, MDM_RESET_ASSERTED);
	k_sleep(1);

	LOG_DBG("MDM_RESET_PIN -> NOT_ASSERTED");
	modem_pin_write(&mctx, MDM_RESET, MDM_RESET_NOT_ASSERTED);

	LOG_INF("... Done!");

	return 0;
}

static void modem_rssi_query_work(struct k_work *work)
{
	struct modem_cmd cmd =
		MODEM_CMD("+CESQ: ", on_cmd_atcmdinfo_rssi_cesq, 6U, ",");
	static char *send_cmd = "AT+CESQ";
	int ret;

	/* query modem RSSI */
	ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
			     &cmd, 1U, send_cmd, &mdata.sem_response,
			     MDM_CMD_TIMEOUT);
	if (ret < 0) {
		LOG_ERR("AT+C[E]SQ ret:%d", ret);
	}

	/* re-start RSSI query work */
	if (work) {
		k_delayed_work_submit_to_queue(&modem_workq,
					       &mdata.rssi_query_work,
					       K_SECONDS(RSSI_TIMEOUT_SECS));
	}
}

static void modem_reset(void)
{
	int ret = 0, retry_count = 0, counter = 0;
	static struct setup_cmd setup_cmds[] = {
		/* query modem info */
		SETUP_CMD("AT+DEV?", "", on_cmd_atcmdinfo_model, 0U, ""),
		SETUP_CMD("AT+VER?", "", on_cmd_atcmdinfo_revision, 0U, ""),
		SETUP_CMD("AT+DEVEUI?", "", on_cmd_atcmdinfo_deveui, 0U, ""),
		/* setup PDP context definition */
		SETUP_CMD_NOHANDLE("AT+CGDCONT=1,\"IP\",\""
					CONFIG_MODEM_MURATA_LORA_APN "\""),
		/* start functionality */
		SETUP_CMD_NOHANDLE("AT+CFUN=1"),
	};

	static struct setup_cmd setup_cmds_post[] = {
		/* set the APN */
		SETUP_CMD_NOHANDLE("AT+UPSD=0,1,\""
				CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO "\""),
		/* set dynamic IP */
		SETUP_CMD_NOHANDLE("AT+UPSD=0,7,\"0.0.0.0\""),
		/* activate the GPRS connection */
		SETUP_CMD_NOHANDLE("AT+UPSDA=0,3"),
	};

	/* bring down network interface */
	atomic_clear_bit(mdata.net_iface->if_dev->flags, NET_IF_UP);

restart:
	/* stop RSSI delay work */
	k_delayed_work_cancel(&mdata.rssi_query_work);

	pin_init();

	LOG_INF("Waiting for modem to respond");

	/* Give the modem a while to start responding to simple 'AT' commands.
	 * Also wait for CSPS=1 or RRCSTATE=1 notification
	 */
	ret = -1;
	while (counter++ < 50 && ret < 0) {
		k_sleep(K_SECONDS(2));
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0, "AT", &mdata.sem_response,
				     MDM_CMD_TIMEOUT);
		if (ret < 0 && ret != -ETIMEDOUT) {
			break;
		}
	}

	if (ret < 0) {
		LOG_ERR("MODEM WAIT LOOP ERROR: %d", ret);
		goto error;
	}

	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds, ARRAY_SIZE(setup_cmds),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		goto error;
	}


	if (strlen(CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO) > 0) {
		/* use manual MCC/MNO entry */
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0,
				     "AT+COPS=1,2,\""
					CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO
					"\"",
				     &mdata.sem_response,
				     MDM_REGISTRATION_TIMEOUT);
	} else {
		/* register operator automatically */
		ret = modem_cmd_send(&mctx.iface, &mctx.cmd_handler,
				     NULL, 0, "AT+COPS=0,0",
				     &mdata.sem_response,
				     MDM_REGISTRATION_TIMEOUT);
	}

	if (ret < 0) {
		LOG_ERR("AT+COPS ret:%d", ret);
		goto error;
	}

	LOG_INF("Waiting for network");

	/*
	 * TODO: A lot of this should be setup as a 3GPP module to handle
	 * basic connection to the network commands / polling
	 */

	/* wait for +CREG: 1(normal) or 5(roaming) */
	counter = 0;
	while (counter++ < 20 && mdata.ev_creg != 1 && mdata.ev_creg != 5) {
		k_sleep(K_SECONDS(1));
	}

	/* query modem RSSI */
	modem_rssi_query_work(NULL);
	k_sleep(MDM_WAIT_FOR_RSSI_DELAY);

	counter = 0;
	/* wait for RSSI < 0 and > -1000 */
	while (counter++ < MDM_WAIT_FOR_RSSI_COUNT &&
	       (mctx.data_rssi >= 0 ||
		mctx.data_rssi <= -1000)) {
		modem_rssi_query_work(NULL);
		k_sleep(MDM_WAIT_FOR_RSSI_DELAY);
	}

	if (mctx.data_rssi >= 0 || mctx.data_rssi <= -1000) {
		retry_count++;
		if (retry_count >= MDM_NETWORK_RETRY_COUNT) {
			LOG_ERR("Failed network init.  Too many attempts!");
			ret = -ENETUNREACH;
			goto error;
		}

		LOG_ERR("Failed network init.  Restarting process.");
		goto restart;
	}

	ret = modem_cmd_handler_setup_cmds(&mctx.iface, &mctx.cmd_handler,
					   setup_cmds_post,
					   ARRAY_SIZE(setup_cmds_post),
					   &mdata.sem_response,
					   MDM_REGISTRATION_TIMEOUT);
	if (ret < 0) {
		goto error;
	}

	LOG_INF("Network is ready.");

	/* Set iface up */
	net_if_up(mdata.net_iface);

	/* start RSSI query */
	k_delayed_work_submit_to_queue(&modem_workq,
				       &mdata.rssi_query_work,
				       K_SECONDS(RSSI_TIMEOUT_SECS));

error:
	return;
}

#define HASH_MULTIPLIER		37
static u32_t hash32(char *str, int len)
{
	u32_t h = 0;
	int i;

	for (i = 0; i < len; ++i) {
		h = (h * HASH_MULTIPLIER) + str[i];
	}

	return h;
}

static inline u8_t *modem_get_mac(struct device *dev)
{
	struct modem_data *data = dev->driver_data;
	u32_t hash_value;

	data->mac_addr[0] = 0x00;
	data->mac_addr[1] = 0x10;

	/* use DEVEUI for mac_addr */
	hash_value = hash32(mdata.mdm_deveui, strlen(mdata.mdm_deveui));

	UNALIGNED_PUT(hash_value, (u32_t *)(data->mac_addr + 2));

	return data->mac_addr;
}

static void modem_net_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct modem_data *data = dev->driver_data;

	net_if_set_link_addr(iface, modem_get_mac(dev),
			     sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	data->net_iface = iface;
}

static struct net_if_api api_funcs = {
	.init = modem_net_iface_init,
};

static struct modem_cmd response_cmds[] = {
	MODEM_CMD("+OK", on_cmd_ok, 0U, ""),
	MODEM_CMD("+ERR", on_cmd_error, 0U, ""),
	MODEM_CMD("+ERR_PARAM", on_cmd_param_error, 0U, ""),
	MODEM_CMD("+ERR_BUSY", on_cmd_busy_error, 0U, ""),
	MODEM_CMD("+ERR_PARAM_OVERFLOW", on_cmd_overflow_error, 0U, ""),
	MODEM_CMD("+ERR_NO_NETWORK", on_cmd_network_error, 0U, ""),
	MODEM_CMD("+ERR_RX", on_cmd_rx_error, 0U, ""),
	MODEM_CMD("+ERR_UNKNOWN", on_cmd_unknown_error, 0U, ""),
};

static struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("+EVENT=0,0", on_cmd_reset, 0U, ""),
	MODEM_CMD("+EVENT=1,1", on_cmd_join, 0U, ""),
	MODEM_CMD("+RECV=", on_cmd_rx_data, 2U, ","),
};

static int modem_init(struct device *dev)
{
	int ret = 0;

	ARG_UNUSED(dev);

	k_sem_init(&mdata.sem_response, 0, 1);

	/* initialize the work queue */
	k_work_q_start(&modem_workq,
		       modem_workq_stack,
		       K_THREAD_STACK_SIZEOF(modem_workq_stack),
		       K_PRIO_COOP(7));

	/* cmd handler */
	mdata.cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	mdata.cmd_handler_data.cmds[CMD_UNSOL] = unsol_cmds;
	mdata.cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsol_cmds);
	mdata.cmd_handler_data.read_buf = &mdata.cmd_read_buf[0];
	mdata.cmd_handler_data.read_buf_len = sizeof(mdata.cmd_read_buf);
	mdata.cmd_handler_data.match_buf = &mdata.cmd_match_buf[0];
	mdata.cmd_handler_data.match_buf_len = sizeof(mdata.cmd_match_buf);
	mdata.cmd_handler_data.buf_pool = &mdm_recv_pool;
	mdata.cmd_handler_data.alloc_timeout = BUF_ALLOC_TIMEOUT;
	ret = modem_cmd_handler_init(&mctx.cmd_handler,
				     &mdata.cmd_handler_data);
	if (ret < 0) {
		goto error;
	}

	/* modem interface */
	mdata.iface_data.isr_buf = &mdata.iface_isr_buf[0];
	mdata.iface_data.isr_buf_len = sizeof(mdata.iface_isr_buf);
	mdata.iface_data.rx_rb_buf = &mdata.iface_rb_buf[0];
	mdata.iface_data.rx_rb_buf_len = sizeof(mdata.iface_rb_buf);
	ret = modem_iface_uart_init(&mctx.iface, &mdata.iface_data,
				    MDM_UART_DEV_NAME);
	if (ret < 0) {
		goto error;
	}

	/* modem data storage */
	mctx.data_manufacturer = mdata.mdm_manufacturer;
	mctx.data_model = mdata.mdm_model;
	mctx.data_revision = mdata.mdm_revision;
	mctx.data_imei = mdata.mdm_deveui;

	/* pin setup */
	mctx.pins = modem_pins;
	mctx.pins_len = ARRAY_SIZE(modem_pins);

	mctx.driver_data = &mdata;

	ret = modem_context_register(&mctx);
	if (ret < 0) {
		LOG_ERR("Error registering modem context: %d", ret);
		goto error;
	}

	/* start RX thread */
	k_thread_create(&modem_rx_thread, modem_rx_stack,
			K_THREAD_STACK_SIZEOF(modem_rx_stack),
			(k_thread_entry_t) modem_rx,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	/* init RSSI query */
	k_delayed_work_init(&mdata.rssi_query_work, modem_rssi_query_work);

	modem_reset();

	mdata.network_joined = false;

error:
	return ret;
}
