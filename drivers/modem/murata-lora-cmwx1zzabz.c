/**
 * @file murata-lora-cmwx1zzabz.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-04
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

#define DT_DRV_COMPAT murata_lora

#include <logging/log.h>
LOG_MODULE_REGISTER(modem_lora, CONFIG_MODEM_LOG_LEVEL);

#include <kernel.h>
#include <device.h>
#include <sys/ring_buffer.h>
#include <sys/util.h>
#include <drivers/console/uart_pipe.h>
#include <drivers/uart.h>
#include <soc.h>

#include <net/net_if.h>
#include <drivers/gpio.h>

#include <modem/modem_context.h>
#include <modem/modem_iface_uart.h>
#include <modem/modem_cmd_handler.h>
#include "secret.h"

/* Murata LoRa devicetree macros */
#define DT_LORA_NODE		DT_INST(0, murata_lora)
#define DT_LORA_UART_LABEL	DT_BUS_LABEL(DT_INST(0, murata_lora))

#define LORA_AT_CMD(cmd_send_, match_cmd_, func_cb_, num_param_, delim_, timeout_) { \
	.send_cmd = cmd_send_, \
	.timeout = timeout_,   \
	MODEM_CMD(match_cmd_, func_cb_, num_param_, delim_) \
}

#define LORA_AT_CMD_NOHANDLE(send_cmd_, timeout_) \
		LORA_AT_CMD(send_cmd_, NULL, NULL, 0U, NULL, timeout_)

/* series of modem setup commands to run */
struct lora_at_cmd {
	const char *send_cmd;
	uint32_t timeout;
	struct modem_cmd handle_cmd;
};

enum lora_mode {
	LORA_ABP = 0,
	LORA_OTAA,
};

enum lora_band {
	AS923 = 0,
	AU915,
	EU868 = 5,
	KR920,
	IN865,
	US915,
	US915_HYBRID,
};

/* modem pin settings */
enum mdm_control_pins {
	MDM_RESET = 0,
	MDM_BOOT0,
	MDM_IRQ,
	MDM_RF_SSN,
	MDM_SCK,
	MDM_MOSI_TX,
	MDM_MISO_RX,
};

static struct modem_pin modem_pins[] = {
	/*
	 * MDM_RESET
	 * MKR WAN 1310 has 1M (R11) pull up
	 * 
	 * DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_CONTROLLER
	 * DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_PIN
	 */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_reset_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_reset_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_boot0_gpios))
	/*
	 * MDM_BOOT0
	 * MKR WAN 1310 has 1M (R7) pull down
	 */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_boot0_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_boot0_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH),
#endif

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_rf_ssn_gpios))
	/*
	 * MDM_RF_SSN
	 * MKR WAN 1310 has 1M (R12) pull up
	 */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_rf_ssn_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_rf_ssn_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_rf_irq_gpios))
	/* MDM_IRQ */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_rf_irq_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_rf_irq_gpios),
		  GPIO_DISCONNECTED),
#endif

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_sck_gpios))
	/* MDM_SCK */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_sck_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_sck_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_mosi_tx_gpios))
	/* MDM_MOSI_TX */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_mosi_tx_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_mosi_tx_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if DT_NODE_EXISTS(DT_PHANDLE(DT_LORA_NODE, mdm_miso_rx_gpios))
	/* MDM_MISO_RX */
	MODEM_PIN(DT_PROP(DT_PHANDLE(DT_LORA_NODE, mdm_miso_rx_gpios), label),
		  DT_GPIO_PIN(DT_LORA_NODE, mdm_miso_rx_gpios),
		  GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif
};


#define LORA_CMD_READ_BUF		128
#define LORA_CMD_AT_TIMEOUT		2		// (sec)
#define LORA_CMD_SETUP_TIMEOUT		6		// (sec)
#define LORA_CMD_JOIN_TIMEOUT		10		// (sec)
#define LORA_RX_STACK_SIZE		512
#define LORA_MAX_DATA_LENGTH		128
#define LORA_RECV_MAX_BUF		4
#define LORA_RECV_BUF_SIZE		128
#define LORA_BUF_ALLOC_TIMEOUT		1		// (sec)

#define LORA_REJOIN_PERIOD            600000
#define LORA_JOIN_RETRIES                  6
#define LORA_UPLINK_TIMEOUT            65000                  ///<  MAX_RX_WINDOW for US915 region is 3000ms per retry.  Max timeout = 3000 * NbTrials (=8) = 24000 ms
#define LORA_RX_WINDOW_PERIOD           3000
#define LORA_NBTRIALS                      8
#define LORA_ACK_TIMEOUT                6000
#define LORA_CONFIRMED_RETRIES             6
#define LORA_UNCONFIRMED_RETRIES           2
#define LORA_MAX_CONFIRMED_UPLINKS_FAILED 10
#define LORA_MAX_LORAMAC_FAILED           10

#define LORA_HEARTBEAT_TIMEOUT		3600
#define LORA_UART_TXC_TIMEOUT		   2

struct lora_status_t
{
	uint32_t heartbeat_period;
	uint32_t join_retries;
	uint32_t join_count;
	uint32_t ack_count;
	uint32_t failed_ack_count;
	uint32_t reset_count;
	int16_t rssi;                /*< Rssi of the received packet */
	uint8_t snr;                 /*< Snr of the received packet */
	uint8_t application_port;    /*< Application port we will receive to */
	uint8_t nbtrials;
	bool network_joined;
	bool req_ack;      /*< ENABLE if acknowledge is requested */
	bool ack_failed;
	bool uart_busy;
};

/*
Firmware Version and Status Advertising in Frame 0
Battery voltage and level
Firmware revision
Received signal levels (device Rx levels from gateway)
Device configuration parameters or CRC/MD5 hash
Device error conditions (e.g., sensor out of calibration)
Temperature (if this is a secondary measurement)
Counts of certain events such as the number of times woken by accelerometer
Wake versus sleep time of the device
*/
static struct lora_modem {
	struct modem_context context;

	struct modem_cmd_handler_data cmd_handler_data;
	uint8_t cmd_read_buf[LORA_CMD_READ_BUF];
	uint8_t cmd_match_buf[LORA_CMD_READ_BUF];
	struct k_sem sem_response;

	struct modem_iface_uart_data lora_data;
	struct k_delayed_work lora_configure_work;
	char lora_isr_buf[LORA_MAX_DATA_LENGTH];
	char lora_rx_rb_buf[LORA_RECV_BUF_SIZE];

	struct lora_status_t status;
	bool setup_done;
	uint8_t *ppp_recv_buf;
	size_t ppp_recv_buf_len;
	uart_pipe_recv_cb ppp_recv_cb;
	struct k_sem ppp_send_sem;
	struct k_delayed_work heartbeat_work;
	struct k_delayed_work req_ack_work;
} lora;

static size_t recv_buf_offset;

NET_BUF_POOL_DEFINE(lora_recv_pool, LORA_RECV_MAX_BUF, LORA_RECV_BUF_SIZE,
		    0, NULL);
K_THREAD_STACK_DEFINE(lora_rx_stack, LORA_RX_STACK_SIZE);

struct k_thread lora_rx_thread;

static void lora_rx(struct lora_modem *lora)
{
	int bytes, ret;

	LOG_DBG("starting");

	while (true) {
		k_sem_take(&lora->lora_data.rx_sem, K_FOREVER);

		if (lora->setup_done == false) {
			lora->context.cmd_handler.process(
						&lora->context.cmd_handler,
						&lora->context.iface);
			continue;
		}

		if (lora->ppp_recv_cb == NULL || lora->ppp_recv_buf == NULL ||
		    lora->ppp_recv_buf_len == 0) {
			LOG_ERR("rx not registered, thread terminated");
			return;
		}

		ret = lora->context.iface.read(
					&lora->context.iface,
					&lora->ppp_recv_buf[recv_buf_offset],
					lora->ppp_recv_buf_len -
					recv_buf_offset,
					&bytes);
		if (ret < 0 || bytes == 0) {
			continue;
		}

		recv_buf_offset += bytes;

		lora->ppp_recv_buf = lora->ppp_recv_cb(lora->ppp_recv_buf,
						     &recv_buf_offset);
	}
}

static char modem_response[33];
static char modem_error[16];

/* AT+OK Handler: +OK[0]=<response>[1] */
MODEM_CMD_DEFINE(lora_cmd_ok)
{
	size_t out_len;

	out_len = net_buf_linearize(modem_response,
				    sizeof(modem_response) - 1,
				    data->rx_buf, 1, len);
	modem_response[out_len] = '\0';
	if (out_len > 0 && modem_response[out_len-1] == '\r') {
		modem_response[out_len-1] = '\0';
	}
	LOG_DBG("+OK: %s", log_strdup(modem_response));

	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(lora_cmd_error)
{
	size_t out_len;

	out_len = net_buf_linearize(modem_error,
				    sizeof(modem_error) - 1,
				    data->rx_buf, 1, len);
	modem_error[out_len] = '\0';
	LOG_WRN("+ERR: %s", log_strdup(modem_error));

	modem_cmd_handler_set_error(data, -EINVAL);
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_true)
{
	LOG_DBG("true");
	modem_response[0] = 1;
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_false)
{
	LOG_DBG("false");
	modem_response[0] = 0;
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}

/*
 * MODEM UNSOLICITED NOTIFICATION HANDLERS
 */


/* Handler: +EVENT=0,0 (Module reset event) */
MODEM_CMD_DEFINE(on_cmd_reset)
{
	lora.status.reset_count++;
	LOG_DBG("device reset");
	modem_cmd_handler_set_error(data, 0);
	// k_sem_give(&lora.sem_response);
	return 0;
}

/* Handler: +EVENT=1,1 (Network join event) */
MODEM_CMD_DEFINE(on_cmd_join)
{
	lora.status.network_joined = true;
	lora.status.join_count++;
	/* TODO:  send device status after join */
	LOG_INF("network join");
	modem_cmd_handler_set_error(data, 0);
	// k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_rx_data)
{
	size_t out_len;
	// struct lora_modem *lora = CONTAINER_OF(data, struct lora_modem,
	// 				     cmd_handler_data);

	LOG_INF("downlink rx");
	LOG_HEXDUMP_DBG(data->rx_buf, len, "+RECV");
	if (lora.ppp_recv_cb == NULL || lora.ppp_recv_buf == NULL ||
		lora.ppp_recv_buf_len == 0) {
		LOG_ERR("rx not registered, downlink ignored");
		modem_cmd_handler_set_error(data, -EPIPE);
		return 0;
	}
	out_len = net_buf_linearize(&lora.ppp_recv_buf[recv_buf_offset],
				    lora.ppp_recv_buf_len - recv_buf_offset,
				    data->rx_buf, 1, len);
	recv_buf_offset += out_len;

	lora.ppp_recv_buf = lora.ppp_recv_cb(lora.ppp_recv_buf,
					     &recv_buf_offset);

	// modem_response[out_len] = '\0';
	// if (out_len > 0 && modem_response[out_len-1] == '\r') {
	// 	modem_response[out_len-1] = '\0';
	// }

	modem_cmd_handler_set_error(data, 0);
	// k_sem_give(&lora.sem_response);
	return 0;
}

/* Handler: +ACK (ack) */
MODEM_CMD_DEFINE(on_cmd_ack)
{
	LOG_INF("network ack");
	lora.status.ack_count++;
	modem_cmd_handler_set_error(data, 0);
	// k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_modem_rx_err)
{
	LOG_WRN("modem announced rx error");
	return 0;
}

static const struct modem_cmd response_cmds[] = {
	MODEM_CMD("+OK", lora_cmd_ok, 0U, ""),
	MODEM_CMD("+ERR", lora_cmd_error, 0U, ""),
	MODEM_CMD("CONNECT", lora_cmd_ok, 0U, ""),
};

static const struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("+EVENT=0,0", on_cmd_reset, 0U, ""),
	MODEM_CMD("+EVENT=1,1", on_cmd_join, 0U, ""),
	MODEM_CMD("+RECV=", on_cmd_rx_data, 2U, ","),
	MODEM_CMD("+ACK", on_cmd_ack, 0U, ""),
	MODEM_CMD("Error when receiving", on_cmd_modem_rx_err, 0U, ""),
};

static const struct modem_cmd bool_response_cmds[] = {
	MODEM_CMD("0", on_cmd_false, 0U, ""),
	MODEM_CMD("1", on_cmd_true, 0U, ""),
};

#if defined(CONFIG_MODEM_SHELL)
#define MDM_MANUFACTURER_LENGTH  10
#define MDM_MODEL_LENGTH         17
#define MDM_REVISION_LENGTH      17
#define MDM_IMEI_LENGTH          17
#define MDM_DEVEUI_LENGTH	 17
#define MDM_DEVADDR_LENGTH	 17
#define MDM_APPKEY_LENGTH	 33
#define MDM_APPSKEY_LENGTH	 33
#define MDM_APPEUI_LENGTH	 17
#define MDM_NWKSKEY_LENGTH	 33
#define MDM_RESPONSE_LENGTH	 33

struct modem_info {
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
	char mdm_deveui[MDM_DEVEUI_LENGTH];
	char mdm_devaddr[MDM_DEVADDR_LENGTH];
	char mdm_appkey[MDM_APPKEY_LENGTH];
	char mdm_appskey[MDM_APPSKEY_LENGTH];
	char mdm_appeui[MDM_APPEUI_LENGTH];
	char mdm_nwkskey[MDM_NWKSKEY_LENGTH];
	char mdm_atresponse[MDM_RESPONSE_LENGTH];
};

static struct modem_info minfo;

/*
 * Provide modem info if modem shell is enabled. This can be shown with
 * "modem list" shell command.
 */

/* Handler: <manufacturer> */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_manufacturer,
				    sizeof(minfo.mdm_manufacturer) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", minfo.mdm_manufacturer);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+DEV Handler: +OK[0]=<model>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_model,
				    sizeof(minfo.mdm_model) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", minfo.mdm_model);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+VER Handler: +OK[0]=<rev>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_revision,
				    sizeof(minfo.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", minfo.mdm_revision);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* Handler: <IMEI> */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_imei, sizeof(minfo.mdm_imei) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", minfo.mdm_imei);
	k_sem_give(&lora.sem_response);

	return 0;
}
#endif /* CONFIG_MODEM_SHELL */


/* AT+DEVEUI Handler: +OK[0]=<DEVEUI>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_deveui)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_DBG("DEVEUI: %s", minfo.mdm_deveui);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+DEVADDR Handler: +OK[0]=<DEVADDR>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_devaddr)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_devaddr, sizeof(minfo.mdm_devaddr) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_devaddr[out_len] = '\0';
	LOG_DBG("DEVADDR: %s", minfo.mdm_devaddr);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+NWKSKEY Handler: +OK[0]=<NWKSKEY>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_nwkskey)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_nwkskey, sizeof(minfo.mdm_nwkskey) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_nwkskey[out_len] = '\0';
	LOG_DBG("NWKSKEY: %s", minfo.mdm_nwkskey);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+APPSKEY Handler: +OK[0]=<APPSKEY>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_appskey)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_appskey, sizeof(minfo.mdm_appskey) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_appskey[out_len] = '\0';
	LOG_DBG("APPSKEY: %s", minfo.mdm_appskey);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+APPKEY Handler: +OK[0]=<APPKEY>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_appkey)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_appkey, sizeof(minfo.mdm_appkey) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_appkey[out_len] = '\0';
	LOG_DBG("APPKEY: %s", minfo.mdm_appkey);
	k_sem_give(&lora.sem_response);

	return 0;
}

/* AT+APPEUI Handler: +OK[0]=<APPEUI>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_appeui)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_appeui, sizeof(minfo.mdm_appeui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_appeui[out_len] = '\0';
	LOG_DBG("APPEUI: %s", minfo.mdm_appeui);
	k_sem_give(&lora.sem_response);

	return 0;
}


static const struct lora_at_cmd setup_cmds[] = {
#if defined(CONFIG_MODEM_SHELL)
	/* query modem info */
	LORA_AT_CMD("AT+DEV?", "+OK=", on_cmd_atcmdinfo_model, 0U, "", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD("AT+VER?", "+OK=", on_cmd_atcmdinfo_revision, 0U, "", LORA_CMD_AT_TIMEOUT),
	// SETUP_CMD_NOHANDLE("AT+DEVEUI?"),
	// SETUP_CMD_NOHANDLE("AT+APPEUI?"),
	// SETUP_CMD_NOHANDLE("AT+APPKEY?"),
#endif
};

static const struct lora_at_cmd config_otaa_cmds[] = {
	LORA_AT_CMD_NOHANDLE("AT+BAND=9", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD_NOHANDLE("AT+CLASS=A", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD_NOHANDLE("AT+PORT=10", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD_NOHANDLE("AT+ADR=1", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD_NOHANDLE("AT+DR=3", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD_NOHANDLE("AT+MODE=1", LORA_CMD_AT_TIMEOUT),
};

static const struct lora_at_cmd join_otaa_cmds[] = {
	LORA_AT_CMD_NOHANDLE("AT+JOIN", LORA_CMD_JOIN_TIMEOUT),
};

static const struct lora_at_cmd set_keys[] = {
	LORA_AT_CMD("AT+APPEUI=" SECRET_APP_EUI, "+OK", lora_cmd_ok, 0U, "", LORA_CMD_AT_TIMEOUT),
	LORA_AT_CMD("AT+APPKEY=" SECRET_APP_KEY, "+OK", lora_cmd_ok, 0U, "", LORA_CMD_AT_TIMEOUT),
};

static const struct lora_at_cmd cfs[] = {
	LORA_AT_CMD("AT+CFS?", "+OK=", lora_cmd_ok, 0U, "", LORA_CMD_AT_TIMEOUT),
};

/* run a set of AT commands */
int lora_at_cmd_seq_send(struct modem_iface *iface,
		     struct modem_cmd_handler *handler,
		     const struct lora_at_cmd *cmds, size_t cmds_len,
		     struct k_sem *sem)
{
	int ret = 0, i;

	for (i = 0; i < cmds_len; i++) {
		if (i) {
			k_sleep(K_MSEC(50));
		}

		if (cmds[i].handle_cmd.cmd && cmds[i].handle_cmd.func) {
			ret = modem_cmd_send(iface, handler,
					     &cmds[i].handle_cmd, 1U,
					     cmds[i].send_cmd,
					     sem, K_SECONDS(cmds[i].timeout));
		} else {
			ret = modem_cmd_send(iface, handler,
					     NULL, 0, cmds[i].send_cmd,
					     sem, K_SECONDS(cmds[i].timeout));
		}

		if (ret < 0) {
			LOG_ERR("command %s ret:%d", cmds[i].send_cmd, ret);
			break;
		}
	}

	return ret;
}

static int lora_setup_keys(struct lora_modem *lora)
{
	int ret;

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPEUI=" 
				SECRET_APP_EUI,
				&lora->sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));
	if (ret < 0) {
		LOG_ERR("set keys error, %d", ret);
	}

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPKEY="
				SECRET_APP_KEY,
				&lora->sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));
	if (ret < 0) {
		LOG_ERR("set keys error, %d", ret);
	}

	return ret;
}

static void lora_heartbeat(struct k_work *work)
{
	lora.status.req_ack = true;
	k_delayed_work_submit(&lora.heartbeat_work, K_SECONDS(LORA_HEARTBEAT_TIMEOUT));
}

static void lora_req_ack(struct k_work *work)
{
	int ret;

	ret = modem_cmd_send(&lora.context.iface,
				&lora.context.cmd_handler,
				(struct modem_cmd *)bool_response_cmds,
				ARRAY_SIZE(bool_response_cmds),
				"AT+CFS?",
				&lora.sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));
	if (ret < 0) {
		LOG_ERR("get ack status error");
		return;
	}
	if (modem_response[0]) {
		LOG_DBG("uplink ack received");
	} else {
		if (lora.status.nbtrials--) {
			LOG_DBG("uplink ack not received, retry...");
			/* send zero length packet to trigger retransmission
			 * of the unacknowledged packet
			 */
			ret = modem_cmd_send(&lora.context.iface,
						&lora.context.cmd_handler,
						(struct modem_cmd *)response_cmds,
						ARRAY_SIZE(response_cmds),
						"AT+CTX 0",
						&lora.sem_response,
						K_SECONDS(LORA_CMD_AT_TIMEOUT));
			k_delayed_work_submit(&lora.req_ack_work, K_MSEC(LORA_ACK_TIMEOUT));
		} else {
			/* LoRaMac will retry sending the confirmed packet up
			 * to 8 times.  If the retries fail, then the uplink
			 * either could not reach the server or the ACK could
			 * not reach the mote
			 */
			lora.status.failed_ack_count++;
			lora.status.ack_failed = true;
			LOG_ERR("ACK timeout");
		}
	}
}

static void lora_configure(struct k_work *work)
{
	int ret = -1;
	int timeout = 0;
	struct lora_modem *lora = CONTAINER_OF(work, struct lora_modem,
					     lora_configure_work);

	LOG_DBG("Starting modem %p configuration", lora);

	while (ret < 0 && timeout < 10) {
		ret = modem_cmd_send(&lora->context.iface,
					&lora->context.cmd_handler,
					(struct modem_cmd *)&response_cmds[0],
					ARRAY_SIZE(response_cmds),
					"AT", &lora->sem_response,
					K_SECONDS(LORA_CMD_AT_TIMEOUT));
		if (ret < 0) {
			LOG_DBG("modem not ready, rc=%d", ret);
			k_sleep(K_MSEC(200));
			timeout++;
		} else {
			LOG_DBG("modem ready");
		}
		if (timeout == 10) {
			LOG_ERR("modem hardware failure");
		}
	}

	lora_setup_keys(lora);

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+DEVEUI?",
				&lora->sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));

	if (ret < 0) {
		LOG_ERR("Get dev eui failed");	
	} else {
		LOG_INF("Dev EUI: %s", log_strdup(modem_response));
	}

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPEUI?",
				&lora->sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));

	if (ret < 0) {
		LOG_ERR("Get app eui failed");	
	} else {
		LOG_DBG("App EUI: %s", log_strdup(modem_response));
	}

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPKEY?",
				&lora->sem_response,
				K_SECONDS(LORA_CMD_AT_TIMEOUT));

	if (ret < 0) {
		LOG_ERR("Get app key failed");	
	} else {
		LOG_DBG("App key: %s", log_strdup(modem_response));
	}

	ret = lora_at_cmd_seq_send(&lora->context.iface,
				&lora->context.cmd_handler,
				setup_cmds,
				ARRAY_SIZE(setup_cmds),
				&lora->sem_response);
	if (ret < 0) {
		LOG_ERR("modem setup error, rc=%d", ret);
		k_sleep(K_MSEC(200));
		timeout++;
	} else {
		LOG_DBG("modem setup complete");
	}

	ret = lora_at_cmd_seq_send(&lora->context.iface,
				&lora->context.cmd_handler,
				config_otaa_cmds,
				ARRAY_SIZE(config_otaa_cmds),
				&lora->sem_response);
	if (ret < 0) {
		LOG_ERR("OTAA configuration error");
	} else {
		LOG_DBG("otaa config complete");
	}

	ret = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+JOIN",
				&lora->sem_response,
				K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("join request error");	
	} else {
		LOG_DBG("join requested");
	}

	// lora->setup_done = true;
	k_sem_give(&lora->ppp_send_sem);
}

static int lora_init(struct device *device)
{
	struct lora_modem *lora = device->driver_data;
	k_tid_t thread;
	int ret;

	LOG_DBG("Generic LoRa modem (%p)", lora);

	/* pin setup */
	lora->context.pins = modem_pins;
	lora->context.pins_len = ARRAY_SIZE(modem_pins);

	k_sem_init(&lora->ppp_send_sem, 0, 1);

	lora->cmd_handler_data.cmds[CMD_RESP] = (struct modem_cmd *)response_cmds;
	lora->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
	lora->cmd_handler_data.cmds[CMD_UNSOL] = (struct modem_cmd *)unsol_cmds;
	lora->cmd_handler_data.cmds_len[CMD_UNSOL] = ARRAY_SIZE(unsol_cmds);
	lora->cmd_handler_data.read_buf = &lora->cmd_read_buf[0];
	lora->cmd_handler_data.read_buf_len = sizeof(lora->cmd_read_buf);
	lora->cmd_handler_data.match_buf = &lora->cmd_match_buf[0];
	lora->cmd_handler_data.match_buf_len = sizeof(lora->cmd_match_buf);
	lora->cmd_handler_data.buf_pool = &lora_recv_pool;
	lora->cmd_handler_data.alloc_timeout = K_SECONDS(LORA_BUF_ALLOC_TIMEOUT);
	lora->cmd_handler_data.eol = "\r";

	k_sem_init(&lora->sem_response, 0, 1);

	ret = modem_cmd_handler_init(&lora->context.cmd_handler,
				   &lora->cmd_handler_data);
	if (ret < 0) {
		LOG_DBG("cmd handler error %d", ret);
		return ret;
	}

#if defined(CONFIG_MODEM_SHELL)
	/* modem information storage */
	lora->context.data_manufacturer = minfo.mdm_manufacturer;
	lora->context.data_model = minfo.mdm_model;
	lora->context.data_revision = minfo.mdm_revision;
	lora->context.data_imei = minfo.mdm_imei;
#endif

	lora->lora_data.isr_buf = &lora->lora_isr_buf[0];
	lora->lora_data.isr_buf_len = sizeof(lora->lora_isr_buf);
	lora->lora_data.rx_rb_buf = &lora->lora_rx_rb_buf[0];
	lora->lora_data.rx_rb_buf_len = sizeof(lora->lora_rx_rb_buf);

	ret = modem_iface_uart_init(&lora->context.iface,
				  &lora->lora_data, DT_LORA_UART_LABEL);
	if (ret < 0) {
		LOG_DBG("iface uart error %d", ret);
		return ret;
	}

	ret = modem_context_register(&lora->context);
	if (ret < 0) {
		LOG_DBG("context error %d", ret);
		return ret;
	}

	LOG_DBG("clear boot0");
	modem_pin_write(&lora->context, MDM_BOOT0, 0);

	LOG_DBG("reset modem");
	modem_pin_write(&lora->context, MDM_RESET, 1);
	k_sleep(K_MSEC(200));
	modem_pin_write(&lora->context, MDM_RESET, 0);

	thread = k_thread_create(&lora_rx_thread, lora_rx_stack,
			K_THREAD_STACK_SIZEOF(lora_rx_stack),
			(k_thread_entry_t) lora_rx,
			lora, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	k_thread_name_set(thread, "lora_rx");
	k_delayed_work_init(&lora->lora_configure_work, lora_configure);
	k_delayed_work_init(&lora->heartbeat_work, lora_heartbeat);
	lora->status.req_ack = false;
	k_delayed_work_init(&lora->req_ack_work, lora_req_ack);

	(void)k_delayed_work_submit(&lora->lora_configure_work, K_SECONDS(0));
	(void)k_delayed_work_submit(&lora->heartbeat_work, K_SECONDS(LORA_HEARTBEAT_TIMEOUT));

	LOG_DBG("iface->read %p iface->write %p",
		lora->context.iface.read, lora->context.iface.write);
	return 0;
}

int uart_pipe_send(const uint8_t *data, int len)
{
	int ret;
	uint8_t buf[12];

	k_sem_take(&lora.ppp_send_sem, K_FOREVER);

	if (lora.status.network_joined && len <= LORA_MAX_DATA_LENGTH) {
		LOG_DBG("uplink queued");
		if (lora.status.req_ack) {
			/* request ack */
			LOG_DBG("uplink ack requested");
			snprintk(buf, sizeof(buf), "AT+CTX %d\r", len);
			lora.status.req_ack = false;
			lora.status.nbtrials = LORA_NBTRIALS;
			k_delayed_work_submit(&lora.req_ack_work, K_MSEC(LORA_ACK_TIMEOUT));
		} else {
			snprintk(buf, sizeof(buf), "AT+UTX %d\r", len);
		}
		(void)lora.context.iface.write(&lora.context.iface, buf, strlen(buf));
		(void)lora.context.iface.write(&lora.context.iface, data, len);

		// k_sem_reset(&lora.sem_response);
		ret = k_sem_take(&lora.sem_response, K_SECONDS(LORA_CMD_AT_TIMEOUT));
		if (ret == 0) {
			ret = lora.cmd_handler_data.last_error;
		} else if (ret == -EAGAIN) {
			LOG_DBG("modem response timeout");
			ret = -ETIMEDOUT;
		}
	} else {
		ret = -ENETUNREACH;
	}
	k_sem_give(&lora.ppp_send_sem);

	return ret;
}

void uart_pipe_register(uint8_t *buf, size_t len, uart_pipe_recv_cb cb)
{
	lora.ppp_recv_buf = buf;
	lora.ppp_recv_buf_len = len;
	lora.ppp_recv_cb = cb;
}

DEVICE_INIT(lora_ppp, "modem_lora", lora_init, &lora, NULL, POST_KERNEL,
	    CONFIG_MODEM_MURATA_LORA_INIT_PRIORITY);
