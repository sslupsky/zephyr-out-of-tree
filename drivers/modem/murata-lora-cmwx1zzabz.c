/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

#include "../../../zephyr/drivers/modem/modem_context.h"
#include "../../../zephyr/drivers/modem/modem_iface_uart.h"
#include "../../../zephyr/drivers/modem/modem_cmd_handler.h"
#include "secret.h"

/* pin settings */
enum mdm_control_pins {
	MDM_RESET = 0,
	MDM_BOOT0,
	MDM_IRQ,
	MDM_RF_SSN,
	MDM_SCK,
	MDM_MOSI_TX,
	MDM_MISO_RX,
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

static struct modem_pin modem_pins[] = {
	/* MDM_RESET */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_RESET_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),

#if defined(DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_CONTROLLER)
	/* MDM_BOOT0 */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_BOOT0_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_RF_SSN_GPIOS_CONTROLLER)
	/* MDM_RF_SSN */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_RF_SSN_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_RF_SSN_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_RF_IRQ_GPIOS_CONTROLLER)
	/* MDM_IRQ */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_RF_IRQ_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_RF_IRQ_GPIOS_PIN, GPIO_DISCONNECTED),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_SCK_GPIOS_CONTROLLER)
	/* MDM_SCK */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_SCK_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_SCK_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_MOSI_TX_GPIOS_CONTROLLER)
	/* MDM_MOSI_TX */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_MOSI_TX_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_MOSI_TX_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif

#if defined(DT_INST_0_MURATA_LORA_MDM_MISO_RX_GPIOS_CONTROLLER)
	/* MDM_MISO_RX */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_MISO_RX_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_MISO_RX_GPIOS_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW),
#endif
};

#define MDM_UART_DEV_NAME		DT_INST_0_MURATA_LORA_BUS_NAME
#define MDM_PIN_RESET			GPIO_ACTIVE_LOW
#define MDM_PIN_BOOT0			GPIO_ACTIVE_HIGH
#define MDM_BOOT0_DISABLE		0
#define MDM_IRQ_NOT_ASSERTED		1
#define MDM_IRQ_ASSERTED		0

#define LORA_CMD_READ_BUF       128
#define LORA_CMD_AT_TIMEOUT     K_SECONDS(2)
#define LORA_CMD_SETUP_TIMEOUT  K_SECONDS(6)
#define LORA_RX_STACK_SIZE      512
#define LORA_MAX_DATA_LENGTH	128
#define LORA_RECV_MAX_BUF       4
#define LORA_RECV_BUF_SIZE      128
#define LORA_BUF_ALLOC_TIMEOUT  K_SECONDS(1)

static struct lora_modem {
	struct modem_context context;

	struct modem_cmd_handler_data cmd_handler_data;
	u8_t cmd_read_buf[LORA_CMD_READ_BUF];
	u8_t cmd_match_buf[LORA_CMD_READ_BUF];
	struct k_sem sem_response;

	struct modem_iface_uart_data lora_data;
	struct k_delayed_work lora_configure_work;
	char lora_isr_buf[LORA_MAX_DATA_LENGTH];
	char lora_rx_rb_buf[LORA_RECV_BUF_SIZE];

	bool setup_done;
	bool network_joined;
	u8_t *ppp_recv_buf;
	size_t ppp_recv_buf_len;
	uart_pipe_recv_cb ppp_recv_cb;
	struct k_sem ppp_send_sem;
} lora;

static size_t recv_buf_offset;

NET_BUF_POOL_DEFINE(lora_recv_pool, LORA_RECV_MAX_BUF, LORA_RECV_BUF_SIZE,
		    0, NULL);
K_THREAD_STACK_DEFINE(lora_rx_stack, LORA_RX_STACK_SIZE);

struct k_thread lora_rx_thread;

static void lora_rx(struct lora_modem *lora)
{
	int bytes, r;

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
			return;
		}

		r = lora->context.iface.read(
					&lora->context.iface,
					&lora->ppp_recv_buf[recv_buf_offset],
					lora->ppp_recv_buf_len -
					recv_buf_offset,
					&bytes);
		if (r < 0 || bytes == 0) {
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

/*
 * MODEM UNSOLICITED NOTIFICATION HANDLERS
 */


/* Handler: +EVENT=0,0 (Module reset event) */
MODEM_CMD_DEFINE(on_cmd_reset)
{
	LOG_DBG("device reset");
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}

/* Handler: +EVENT=1,1 (Network join event) */
MODEM_CMD_DEFINE(on_cmd_join)
{
	lora.network_joined = true;
	LOG_INF("network join");
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_rx_data)
{
	LOG_DBG("downlink received");
	modem_cmd_handler_set_error(data, 0);
	k_sem_give(&lora.sem_response);
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
	LOG_INF("Manufacturer: %s", log_strdup(minfo.mdm_manufacturer));
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
	LOG_INF("Model: %s", log_strdup(minfo.mdm_model));
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
	LOG_INF("Revision: %s", log_strdup(minfo.mdm_revision));
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
	LOG_INF("IMEI: %s", log_strdup(minfo.mdm_imei));
	k_sem_give(&lora.sem_response);

	return 0;
}
#endif /* CONFIG_MODEM_SHELL */

/* AT+REBOOT Handler: +OK[0]=<REBOOT>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_reboot)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("DEVEUI: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

/* AT+BAND Handler: +OK[0]=<BAND>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_band)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_band, sizeof(minfo.mdm_band) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_band[out_len] = '\0';
// 	LOG_INF("BAND: %s", log_strdup(minfo.mdm_band));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

/* AT+DEVEUI Handler: +OK[0]=<DEVEUI>[1] */
__attribute__((unused))
MODEM_CMD_DEFINE(on_cmd_atcmd_deveui)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_DBG("DEVEUI: %s", log_strdup(minfo.mdm_deveui));
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
	LOG_DBG("DEVADDR: %s", log_strdup(minfo.mdm_devaddr));
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
	LOG_DBG("NWKSKEY: %s", log_strdup(minfo.mdm_nwkskey));
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
	LOG_DBG("APPSKEY: %s", log_strdup(minfo.mdm_appskey));
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
	LOG_DBG("APPKEY: %s", log_strdup(minfo.mdm_appkey));
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
	LOG_DBG("APPEUI: %s", log_strdup(minfo.mdm_appeui));
	k_sem_give(&lora.sem_response);

	return 0;
}

// /* AT+ADR Handler: +OK[0]=<ADR>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_adr)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_adr, sizeof(minfo.mdm_adr) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_adr[out_len] = '\0';
// 	LOG_INF("ADR: %s", log_strdup(minfo.mdm_adr));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+RFPOWER Handler: +OK[0]=<RFPOWER>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_rfpower)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_txpower, sizeof(minfo.mdm_txpower) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_txpower[out_len] = '\0';
// 	LOG_INF("RFPOWER: %s", log_strdup(minfo.mdm_txpower));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+DFORMAT Handler: +OK[0]=<DFORMAT>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_dformat)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_atformat, sizeof(minfo.mdm_atformat) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_atformat[out_len] = '\0';
// 	LOG_INF("DFORMAT: %s", log_strdup(minfo.mdm_atformat));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+DR Handler: +OK[0]=<DR>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_dr)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_dr, sizeof(minfo.mdm_dr) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_dr[out_len] = '\0';
// 	LOG_INF("DR: %s", log_strdup(minfo.mdm_dr));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+DUTYCYCLE Handler: +OK[0]=<DUTYCYCLE>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_dutycycle)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_dutycycle, sizeof(minfo.mdm_dutycycle) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_dutycycle[out_len] = '\0';
// 	LOG_INF("DUTYCYCLE: %s", log_strdup(minfo.mdm_dutycycle));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+NWK Handler: +OK[0]=<NWK>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_nwk)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("NWK: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+MODE Handler: +OK[0]=<MODE>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_mode)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_mode, sizeof(minfo.mdm_mode) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_mode[out_len] = '\0';
// 	LOG_INF("MODE: %s", log_strdup(minfo.mdm_mode));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+FCU Handler: +OK[0]=<FCU>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_fcu)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_fcu, sizeof(minfo.mdm_fcu) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_fcu[out_len] = '\0';
// 	LOG_INF("FCU: %s", log_strdup(minfo.mdm_fcu));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+FCD Handler: +OK[0]=<FCD>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_fcd)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_fcd, sizeof(minfo.mdm_fcd) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_fcd[out_len] = '\0';
// 	LOG_INF("FCD: %s", log_strdup(minfo.mdm_fcd));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+CLASS Handler: +OK[0]=<CLASS>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_class)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_class, sizeof(minfo.mdm_class) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_class[out_len] = '\0';
// 	LOG_INF("CLASS: %s", log_strdup(minfo.mdm_class));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+JOIN Handler: +OK[0]=<JOIN>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_join)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("JOIN: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+NJS Handler: +OK[0]=<NJS>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_njs)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_njs, sizeof(minfo.mdm_njs) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_njs[out_len] = '\0';
// 	LOG_INF("NJS: %s", log_strdup(minfo.mdm_njs));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+UTX Handler: +OK[0]=<UTX>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_utx)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("UTX: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+CTX Handler: +OK[0]=<CTX>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_ctx)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("CTX: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+PORT Handler: +OK[0]=<PORT>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_port)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_port, sizeof(minfo.mdm_port) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_port[out_len] = '\0';
// 	LOG_INF("PORT: %s", log_strdup(minfo.mdm_port));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+CFM Handler: +OK[0]=<CFM>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_cfm)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_cfm, sizeof(minfo.mdm_cfm) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_cfm[out_len] = '\0';
// 	LOG_INF("CFM: %s", log_strdup(minfo.mdm_cfm));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+CFS Handler: +OK[0]=<CFS>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_cfs)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_cf, sizeof(minfo.mdm_cf) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_cf[out_len] = '\0';
// 	LOG_INF("CFS: %s", log_strdup(minfo.mdm_cf));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+SNR Handler: +OK[0]=<SNR>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_snr)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_snr, sizeof(minfo.mdm_snr) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_snr[out_len] = '\0';
// 	LOG_INF("SNR: %s", log_strdup(minfo.mdm_snr));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+RSSI Handler: +OK[0]=<RSSI>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_rssi)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_rssi, sizeof(minfo.mdm_rssi) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_rssi[out_len] = '\0';
// 	LOG_INF("RSSI: %s", log_strdup(minfo.mdm_rssi));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+BAT Handler: +OK[0]=<BAT>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_bat)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_bat, sizeof(minfo.mdm_bat) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_bat[out_len] = '\0';
// 	LOG_INF("BAT: %s", log_strdup(minfo.mdm_bat));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+UART Handler: +OK[0]=<UART>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_uart)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("UART: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+FACNEW Handler: +OK[0]=<FACNEW>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_facnew)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("FACNEW: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+SLEEP Handler: +OK[0]=<SLEEP>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_sleep)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("SLEEP: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

// /* AT+MSIZE Handler: +OK[0]=<MSIZE>[1] */
// MODEM_CMD_DEFINE(on_cmd_atcmd_msize)
// {
// 	size_t out_len;

// 	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
// 				    data->rx_buf, 0, len);
// 	minfo.mdm_deveui[out_len] = '\0';
// 	LOG_INF("MSIZE: %s", log_strdup(minfo.mdm_deveui));
// 	k_sem_give(&lora.sem_response);

// 	return 0;
// }

static struct setup_cmd setup_cmds[] = {
#if defined(CONFIG_MODEM_SHELL)
	/* query modem info */
	SETUP_CMD("AT+DEV?", "+OK=", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+VER?", "+OK=", on_cmd_atcmdinfo_revision, 0U, ""),
	// SETUP_CMD_NOHANDLE("AT+DEVEUI?"),
	// SETUP_CMD_NOHANDLE("AT+APPEUI?"),
	// SETUP_CMD_NOHANDLE("AT+APPKEY?"),
	SETUP_CMD_NOHANDLE("AT+BAND=9"),
	SETUP_CMD_NOHANDLE("AT+CLASS=A"),
	SETUP_CMD_NOHANDLE("AT+PORT=10"),
	SETUP_CMD_NOHANDLE("AT+ADR=1"),
	SETUP_CMD_NOHANDLE("AT+DR=3"),
	SETUP_CMD_NOHANDLE("AT+MODE=1"),
	// SETUP_CMD_NOHANDLE("AT+JOIN"),
#endif
};

static int lora_setup_keys(struct lora_modem *lora)
{
	int ret;

//	if (CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO[0]) {
		/* use manual MCC/MNO entry */
		ret = modem_cmd_send(&lora->context.iface,
				     &lora->context.cmd_handler,
				     NULL, 0,
				     "AT+APPEUI=" 
				     SECRET_APP_EUI,
				     &lora->sem_response,
				     LORA_CMD_AT_TIMEOUT);
//	}
	if (ret < 0) {
		LOG_ERR("set keys error, %d", ret);
	}

//	if (CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO[0]) {
		/* register operator automatically */
		ret = modem_cmd_send(&lora->context.iface,
				     &lora->context.cmd_handler,
				     NULL, 0,
				     "AT+APPKEY="
				     SECRET_APP_KEY,
				     &lora->sem_response,
				     LORA_CMD_AT_TIMEOUT);
//	}

	if (ret < 0) {
		LOG_ERR("set keys error, %d", ret);
	}

	return ret;
}

// set port (10)
// set ADR (true)
// set data rate (3)
// OTAA join (appEUI, appKey)

static void lora_configure(struct k_work *work)
{
	int r = -1;
	int timeout = 0;
	struct lora_modem *lora = CONTAINER_OF(work, struct lora_modem,
					     lora_configure_work);

	LOG_DBG("Starting modem %p configuration", lora);

	while (r < 0 && timeout < 10) {
		r = modem_cmd_send(&lora->context.iface,
					&lora->context.cmd_handler,
					(struct modem_cmd *)&response_cmds[0],
					ARRAY_SIZE(response_cmds),
					"AT", &lora->sem_response,
					LORA_CMD_AT_TIMEOUT);
		if (r < 0) {
			LOG_DBG("modem not ready, rc=%d", r);
			k_sleep(K_MSEC(200));
			timeout++;
		} else {
			LOG_DBG("modem ready");
			(void)lora_setup_keys(lora);
		}
		if (timeout == 10) {
			LOG_ERR("modem hardware failure");
		}
	}

	r = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+DEVEUI?",
				&lora->sem_response,
				LORA_CMD_AT_TIMEOUT);

	if (r < 0) {
		LOG_ERR("Get dev eui failed");	
	} else {
		LOG_INF("Dev EUI: %s", log_strdup(modem_response));
	}

	r = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPEUI?",
				&lora->sem_response,
				LORA_CMD_AT_TIMEOUT);

	if (r < 0) {
		LOG_ERR("Get app eui failed");	
	} else {
		LOG_DBG("App EUI: %s", log_strdup(modem_response));
	}

	r = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+APPKEY?",
				&lora->sem_response,
				LORA_CMD_AT_TIMEOUT);

	if (r < 0) {
		LOG_ERR("Get app key failed");	
	} else {
		LOG_DBG("App key: %s", log_strdup(modem_response));
	}

	r = -1;
	timeout = 0;
	while (r < 0 && timeout < 10) {
		r = modem_cmd_handler_setup_cmds(&lora->context.iface,
						 &lora->context.cmd_handler,
						 setup_cmds,
						 ARRAY_SIZE(setup_cmds),
						 &lora->sem_response,
						 LORA_CMD_SETUP_TIMEOUT);
		if (r < 0) {
			LOG_DBG("modem setup error, rc=%d", r);
			k_sleep(K_MSEC(200));
			timeout++;
		} else {
			LOG_DBG("modem setup complete");
		}
		if (timeout == 10) {
			LOG_ERR("modem hardware failure");
		}
	}

	r = modem_cmd_send(&lora->context.iface,
				&lora->context.cmd_handler,
				NULL, 0,
				"AT+JOIN",
				&lora->sem_response,
				K_NO_WAIT);


	if (r < 0) {
		LOG_ERR("Join failed");	
	}

	// lora->setup_done = true;
	k_sem_give(&lora->ppp_send_sem);
}

static int lora_init(struct device *device)
{
	struct lora_modem *lora = device->driver_data;
	k_tid_t thread;
	int r;

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
	lora->cmd_handler_data.alloc_timeout = LORA_BUF_ALLOC_TIMEOUT;
	lora->cmd_handler_data.eol = "\r";

	k_sem_init(&lora->sem_response, 0, 1);

	r = modem_cmd_handler_init(&lora->context.cmd_handler,
				   &lora->cmd_handler_data);
	if (r < 0) {
		LOG_DBG("cmd handler error %d", r);
		return r;
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

	r = modem_iface_uart_init(&lora->context.iface,
				  &lora->lora_data, MDM_UART_DEV_NAME);
	if (r < 0) {
		LOG_DBG("iface uart error %d", r);
		return r;
	}

	r = modem_context_register(&lora->context);
	if (r < 0) {
		LOG_DBG("context error %d", r);
		return r;
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

	k_thread_name_set(thread, "lora rx");
	k_delayed_work_init(&lora->lora_configure_work, lora_configure);

	(void)k_delayed_work_submit(&lora->lora_configure_work, 0);

	LOG_DBG("iface->read %p iface->write %p",
		lora->context.iface.read, lora->context.iface.write);
	return 0;
}

int uart_pipe_send(const u8_t *data, int len)
{
	k_sem_take(&lora.ppp_send_sem, K_FOREVER);

	(void)lora.context.iface.write(&lora.context.iface, data, len);

	k_sem_give(&lora.ppp_send_sem);

	return 0;
}

void uart_pipe_register(u8_t *buf, size_t len, uart_pipe_recv_cb cb)
{
	lora.ppp_recv_buf = buf;
	lora.ppp_recv_buf_len = len;
	lora.ppp_recv_cb = cb;
}

DEVICE_INIT(lora_ppp, "modem_lora", lora_init, &lora, NULL, POST_KERNEL,
	    CONFIG_MODEM_MURATA_LORA_INIT_PRIORITY);
