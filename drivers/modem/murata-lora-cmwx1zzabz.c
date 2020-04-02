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

#include <net/net_if.h>
#include <drivers/gpio.h>

#include "../../../zephyr/drivers/modem/modem_context.h"
#include "../../../zephyr/drivers/modem/modem_iface_uart.h"
#include "../../../zephyr/drivers/modem/modem_cmd_handler.h"

/* pin settings */
enum mdm_control_pins {
	MDM_RESET = 0,
	MDM_BOOT0,
	MDM_IRQ,
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

#if defined(DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_CONTROLLER)
	/* MDM_IRQ */
	MODEM_PIN(DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_CONTROLLER,
		  DT_INST_0_MURATA_LORA_MDM_IRQ_GPIOS_PIN, GPIO_INPUT),
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
#define LORA_RX_STACK_SIZE      1024
#define LORA_MAX_DATA_LENGTH	1024
#define LORA_RECV_MAX_BUF       8
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

MODEM_CMD_DEFINE(lora_cmd_ok)
{
	modem_cmd_handler_set_error(data, 0);
	LOG_DBG("ok");
	k_sem_give(&lora.sem_response);
	return 0;
}

MODEM_CMD_DEFINE(lora_cmd_error)
{
	modem_cmd_handler_set_error(data, -EINVAL);
	LOG_DBG("error");
	k_sem_give(&lora.sem_response);
	return 0;
}

/*
 * MODEM UNSOLICITED NOTIFICATION HANDLERS
 */


/* Handler: +EVENT=0,0 (Module reset event) */
MODEM_CMD_DEFINE(on_cmd_reset)
{
	LOG_DBG("murata reset");
	return 0;
}

/* Handler: +EVENT=1,1 (Network join event) */
MODEM_CMD_DEFINE(on_cmd_join)
{
	lora.network_joined = true;
	LOG_DBG("network join");
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_rx_data)
{
	LOG_DBG("downlink received");
	return 0;
}

static struct modem_cmd response_cmds[] = {
	MODEM_CMD("+OK", lora_cmd_ok, 0U, ""),
	MODEM_CMD("+ERR", lora_cmd_error, 0U, ""),
	MODEM_CMD("CONNECT", lora_cmd_ok, 0U, ""),
};

static struct modem_cmd unsol_cmds[] = {
	MODEM_CMD("+EVENT=0,0", on_cmd_reset, 0U, ""),
	MODEM_CMD("+EVENT=1,1", on_cmd_join, 0U, ""),
	MODEM_CMD("+RECV=", on_cmd_rx_data, 2U, ","),
};

#if defined(CONFIG_MODEM_SHELL)
#define MDM_MANUFACTURER_LENGTH  10
#define MDM_MODEL_LENGTH         16
#define MDM_REVISION_LENGTH      64
#define MDM_IMEI_LENGTH          16
#define MDM_DEVEUI_LENGTH	 16

struct modem_info {
	char mdm_manufacturer[MDM_MANUFACTURER_LENGTH];
	char mdm_model[MDM_MODEL_LENGTH];
	char mdm_revision[MDM_REVISION_LENGTH];
	char mdm_imei[MDM_IMEI_LENGTH];
	char mdm_deveui[MDM_DEVEUI_LENGTH];
};

static struct modem_info minfo;

/*
 * Provide modem info if modem shell is enabled. This can be shown with
 * "modem list" shell command.
 */

/* Handler: <manufacturer> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_manufacturer)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_manufacturer,
				    sizeof(minfo.mdm_manufacturer) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_manufacturer[out_len] = '\0';
	LOG_INF("Manufacturer: %s", log_strdup(minfo.mdm_manufacturer));

	return 0;
}

/* Handler: <model> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_model)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_model,
				    sizeof(minfo.mdm_model) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_model[out_len] = '\0';
	LOG_INF("Model: %s", log_strdup(minfo.mdm_model));

	return 0;
}

/* Handler: <rev> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_revision)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_revision,
				    sizeof(minfo.mdm_revision) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_revision[out_len] = '\0';
	LOG_INF("Revision: %s", log_strdup(minfo.mdm_revision));

	return 0;
}

/* Handler: <IMEI> */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_imei)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_imei, sizeof(minfo.mdm_imei) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_imei[out_len] = '\0';
	LOG_INF("IMEI: %s", log_strdup(minfo.mdm_imei));

	return 0;
}
#endif /* CONFIG_MODEM_SHELL */

/* Handler: +OK[0]=<DEVEUI>[1] */
MODEM_CMD_DEFINE(on_cmd_atcmdinfo_deveui)
{
	size_t out_len;

	out_len = net_buf_linearize(minfo.mdm_deveui, sizeof(minfo.mdm_deveui) - 1,
				    data->rx_buf, 0, len);
	minfo.mdm_deveui[out_len] = '\0';
	LOG_INF("DEVEUI: %s", log_strdup(minfo.mdm_deveui));
	return 0;
}

static struct setup_cmd setup_cmds[] = {
#if defined(CONFIG_MODEM_SHELL)
	/* query modem info */
	SETUP_CMD("AT+DEV?", "+OK=", on_cmd_atcmdinfo_model, 0U, ""),
	SETUP_CMD("AT+VER?", "+OK=", on_cmd_atcmdinfo_revision, 0U, ""),
	SETUP_CMD("AT+DEVEUI?", "+OK=", on_cmd_atcmdinfo_deveui, 0U, ""),
#endif
};

static int lora_setup_keys(struct lora_modem *lora)
{
	int ret;

	if (CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO[0]) {
		/* use manual MCC/MNO entry */
		ret = modem_cmd_send(&lora->context.iface,
				     &lora->context.cmd_handler,
				     NULL, 0,
				     "AT+COPS=1,2,\""
				     CONFIG_MODEM_MURATA_LORA_MANUAL_MCCMNO
				     "\"",
				     &lora->sem_response,
				     LORA_CMD_AT_TIMEOUT);
	} else {
		/* register operator automatically */
		ret = modem_cmd_send(&lora->context.iface,
				     &lora->context.cmd_handler,
				     NULL, 0, "AT+COPS=0,0",
				     &lora->sem_response,
				     LORA_CMD_AT_TIMEOUT);
	}

	if (ret < 0) {
		LOG_ERR("AT+COPS ret:%d", ret);
	}

	return ret;
}

static void lora_configure(struct k_work *work)
{
	int r = -1;
	struct lora_modem *lora = CONTAINER_OF(work, struct lora_modem,
					     lora_configure_work);

	LOG_DBG("Starting modem %p configuration", lora);

	while (r < 0) {
		while (true) {
			r = modem_cmd_send(&lora->context.iface,
					   &lora->context.cmd_handler,
					   &response_cmds[0],
					   ARRAY_SIZE(response_cmds),
					   "AT", &lora->sem_response,
					   LORA_CMD_AT_TIMEOUT);
			if (r < 0) {
				LOG_DBG("modem not ready %d", r);
			} else {
				LOG_DBG("connect with modem %d", r);
				// (void)lora_setup_keys(lora);
				break;
			}
		}

		r = modem_cmd_handler_setup_cmds(&lora->context.iface,
						 &lora->context.cmd_handler,
						 setup_cmds,
						 ARRAY_SIZE(setup_cmds),
						 &lora->sem_response,
						 LORA_CMD_SETUP_TIMEOUT);
		if (r < 0) {
			LOG_DBG("modem setup returned %d, %s",
				r, "retrying...");
		} else {
			LOG_DBG("modem setup returned %d, %s",
				r, "enable PPP");
			break;
		}
	}

	lora->setup_done = true;
	k_sem_give(&lora->ppp_send_sem);
}

static int lora_init(struct device *device)
{
	struct lora_modem *lora = device->driver_data;
	int r;

	k_sleep(1000);
	LOG_DBG("Generic LoRa modem (%p)", lora);

	/* pin setup */
	lora->context.pins = modem_pins;
	lora->context.pins_len = ARRAY_SIZE(modem_pins);

	k_sem_init(&lora->ppp_send_sem, 0, 1);

	lora->cmd_handler_data.cmds[CMD_RESP] = response_cmds;
	lora->cmd_handler_data.cmds_len[CMD_RESP] = ARRAY_SIZE(response_cmds);
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
	k_sleep(200);
	modem_pin_write(&lora->context, MDM_RESET, 0);

	k_thread_create(&lora_rx_thread, lora_rx_stack,
			K_THREAD_STACK_SIZEOF(lora_rx_stack),
			(k_thread_entry_t) lora_rx,
			lora, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	k_delayed_work_init(&lora->lora_configure_work, lora_configure);

	// (void)k_delayed_work_submit(&lora->lora_configure_work, 0);

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
