/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "log_backend_file.h"
#include <logging/log_ctrl.h>
#include <init.h>
#include <logging/log.h>
#include <fs/fs.h>

struct fs_file_t file;

int log_backend_file_output_func(u8_t *data, size_t length, void *ctx)
{
	int ret;

	// print_stream(ctx, data, length);
	ret = fs_write(&file, data, length);
	return length;
}

void log_backend_file_enable(const struct log_backend_file *backend,
			      void *ctx, u32_t init_log_level)
{
	const struct log_file *log_file = (const struct log_file *)ctx;
	int ret = 0;

	if (IS_ENABLED(CONFIG_LOG_IMMEDIATE)) {
		// const struct log_file *log_file;

		// log_file = (const struct log_file *)ctx;

		// /* Reenable transport in blocking mode */
		// err = log_file->iface->api->enable(log_file->iface, true);
	}

	if (ret == 0) {
		log_backend_enable(backend->backend, ctx, init_log_level);
		log_output_ctx_set(backend->log_output, ctx);
		backend->control_block->dropped_cnt = 0;
		ret = fs_open(&file, "/spi/witap_log.txt");
		if (ret < 0) {
			return;
		}
		backend->control_block->state = LOG_BACKEND_FILE_ENABLED;
		ret = fs_seek(&file, 0, FS_SEEK_END);
	}
}

static struct log_msg *msg_from_fifo(const struct log_backend_file *backend)
{
	struct log_backend_file_msg msg;
	int err;

	err = k_msgq_get(backend->msgq, &msg, K_NO_WAIT);

	return (err == 0) ? msg.msg : NULL;
}

static void fifo_flush(const struct log_backend_file *backend)
{
	struct log_msg *msg;

	/* Flush log messages. */
	while ((msg = msg_from_fifo(backend)) != NULL) {
		log_msg_put(msg);
	}
}

static void flush_expired_messages(const struct log_file *log_file)
{
	int err;
	struct log_backend_file_msg msg;
	struct k_msgq *msgq = log_file->log_backend->msgq;
	u32_t timeout = log_file->log_backend->timeout;
	u32_t now = k_uptime_get_32();

	while (1) {
		err = k_msgq_peek(msgq, &msg);

		if (err == 0 && ((now - msg.timestamp) > timeout)) {
			(void)k_msgq_get(msgq, &msg, K_NO_WAIT);
			log_msg_put(msg.msg);

			if (IS_ENABLED(CONFIG_SHELL_STATS)) {
				// atomic_inc(&log_file->stats->log_lost_cnt);
			}
		} else {
			break;
		}
	}
}

static void msg_to_fifo(const struct log_file *log_file,
			struct log_msg *msg)
{
	int err;
	struct log_backend_file_msg t_msg = {
		.msg = msg,
		.timestamp = k_uptime_get_32()
	};

	err = k_msgq_put(log_file->log_backend->msgq, &t_msg,
			 K_MSEC(log_file->log_backend->timeout));

	switch (err) {
	case 0:
		break;
	case -EAGAIN:
	case -ENOMSG:
	{
		flush_expired_messages(log_file);

		err = k_msgq_put(log_file->log_backend->msgq, &msg, K_NO_WAIT);
		if (err) {
			/* Unexpected case as we just freed one element and
			 * there is no other context that puts into the msgq.
			 */
			__ASSERT_NO_MSG(0);
		}
		break;
	}
	default:
		/* Other errors are not expected. */
		__ASSERT_NO_MSG(0);
		break;
	}
}

void log_backend_file_disable(const struct log_backend_file *backend)
{
	fifo_flush(backend);
	log_backend_disable(backend->backend);
	backend->control_block->state = LOG_BACKEND_FILE_DISABLED;
}

static void msg_process(const struct log_output *log_output,
			struct log_msg *msg)
{
	u32_t flags = LOG_OUTPUT_FLAG_LEVEL |
		      LOG_OUTPUT_FLAG_TIMESTAMP |
		      LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;

	log_output_msg_process(log_output, msg, flags);
	log_msg_put(msg);
}

bool log_backend_file_process(const struct log_backend_file *backend)
{
	u32_t dropped;
	const struct log_file *log_file =
			(const struct log_file *)backend->backend->cb->ctx;
	struct log_msg *msg = msg_from_fifo(backend);

	if (!msg) {
		return false;
	}

	dropped = atomic_set(&backend->control_block->dropped_cnt, 0);
	if (dropped) {
		log_output_dropped_process(backend->log_output, dropped);
	}

	msg_process(log_file->log_backend->log_output, msg);

	return true;
}

static void put(const struct log_backend *const backend, struct log_msg *msg)
{
	const struct log_file *log_file = (const struct log_file *)backend->cb->ctx;
	struct k_poll_signal *signal;

	log_msg_get(msg);

	switch (log_file->log_backend->control_block->state) {
	case LOG_BACKEND_FILE_ENABLED:
		msg_to_fifo(log_file, msg);
		log_backend_file_process(log_file->log_backend);
		
		// if (IS_ENABLED(CONFIG_MULTITHREADING)) {
		// 	signal = &log_file->ctx->signals[SHELL_SIGNAL_LOG_MSG];
		// 	k_poll_signal_raise(signal, 0);
		// }

		break;
	case LOG_BACKEND_FILE_PANIC:
		msg_process(log_file->log_backend->log_output, msg);

		break;

	case LOG_BACKEND_FILE_DISABLED:
		/* fall through */
		/* no break */
	default:
		/* Discard message. */
		log_msg_put(msg);
	}
}

static void put_sync_string(const struct log_backend *const backend,
			    struct log_msg_ids src_level, u32_t timestamp,
			    const char *fmt, va_list ap)
{
	const struct log_file *log_file = (const struct log_file *)backend->cb->ctx;
	u32_t key;
	u32_t flags = LOG_OUTPUT_FLAG_LEVEL |
		      LOG_OUTPUT_FLAG_TIMESTAMP |
		      LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;

	key = irq_lock();
	log_output_string(log_file->log_backend->log_output, src_level, timestamp,
			  fmt, ap, flags);
	irq_unlock(key);
}

static void put_sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, u32_t timestamp,
			 const char *metadata, const u8_t *data, u32_t length)
{
	const struct log_file *log_file = (const struct log_file *)backend->cb->ctx;
	u32_t key;
	u32_t flags = LOG_OUTPUT_FLAG_LEVEL |
		      LOG_OUTPUT_FLAG_TIMESTAMP |
		      LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;

	key = irq_lock();
	log_output_hexdump(log_file->log_backend->log_output, src_level, timestamp,
			   metadata, data, length, flags);
	irq_unlock(key);
}

static void panic(const struct log_backend *const backend)
{
	const struct log_file *log_file = (const struct log_file *)backend->cb->ctx;
	int err;

	if (IS_ENABLED(CONFIG_LOG_IMMEDIATE)) {
		return;
	}

	// err = log_file->iface->api->enable(log_file->iface, true);

	if (err == 0) {
		log_file->log_backend->control_block->state =
							LOG_BACKEND_FILE_PANIC;

		/* Move to the start of next line. */

		while (log_backend_file_process(log_file->log_backend)) {
			/* empty */
		}
	} else {
		log_backend_file_disable(log_file->log_backend);
	}
}

static void dropped(const struct log_backend *const backend, u32_t cnt)
{
	const struct log_file *log_file = (const struct log_file *)backend->cb->ctx;
	const struct log_backend_file *log_backend = log_file->log_backend;

	// atomic_add(&log_file->stats->log_lost_cnt, cnt);
	atomic_add(&log_backend->control_block->dropped_cnt, cnt);
}

static void log_backend_file_init(struct device *arg)
{
	ARG_UNUSED(arg);
	struct device *dev =
			device_get_binding(CONFIG_UART_SHELL_ON_DEV_NAME);
	bool log_backend = CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > 0;
	u32_t level =
		(CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL > LOG_LEVEL_DBG) ?
		CONFIG_LOG_MAX_LEVEL : CONFIG_SHELL_BACKEND_SERIAL_LOG_LEVEL;

	// shell_init(&shell_uart, dev, true, log_backend, level);

	return 0;
}

const struct log_backend_api log_backend_file_api = {
	.put = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : put,
	.put_sync_string = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			put_sync_string : NULL,
	.put_sync_hexdump = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			put_sync_hexdump : NULL,
	.dropped = dropped,
	.panic = panic,
};

SYS_INIT(log_backend_file_init, POST_KERNEL, 0);
