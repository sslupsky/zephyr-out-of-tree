/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <stddef.h>
#include <logging/log_backend.h>
#include <logging/log_core.h>
#include <logging/log_ctrl.h>
#include <logging/log_msg.h>
#include <logging/log_output.h>
#include <irq.h>

#include <fs/fs.h>


#define _LOG_OUTPUT_BUF_SIZE 80

struct log_backend_ctx {
	struct fs_file_t file;
	char fname[20];
	struct k_timer timer;
	bool sync_pending;
	uint16_t char_count;
	k_timeout_t timeout;
};

/* forward declare witap_log_backend */
static const struct log_backend witap_log_backend;

static struct log_backend_ctx witap_log_ctx;
static u8_t buf[_LOG_OUTPUT_BUF_SIZE];

static struct k_work witap_log_backend_work;

static void witap_log_backend_timer_handler(struct k_timer *timer_id) {
	k_work_submit(&witap_log_backend_work);
}

static void witap_log_backend_sync_work_handler(struct k_work *work)
{
	int ret;

	ret = fs_sync(&witap_log_ctx.file);
	witap_log_ctx.sync_pending = false;
	witap_log_ctx.char_count = 0;
	return;
}

static int char_out(u8_t *data, size_t length, void *backend_ctx)
{
	struct log_backend_ctx *ctx = (struct log_backend_ctx *)backend_ctx;
	int ret;

	ret = fs_write(&ctx->file, data, length);
	ctx->char_count += length;
	/*
	 *  if we are close to overflowing the log buffer,
	 *  we need to sync the file
	 */
	if (ctx->char_count + 128 > CONFIG_LOG_BUFFER_SIZE) {
		k_work_submit(&witap_log_backend_work);
	}
	if (!ctx->sync_pending) {
		k_timer_start(&ctx->timer, ctx->timeout, K_MSEC(0));
		ctx->sync_pending = true;
	}

	return ret;
}

LOG_OUTPUT_DEFINE(witap_log_output, char_out, buf, sizeof(buf));

int witap_log_backend_enable(char *fname, u32_t init_log_level, k_timeout_t timeout)
{
	int ret = 0;

	strncpy(witap_log_ctx.fname, fname, sizeof(witap_log_ctx.fname));

	ret = fs_open(&witap_log_ctx.file, fname);
	if (ret < 0) {
		return ret;
	}
	ret = fs_seek(&witap_log_ctx.file, 0, FS_SEEK_END);

	k_work_init(&witap_log_backend_work, witap_log_backend_sync_work_handler);
	k_timer_init(&witap_log_ctx.timer, witap_log_backend_timer_handler, NULL);
	witap_log_ctx.timeout = timeout;
	witap_log_ctx.sync_pending = false;
	witap_log_ctx.char_count = 0;
	log_output_ctx_set(&witap_log_output, &witap_log_ctx);
	log_backend_enable(&witap_log_backend, &witap_log_ctx, init_log_level);
	return ret;
}

int witap_log_backend_disable()
{
	int ret;

	k_timer_stop(&witap_log_ctx.timer);
	log_backend_disable(&witap_log_backend);
	ret = fs_close(&witap_log_ctx.file);

	return ret;
}

static void put(const struct log_backend *const backend,
		struct log_msg *msg)
{
	log_msg_get(msg);

	u32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	log_output_msg_process(&witap_log_output, msg, flags);

	log_msg_put(msg);

}

static void panic(struct log_backend const *const backend)
{
	int ret;

	log_output_flush(&witap_log_output);
	ret = fs_sync(&witap_log_ctx.file);
	witap_log_backend_disable();
}

static void dropped(const struct log_backend *const backend, u32_t cnt)
{
	ARG_UNUSED(backend);

	log_output_dropped_process(&witap_log_output, cnt);
}

static void sync_string(const struct log_backend *const backend,
		     struct log_msg_ids src_level, u32_t timestamp,
		     const char *fmt, va_list ap)
{
	u32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;
	u32_t key;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	key = irq_lock();
	log_output_string(&witap_log_output, src_level, timestamp, fmt, ap, flags);
	irq_unlock(key);
}

static void sync_hexdump(const struct log_backend *const backend,
			 struct log_msg_ids src_level, u32_t timestamp,
			 const char *metadata, const u8_t *data, u32_t length)
{
	u32_t flags = LOG_OUTPUT_FLAG_LEVEL | LOG_OUTPUT_FLAG_TIMESTAMP;
	u32_t key;

	if (IS_ENABLED(CONFIG_LOG_BACKEND_FORMAT_TIMESTAMP)) {
		flags |= LOG_OUTPUT_FLAG_FORMAT_TIMESTAMP;
	}

	key = irq_lock();
	log_output_hexdump(&witap_log_output, src_level, timestamp,
			metadata, data, length, flags);
	irq_unlock(key);
}

static void init(void)
{
	return;
}

const struct log_backend_api witap_log_backend_api = {
	.put = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : put,
	.put_sync_string = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			sync_string : NULL,
	.put_sync_hexdump = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ?
			sync_hexdump : NULL,
	.panic = panic,
	.dropped = IS_ENABLED(CONFIG_LOG_IMMEDIATE) ? NULL : dropped,
	.init = init,
};

LOG_BACKEND_DEFINE(witap_log_backend,
		   witap_log_backend_api,
		   false);
