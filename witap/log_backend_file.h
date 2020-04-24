/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOG_BACKEND_FILE_H__
#define LOG_BACKEND_FILE_H__

#include <zephyr.h>
#include <logging/log_backend.h>
#include <logging/log_output.h>
#include <sys/atomic.h>
#include <fs/fs.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const struct log_backend_api log_backend_file_api;

/** @brief File log backend states. */
enum log_backend_file_state {
	LOG_BACKEND_FILE_UNINIT,
	LOG_BACKEND_FILE_ENABLED,
	LOG_BACKEND_FILE_DISABLED,
	LOG_BACKEND_FILE_PANIC,
};

/** @brief File log backend control block (RW data). */
struct log_backend_file_control_block {
	atomic_t dropped_cnt;
	enum log_backend_file_state state;
};

/** @brief File log backend instance structure (RO data). */
struct log_backend_file {
	const struct log_backend *backend;
	struct k_msgq *msgq;
	const struct log_output *log_output;
	struct log_backend_file_control_block *control_block;
	u32_t timeout;
};

/** @brief File log backend message structure. */
struct log_backend_file_msg {
	struct log_msg *msg;
	u32_t timestamp;
};

struct log_file {
	struct fs_file_t *file;
	const char *fname;
	const struct log_backend_file *log_backend;
};

/** @brief Prototype of function outputing processed data. */
int log_backend_file_output_func(u8_t *data, size_t length, void *ctx);

/** @def LOG_BACKEND_FILE_DEFINE
 *  @brief Macro for creating instance of file log backend.
 *
 *  @param _name	File name.
 *  @param _buf		Output buffer.
 *  @param _size	Output buffer size.
 *  @param _queue_size	Log message queue size.
 *  @param _timeout	Timeout in milliseconds for pending on queue full.
 *			Message is dropped on timeout.
 */
/** @def LOG_BACKEND_FILE_PTR
 *  @brief Macro for retrieving pointer to the instance of file log backend.
 *
 *  @param _name File name.
 */
#ifdef CONFIG_LOG_BACKEND_FILE
#define LOG_BACKEND_FILE_DEFINE(_name, _buf, _size, _queue_size, _timeout)  \
	LOG_BACKEND_DEFINE(_name##_backend, log_backend_file_api, false);   \
	K_MSGQ_DEFINE(_name##_msgq, sizeof(struct log_backend_file_msg),    \
			_queue_size, sizeof(void *));			     \
	LOG_OUTPUT_DEFINE(_name##_log_output, log_backend_file_output_func, \
			  _buf, _size);					     \
	static struct log_backend_file_control_block _name##_control_block; \
	static const struct log_backend_file _name##_log_backend = {	     \
		.backend = &_name##_backend,				     \
		.msgq = &_name##_msgq,					     \
		.log_output = &_name##_log_output,			     \
		.control_block = &_name##_control_block,		     \
		.timeout = _timeout					     \
	}

#define LOG_BACKEND_FILE_PTR(_name) (&_name##_log_backend)
#else /* CONFIG_LOG */
#define LOG_BACKEND_FILE_DEFINE(_name, _buf, _size, _queue_size, _timeout)
#define LOG_BACKEND_FILE_PTR(_name) NULL
#endif /* CONFIG_LOG */

/** @brief Enable file log backend.
 *
 * @param backend		File log backend instance.
 * @param ctx			Pointer to file instance.
 * @param init_log_level	Initial log level set to all logging sources.
 */
void log_backend_file_enable(const struct log_backend_file *backend,
			      void *ctx, u32_t init_log_level);

/** @brief Disable file log backend.
 *
 * @param backend File log backend instance.
 */
void log_backend_file_disable(const struct log_backend_file *backend);

/** @brief Trigger processing of one log entry.
 *
 * @param backend File log backend instance.
 *
 * @return True if message was processed, false if FIFO was empty
 */
bool log_backend_file_process(const struct log_backend_file *backend);

#ifdef __cplusplus
}
#endif

#endif /* LOG_BACKEND_FILE_H__ */
