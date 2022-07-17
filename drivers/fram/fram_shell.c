/*
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief FRAM shell commands.
 */

#include <shell/shell.h>
#include <drivers/fram.h>
#include <stdlib.h>

#include "fram_mb85rs.h"

struct args_index {
	uint8_t device;
	uint8_t offset;
	uint8_t length;
	uint8_t data;
	uint8_t pattern;
};

static const struct args_index args_indx = {
	.device = 1,
	.offset = 2,
	.length = 3,
	.data = 3,
	.pattern = 4,
};

static int cmd_read(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
	struct device *fram;
	off_t offset;
	size_t len;
	int err;

	offset = strtoul(argv[args_indx.offset], NULL, 0);
	len = strtoul(argv[args_indx.length], NULL, 0);

	if (len > sizeof(buf)) {
		shell_error(shell, "Read buffer size (%d bytes) exceeded",
			    sizeof(buf));
		return -EINVAL;
	}

	fram = device_get_binding(argv[args_indx.device]);
	if (!fram) {
		shell_error(shell, "FRAM device not found");
		return -EINVAL;
	}

	shell_print(shell, "Reading %d bytes from FRAM, offset %d...", len,
		    offset);

	err = fram_read(fram, offset, buf, len);
	if (err) {
		shell_error(shell, "FRAM read failed (err %d)", err);
		return err;
	}

	shell_hexdump(shell, buf, len);

	return 0;
}

static int cmd_write(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t wr_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
	uint8_t rd_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
	struct device *fram;
	unsigned long byte;
	off_t offset;
	size_t len;
	int err;
	int i;

	offset = strtoul(argv[args_indx.offset], NULL, 0);
	len = argc - args_indx.data;

	if (len > sizeof(wr_buf)) {
		shell_error(shell, "Write buffer size (%d bytes) exceeded",
			    sizeof(wr_buf));
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		byte = strtoul(argv[args_indx.data + i], NULL, 0);
		if (byte > UINT8_MAX) {
			shell_error(shell, "Error parsing data byte %d", i);
			return -EINVAL;
		}
		wr_buf[i] = byte;
	}

	fram = device_get_binding(argv[args_indx.device]);
	if (!fram) {
		shell_error(shell, "FRAM device not found");
		return -EINVAL;
	}

	shell_print(shell, "Writing %d bytes to FRAM...", len);

	err = fram_write(fram, offset, wr_buf, len);
	if (err) {
		shell_error(shell, "FRAM write failed (err %d)", err);
		return err;
	}

	shell_print(shell, "Verifying...");

	err = fram_read(fram, offset, rd_buf, len);
	if (err) {
		shell_error(shell, "FRAM read failed (err %d)", err);
		return err;
	}

	if (memcmp(wr_buf, rd_buf, len) != 0) {
		shell_error(shell, "Verify failed");
		return -EIO;
	}

	shell_print(shell, "Verify OK");

	return 0;
}

static int cmd_size(const struct shell *shell, size_t argc, char **argv)
{
	struct device *fram;

	fram = device_get_binding(argv[args_indx.device]);
	if (!fram) {
		shell_error(shell, "FRAM device not found");
		return -EINVAL;
	}

	shell_print(shell, "%d bytes", fram_get_size(fram));
	return 0;
}

static int cmd_fill(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t wr_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
	uint8_t rd_buf[CONFIG_FRAM_SHELL_BUFFER_SIZE];
	struct device *fram;
	unsigned long pattern;
	off_t offset;
	size_t len;
	int err;

	offset = strtoul(argv[args_indx.offset], NULL, 0);
	len = strtoul(argv[args_indx.length], NULL, 0);

	if (len > sizeof(wr_buf)) {
		shell_error(shell, "Write buffer size (%d bytes) exceeded",
			    sizeof(wr_buf));
		return -EINVAL;
	}

	pattern = strtoul(argv[args_indx.pattern], NULL, 0);
	if (pattern > UINT8_MAX) {
		shell_error(shell, "Error parsing pattern byte");
		return -EINVAL;
	}
	memset(wr_buf, pattern, len);

	fram = device_get_binding(argv[args_indx.device]);
	if (!fram) {
		shell_error(shell, "FRAM device not found");
		return -EINVAL;
	}

	shell_print(shell, "Writing %d bytes of 0x%02x to FRAM...", len,
		    pattern);

	err = fram_write(fram, offset, wr_buf, len);
	if (err) {
		shell_error(shell, "FRAM write failed (err %d)", err);
		return err;
	}

	shell_print(shell, "Verifying...");

	err = fram_read(fram, offset, rd_buf, len);
	if (err) {
		shell_error(shell, "FRAM read failed (err %d)", err);
		return err;
	}

	if (memcmp(wr_buf, rd_buf, len) != 0) {
		shell_error(shell, "Verify failed");
		return -EIO;
	}

	shell_print(shell, "Verify OK");

	return 0;
}

static int cmd_id(const struct shell *shell, size_t argc, char **argv)
{
	struct device *fram;
	struct fram_mb85rsx_device_id id;

	fram = device_get_binding(argv[args_indx.device]);
	if (!fram) {
		shell_error(shell, "FRAM device not found");
		return -EINVAL;
	}

	id = fram_mb85rsx_id(fram);
	shell_print(shell, "Manufacturer ID: 0x%02x", id.manufacturer_id);
	shell_print(shell, "Continuation Code: 0x%02x", id.continuation_code);
	shell_print(shell, "Product ID byte 1: 0x%02x", id.product_id[0]);
	shell_print(shell, "Manufacturer ID byte 2: 0x%02x", id.product_id[1]);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(fram_cmds,
	SHELL_CMD_ARG(read, NULL, "<device> <offset> <length>", cmd_read, 4, 0),
	SHELL_CMD_ARG(write, NULL,
		      "<device> <offset> [byte0] <byte1> .. <byteN>", cmd_write,
		      4, CONFIG_FRAM_SHELL_BUFFER_SIZE - 1),
	SHELL_CMD_ARG(size, NULL, "<device>", cmd_size, 2, 0),
	SHELL_CMD_ARG(fill, NULL, "<device> <offset> <length> <pattern>",
		      cmd_fill, 5, 0),
	SHELL_CMD_ARG(id, NULL, "<device>", cmd_id, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fram, &fram_cmds, "FRAM shell commands", NULL);
