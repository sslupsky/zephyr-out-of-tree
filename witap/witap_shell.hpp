/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-11
 * 
 * @copyright Copyright (c) 2020
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


#ifndef _WITAP_SHELL_HPP_
#define _WITAP_SHELL_HPP_

#include <getopt.h>
#include <ctype.h>
#include <shell/shell.h>

static int cmd_show_version(const struct shell *shell)
{
	shell_print(shell, "WiTAP G3 Wireless Industrial Sensor");
	shell_print(shell, "(c) Copyright 2022");
	shell_print(shell, "Scanimetrics Inc.");
	shell_print(shell, "All rights reserved");
	shell_print(shell, "Zephyr build: %s", STRINGIFY(BUILD_VERSION));
	shell_print(shell, "Zephyr Out Of Tree build: %s", STRINGIFY(OOT_BUILD_VERSION));
	shell_print(shell, "Application build: %s", STRINGIFY(APPLICATION_VERSION));
	shell_print(shell, "Scanimetrics WiTAP command shell");
	return 0;
}

static int cmd_show_banner(const struct shell *shell)
{
	shell_print(shell, "%s", STRING_SCAN_LOGO);
	shell_print(shell, "%s", STRING_WITAP_LOGO);
	return 0;
}

static void forward(const struct shell *shell, struct fs_file_t *file, int num_lines)
{
#define _TAIL_BUFFER_SIZE 16
	u8_t buf[_TAIL_BUFFER_SIZE+1];
	off_t offset, size, pos;
	bool last_char_is_eol;
	int i, ret;

	ret = fs_seek(file, 0, FS_SEEK_END);
	if (ret) {
		shell_error(shell, "Failed to seek to end of file");
		fs_close(file);
		return;
	}

	/* count back the number of lines to determine the file position */
	last_char_is_eol = false;
	size = fs_tell(file);
	pos = MAX(0, size - _TAIL_BUFFER_SIZE);
	offset = size;
	while ((pos > 0) && (num_lines > 0)) {
		ssize_t read;

		ret = fs_seek(file, pos, FS_SEEK_SET);
		if (ret < 0) {
			shell_error(shell, "Failed to seek to offset %d", pos);
			fs_close(file);
			return;
		}

		read = fs_read(file, buf, MIN(pos, _TAIL_BUFFER_SIZE));
		if (read <= 0) {
			break;
		}

		for (i = read; i > 0; i--) {
			if (buf[i-1] == '\n') {
				if ((pos + i) == size) {
					/*
					 * If the last character in the file
					 * is eol then ignore it.
					 */
					last_char_is_eol = true;
				} else {
					/*
					 * Capture the position immediately
					 * after the eol char which is the
					 * beginning of the next line.
					 */
					offset = pos + i;
					num_lines--;
				}
				if (num_lines == 0) {
					break;
				}
			}
		}
		pos -= read;
	}

	/* if we did not find all the lines, then start
	 * at the beginning of the file
	 */
	if (num_lines > 0) {
		offset = 0;
	}

	fs_seek(file, offset, FS_SEEK_SET);
	while (offset < size) {
		ssize_t read;

		read = fs_read(file, buf, MIN(size - offset, _TAIL_BUFFER_SIZE));
		if (read <= 0) {
			break;
		}

		// for (i = 0; i < MIN(size - offset, _TAIL_BUFFER_SIZE); i++) {
		// 	shell_fprintf(shell, SHELL_NORMAL, "%c", buf[i]);
		// }

		/* replace unprintable characters with '.' */
		for (i = 0; i < MIN(size - offset, _TAIL_BUFFER_SIZE); i++) {
			buf[i] = buf[i] == 0 ? '.' : buf[i];
		}
		buf[MIN(size - offset, _TAIL_BUFFER_SIZE)] = '\0';
		shell_fprintf(shell, SHELL_OPTION, "%s", buf);

		offset += read;
	}

	if (!last_char_is_eol) {
		shell_print(shell, "");
	}
	fs_close(file);

}

/**
 * @brief Tail
 *        see reference:  https://github.com/openbsd/src/tree/master/usr.bin/tail
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_tail(const struct shell *shell, size_t argc, char **argv)
{
#define _TAIL_NUM_LINES_DEFAULT 10
#define _TAIL_NUM_BYTES_DEFAULT 16

	char *fname;
	struct fs_file_t file;
	bool qflag;
	int num_lines, num_bytes, num_files, first, ch, ret;

	num_lines = _TAIL_NUM_LINES_DEFAULT;
	num_bytes = _TAIL_NUM_BYTES_DEFAULT;
	qflag = false;
	/* reset getopt() with optind=0 so that successive calls
	 * to getopts() parses the args.  Otherwise getops()
	 * remembers the last parse and returns -1 when
	 * tail is run more than once.
	 * see linux man page
	 * http://man7.org/linux/man-pages/man3/getopt.3.html
	 */
	optind = 0;

	while ((ch = getopt(argc, argv, "c:n:q")) != -1 ) {
		switch (ch) {
		case 'c':
			num_bytes = strtol(optarg, NULL, 0);
			break;
		case 'n':
			num_lines = strtol(optarg, NULL, 0);
			break;
		case 'q':
			qflag = true;
			break;
		case '?':
		default:
			return -ENOTSUP;
		}
	}
	argc -= optind;
	argv += optind;

	num_files = argc ? argc : 1;

	if (*argv) {
		for (first = 1; (fname = *argv++);) {
			ret = fs_open(&file, fname);
			if (ret < 0) {
				shell_error(shell, "failed to open %s (ret=%d)", fname, ret);
				continue;
			}
			if (argc > 1 && !qflag) {
				shell_print(shell, "%s==> %s <==",
					    first ? "" : "\n", fname);
				first = 0;
			}
			forward(shell, &file, num_lines);
		}
	}

	return 0;
}

/**
 * @brief cat support
 * 
 */
#define isascii(__c)	((unsigned)(__c)<=0177)
#define _CAT_BUFFER_SIZE 16

struct cat_flags {
	int bflag, eflag, nflag, sflag, tflag, vflag;
};

int
cook_buf(const struct shell *shell, struct fs_file_t *fp, const char *filename, cat_flags *flags)
{
	unsigned long long line;
	char ch, prev;
	int gobble;
	ssize_t len;

	line = gobble = 0;
	for (prev = '\n'; (len = fs_read(fp, &ch, 1)) > 0; prev = ch) {
		if (prev == '\n') {
			if (flags->sflag) {
				if (ch == '\n') {
					if (gobble)
						continue;
					gobble = 1;
				} else
					gobble = 0;
			}
			if (flags->nflag) {
				if (!flags->bflag || ch != '\n') {
					shell_fprintf(shell, SHELL_OPTION, "%6llu\t", ++line);
				} else if (flags->eflag) {
					shell_fprintf(shell, SHELL_OPTION, "%6s\t", "");
				}
			}
		}
		if (ch == '\n') {
			if (flags->eflag) {
				shell_fprintf(shell, SHELL_OPTION, "$");
			}
		} else if (ch == '\t') {
			if (flags->tflag) {
				shell_fprintf(shell, SHELL_OPTION, "^I");
				continue;
			}
		} else if (flags->vflag) {
			if (!isascii(ch)) {
				shell_fprintf(shell, SHELL_OPTION, "M-");
				ch = ch & 0x7f;
			}
			if (iscntrl(ch)) {
				ch = (ch == '\177' ? '?' : ch | 0100);
				shell_fprintf(shell, SHELL_OPTION, "^%c", ch);
				continue;
			}
		}
		shell_fprintf(shell, SHELL_OPTION, "%c", ch);
	}
	if (len < 0) {
		shell_print(shell,"could not read %s", filename);
		return 1;
	}
	return 0;
}

void
raw_cat(const struct shell *shell, struct fs_file_t *fp, const char *filename)
{
	ssize_t nr, nw, off;

	// static size_t bsize;
	// static char *buf = NULL;
	// struct fs_dirent sbuf;

	// if (buf == NULL) {
	// 	if (fs_stat(filename, &sbuf) < 0)
	// 		shell_print(shell,"file stat error");
	// 	bsize = MIN(sbuf.size, 128);
	// 	if ((buf = (char *)k_malloc(bsize)) == NULL) {
	// 		shell_print(shell,"malloc error");
	// 		return;
	// 	}
	// }

	u8_t buf[_CAT_BUFFER_SIZE+1];
	ssize_t bsize = _CAT_BUFFER_SIZE;

	while ((nr = fs_read(fp, buf, bsize)) > 0) {
		nw = MIN(bsize,nr);
		for (off = 0; nr; nr -= nw, off += nw) {
			buf[nr] = 0;
			shell_fprintf(shell, SHELL_OPTION, "%s", buf);
		}
	}
}

int
cat_file(const struct shell *shell, const char *path, cat_flags *flags)
{
	struct fs_file_t fp;
	int ret;

	if (flags->bflag || flags->eflag || flags->nflag || flags->sflag || flags->tflag || flags->vflag) {
		if (path == NULL || strcmp(path, "-") == 0) {
			shell_print(shell, "stdin unsupported");
			return 1;
		} else {
			if ((ret = fs_open(&fp, path)) < 0) {
				shell_print(shell,"failed to open %s", path);
				return 1;
			}
			cook_buf(shell, &fp, path, flags);
			fs_close(&fp);
		}
	} else {
		if (path == NULL || strcmp(path, "-") == 0) {
			shell_print(shell, "stdin unsupported");
			return 1;
		} else {
			if ((ret = fs_open(&fp, path)) < 0) {
				shell_print(shell,"%s", path);
				return 1;
			}
			raw_cat(shell, &fp, path);
			fs_close(&fp);
		}
	}
	return 0;
}

/**
 * @brief cat
 *        see reference https://github.com/openbsd/src/blob/master/bin/cat/cat.c
 * 
 * @param shell 
 * @param argc 
 * @param argv 
 * @return int 
 */
static int cmd_cat(const struct shell *shell, size_t argc, char **argv)
{
	cat_flags flags = {0};
	int ch, rval;

	// if (pledge("stdio rpath", NULL) == -1)
	// 	err(1, "pledge");

	/* reset getopt() with optind=0 so that successive calls
	 * to getopts() parses the args.  Otherwise getops()
	 * remembers the last parse and returns -1 when
	 * tail is run more than once.
	 * see linux man page
	 * http://man7.org/linux/man-pages/man3/getopt.3.html
	 */
	optind = 0;
	while ((ch = getopt(argc, argv, "benstuv")) != -1) {
		switch (ch) {
		case 'b':
			flags.bflag = flags.nflag = 1;	/* -b implies -n */
			break;
		case 'e':
			flags.eflag = flags.vflag = 1;	/* -e implies -v */
			break;
		case 'n':
			flags.nflag = 1;
			break;
		case 's':
			flags.sflag = 1;
			break;
		case 't':
			flags.tflag = flags.vflag = 1;	/* -t implies -v */
			break;
		case 'u':
			// setvbuf(stdout, NULL, _IONBF, 0);
			break;
		case 'v':
			flags.vflag = 1;
			break;
		default:
			shell_fprintf(shell, SHELL_OPTION, "usage: cat [-benstuv] [file ...]\n");
			return 1;
		}
	}
	argc -= optind;
	argv += optind;

	if (argc == 0) {
		// if (pledge("stdio", NULL) == -1)
		// 	err(1, "pledge");

		rval = cat_file(shell, NULL, &flags);
	} else {
		for (; *argv != NULL; argv++)
			rval = cat_file(shell, *argv, &flags);
	}

	return rval;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_show,
	SHELL_CMD(banner, NULL, "display banner", cmd_show_banner),
	SHELL_CMD(version, NULL, "display version info", cmd_show_version),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);
SHELL_CMD_REGISTER(show, &sub_show, "show [version | banner] - show witap info", NULL);

SHELL_CMD_ARG_REGISTER(tail, NULL, "tail [-q] [-n number] file - display the last part of a file", cmd_tail, 2, 3);
SHELL_CMD_ARG_REGISTER(cat, NULL, "cat -- concatenate and print files", cmd_cat, 2, 3);

#endif /*_WITAP_SHELL_HPP */