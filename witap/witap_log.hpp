/**
 * @file witap_log.hpp
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-23
 * 
 * @copyright Copyright (c) 2020
 * SPDX-License-Identifier: Apache-2.0
 * 
   _____                 _                _        _          
  / ____|               (_)              | |      (_)         
 | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
  \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
  ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
                                                              
 */


#ifndef WITAP_LOG_H
#define WITAP_LOG_H

#include <stdio.h>
#include <zephyr.h>
#include <soc.h>
#include <fs/fs.h>
#include <posix/time.h>
#include <sys/timeutil.h>

#include "witap_types.h"

#define WITAP_LOG_DEFAULT_SYNC_TIMEOUT		K_SECONDS(30)
#define WITAP_LOG_INITIAL_SYNC_TIMEOUT		K_SECONDS(30)

char const boot_count_fname[] = "boot_count";
char const boot_log_fname[] = "boot.log";
char const witap_log_fname[] = "witap.log";
char const test_log_fname[] = "test.log";
char const data_log_fname[] = "data.log";
char const test_string[] = "this is a 32 byte test string. \n";


extern "C" {
	int witap_log_backend_enable(char *fname, u32_t init_log_level, k_timeout_t timeout);
}

class WITAP_LOG
{
public:
	WITAP_LOG() {}
	~WITAP_LOG() {}
	void begin(struct fs_mount_t *boot_mp, struct fs_mount_t *log_mp, struct boot_status *boot, k_timeout_t timeout);
	int update_bootcount(struct fs_mount_t *mp, struct boot_status *boot);
	int update_bootlog(struct fs_mount_t *mp, struct boot_status *boot);
	int update_testlog(struct fs_mount_t *mp);
	int init_datalog(struct fs_mount_t *mp);
	int update_datalog(char *buf);
	int stop_datalog();
	bool initialized = false;
	struct fs_file_t data_file;

private:
//     void _initialize_log();
	boot_status bootStatus;
};

void WITAP_LOG::begin(struct fs_mount_t *boot_mp, struct fs_mount_t *log_mp, struct boot_status *boot, k_timeout_t timeout)
{
	char fname[20];

	if (log_mp) {
		snprintk(fname, sizeof(fname), "%s/%s", log_mp->mnt_point, witap_log_fname);
		witap_log_backend_enable(fname, CONFIG_WITAP_LOG_LEVEL, timeout);
	}
	if (boot_mp) {
		update_bootcount(boot_mp, boot);
	}
}

/**
 * @brief	Track and update the system boot count
 * 
 * @param mp 
 * @return int 
 */
int WITAP_LOG::update_bootcount(struct fs_mount_t *mp, struct boot_status *boot) {
	char fname[20];
	struct fs_file_t file;
	int ret;

	snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, boot_count_fname);
	LOG_DBG("opening file: %s", log_strdup(fname));

	ret = fs_open(&file, fname);
	if (ret < 0) {
		LOG_ERR("file open error, ret=%d", ret);
		goto out;
	}

	ret = fs_read(&file, &boot->count, sizeof(boot->count));
	if (ret < 0) {
		LOG_ERR("file read error, count %u, ret=%d", boot->count, ret);
		goto out;
	}

	ret = fs_seek(&file, 0, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("file seek error, ret=%d", ret);
		goto out;
	}

	boot->count += 1;

	ret = fs_write(&file, &boot->count, sizeof(boot->count));
	if (ret < 0) {
		LOG_ERR("file write error, count %u, ret=%d", boot->count, ret);
		goto out;
	}

	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("file close error, ret=%d", ret);
		goto out;
	}

	LOG_DBG("Boot count: %d", boot->count);

out:
	return ret;
}

/**
 * @brief	Track and update the system boot log
 * 
 * @param mp 
 * @return int 
 */
int WITAP_LOG::update_bootlog(struct fs_mount_t *mp, struct boot_status *boot) {
	char fname[20], buf[43];
	struct fs_file_t file;
	int ret;
	struct timespec tp;
	struct tm tm;

	clock_gettime(CLOCK_REALTIME, &tp);
	gmtime_r(&tp.tv_sec, &tm);

	snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, boot_log_fname);
	LOG_DBG("opening file: %s", log_strdup(fname));

	ret = fs_open(&file, fname);
	if (ret < 0) {
		LOG_ERR("file open error, ret=%d", ret);
		goto out;
	}

	ret = fs_seek(&file, 0, FS_SEEK_END);
	if (ret < 0) {
		LOG_ERR("file seek error, ret=%d", ret);
		goto out;
	}

	/* convert time to seconds */
	snprintk(buf, sizeof(buf), "[%09d] cause: %#02x, count: %d\n", (u32_t) (boot->time / 1000), boot->cause, boot->count);
	ret = fs_write(&file, buf, strlen(buf));
	if (ret < 0) {
		LOG_ERR("file write error, ret=%d", ret);
		goto out;
	}

	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("file close error, ret=%d", ret);
		goto out;
	}

out:
	return ret;
}

/**
 * @brief	write a string to test log
 * 
 * @param mp 
 * @return int 
 */
int WITAP_LOG::update_testlog(struct fs_mount_t *mp) {
	char fname[20];
	struct fs_file_t file;
	int ret;

	snprintk(fname, sizeof(fname), "%s/%s", mp->mnt_point, test_log_fname);
	LOG_DBG("opening file: %s", log_strdup(fname));

	ret = fs_open(&file, fname);
	if (ret < 0) {
		LOG_ERR("file open error, ret=%d", ret);
		goto out;
	}

	ret = fs_seek(&file, 0, FS_SEEK_END);
	if (ret < 0) {
		LOG_ERR("file seek error, ret=%d", ret);
		goto out;
	}

	/* do not print string null */
	ret = fs_write(&file, test_string, sizeof(test_string) - 1);
	if (ret < 0) {
		LOG_ERR("file write error, ret=%d", ret);
		goto out;
	}

	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("file close error, ret=%d", ret);
		goto out;
	}

out:
	return ret;
}

int WITAP_LOG::init_datalog(struct fs_mount_t *mp) {
	struct timespec tp;
	char buf[64];
	int ret;

	snprintk(buf, sizeof(buf), "%s/%s", mp->mnt_point, data_log_fname);
	LOG_DBG("opening file: %s", log_strdup(buf));

	ret = fs_open(&(WITAP_LOG::data_file), buf);
	if (ret < 0) {
		LOG_ERR("file open error, ret=%d", ret);
		goto out;
	}

	ret = fs_seek(&(WITAP_LOG::data_file), 0, FS_SEEK_END);
	if (ret < 0) {
		LOG_ERR("file seek error, ret=%d", ret);
		goto out;
	}

	clock_gettime(CLOCK_REALTIME, &tp);
	if (tp.tv_sec < 1657918426) {
		LOG_WRN("Date is set earlier than July 15, 2022");
	}
	snprintk(buf, sizeof(buf), "%012llu,<<LOG START>>\n", tp.tv_sec);
	/* do not print string null */
	ret = fs_write(&(WITAP_LOG::data_file), buf, strlen(buf));
	if (ret < 0) {
		LOG_ERR("file write error, ret=%d", ret);
		goto out;
	}
	// Initialize Internal File System
	initialized = true;
out:
	return ret;
}

int WITAP_LOG::update_datalog(char *buf) {
	struct timespec tp;
	char csv[64];
	int ret;

	if (initialized) {
		clock_gettime(CLOCK_REALTIME, &tp);
		snprintk(csv, sizeof(csv), "%012llu,%s\n", tp.tv_sec, buf);
		/* do not print string null */
		ret = fs_write(&(WITAP_LOG::data_file), csv, strlen(csv));
		if (ret < 0) {
			LOG_ERR("file write error, ret=%d", ret);
			goto out;
		}
	} else {
		ret = -EINVAL;
	}

out:
	return ret;
}

int WITAP_LOG::stop_datalog() {
	struct timespec tp;
	char buf[64];
	int ret;

	LOG_DBG("stop data logging");

	clock_gettime(CLOCK_REALTIME, &tp);
	snprintk(buf, sizeof(buf), "%012llu,<<LOG STOP>>\n", tp.tv_sec);
	/* do not print string null */
	ret = fs_write(&(WITAP_LOG::data_file), buf, strlen(buf));
	if (ret < 0) {
		LOG_ERR("file write error, ret=%d", ret);
		goto out;
	}

	ret = fs_close(&(WITAP_LOG::data_file));
	if (ret < 0) {
		LOG_ERR("file close error, ret=%d", ret);
		goto out;
	}
	initialized = false;

out:
	return ret;
}

#endif /* WITAP_LOG_H */
