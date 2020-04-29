/**
 * @file witap_log.hpp
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-23
 * 
 * @copyright Copyright (c) 2020
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

#include "witap_types.h"

#define WITAP_LOG_DEFAULT_SYNC_TIMEOUT		K_SECONDS(30)
#define WITAP_LOG_INITIAL_SYNC_TIMEOUT		K_SECONDS(30)

char const boot_count_fname[] = "/boot_count";
char const boot_log_fname[] = "/boot_log.txt";
char const boot_test_string[] = "this is a test string\n";
char const witap_log_fname[] = "/witap_log.txt";


extern "C" {
	int witap_log_backend_enable(char *fname, u32_t init_log_level, k_timeout_t timeout);
}

class WITAP_LOG
{
public:
    WITAP_LOG() {}
    ~WITAP_LOG() {}
    void begin(struct fs_mount_t *boot_mp, struct fs_mount_t *log_mp, k_timeout_t timeout);
    int update_bootcount(struct fs_mount_t *mp);
    int update_bootlog(struct fs_mount_t *mp);
    bool initialized = false;

private:
//     void _initialize_log();
    BOOT_STATUS bootStatus;
};

void WITAP_LOG::begin(struct fs_mount_t *boot_mp, struct fs_mount_t *log_mp, k_timeout_t timeout)
{
	char fname[20];

	snprintf(fname, sizeof(fname), "%s%s", log_mp->mnt_point, witap_log_fname);
	witap_log_backend_enable(fname, CONFIG_LOG_DEFAULT_LEVEL, timeout);
	update_bootcount(boot_mp);
	// Initialize Internal File System
	initialized = true;
}

/**
 * @brief	Track and update the system boot count
 * 
 * @param mp 
 * @return int 
 */
int WITAP_LOG::update_bootcount(struct fs_mount_t *mp) {
	char fname[20];
	struct fs_file_t file;
	u32_t boot_count = 0;
	int ret;

	snprintf(fname, sizeof(fname), "%s%s", mp->mnt_point, boot_count_fname);
	LOG_DBG("opening file: %s", log_strdup(fname));

	ret = fs_open(&file, fname);
	if (ret < 0) {
		LOG_ERR("file open error, ret=%d", ret);
		goto out;
	}

	ret = fs_read(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		LOG_ERR("file read error, count %u, ret=%d", boot_count, ret);
		goto out;
	}

	ret = fs_seek(&file, 0, FS_SEEK_SET);
	if (ret < 0) {
		LOG_ERR("file seek error, ret=%d", ret);
		goto out;
	}

	boot_count += 1;

	ret = fs_write(&file, &boot_count, sizeof(boot_count));
	if (ret < 0) {
		LOG_ERR("file write error, count %u, ret=%d", boot_count, ret);
		goto out;
	}

	ret = fs_close(&file);
	if (ret < 0) {
		LOG_ERR("file close error, ret=%d", ret);
		goto out;
	}

	LOG_INF("Boot count: %d", boot_count);

out:
	return ret;
}

/**
 * @brief	Track and update the system boot log
 * 
 * @param mp 
 * @return int 
 */
int WITAP_LOG::update_bootlog(struct fs_mount_t *mp) {
	char fname[20];
	struct fs_file_t file;
	u32_t boot_count = 0;
	int ret;

	snprintf(fname, sizeof(fname), "%s%s", mp->mnt_point, boot_log_fname);
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

	ret = fs_write(&file, &boot_test_string, sizeof(boot_test_string));
	if (ret < 0) {
		LOG_ERR("file write error, count %u, ret=%d", boot_count, ret);
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

#endif /* WITAP_LOG_H */