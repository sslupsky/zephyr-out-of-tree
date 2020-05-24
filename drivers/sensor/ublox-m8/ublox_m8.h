/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-18
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

/*
 *  This is a library for the Ublox M8 series GNSS modules.
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef __GNSS_UBLOX_M8_H__
#define __GNSS_UBLOX_M8_H__

#include <device.h>
#include <sys/util.h>
#include <drivers/gnss.h>
#include <drivers/gpio.h>
#include <net/buf.h>
#include <sys/ring_buffer.h>
#include "ubx.h"


#define UBLOX_M8_STARTUP_TIME_USEC	1000
#define UBLOX_M8_MESSAGE_TIMEOUT_MSEC	1000

#define UBLOX_M8_RECV_BUF_SIZE		128
#define UBLOX_MAX_DATA_LENGTH		256

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define UBLOX_M8_ADDRESS	(0x42)		/**< The default I2C address for the gnss. */
#define UBLOX_M8_CHIPID		(0x23)		/**< Default chip ID. */
#define UBLOX_M8_CHIPID_MASK	0b11111011


// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce defaultMaxWait to 250.
#ifndef defaultMaxWait // Let's allow the user to define their own value if they want to
#define defaultMaxWait 1100
#endif

// getPVT will only return data once in each navigation cycle. By default, that is once per second.
// Therefore we should set getPVTmaxWait to slightly longer than that.
// If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
// then you should use a shorter maxWait for getPVT. 300msec would be about right: getPVT(300)
// The same is true for getHPPOSLLH.
#define getPVTmaxWait 1100		// Default maxWait for getPVT and all functions which call it
#define getHPPOSLLHmaxWait 1100 // Default maxWait for getHPPOSLLH and all functions which call it


#define UBLOX_M8_REGISTER_LEN		0xFD
#define UBLOX_M8_REGISTER_MSG		0xFF
#define UBLOX_SAM_M8Q_TXREADY_PIO	6

/**
 * Driver for the UBLOX_M8 gnss.
 */

/**
 * @brief Registers available on the gnss.
 * 
 */

// public:
// Global Status Returns
enum ublox_status {
	UBLOX_STATUS_SUCCESS,
	UBLOX_STATUS_FAIL,
	UBLOX_STATUS_CRC_FAIL,
	UBLOX_STATUS_TIMEOUT,
	UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
	UBLOX_STATUS_OUT_OF_RANGE,
	UBLOX_STATUS_INVALID_ARG,
	UBLOX_STATUS_INVALID_OPERATION,
	UBLOX_STATUS_MEM_ERR,
	UBLOX_STATUS_HW_ERR,
	UBLOX_STATUS_DATA_SENT,		// This indicates that a 'set' was successful
	UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
	UBLOX_STATUS_I2C_COMM_FAILURE,
	UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
};


/*!
 *  Struct to hold registers.
 */
typedef struct {
} UBLOX_M8_device_t;

typedef union {
	u8_t reg;
} UBLOX_M8_register_t;

typedef union {
	UBLOX_M8_device_t reg;
	u8_t raw[sizeof(UBLOX_M8_device_t)];
} gnss_t;


struct msg_handler_data {
	struct net_buf_pool *buf_pool;
	struct k_sem msg_sem;
	struct k_work msg_work;
	int last_error;
};

/* driver structs */
struct ublox_m8_data {
	struct device *i2c;
	gnss_t gnss;
	void (*state_change_cb)(u8_t state);
	struct device *dev;
	struct device *extint_gpio;
	struct device *reset_gpio;
	struct device *safeboot_gpio;
	struct device *rxd_gpio;
	struct device *txd_gpio;

	u8_t rx_buf[UBLOX_M8_RECV_BUF_SIZE];
	struct ring_buf rx_rb;
	u8_t rx_rb_buf[UBX_MAX_FRAME_SIZE];

	struct net_buf_pool *buf_pool;
	struct k_sem msg_sem;
	struct k_work msg_work;
	int last_error;
	u8_t msg_read_buf[UBLOX_M8_RECV_BUF_SIZE];
	u16_t msg_read_buf_len;
	// u8_t msg_match_buf[UBLOX_M8_RECV_BUF_SIZE];
	// u16_t msg_match_buf_len;

	struct k_sem response_sem;
	k_timeout_t timeout;

#ifdef CONFIG_UBLOX_M8_TRIGGER
	struct device *txready_gpio;
	struct gpio_callback txready_gpio_cb;

	struct gnss_trigger data_ready_trigger;
	gnss_trigger_handler_t data_ready_handler;

	struct device *timepulse_gpio;
	struct gpio_callback timepulse_gpio_cb;

	struct gnss_trigger timepulse_trigger;
	gnss_trigger_handler_t timepulse_handler;

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_UBLOX_M8_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem txready_gpio_sem;
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_UBLOX_M8_TRIGGER */
};

struct ublox_m8_dev_config {
	char *i2c_name;
	u8_t i2c_addr;
	char *txready_gpio_name;
	u8_t txready_gpio_pin;
	unsigned int txready_gpio_flags;
	char *reset_gpio_name;
	u8_t reset_gpio_pin;
	unsigned int reset_gpio_flags;
	char *timepulse_gpio_name;
	u8_t timepulse_gpio_pin;
	unsigned int timepulse_gpio_flags;
	char *safeboot_gpio_name;
	u8_t safeboot_gpio_pin;
	unsigned int safeboot_gpio_flags;
	char *extint_gpio_name;
	u8_t extint_gpio_pin;
	unsigned int extint_gpio_flags;
	char *rxd_gpio_name;
	u8_t rxd_gpio_pin;
	unsigned int rxd_gpio_flags;
	char *txd_gpio_name;
	u8_t txd_gpio_pin;
	unsigned int txd_gpio_flags;
};

#ifdef CONFIG_UBLOX_M8_TRIGGER
int ublox_m8_trigger_set(struct device *dev,
			const struct gnss_trigger *trig,
			gnss_trigger_handler_t handler);

int ublox_m8_init_interrupt(struct device *dev);
#endif

#endif /* __GNSS_UBLOX_M8_H__ */
