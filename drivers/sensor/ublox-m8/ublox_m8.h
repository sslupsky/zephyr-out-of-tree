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
// #include <net/buf.h>
#include <sys/ring_buffer.h>
#include <kernel.h>
#include "ubx.h"


#define UBLOX_M8_STARTUP_TIME_USEC	1000
#define UBLOX_M8_MESSAGE_TIMEOUT_MSEC	1100
#define UBLOX_M8_PVT_TIMEOUT_MSEC	10000

#define UBLOX_M8_RECV_BUF_SIZE		128

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define UBLOX_M8_ADDRESS	(0x42)		/**< The default I2C address for the gnss. */


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
#define UBLOX_SAM_M8Q_TXD_PIO		6
#define UBLOX_SAM_M8Q_EXTINT_PIO	13
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

enum ublox_comm_port {
	UBLOX_COMM_PORT_DDC,
	UBLOX_COMM_PORT_UART1,
	UBLOX_COMM_PORT_USB,
	UBLOX_COMM_PORT_SPI,
};

#define UBLOX_PROTO_UBX		BIT(0)
#define UBLOX_PROTO_NMEA	BIT(1)
#define UBLOX_PROTO_RTCM	BIT(2)
#define UBLOX_PROTO_RTCM3	BIT(3)

/*!
 *  Struct to hold registers.
 */
union ublox_m8_register_len {
	u16_t len;
};

union ublox_m8_register_msg {
	u8_t msg;
};

struct ublox_m8_device {
	union ublox_m8_register_len len;
	union ublox_m8_register_msg msg;
};

union ublox_m8 {
	struct ublox_m8_device reg;
	u8_t raw[sizeof(struct ublox_m8_device)];
};


struct msg_handler_data {
	struct net_buf_pool *buf_pool;
	struct k_sem msg_sem;
	struct k_work msg_work;
	int last_error;
};

union ublox_device_id {
	struct {
		u8_t reserved[3];
		u8_t id[5];
	};
	u64_t be_word;
};

struct ublox_frame {
	struct ubx_frame ubx;
	u8_t nmea[128];
	u8_t rtcm[128];
};

/*
 * used to evaluated size of structures and types
 */
// u8_t x = sizeof(struct k_thread);
// u8_t x = sizeof(struct k_timer);
// u8_t x = sizeof(struct k_delayed_work);
// u8_t x = sizeof(struct k_pipe);
// u8_t x = sizeof(struct k_sem);
// u8_t x = sizeof(struct k_mutex);
// u8_t x = sizeof(struct k_work);
// u8_t x = sizeof(k_timeout_t);


// u8_t x = sizeof(struct ublox_frame);
// u8_t x = sizeof(struct gpio_callback);
// u8_t x = sizeof(struct gnss_trigger);
// u8_t x = sizeof(gnss_trigger_handler_t);

/* driver structs */
struct ublox_m8_data {
	struct device *i2c;
	enum gnss_device_state device_state;
	enum gnss_sentence_state sentence_state;
	void (*state_change_cb)(u8_t state);
	struct device *dev;
	struct device *extint_gpio;
	struct device *reset_gpio;
	struct device *safeboot_gpio;
	struct device *rxd_gpio;
	struct device *txd_gpio;

	u8_t rx_buf[UBLOX_M8_RECV_BUF_SIZE];
	u8_t __aligned(4) rb_buf[UBX_MAX_FRAME_SIZE];

	// struct net_buf_pool *buf_pool;
	struct k_sem msg_sem;
	struct k_work msg_work;
	int last_error;
	int poll_status;

	struct k_sem ubx_frame_sem;
	struct k_sem ubx_get_sem;
	struct k_sem ubx_ack_sem;
	struct k_sem ubx_poll_sem;
	k_timeout_t timeout;
	struct k_delayed_work config_work;
	struct k_mutex msg_get_mtx;
	struct k_mutex msg_send_mtx;

	K_THREAD_STACK_MEMBER(msg_thread_stack, CONFIG_UBLOX_M8_MSG_THREAD_STACK_SIZE);
	struct k_thread msg_thread;

	struct ublox_frame frame;
	union ublox_device_id device_id;

	struct gnss_pvt pvt;
	bool pvt_ready;
	struct gnss_status nav_status;
	char sw_version[30];
	char hw_version[10];
	u8_t ubx_get_buf[UBX_MAX_PAYLOAD_SIZE];
	struct ubx_payload_ack ubx_ack_buf;

#ifdef CONFIG_UBLOX_M8_TRIGGER
	struct device *txready_gpio;
	struct gpio_callback txready_gpio_cb;

	struct gnss_trigger txready_trigger;
	gnss_trigger_handler_t txready_handler;

	struct device *timepulse_gpio;
	struct gpio_callback timepulse_gpio_cb;

	struct gnss_trigger timepulse_trigger;
	gnss_trigger_handler_t timepulse_handler;

	struct gnss_trigger pvt_trigger;
	gnss_trigger_handler_t pvt_handler;

	struct gnss_trigger poll_trigger;
	gnss_trigger_handler_t poll_handler;

#if defined(CONFIG_UBLOX_M8_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(trigger_thread_stack, CONFIG_UBLOX_M8_TRIGGER_THREAD_STACK_SIZE);
	struct k_thread trigger_thread;
	struct k_sem txready_gpio_sem;
	struct k_poll_event events[3];
	struct k_poll_signal txready_signal;
	struct k_poll_signal timepulse_signal;
	struct k_poll_signal pvt_signal;
#elif defined(CONFIG_UBLOX_M8_TRIGGER_GLOBAL_THREAD)
	struct k_work trigger_txready_work;
	struct k_work trigger_timepulse_work;
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
