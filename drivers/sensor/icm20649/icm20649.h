/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2022 Scanimetrics Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM20649_ICM20649_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM20649_ICM20649_H_

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <zephyr/types.h>

//  All Banks
#define ICM20649_REG_BANK_SEL           0x7F
#define ICM20649_USER_BANK_POS          0x04

//  User Bank 0
#define ICM20649_REG_CHIP_ID		    0x00
#define ICM20649_CHIP_ID			    0xE1

#define ICM20649_REG_USER_CTRL          0x03

#define ICM20649_REG_LP_CONFIG          0x05
#define ICM20649_MST_CYCLE              BIT(6)
#define ICM20649_ACCEL_CYCLE            BIT(5)
#define ICM20649_GYRO_CYCLE             BIT(4)

#define ICM20649_REG_PWR_MGMT_1		    0x06
#define ICM20649_SLEEP_EN		        BIT(6)

#define ICM20649_REG_PWR_MGMT_2		    0x07

#define ICM20649_INT_PIN_CONFIG         0x0F

#define ICM20649_REG_INT_ENABLE         0x10
#define ICM20649_REG_INT_ENABLE_1	    0x11
#define ICM20649_REG_INT_ENABLE_2   	0x12
#define ICM20649_REG_INT_ENABLE_3	    0x13
#define ICM20649_RAW_DATA_0_RDY_EN	    BIT(0)

#define ICM20649_REG_I2C_MST_STATUS     0x17

#define ICM20649_REG_INT_STATUS         0x19
#define ICM20649_REG_INT_STATUS_1		0x1A
#define ICM20649_REG_INT_STATUS_2		0x1B
#define ICM20649_REG_INT_STATUS_3		0x1C

#define ICM20649_REG_DELAY_TIME_H 		0x28
#define ICM20649_REG_DELAY_TIME_L 		0x29

#define ICM20649_REG_DATA_START		    0x2D
#define ICM20649_REG_ACCEL_XOUT_H 		0x2D
#define ICM20649_REG_ACCEL_XOUT_L 		0x2E
#define ICM20649_REG_ACCEL_YOUT_H 		0x2F
#define ICM20649_REG_ACCEL_YOUT_L 		0x30
#define ICM20649_REG_ACCEL_ZOUT_H 		0x31
#define ICM20649_REG_ACCEL_ZOUT_L 		0x32
#define ICM20649_REG_GYRO_XOUT_H 	    0x33
#define ICM20649_REG_GYRO_XOUT_L 	    0x34
#define ICM20649_REG_GYRO_YOUT_H 	    0x35
#define ICM20649_REG_GYRO_YOUT_L 	    0x36
#define ICM20649_REG_GYRO_ZOUT_H 	    0x37
#define ICM20649_REG_GYRO_ZOUT_L 	    0x38
#define ICM20649_REG_TEMP_OUT_H 	    0x39
#define ICM20649_REG_TEMP_OUT_L 	    0x3A

#define ICM20649_REG_FIFO_EN_1          0x66
#define ICM20649_REG_FIFO_EN_2          0x67
#define ICM20649_REG_FIFO_RST           0x68
#define ICM20649_REG_FIFO_MODE          0x69
#define ICM20649_REG_FIFO_COUNTH        0x70
#define ICM20649_REG_FIFO_COUNTL        0x71
#define ICM20649_REG_FIFO_R_W           0x72
#define ICM20649_REG_DATA_RDY_STATUS    0x74
#define ICM20649_REG_FIFO_CONFIG        0x76

//  User Bank 1
#define ICM20649_REG_XA_OFFS_USRH       0x14
#define ICM20649_REG_XA_OFFS_USRL       0x15
#define ICM20649_REG_YA_OFFS_USRH       0x17
#define ICM20649_REG_YA_OFFS_USRL       0x18
#define ICM20649_REG_ZA_OFFS_USRH       0x1A
#define ICM20649_REG_ZA_OFFS_USRL       0x1B
#define ICM20649_REG_TIMEBASE_CORRECTION_PLL    0x28

//  User Bank 2
#define ICM20649_REG_GYRO_SMPLRT_DIV    0x00
#define ICM20649_REG_GYRO_CONFIG_1		0x01
#define ICM20649_REG_GYRO_CONFIG_2		0x02
#define ICM20649_GYRO_FS_POS		1

#define ICM20649_REG_XG_OFFS_USRH       0x03
#define ICM20649_REG_XG_OFFS_USRL       0x04
#define ICM20649_REG_YG_OFFS_USRH       0x05
#define ICM20649_REG_YG_OFFS_USRL       0x06
#define ICM20649_REG_ZG_OFFS_USRH       0x07
#define ICM20649_REG_ZG_OFFS_USRL       0x08
#define ICM20649_REG_ODR_ALIGN_EN       0x09
#define ICM20649_REG_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20649_REG_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20649_REG_INTEL_CTRL         0x12
#define ICM20649_REG_WOM_THR            0x13
#define ICM20649_REG_ACCEL_CONFIG_1		0x14
#define ICM20649_REG_ACCEL_CONFIG_2		0x15
#define ICM20649_ACCEL_FS_POS		1
#define ICM20649_REG_FSYNC_CONFIG       0x52
#define ICM20649_REG_TEMP_CONFIG        0x53
#define ICM20649_REG_MOD_CTRL_USR       0x54

#define ICM20648_STARTUP_TIME_USEC      100000

typedef enum{
	ICM20649_Sample_Mode_Continuous = 0x00,
	ICM20649_Sample_Mode_Cycled,
} ICM20649_LP_CONFIG_CYCLE_e;


typedef enum{
	ICM20649_Clock_Internal_20MHz = 0x00,
	ICM20649_Clock_Auto,
	ICM20649_Clock_TimingReset = 0x07
} ICM20649_PWR_MGMT_1_CLKSEL_e;

typedef enum{
	gpm4 = 0x00,
	gpm8,
	gpm16,
    gpm30,
} ICM20649_ACCEL_CONFIG_FS_SEL_e;

typedef enum{							// Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
	acc_d246bw_n265bw = 0x00,
	acc_d246bw_n265bw_1,
	acc_d111bw4_n136bw,
	acc_d50bw4_n68bw8,
	acc_d23bw9_n34bw4,
	acc_d11bw5_n17bw,
	acc_d5bw7_n8bw3,
	acc_d473bw_n499bw,
} ICM20649_ACCEL_CONFIG_DLPCFG_e;

typedef enum{                       // Full scale range options in degrees per second
	dps500 = 0x00,
	dps1000,
	dps2000,
    dps4000,
} ICM20649_GYRO_CONFIG_1_FS_SEL_e;

typedef enum{                       // Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
	gyr_d196bw6_n229bw8 = 0x00,
	gyr_d151bw8_n187bw6,
	gyr_d119bw5_n154bw3,
	gyr_d51bw2_n73bw3,
	gyr_d23bw9_n35bw9,
	gyr_d11bw6_n17bw8,
	gyr_d5bw7_n8bw9,
	gyr_d361bw4_n376bw5,
} ICM20649_GYRO_CONFIG_1_DLPCFG_e;

/* measured in degrees/sec x10 to avoid floating point */
static const u16_t icm20649_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

struct icm20649_data {
	struct device *i2c;
	u16_t i2c_slave_addr;

	s16_t accel_x;
	s16_t accel_y;
	s16_t accel_z;
	u16_t accel_sensitivity_shift;

	s16_t temp;

	s16_t gyro_x;
	s16_t gyro_y;
	s16_t gyro_z;
	u16_t gyro_sensitivity_x10;

#ifdef CONFIG_ICM20649_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_ICM20649_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_ICM20649_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICM20649_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_ICM20649_TRIGGER */
};

#ifdef CONFIG_ICM20649_TRIGGER
int icm20649_trigger_set(struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int icm20649_init_interrupt(struct device *dev);
#endif

int icm20649_set_reg_bank(struct icm20649_data *drv_data, int bank);

#endif /* __SENSOR_ICM20649__ */
