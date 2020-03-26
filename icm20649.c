/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <init.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "icm20649.h"

LOG_MODULE_REGISTER(ICM20649, CONFIG_SENSOR_LOG_LEVEL);

int icm20649_set_reg_bank(struct icm20649_data *drv_data, int bank) 
{
	if (i2c_reg_write_byte(drv_data->i2c, drv_data->i2c_slave_addr,
			       ICM20649_REG_BANK_SEL,
			       bank << ICM20649_USER_BANK_POS) < 0) {
		LOG_ERR("Failed to write bank select.");
		return -EIO;
	}
    return 0;
}

/* see "Accelerometer Measurements" section from register map description */
static void icm20649_convert_accel(struct sensor_value *val, s16_t raw_val,
				  u16_t sensitivity_shift)
{
	s64_t conv_val;

	conv_val = ((s64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void icm20649_convert_gyro(struct sensor_value *val, s16_t raw_val,
				 u16_t sensitivity_x10)
{
	s64_t conv_val;

	conv_val = ((s64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void icm20649_convert_temp(struct sensor_value *val,
					s16_t raw_val)
{
	val->val1 = raw_val / 340 + 36;
	val->val2 = ((s64_t)(raw_val % 340) * 1000000) / 340 + 530000;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int icm20649_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct icm20649_data *drv_data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		icm20649_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		icm20649_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		icm20649_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20649_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20649_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		icm20649_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		icm20649_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		icm20649_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20649_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20649_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	default: /* chan == SENSOR_CHAN_DIE_TEMP */
		icm20649_convert_temp(val, drv_data->temp);
	}

	return 0;
}

static int icm20649_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct icm20649_data *drv_data = dev->driver_data;
	s16_t buf[7];

    //  Select Bank 0
    if ( icm20649_set_reg_bank(drv_data, 0) < 0 ) {
        return -EIO;
    }

	if (i2c_burst_read(drv_data->i2c, drv_data->i2c_slave_addr,
			   ICM20649_REG_DATA_START, (u8_t *)buf, 14) < 0) {
		LOG_ERR("Failed to read data sample.");
		return -EIO;
	}

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cpu(buf[2]);
	drv_data->gyro_x  = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_y  = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_z  = sys_be16_to_cpu(buf[5]);
	drv_data->temp    = sys_be16_to_cpu(buf[6]);

	return 0;
}

static const struct sensor_driver_api icm20649_driver_api = {
#if CONFIG_ICM20649_TRIGGER
	.trigger_set = icm20649_trigger_set,
#endif
	.sample_fetch = icm20649_sample_fetch,
	.channel_get = icm20649_channel_get,
};

int icm20649_init(struct device *dev)
{
	struct icm20649_data *drv_data = dev->driver_data;
	u8_t id, i;
    s64_t now;

    now = k_uptime_get() * 1000;
    if (now < ICM20648_STARTUP_TIME_USEC ) {
        k_busy_wait(ICM20648_STARTUP_TIME_USEC - now);
    }

	drv_data->i2c = device_get_binding(DT_INST_0_INVENSENSE_ICM20649_BUS_NAME);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    DT_INST_0_INVENSENSE_ICM20649_BUS_NAME);
		return -EINVAL;
	}
	drv_data->i2c_slave_addr = DT_INST_0_INVENSENSE_ICM20649_BASE_ADDRESS;

    //  Select Bank 0
    if ( icm20649_set_reg_bank(drv_data, 0) < 0 ) {
        return -EIO;
    }

	/* check chip ID */
	if (i2c_reg_read_byte(drv_data->i2c, drv_data->i2c_slave_addr,
			      ICM20649_REG_CHIP_ID, &id) < 0) {
		LOG_ERR("Failed to read chip ID.");
		return -EIO;
	}

	if (id != ICM20649_CHIP_ID) {
		LOG_ERR("Invalid chip ID.");
		return -EINVAL;
	}

    LOG_DBG("ICM20648 chip detected");

	/* wake up chip */
	if (i2c_reg_update_byte(drv_data->i2c, drv_data->i2c_slave_addr,
				ICM20649_REG_PWR_MGMT_1, ICM20649_SLEEP_EN,
				0) < 0) {
		LOG_ERR("Failed to wake up chip.");
		return -EIO;
	}

    // Set duty cycle mode
	if (i2c_reg_update_byte(drv_data->i2c, drv_data->i2c_slave_addr,
				ICM20649_REG_LP_CONFIG, 
                ICM20649_ACCEL_CYCLE | ICM20649_GYRO_CYCLE | ICM20649_MST_CYCLE,
				0) < 0) {
		LOG_ERR("Failed to set duty cycle mode.");
		return -EIO;
	}

	/* set accelerometer full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i+1) == CONFIG_ICM20649_ACCEL_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for accel full-scale range.");
		return -EINVAL;
	}

    //  Select Bank 2
    if ( icm20649_set_reg_bank(drv_data, 2) < 0 ) {
        return -EIO;
    }

	if (i2c_reg_write_byte(drv_data->i2c, drv_data->i2c_slave_addr,
			       ICM20649_REG_ACCEL_CONFIG_1,
			       i << ICM20649_ACCEL_FS_POS) < 0) {
		LOG_ERR("Failed to write accel full-scale range.");
		return -EIO;
	}

	drv_data->accel_sensitivity_shift = 13 - i;

	/* set gyroscope full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i) * 500 == CONFIG_ICM20649_GYRO_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for gyro full-scale range.");
		return -EINVAL;
	}

	if (i2c_reg_write_byte(drv_data->i2c, drv_data->i2c_slave_addr,
			       ICM20649_REG_GYRO_CONFIG_1,
			       i << ICM20649_GYRO_FS_POS) < 0) {
		LOG_ERR("Failed to write gyro full-scale range.");
		return -EIO;
	}

	drv_data->gyro_sensitivity_x10 = icm20649_gyro_sensitivity_x10[i];

#ifdef CONFIG_ICM20649_TRIGGER
	if (icm20649_init_interrupt(dev) < 0) {
		LOG_DBG("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

	return 0;
}

struct icm20649_data icm20649_driver;

DEVICE_AND_API_INIT(icm20649, CONFIG_ICM20649_NAME, icm20649_init, &icm20649_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &icm20649_driver_api);
