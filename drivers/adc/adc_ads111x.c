/*
 * Copyright (c) 2020 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ADC driver for the ADS1115/ADS1114 ADCs.
 */

#include <device.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>

#include "adc_ads111x.h"

LOG_MODULE_REGISTER(adc_ads111x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "../../zephyr/drivers/adc/adc_context.h"

#define ADS111X_RESOLUTION 16U
#define ADS101X_RESOLUTION 12U

#define ADS111X_SEL(x)         ((x & BIT_MASK(3)) << 9)

#define KERNEL_TIMER_LATENCY 1

/* Macro for checking if Data Ready Bar IRQ is in use */
#define ADS111X_HAS_ALERT(config) (config->alert_dev_name != NULL)

struct ads111x_config {
	const char *i2c_bus_name;
	u8_t i2c_addr;
	u8_t channels;
	const char *alert_dev_name;
	gpio_pin_t alert_pin;
	gpio_dt_flags_t alert_flags;
};

struct ads111x_data {
	struct adc_context ctx;
	struct device *i2c_dev;
	u16_t *buffer;
	u16_t *repeat_buffer;
	u8_t channels;
	u8_t differential;
	u16_t ch_config[4];
	struct k_thread thread;
	struct k_sem sem;
	struct device *alert_gpio_dev;
	gpio_pin_t alert_gpio_pin;
	struct gpio_callback alert_cb;
	struct k_sem alert_sem;

	K_THREAD_STACK_MEMBER(stack,
			CONFIG_ADC_ADS111X_ACQUISITION_THREAD_STACK_SIZE);
};

__attribute__((unused))
static int ads111x_channel_config(struct device *dev, u8_t channel_id)
{
	const struct ads111x_config *config = dev->config->config_info;
	struct ads111x_data *data = dev->driver_data;
	u8_t tx_bytes[3];
	int ret;

	tx_bytes[0] = ADS111X_REG_POINTER_CONFIG;
	sys_put_be16(data->ch_config[channel_id], &tx_bytes[1]);

	/* Write config register to the ADC */
	ret = i2c_write(data->i2c_dev, tx_bytes, sizeof(tx_bytes),
			config->i2c_addr);
	if (ret < 0) {
		LOG_ERR("failed to write adc config register");
		return -EIO;
	}
}


static int ads111x_channel_setup(struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	const struct ads111x_config *config = dev->config->config_info;
	struct ads111x_data *data = dev->driver_data;

	if (channel_cfg->channel_id >= config->channels) {
		LOG_ERR("unsupported channel id '%d'", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("unsupported channel reference '%d'",
			channel_cfg->reference);
		return -ENOTSUP;
	}

	/* select single shot mode and disable ALERT/RDY */
	data->ch_config[channel_cfg->channel_id] = ADS111X_CONFIG_MODE | ADS111X_CONFIG_CQUE_SEL(3);

	if (channel_cfg->differential) {
		/*  select differential input pair  */
		if (channel_cfg->input_positive == 0 && channel_cfg->input_negative == 1) {
			data->ch_config[channel_cfg->channel_id] &= ~ADS111X_CONFIG_MUX_SEL(7);
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_MUX_SEL(0);
		} else if (channel_cfg->input_positive == 0 && channel_cfg->input_negative == 3) {
			data->ch_config[channel_cfg->channel_id] &= ~ADS111X_CONFIG_MUX_SEL(7);
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_MUX_SEL(1);
		} else if (channel_cfg->input_positive == 1 && channel_cfg->input_negative == 3) {
			data->ch_config[channel_cfg->channel_id] &= ~ADS111X_CONFIG_MUX_SEL(7);
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_MUX_SEL(2);
		} else if (channel_cfg->input_positive == 2 && channel_cfg->input_negative == 3) {
			data->ch_config[channel_cfg->channel_id] &= ~ADS111X_CONFIG_MUX_SEL(7);
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_MUX_SEL(3);
		} else {
			LOG_ERR("unsupported differential input configuration +:%d, -:%d", channel_cfg->input_positive, channel_cfg->input_negative);
			return -ENOTSUP;
		}
	} else {
		/*  select single ended input channel  */
		if (channel_cfg->channel_id <= 3) {
			data->ch_config[channel_cfg->channel_id] &= ~ADS111X_CONFIG_MUX_SEL(7);
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_MUX_SEL((4 | channel_cfg->channel_id));
		} else {
			LOG_ERR("unsupported single ended input %d", channel_cfg->channel_id);
			return -ENOTSUP;
		}
	}

	/**
	 * The datasheet does not specify the reference voltage.  We can
	 * infer the reference voltage is 2.048 V from the default FSR
	 * of 2.048V.
	 * 
	 * Gain selection is relative to the default FSR / reference.
	 * ie:  GAIN = 1 --> FSR = 2.048V
	 */
	switch (channel_cfg->gain) {
	case ADC_GAIN_1_3:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(0);
		break;
	case ADC_GAIN_1_2:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(1);
		break;
	case ADC_GAIN_1:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(2);
		break;
	case ADC_GAIN_2:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(3);
		break;
	case ADC_GAIN_4:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(4);
		break;
	case ADC_GAIN_8:
		data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_GAIN_SEL(5);
		break;
	default:
		LOG_ERR("unsupported channel gain '%d'", channel_cfg->gain);
		return -ENOTSUP;
	}

	/**
	 * ADS111X uses acquisition time units which range from 1.1ms to 125ms.
	 * 
	 * We define the "ADC TIME TICKS" as a millisecond time scale since
	 * "ADC_ACQ_TIME_MICROSECONDS" only has 14 bits of range (16.3ms)
	 * 
	 * See the datasheet Table 8 for more info 
	 */
	if (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time) == ADC_ACQ_TIME_TICKS) {
		switch (ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time)) {
		case ADS111X_ODR_PERIOD_8SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_8SPS);
			break;
		case ADS111X_ODR_PERIOD_16SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_16SPS);
			break;
		case ADS111X_ODR_PERIOD_32SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_32SPS);
			break;
		case ADS111X_ODR_PERIOD_64SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_64SPS);
			break;
		case ADS111X_ODR_PERIOD_128SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_128SPS);
			break;
		case ADS111X_ODR_PERIOD_250SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_250SPS);
			break;
		case ADS111X_ODR_PERIOD_475SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_475SPS);
			break;
		case ADS111X_ODR_PERIOD_860SPS:
			data->ch_config[channel_cfg->channel_id] |= ADS111X_CONFIG_ODR_SEL(ADS111X_CONFIG_ODR_860SPS);
			break;
		default:
			LOG_ERR("unsupported acquisition_time '%d'",
				channel_cfg->acquisition_time);
			return -ENOTSUP;
		}
	} else {
		LOG_ERR("unsupported acquisition_time '%d'",
			channel_cfg->acquisition_time);
		return -ENOTSUP;
	}

	return 0;
}

static int ads111x_validate_buffer_size(struct device *dev,
					const struct adc_sequence *sequence)
{
	const struct ads111x_config *config = dev->config->config_info;
	u8_t channels = 0;
	size_t needed;
	u32_t mask;

	for (mask = BIT(config->channels - 1); mask != 0; mask >>= 1) {
		if (mask & sequence->channels) {
			channels++;
		}
	}

	needed = channels * sizeof(u16_t);
	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int ads111x_start_read(struct device *dev,
			      const struct adc_sequence *sequence)
{
	const struct ads111x_config *config = dev->config->config_info;
	struct ads111x_data *data = dev->driver_data;
	int err;

	if (sequence->resolution != ADS111X_RESOLUTION) {
		LOG_ERR("unsupported resolution %d", sequence->resolution);
		return -ENOTSUP;
	}

	if (find_msb_set(sequence->channels) > config->channels) {
		LOG_ERR("unsupported channels in mask: 0x%08x",
			sequence->channels);
		return -ENOTSUP;
	}

	err = ads111x_validate_buffer_size(dev, sequence);
	if (err) {
		LOG_ERR("buffer size too small");
		return err;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int ads111x_read_async(struct device *dev,
			      const struct adc_sequence *sequence,
			      struct k_poll_signal *async)
{
	struct ads111x_data *data = dev->driver_data;
	int err;

	adc_context_lock(&data->ctx, async ? true : false, async);
	err = ads111x_start_read(dev, sequence);
	adc_context_release(&data->ctx, err);

	return err;
}

static int ads111x_read(struct device *dev,
			const struct adc_sequence *sequence)
{
	return ads111x_read_async(dev, sequence, NULL);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads111x_data *data = CONTAINER_OF(ctx, struct ads111x_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct ads111x_data *data = CONTAINER_OF(ctx, struct ads111x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int ads111x_read_channel(struct device *dev, u8_t channel_id, u16_t *result)
{
	const struct ads111x_config *config = dev->config->config_info;
	struct ads111x_data *data = dev->driver_data;
	u8_t tx_bytes[3];
	u8_t rx_bytes[2];
	u16_t ch_config;
	k_timeout_t sample_period;
	int ret;

	if (channel_id > 3) {
		return 0;
	}
	
	tx_bytes[0] = ADS111X_REG_POINTER_CONFIG;
	ch_config = data->ch_config[channel_id];
	/* set OS bit to start conversion */
	WRITE_BIT(ch_config, ADS111X_REG_CONFIG_OS_POS, 1);
	sys_put_be16(ch_config, &tx_bytes[1]);

	/* Write config register to the ADC */
	ret = i2c_write(data->i2c_dev, tx_bytes, sizeof(tx_bytes),
			config->i2c_addr);
	if (ret < 0) {
		LOG_ERR("failed to write config register");
		return -EIO;
	}

	/* Wait for the conversion to complete */
	sample_period = ads111x_odr_delay_tbl[(ch_config >> ADS111X_CONFIG_ODR_POS) & BIT_MASK(3)] - KERNEL_TIMER_LATENCY;
	k_sleep(K_MSEC(sample_period));

	int i = 0;
	do {
		if (i++) {
			k_sleep(K_MSEC(1));
		}
		/* pointer register is already set to config register */
		/* so we can use i2c_read() here */
		ret = i2c_read(data->i2c_dev, rx_bytes, sizeof(rx_bytes),
			       config->i2c_addr);
		if (ret < 0) {
			LOG_ERR("failed to read config register");
			return -EIO;
		}
		ch_config = sys_get_be16(rx_bytes);
		if (i > 10) {
			LOG_ERR("conversion complete timeout");
			return -EIO;
		}
	} while (!(ch_config & ADS111X_CONFIG_OS));
	
	/* Read the conversion results */
	ret = i2c_burst_read(data->i2c_dev, config->i2c_addr,
			     ADS111X_REG_POINTER_CONVERT, rx_bytes, sizeof(rx_bytes));
	if (ret < 0) {
		LOG_ERR("failed to read conversion result register");
		return -EIO;
	}
	*result = sys_get_be16(rx_bytes);
	*result &= BIT_MASK(ADS111X_RESOLUTION);

	return 0;
}

static void ads111x_acquisition_thread(struct device *dev)
{
	struct ads111x_data *data = dev->driver_data;
	u16_t result = 0;
	u8_t channel;
	int err;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		while (data->channels) {
			channel = find_lsb_set(data->channels) - 1;

			LOG_DBG("reading channel %d", channel);

			err = ads111x_read_channel(dev, channel, &result);
			if (err) {
				LOG_ERR("failed to read channel %d (err %d)",
					channel, err);
				adc_context_complete(&data->ctx, err);
				break;
			}

			LOG_DBG("read channel %d, result = %d", channel,
				result);

			*data->buffer++ = result;
			WRITE_BIT(data->channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

__attribute__((unused))
static void ads111x_alert_callback(struct device *port,
				   struct gpio_callback *cb, u32_t pins)
{
	struct ads111x_data *data =
		CONTAINER_OF(cb, struct ads111x_data, alert_cb);

	/* Signal thread that data is now ready */
	k_sem_give(&data->alert_sem);
}

static int ads111x_init(struct device *dev)
{
	const struct ads111x_config *config = dev->config->config_info;
	struct ads111x_data *data = dev->driver_data;
	u8_t rx_bytes[2];
	u16_t ch_config;
	k_tid_t thread;
	int ret;

	while (k_uptime_ticks() < k_us_to_cyc_ceil32(ADS111X_POWER_ON_TIME_USEC)) {
		/* wait for chip to power up */
	}

	k_sem_init(&data->sem, 0, 1);
	data->i2c_dev = device_get_binding(config->i2c_bus_name);

	if (!data->i2c_dev) {
		LOG_ERR("I2C bus '%s' not found",
			config->i2c_bus_name);
		return -EINVAL;
	}

	if (config->alert_dev_name) {
		data->alert_gpio_dev =
			device_get_binding(config->alert_dev_name);
		if (!data->alert_gpio_dev) {
			LOG_ERR("ALERT GPIO device '%s' not found",
				config->alert_dev_name);
			return -EINVAL;
		}

		data->alert_gpio_pin = config->alert_pin;
		
		ret = gpio_pin_configure(data->alert_gpio_dev, config->alert_pin,
					 GPIO_INPUT | config->alert_flags);
		if (ret) {
			LOG_ERR("failed to configure ALERT GPIO pin (ret %d)",
				ret);
			return -EINVAL;
		}

		// gpio_init_callback(&data->alert_cb, ads111x_alert_callback,
		// 		   BIT(config->alert_pin));

		// ret = gpio_add_callback(data->alert_gpio_dev, &data->alert_cb);
		// if (ret) {
		// 	LOG_ERR("failed to add ALERT callback (ret %d)", ret);
		// 	return -EINVAL;
		// }

		// ret = gpio_pin_interrupt_configure(data->alert_gpio_dev, config->alert_pin,
		// 				   GPIO_INT_EDGE_TO_ACTIVE);
		// if (ret) {
		// 	LOG_ERR("failed to configure ALERT interrupt (ret %d)",
		// 		ret);
		// 	return -EINVAL;
		// }
	}

	/* Read config register from the ADC */
	ret = i2c_burst_read(data->i2c_dev, config->i2c_addr,
			      ADS111X_REG_POINTER_CONFIG, rx_bytes, sizeof(rx_bytes));
	if (ret < 0) {
		LOG_ERR("failed to read config register");
	}
	ch_config = sys_get_be16(rx_bytes);
	LOG_DBG("config reg: %04x", ch_config);

	thread = k_thread_create(&data->thread, data->stack,
			CONFIG_ADC_ADS111X_ACQUISITION_THREAD_STACK_SIZE,
			(k_thread_entry_t)ads111x_acquisition_thread,
			dev, NULL, NULL,
			CONFIG_ADC_ADS111X_ACQUISITION_THREAD_PRIO,
			0, K_NO_WAIT);

	k_thread_name_set(thread, "ads111x");
	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api ads111x_adc_api = {
	.channel_setup = ads111x_channel_setup,
	.read = ads111x_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ads111x_read_async,
#endif
};

#define DT_INST_ADS111X(inst, t) DT_INST(inst, ti_ads##t)

#define ADS111X_DEVICE(t, n, ch) \
	static struct ads111x_data ads##t##_data_##n = { \
		ADC_CONTEXT_INIT_TIMER(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_LOCK(ads##t##_data_##n, ctx), \
		ADC_CONTEXT_INIT_SYNC(ads##t##_data_##n, ctx), \
	}; \
	static const struct ads111x_config ads##t##_config_##n = { \
		.i2c_bus_name = DT_BUS_LABEL(DT_INST_ADS111X(n, t)), \
		.i2c_addr = DT_REG_ADDR(DT_INST_ADS111X(n, t)), \
		.channels = ch, \
		.alert_dev_name = UTIL_AND( \
			DT_NODE_HAS_PROP(DT_INST_ADS111X(n, t), alert_gpios), \
			DT_GPIO_LABEL(DT_INST_ADS111X(n, t), alert_gpios) \
			), \
		.alert_pin = UTIL_AND( \
			DT_NODE_HAS_PROP(DT_INST_ADS111X(n, t), alert_gpios), \
			DT_GPIO_PIN(DT_INST_ADS111X(n, t), alert_gpios) \
			), \
		.alert_flags = UTIL_AND( \
			DT_NODE_HAS_PROP(DT_INST_ADS111X(n, t), alert_gpios), \
			DT_GPIO_FLAGS(DT_INST_ADS111X(n, t), alert_gpios) \
			), \
	}; \
	DEVICE_AND_API_INIT(ads##t##_##n, \
			    DT_LABEL(DT_INST_ADS111X(n, t)), \
			    &ads111x_init, &ads##t##_data_##n, \
			    &ads##t##_config_##n, POST_KERNEL, \
			    CONFIG_ADC_ADS111X_INIT_PRIORITY, \
			    &ads111x_adc_api)

/*
 * ADS1114: 2 single ended channels / 1 differential channel
 */
#define ADS1114_DEVICE(n) ADS111X_DEVICE(1114, n, 8)

/*
 * ADS1115: 4 single ended channels / 2 differential channels
 */
#define ADS1115_DEVICE(n) ADS111X_DEVICE(1115, n, 4)

#define DT_INST_ADS111X_FOREACH(t, inst_expr) \
	UTIL_LISTIFY(DT_NUM_INST(ti_ads##t), DT_CALL_WITH_ARG, inst_expr)

DT_INST_ADS111X_FOREACH(1114, ADS1114_DEVICE);
DT_INST_ADS111X_FOREACH(1115, ADS1115_DEVICE);
