/*
 * Copyright (c) 2021 Eug Krashtan
 * Copyright (c) 2022 Wouter Cappelle
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(stm32_vdda, CONFIG_SENSOR_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_vdda)
#define DT_DRV_COMPAT st_stm32_vdda
#define HAS_CALIBRATION 1
#endif

struct stm32_vdda_data {
	const struct device *adc;
	const struct adc_channel_cfg adc_cfg;
	struct adc_sequence adc_seq;
	struct k_mutex mutex;
	int16_t sample_buffer;
	int16_t raw; /* raw adc Sensor value */
};

struct stm32_vdda_config {
	uint16_t *ref_value;
	uint16_t ref_analog_value;
};

static int stm32_vdda_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct stm32_vdda_data *data = dev->data;
	struct adc_sequence *sp = &data->adc_seq;
	int rc;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);

	rc = adc_channel_setup(data->adc, &data->adc_cfg);
	if (rc) {
		LOG_DBG("Setup AIN%u got %d", data->adc_cfg.channel_id, rc);
		goto unlock;
	}

	rc = adc_read(data->adc, sp);
	if (rc == 0) {
		data->raw = data->sample_buffer;
	}

unlock:
	k_mutex_unlock(&data->mutex);

	return rc;
}

static int stm32_vdda_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct stm32_vdda_data *data = dev->data;
	const struct stm32_vdda_config *cfg = dev->config;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	// calculate Vdda according to STM32L1 RM0038 Rev 17, chapter 12.12
	int16_t* pV_ref_int_cal = ( int16_t *) cfg->ref_value;
	val->val1 = cfg->ref_analog_value * (*pV_ref_int_cal) / data->raw;

	return 0;
}

static const struct sensor_driver_api stm32_vdda_driver_api = {
	.sample_fetch = stm32_vdda_sample_fetch,
	.channel_get = stm32_vdda_channel_get,
};

static int stm32_vdda_init(const struct device *dev)
{
	struct stm32_vdda_data *data = dev->data;
	struct adc_sequence *asp = &data->adc_seq;

	k_mutex_init(&data->mutex);

	if (!device_is_ready(data->adc)) {
		LOG_ERR("Device %s is not ready", data->adc->name);
		return -ENODEV;
	}

	*asp = (struct adc_sequence) {
		.channels = BIT(data->adc_cfg.channel_id),
		.buffer = &data->sample_buffer,
		.buffer_size = sizeof(data->sample_buffer),
		.resolution = 12U,
	};

	return 0;
}

#define STM32_VDDA_DEFINE(inst)									\
	static struct stm32_vdda_data stm32_vdda_dev_data_##inst = {				\
		.adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR(inst)),				\
		.adc_cfg = {									\
			.gain = ADC_GAIN_1,							\
			.reference = ADC_REF_INTERNAL,						\
			.acquisition_time = ADC_ACQ_TIME_MAX,					\
			.channel_id = DT_INST_IO_CHANNELS_INPUT(inst),				\
			.differential = 0							\
		},										\
	};											\
												\
	static const struct stm32_vdda_config stm32_vdda_dev_config_##inst = {	\
		.ref_value = (uint16_t *)DT_INST_PROP(inst, vref_addr),		\
		.ref_analog_value = (uint16_t)DT_INST_PROP(inst, analog_value),	\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, stm32_vdda_init, NULL,					\
				  &stm32_vdda_dev_data_##inst, &stm32_vdda_dev_config_##inst,	\
				  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,				\
				  &stm32_vdda_driver_api);						\

DT_INST_FOREACH_STATUS_OKAY(STM32_VDDA_DEFINE)
