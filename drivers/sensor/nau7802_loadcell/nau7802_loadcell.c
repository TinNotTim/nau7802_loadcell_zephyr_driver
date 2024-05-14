/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include "nau7802_loadcell.h"

/* Register the module to logging submodule*/
LOG_MODULE_REGISTER(NAU7802_LOADCELL, CONFIG_SENSOR_LOG_LEVEL);

/**************************************************************************/
/*!
    @brief Perform a soft reset
	@return 0 if success
*/
/**************************************************************************/
static int nau7802_reset(const struct nau7802_loadcell_config *config)
{
	int ret;
	/* Set the RR bit to 1 in R0x00, to guarantee a reset of all register values.*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_RR, 1);
	if(ret != 0){
		LOG_ERR("Chip reset failed");
		return -EIO;
	}
	/* Set the RR bit to 0 and PUD bit 1, in R0x00, to enter normal operation*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_RR, 0);
	if(ret != 0){
		LOG_ERR("Chip reset RR bit failed");
		return -EIO;
	}
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUD, 1);
	if(ret != 0){
		LOG_ERR("Chip set PUD bit failed");
		return -EIO;
	}
	/*!
	After about 200 microseconds, the PWRUP bit will be Logic=1 indicating the
	device is ready for the remaining programming setup.
	*/
	k_sleep(K_USEC(300));
	uint8_t pu_ctrl_val;
	ret = i2c_reg_read_byte_dt(config->bus, NAU7802_PU_CTRL, &pu_ctrl_val);
	if(ret != 0){
		LOG_ERR("Read PUR bit failed");
		return -EIO;
	}

	if((pu_ctrl_val & NAU7802_MASK_PU_CTRL_PUR) == 0){
		LOG_ERR("Chip is not powered up, PUR bit is 0.");
		return -EIO;
	}

	return 0;
}

/**************************************************************************/
/*!
    @brief  Whether to have the sensor enabled and working or in power down mode
    @param  flag True to be in powered mode, False for power down mode
    @return 0 if success
*/
/**************************************************************************/
static int nau7802_enable(const struct nau7802_loadcell_config *config, bool flag)
{
	int ret;
	/* Turn off the IC*/
	if(!flag){
		/* Reset the PUA bit*/
		ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUA, 0);
		if(ret != 0){
			LOG_ERR("Chip reset PUA bit failed");
			return -EIO;
		}

		/* Reset the PUD bit*/
		ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUD, 0);
		if(ret != 0){
			LOG_ERR("Chip reset PUD bit failed");
			return -EIO;
		}
		/* success*/
		return 0;
	}

	/* Turn on the IC*/
	/* Set the PUA bit*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUA, 1);
	if(ret != 0){
		LOG_ERR("Chip set PUA bit failed");
		return -EIO;
	}

	/* Set the PUD bit*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_PUD, 1);
	if(ret != 0){
		LOG_ERR("Chip set PUD bit failed");
		return -EIO;
	}

	/*!
		RDY: Analog part wakeup stable plus Data Ready after exiting power-down
		mode 600ms
	*/
	k_sleep(K_MSEC(600));

	uint8_t pu_ctrl_val;
	ret = i2c_reg_read_byte_dt(config->bus, NAU7802_PU_CTRL, &pu_ctrl_val);
	if(ret != 0){
		LOG_ERR("Read PUR bit failed");
		return -EIO;
	}

	if((pu_ctrl_val & NAU7802_MASK_PU_CTRL_PUR) == 0){
		LOG_ERR("Chip is not powered up, PUR bit is 0.");
		return -EIO;
	}
	/* success*/
	return 0;


}

/**************************************************************************/
/*!
    @brief  The desired LDO voltage setter
    @param voltage The LDO setting: NAU7802_4V5, NAU7802_4V2, NAU7802_3V9,
    NAU7802_3V6, NAU7802_3V3, NAU7802_3V0, NAU7802_2V7, NAU7802_2V4, or
    NAU7802_EXTERNAL if we are not using the internal LDO
    @return 0 if success
*/
/**************************************************************************/
static int nau7802_setLDO(const struct nau7802_loadcell_config *config, NAU7802_LDOVoltage voltage)
{
	int ret;

	if (voltage == NAU7802_EXTERNAL){
		/* Reset the AVDD bit in PU_CTRL register*/
		ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_AVDDS, 0);
		if(ret != 0){
			LOG_ERR("Chip reset AVDDS bit failed");
			return -EIO;
		}

		/* success*/
		return 0;
	}

	/* Set the AVDD bit in PU_CTRL register*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_PU_CTRL, NAU7802_MASK_PU_CTRL_AVDDS, 1);
	if(ret != 0){
		LOG_ERR("Chip set AVDDS bit failed");
		return -EIO;
	}

	/* Write the LDO voltage to CTRL1 register*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_CTRL1, NAU7802_MASK_CTRL1_VLDO, voltage);
	if(ret != 0){
		LOG_ERR("Chip set VLDO bits failed");
		return -EIO;
	}
	/* success*/
	return 0;
}

/**************************************************************************/
/*!
    @brief  The desired ADC gain setter
    @param  gain Desired gain: NAU7802_GAIN_1, NAU7802_GAIN_2, NAU7802_GAIN_4,
    NAU7802_GAIN_8, NAU7802_GAIN_16, NAU7802_GAIN_32, NAU7802_GAIN_64,
    or NAU7802_GAIN_128
    @returns 0 if seccess
*/
/**************************************************************************/
static int nau7802_setGain(const struct nau7802_loadcell_config *config, NAU7802_Gain gain)
{
	/* Write the LDO voltage to CTRL1 register*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_CTRL1, NAU7802_MASK_CTRL1_GAINS, gain);
	if(ret != 0){
		LOG_ERR("Chip set GAINS bits failed");
		return -EIO;
	}
	/* success*/
	return 0;
}

/**************************************************************************/
/*!
    @brief  The desired conversion rate setter
    @param rate The desired rate: NAU7802_RATE_10SPS, NAU7802_RATE_20SPS,
    NAU7802_RATE_40SPS, NAU7802_RATE_80SPS, or NAU7802_RATE_320SPS
    @returns 0 if seccess
*/
/**************************************************************************/
static int nau7802_setRate(const struct nau7802_loadcell_config *config, NAU7802_SampleRate rate)
{
	/* Write the sample rate to CTRL2 register*/
	ret = i2c_update_byte_dt(config->bus, NAU7802_CTRL2, NAU7802_MASK_CTRL2_CRS, rate);
	if(ret != 0){
		LOG_ERR("Chip set CRS bits failed");
		return -EIO;
	}
	/* success*/
	return 0;
}


/* Sensor API function implementation*/
static int nau7802_loadcell_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct nau7802_loadcell_data *data = dev->data;
	const struct nau7802_loadcell_config *config = dev->config;
	uint8_t out[3];

	if ((enum sensor_channel_nuvoton_nau7802_loadcell)chan == SENSOR_CHAN_FORCE) {
		if (i2c_burst_read_dt(&config->bus, NAU7802_ADCO_B2,
			      out, 3) < 0) {
            LOG_DBG("Failed to read sample");
            return -EIO;
	    }
	} else {
		LOG_ERR("Invalid channel");
		return -ENOTSUP;
	}

    /* Reconstruct the 24-bit output data*/
	data->sample = (int32_t)((uint32_t)(out[0]) |
				     ((uint32_t)(out[1]) << 8) |
				     ((uint32_t)(out[2]) << 16));


	return 0;
}

static int nau7802_loadcell_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct nau7802_loadcell_data *data = dev->data;
    struct nau7802_loadcell_config *config = dev->config;
	float uval;
    double integer, fraction;

	if (chan != SENSOR_CHAN_FORCE) {
		return -ENOTSUP;
	}

    /* convert the ADC value to force value */
	uval = (float)(data->sample - data.zero_offset) * config->calibration_factor;
    fraction = modf(uval, &integer)
	val->val1 = (int32_t)integer;
	val->val2 = (int32_t)fraction;

	return 0;
}

/* Define API structure*/
static const struct sensor_driver_api nau7802_loadcell_api = {
	.sample_fetch = nau7802_loadcell_sample_fetch,
	.channel_get = nau7802_loadcell_channel_get,
};

/* Init function*/
static int nau7802_loadcell_init(const struct device *dev)
{
    const struct nau7802_loadcell_config * const config = dev->config;
	int ret;

	/* Check if the i2c bus is ready*/
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	/* Reset the IC*/
	ret = nau7802_reset(config);
	if(ret != 0){
		LOG_ERR("Reset process failed");
		return -EIO;
	}

	/* Power on the IC*/
	ret = nau7802_enable(config, true);
	if(ret != 0){
		LOG_ERR("Enable process failed");
		return -EIO;
	}

	/* Check and assert the IC revision ID*/
	// Do I need to do this?

	/* Set the LDO*/
	ret = nau7802_setLDO(NAU7802_3V0);
	if(ret != 0){
		LOG_ERR("SetLDO process failed");
		return -EIO;
	}
	/* Configure the PGA gain*/
	swich(config->gain){
		case 1:
		break;

		vase
	}

	/* Configure the output data rate*/
	swich(config->conversions_per_second){
		case 10:
		break;

		vase
	}

	/* Disable ADC chopper clock*/

	/* Use low ESR caps*/

	/* PGA stabilizer cap on output*/

}

/* Use the Instance-based APIs*/
#define CREATE_NAU7802_LOADCELL_INST(inst)								  \
	static struct nau7802_loadcell_data nau7802_loadcell_data_##inst;				  \
	static const struct nau7802_loadcell_config nau7802_loadcell_config_##inst = {			  \
		.bus = I2C_DT_SPEC_INST_GET(inst),					  \
		.conversions_per_second = DT_INST_ENUM_IDX(inst, conversions_per_second), \
		.gain = DT_INST_ENUM_IDX(inst, gain),			  \
		.calibration_factor = DT_INST_PROP(inst, calibration_factor),			  \
	};										  \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, nau7802_loadcell_init, NULL, &nau7802_loadcell_data_##inst,	  \
			      &nau7802_loadcell_config_##inst, POST_KERNEL,			  \
			      CONFIG_SENSOR_INIT_PRIORITY, &nau7802_loadcell_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NAU7802_LOADCELL_INST)