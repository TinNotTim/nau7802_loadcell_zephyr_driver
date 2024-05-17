/*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_NAU7802_LOADCELL_NAU7802_LOADCELL_H_
#define ZEPHYR_DRIVERS_SENSOR_NAU7802_LOADCELL_NAU7802_LOADCELL_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

/* Use the Instance-based APIs*/
#define DT_DRV_COMPAT nuvoton_nau7802_loadcell

/**************************************************************************/
/*!
    @brief  NAU7802 register definition from Adafruit_NAU7802 library.
    @brief  https://github.com/adafruit/Adafruit_NAU7802/tree/master
*/
/**************************************************************************/

/** Default NAU7802 I2C address. */
#define NAU7802_I2CADDR_DEFAULT 0x2A ///< I2C address
#define NAU7802_PU_CTRL 0x00         ///< Power control register
#define NAU7802_MASK_PU_CTRL_AVDDS BIT(7)
#define NAU7802_SHIFT_PU_CTRL_AVDDS 7
#define NAU7802_MASK_PU_CTRL_PUR BIT(3)
#define NAU7802_SHIFT_PU_CTRL_PUR 3
#define NAU7802_MASK_PU_CTRL_PUA BIT(2)
#define NAU7802_SHIFT_PU_CTRL_PUA 2
#define NAU7802_MASK_PU_CTRL_PUD BIT(1)
#define NAU7802_SHIFT_PU_CTRL_PUD 1
#define NAU7802_MASK_PU_CTRL_RR BIT(0)
#define NAU7802_SHIFT_PU_CTRL_RR 0

#define NAU7802_CTRL1 0x01           ///< Control/config register #1
#define NAU7802_MASK_CTRL1_VLDO (BIT(5) | BIT(4) | BIT(3))
#define NAU7802_SHIFT_CTRL1_VLDO 3
#define NAU7802_MASK_CTRL1_GAINS (BIT(2) | BIT(1) | BIT(0))
#define NAU7802_SHIFT_CTRL1_GAINS 0

#define NAU7802_CTRL2 0x02           ///< Control/config register #2
#define NAU7802_MASK_CTRL2_CRS (BIT(6) | BIT(5) | BIT(4))
#define NAU7802_SHIFT_CTRL2_CRS 4

#define NAU7802_ADCO_B2 0x12         ///< ADC ouput LSB

#define NAU7802_ADC 0x15             ///< ADC / chopper control
#define NAU7802_MASK_ADC_REG_CHPS (BIT(5) | BIT(4))
#define NAU7802_SHIFT_ADC_REG_CHPS 4

#define NAU7802_PGA 0x1B             ///< PGA control
#define NAU7802_MASK_PGA_LDOMODE BIT(6)
#define NAU7802_SHIFT_PGA_LDOMODE 6

#define NAU7802_POWER 0x1C           ///< power control
#define NAU7802_MASK_POWER_PGA_CAP_EN BIT(7)
#define NAU7802_SHIFT_POWER_PGA_CAP_EN 7

#define NAU7802_REVISION_ID 0x1F     ///< Chip revision ID

/*! The possible LDO voltages */
typedef enum _ldovoltages {
    NAU7802_4V5,
    NAU7802_4V2,
    NAU7802_3V9,
    NAU7802_3V6,
    NAU7802_3V3,
    NAU7802_3V0,
    NAU7802_2V7,
    NAU7802_2V4,
    NAU7802_EXTERNAL,
} NAU7802_LDOVoltage;

/*! The possible gains */
typedef enum _gains {
    NAU7802_GAIN_1,
    NAU7802_GAIN_2,
    NAU7802_GAIN_4,
    NAU7802_GAIN_8,
    NAU7802_GAIN_16,
    NAU7802_GAIN_32,
    NAU7802_GAIN_64,
    NAU7802_GAIN_128,
} NAU7802_Gain;
/* Array to map index to the enum values*/
static const NAU7802_Gain GainMap[] = {
    NAU7802_GAIN_1,  // Index 0
    NAU7802_GAIN_2,  // Index 1
    NAU7802_GAIN_4,  // Index 2
    NAU7802_GAIN_8,  // Index 3
    NAU7802_GAIN_16, // Index 4
    NAU7802_GAIN_32, // Index 5
    NAU7802_GAIN_64, // Index 6
    NAU7802_GAIN_128, // Index 7
};

/*! The possible sample rates */
typedef enum _sample_rates {
    NAU7802_RATE_10SPS = 0,
    NAU7802_RATE_20SPS = 1,
    NAU7802_RATE_40SPS = 2,
    NAU7802_RATE_80SPS = 3,
    NAU7802_RATE_320SPS = 7,
} NAU7802_SampleRate;

/* Array to map index to the enum values*/
static const NAU7802_SampleRate sampleRateMap[] = {
    NAU7802_RATE_10SPS,  // Index 0
    NAU7802_RATE_20SPS,  // Index 1
    NAU7802_RATE_40SPS,  // Index 2
    NAU7802_RATE_80SPS,  // Index 3
    NAU7802_RATE_320SPS, // Index 4
};

/*! The possible calibration modes */
typedef enum _calib_mode {
    NAU7802_CALMOD_INTERNAL = 0,
    NAU7802_CALMOD_OFFSET = 2,
    NAU7802_CALMOD_GAIN = 3,
} NAU7802_Calibration;

/* Define a channel for force reading*/
enum sensor_channel_nuvoton_nau7802_loadcell {
	/* Force reading in Newton*/
	SENSOR_CHAN_FORCE = SENSOR_CHAN_PRIV_START,
};

/* Define data (RAM) and configuration (ROM) structures: */
struct nau7802_loadcell_data {
    /* per-device values to store in RAM */
    float32_t zero_offset;
    float32_t calibration_factor;
    int32_t sample;

#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER
    struct gpio_callback gpio_cb;
    sensor_trigger_handler_t handler_drdy;
    const struct sensor_trigger *trig_drdy;

#if defined(CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_NAU7802_LOADCELL_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem gpio_sem;
#elif defined(CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER_MODE */
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER */

#if defined(CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD) || \
defined(CONFIG_NAU7802_LOADCELL_TRIGGER_DIRECT)
    const struct device *dev;
#endif
};
struct nau7802_loadcell_config {
    /* other configuration to store in ROM */
    const struct i2c_dt_spec bus;
	uint16_t conversions_per_second_idx;
	uint8_t gain_idx;

#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER
    const struct gpio_dt_spec drdy_gpios;
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER */
};

#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER
int nau7802_loadcell_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int nau7802_loadcell_init_interrupt(const struct device *dev);
#endif

#endif