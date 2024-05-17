# Out of tree Adafruit NAU7802 loadcell driver module for Zephyr RTOS
*NAU7802 is a precision 24-bit analog-to-digital converter (ADC) designed for weigh scale applications.*

## Supported Zephyr versions
* 3.7.0 (May 2024)
## Usage
### Module installation
Add this project to your `west.yml` manifest:
```yaml
 # out of tree device driver
- name: NAU7802_LOADCELL
    path: modules/nau7802_loadcell
    revision: refs/tags/zephyr-v3.7.0
    url: https://github.com/nobodyguy/nau7802_loadcell_zephyr_driver
```

So your projects should look something like this:
```yaml
manifest:
  projects:
    - name: zephyr
      url: https://github.com/zephyrproject-rtos/zephyr
      revision: refs/tags/zephyr-v3.2.0
      import: true
# out of tree device driver
    - name: NAU7802_LOADCELL
      path: modules/nau7802_loadcell
      revision: refs/tags/zephyr-v3.7.0
      url: https://github.com/nobodyguy/nau7802_loadcell_zephyr_driver
```

This will import the driver and allow you to use it in your code.

Additionally make sure that you run `west update` when you've added this entry to your `west.yml`.

### Driver configuration
Enable sensor driver subsystem and NAU7802 driver by adding these entries to your `prj.conf`:
```ini
CONFIG_SENSOR=y
CONFIG_NAU7802_LOADCELL=y
# Interrut triggerred configurations
CONFIG_NAU7802_LOADCELL_TRIGGER_NONE=y
CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD=y
CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD=y
CONFIG_NAU7802_LOADCELL_TRIGGER_DIRECT=y
```

Define NAU7802 in your board `.overlay` like this example:
```dts
/{	
    force_sens_1: nau7802_loadcell@2A {
        // label = "NAU7802_1";
        compatible = "nuvoton,nau7802_loadcell";
        reg = <0x2A>;
        drdy-gpios = <&gpiod 1 GPIO_ACTIVE_HIGH>;
        conversions-per-second = <7>; // 320 SPS
        gain = <7>; // 128
    };
};
```

### Driver usage
```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include <sensor/hx711/hx711.h>
#include <stddef.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

const struct device *hx711_dev;

void set_rate(enum hx711_rate rate)
{
	static struct sensor_value rate_val;

	rate_val.val1 = rate;
	sensor_attr_set(hx711_dev,
			HX711_SENSOR_CHAN_WEIGHT,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&rate_val);
}

void measure(void)
{
	static struct sensor_value weight;
	int ret;

	ret = sensor_sample_fetch(hx711_dev);
	if (ret != 0) {
		LOG_ERR("Cannot take measurement: %d", ret);
	} else {
		sensor_channel_get(hx711_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
		LOG_INF("Weight: %d.%06d grams", weight.val1, weight.val2);
	}
}

void main(void)
{
	int calibration_weight = 100; // grams
	hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);
	__ASSERT(hx711_dev == NULL, "Failed to get device binding");

	LOG_INF("Device is %p, name is %s", hx711_dev, hx711_dev->name);
	LOG_INF("Calculating offset...");
	avia_hx711_tare(hx711_dev, 5);

	LOG_INF("Waiting for known weight of %d grams...",
		calibration_weight);

	for (int i = 5; i >= 0; i--) {
		LOG_INF(" %d..", i);
		k_msleep(1000);
	}

	LOG_INF("Calculating slope...");
	avia_hx711_calibrate(hx711_dev, calibration_weight, 5);

	while (true) {
		k_msleep(1000);
		LOG_INF("== Test measure ==");
		LOG_INF("= Setting sampling rate to 10Hz.");
		set_rate(HX711_RATE_10HZ);
		measure();

		k_msleep(1000);
		LOG_INF("= Setting sampling rate to 80Hz.");
		set_rate(HX711_RATE_80HZ);
		measure();
	}
}
```
Relevant `prj.conf`:
```ini
CONFIG_SENSOR=y
CONFIG_HX711=y
CONFIG_LOG=y
```