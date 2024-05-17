# /*
 * Copyright (c) 2024, Tin Chiang
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/gpio.h>
#include "nau7802_loadcell.h"

#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER
/* Declare the module to logging submodule*/
LOG_MODULE_DECLARE(NAU7802_LOADCELL, LOG_LEVEL_DBG);
// LOG_MODULE_DECLARE(NAU7802_LOADCELL, CONFIG_I2C_LOG_LEVEL);

/**
 * nau7802_loadcell_handle_interrupt - send the sensor device to user defined handler function
 */
static void nau7802_loadcell_handle_interrupt(const void *arg)
{
	const struct device *nau7802_loadcell = (const struct device *)arg;
	struct nau7802_loadcell_data *data = nau7802_loadcell->data;

	if (data->handler_drdy != NULL) {
		data->handler_drdy(nau7802_loadcell, data->trig_drdy);
	}
}



/**
 * nau7802_loadcell_gpio_callback - the drdy interrupt callback function
 */
static void nau7802_loadcell_gpio_callback(const struct device *port,
				 struct gpio_callback *cb,
				 uint32_t pin)
{
	struct nau7802_loadcell_data *data = CONTAINER_OF(cb,
						struct nau7802_loadcell_data,
						gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pin);

#if defined(CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD)
	k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->work);
#elif defined(CONFIG_NAU7802_LOADCELL_TRIGGER_DIRECT)
	nau7802_loadcell_handle_interrupt(data->dev);
#endif
}


#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD
static void nau7802_loadcell_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *nau7802_loadcell = (const struct device *)p1;
    struct nau7802_loadcell_data *data = nau7802_loadcell->data;


	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		nau7802_loadcell_handle_interrupt(nau7802_loadcell);
	}
}
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD */

#ifdef CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD
static void nau7802_loadcell_work_cb(struct k_work *work)
{
	struct nau7802_loadcell_data *nau7802_loadcell =
		CONTAINER_OF(work, struct nau7802_loadcell_data, work);

	nau7802_loadcell_handle_interrupt(nau7802_loadcell->dev);
}
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD */


/**
 * nau7802_loadcell_trigger_set - link external trigger to event data ready
 */
int nau7802_loadcell_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler)
{
	struct nau7802_loadcell_data *data = dev->data;

    /* trig and handler are user-defined */
	/* Make sure the trigger type is correct*/
	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	/* Put them into sensor device struct*/
	data->trig_drdy = trig;
	data->handler_drdy = handler;

	/* kick start the read process*/
	nau7802_loadcell_handle_interrupt(dev);

	/* success*/
	LOG_DBG("Trigger set process done");
	return 0;
}


/**
 * nau7802_loadcell_trigger_set - Configure the gpio pin to be interrpt pin
 */
int nau7802_loadcell_init_interrupt(const struct device *dev)
{
	const struct nau7802_loadcell_config *config = dev->config;
	struct nau7802_loadcell_data *data = dev->data;

    int ret;

    /* Check if the gpio port exists */
    if(config->drdy_gpios.port == NULL){
        LOG_ERR("gpio_drdy not defined in DT");
        return ENODEV;
    }

    /* Check if the gpio port is ready */
    if (!gpio_is_ready_dt(&config->drdy_gpios)) {
		LOG_ERR("device %s is not ready", config->drdy_gpios.port->name);
		return -ENODEV;
	}

    /* set up data ready gpio as input */
    ret = gpio_pin_configure_dt(&config->drdy_gpios, GPIO_INPUT|GPIO_ACTIVE_HIGH);
    if(ret != 0){
        LOG_ERR("ret:%d, Setting %s to input fail", ret, config->drdy_gpios.port->name);
        return ret;
    }

    /* Register the handler to GPIO pin*/
    gpio_init_callback(&data->gpio_cb, nau7802_loadcell_gpio_callback, BIT(config->drdy_gpios.pin));
    ret = gpio_add_callback(config->drdy_gpios.port, &data->gpio_cb);
    if(ret != 0){
        LOG_ERR("ret:%d, setting up callback fail", ret);
        return ret;
    }

    /* setup data ready gpio interrupt */
    ret = gpio_pin_interrupt_configure_dt(&config->drdy_gpios, GPIO_INT_EDGE_TO_ACTIVE);
    if(ret != 0){
        LOG_ERR("ret:%d, Setting %s to interrupt pin fail", ret, config->drdy_gpios.port->name);
    }

    /* Depend on the trigger mode, enable thread*/
#if defined(CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
			CONFIG_NAU7802_LOADCELL_THREAD_STACK_SIZE,
			nau7802_loadcell_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(CONFIG_NAU7802_LOADCELL_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD)
	data->work.handler = nau7802_loadcell_work_cb;
#endif /* CONFIG_NAU7802_LOADCELL_TRIGGER_OWN_THREAD */

#if defined(CONFIG_NAU7802_LOADCELL_TRIGGER_GLOBAL_THREAD) || \
	defined(CONFIG_NAU7802_LOADCELL_TRIGGER_DIRECT)
	data->dev = dev;
#endif
	
    /* Success*/
    LOG_DBG("Trigger init success");
	return 0;
}

#endif


