/* as7341.c - Driver for AMS AS7341 spectral sensor */

#include <kernel.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <device.h>
#include <sys/util.h>
#include <kernel.h>
#

#include <logging/log.h>

#include <drivers/sensor/as7341.h>

#define DT_DRV_COMPAT ams_as7341

#define AS7341_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#if !AS7341_BUS_I2C
#error "Failure: AS7341 not configured as okay status on I2C bus"
#endif

LOG_MODULE_REGISTER(AS7341, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "AS7341 driver enabled without any devices"
#endif

int as7341_attr_set(const struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val)
{
	return -ENOTSUP;
}

int as7341_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	struct as7341_data *data = to_data(dev);
	int32_t rc;
	if((rc = as7341_setup_interrupt(data, false)) < 0) {
		LOG_ERR("Failed to setup interrupt as %d with %d", (int) false, (int) rc);
		return rc;
	}
	if(trig == NULL || handler == NULL) {
		LOG_WRN("Disabling interrupts due to NULL argument(s)");
		return 0;
	}

	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
		if (trig->chan == SENSOR_CHAN_ALL) {
			data->trigger_handler = handler;
			data->trigger = *trig;
			// if (i2c_reg_update_byte(data->i2c,
			// 			config->i2c_address,
			// 			AS7341_ENABLE_REG,
			// 			AS7341_ENABLE_PIEN,
			// 			AS7341_ENABLE_PIEN)) {
			// 	return -EIO;
			// }
		} else {
			rc = -ENOTSUP;
			goto done;
		}
		break;
	default:
		LOG_ERR("Unsupported sensor trigger");
		rc = -ENOTSUP;
		goto done;
	}

	rc = 0;

done:

	LOG_INF("Enabling interrupts for trigger");
	int rc1;
	if((rc1 = as7341_setup_interrupt(data, true)) < 0) {
		LOG_ERR("Failed to setup interrupt as %d with %d", (int) true, (int) rc1);
		return rc1;
	}
	if ((rc1 = gpio_pin_get(data->gpio, data->gpio_pin)) > 0) {
		LOG_DBG("Interrupt pin already active, submitting trigger handler work");
		k_work_submit(&data->work);
	} else if(rc1 < 0) {
		LOG_ERR("Failed to check interrupt pin with %d", rc1);
		return rc1;
	}

	return rc;
}

static void as7341_handle_cb(struct as7341_data* );

void as7341_work_cb(struct k_work *work)
{
	struct as7341_data *data = CONTAINER_OF(work,
						  struct as7341_data,
						  work);
	const struct device *dev = data->dev;

	if (data->trigger_handler != NULL) {
		LOG_DBG("Firing user registered work handler");
		data->trigger_handler(dev, &data->trigger);
	} else {
		LOG_WRN("Fired work handler without user registered handler");
	}

	int32_t rc;
	if((rc = gpio_pin_get(data->gpio, data->gpio_pin)) > 0) {
		LOG_WRN("Interrupt ready again before re-enabling trigger");
		as7341_handle_cb(data);
	} else {
		if(rc < 0) {
			LOG_ERR("Failed to check interrupt pin with %d", rc);
		}
		LOG_DBG("Re-enabling interrupts for trigger");
		if((rc = as7341_setup_interrupt(data, true)) < 0) {
			LOG_ERR("Failed to setup interrupt as %d with %d", (int) true, (int) rc);
		}
	}
}

static void as7341_handle_cb(struct as7341_data* drv_data) {
	int32_t rc;
	LOG_DBG("Disabling interrupts for trigger to handle current interrupt");
	if((rc = as7341_setup_interrupt(drv_data, false)) < 0) {
		LOG_ERR("Failed to setup interrupt as %d with %d", (int) false, (int) rc);
	}
	if(k_work_pending(&drv_data->work)) {
		LOG_ERR("Work is still pending");
		if((rc = as7341_setup_interrupt(drv_data, true)) < 0) {
			LOG_ERR("Failed to setup interrupt as %d with %d", (int) true, (int) rc);
		}
		return;
	}
	k_work_init(&drv_data->work, as7341_work_cb);
	k_work_submit(&drv_data->work);
}

static void as7341_gpio_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins) {
	struct as7341_data *drv_data =
		CONTAINER_OF(cb, struct as7341_data, gpio_cb);

	as7341_handle_cb(drv_data);
}

static int as7341_sample_fetch(const struct device* dev, enum sensor_channel chan) {
	// const struct as7341_config * config = dev->config;
	struct as7341_data* data = dev->data;

	if(chan != SENSOR_CHAN_ALL) {
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	return as7341_setup_interrupt(data, true);
}

static int as7341_channel_get(const struct device* dev, enum sensor_channel chan, struct sensor_value * val) {
	return -ENOTSUP;
}

static int as7341_reg_read_i2c(const struct device *dev, uint8_t start, uint8_t *buf, int size) {
	return i2c_burst_read(to_data(dev)->bus, to_config(dev)->i2c_addr, start, buf, size);
}

static int as7341_reg_write_i2c(const struct device *dev, uint8_t start, uint8_t *buf, int size) {
	int rc;
	if((rc = i2c_burst_write(to_data(dev)->bus, to_config(dev)->i2c_addr, start, buf, size)) < 0) {
		LOG_ERR("Write failed for I2C addr %d, Reg %d with value %p of len %d", (int)to_config(dev)->i2c_addr,(int)start, buf, size);
		return rc;
	}
	uint8_t rval[128];
	if(size > 120) {
		LOG_ERR("Can't check write success");
		return 0;
	}
	if((rc = i2c_burst_read(to_data(dev)->bus, to_config(dev)->i2c_addr, start, rval, size)) < 0) {
		LOG_ERR("Read-after-Write failed for I2C addr %d, Reg %d with value %p of len %d", (int)to_config(dev)->i2c_addr,(int)start, buf, size);
		return rc;
	}
	for(size_t i = 0; i < size; i++) {
		if(rval[i] != buf[i]) {
			LOG_ERR("At i=%d, read %d after wrote %d", (int) i, (int) rval[i], (int) buf[i]);
		}
	}
	return 0;
}

static const struct as7341_reg_io as7341_reg_io_i2c = {
	.read = as7341_reg_read_i2c,
	.write = as7341_reg_write_i2c,
};

static const struct sensor_driver_api as7341_api_funcs = {
	.sample_fetch = as7341_sample_fetch,
	.channel_get = as7341_channel_get,
	.attr_set = as7341_attr_set,
	.trigger_set = as7341_trigger_set,
};

static int as7341_chip_init(const struct device *dev)
{
	// Chip says not to talk to it while it is initializing
	/*
	 * INT_BUSY
	 * Indicates that the device is initializing.
	 * This bit will remain 1 for about 300μs after power on.
	 * Do not interact with the device until initialization is complete
	 */
	k_sleep(K_MSEC(1));

	LOG_DBG("Checking chip part identification number");
	int rc;
	uint8_t id;
	if((rc = as7341_reg_read_i2c(dev, AS7341_REG_ID, &id, 1)) < 0) {
		LOG_ERR("Failed to read chip part identification number with %d", rc);
		return -rc;
	}

	id = (id & AS7341_ID_MASK) >> AS7341_ID_SHIFT;
	if(id != AS7341_DEFAULT_ID) {
		LOG_ERR("The part identification number %d did not match default %d", (int) id, (int) AS7341_DEFAULT_ID);
		return -ENOTSUP;
	}

	LOG_DBG("Part number %d matches default", (int) AS7341_DEFAULT_ID);

	return 0;
}

int as7341_init(const struct device *dev)
{
	const char *name = dev->name;
	struct as7341_data *data = to_data(dev);
	const struct as7341_config *config = to_config(dev);
	int rc;

	LOG_DBG("Initializing %s", name);
	data->dev = dev;

	data->bus = device_get_binding(config->bus_label);
	if (!data->bus) {
		LOG_DBG("bus \"%s\" not found", config->bus_label);
		rc = -EINVAL;
		goto done;
	}

	rc = as7341_chip_init(dev);
	if (rc < 0) {
		rc = -EINVAL;
		goto done;
	}

	data->gpio = device_get_binding(config->gpio_label);
	if(data->gpio == NULL) {
		LOG_ERR("gpio \"%s\" not found", config->gpio_label);
		rc = -EIO;
		goto done;
	}

	data->gpio_pin = config->gpio_pin;
	gpio_pin_configure(data->gpio, config->gpio_pin, GPIO_INPUT | config->gpio_flags);
	gpio_init_callback(&data->gpio_cb, as7341_gpio_callback, BIT(config->gpio_pin));
	if (gpio_add_callback(data->gpio, &data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		rc = -EIO;
		goto done;
	}

	if((rc = as7341_setup_interrupt(data, true)) < 0) {
		LOG_ERR("Failed to setup interrupt as %d with %d", (int) true, (int) rc);
		goto done;
	}

	if ((rc = gpio_pin_get(data->gpio, data->gpio_pin)) > 0) {
		LOG_DBG("Interrupt pin is already set after init");
		// as7341_handle_cb(data);
	} else if(rc < 0) {
		LOG_ERR("Failed to check interrupt pin on init");
	}

	rc = 0;

done:
	if (rc == 0) {
		LOG_DBG("%s init done", name);
	} else {
		LOG_ERR("%s init failed", name);
	}
	return rc;
}

/*
 * Device creation macros, defines each device
 */

#define AS7341_DEVICE_INIT(inst)					\
	DEVICE_AND_API_INIT(as7341_##inst, \
		DT_INST_LABEL(inst), \
		as7341_init, \
		&as7341_data_##inst, \
		&as7341_config_##inst, \
		POST_KERNEL, \
		CONFIG_SENSOR_INIT_PRIORITY, \
		&as7341_api_funcs); \

#define AS7341_CONFIG_I2C(inst)						\
	{								\
		.bus_label = DT_INST_BUS_LABEL(inst),			\
		.reg_io = &as7341_reg_io_i2c,				\
		.i2c_addr = DT_INST_REG_ADDR(inst), \
		.gpio_label = DT_INST_GPIO_LABEL(inst, int_gpios), \
		.gpio_pin = DT_INST_GPIO_PIN(inst, int_gpios), \
		.gpio_flags = DT_INST_GPIO_PIN(inst, int_gpios), \
	}

#define AS7341_DEFINE_I2C(inst)						\
	static struct as7341_data as7341_data_##inst;			\
	static const struct as7341_config as7341_config_##inst =	\
		AS7341_CONFIG_I2C(inst);				\
	AS7341_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(AS7341_DEFINE_I2C)
