#ifndef ZEPHYR_DRIVERS_SENSOR_AS7341_AS7341_H_
#define ZEPHYR_DRIVERS_SENSOR_AS7341_AS7341_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <device.h>
#include <drivers/sensor.h>

#if defined CONFIG_AS7341_TEST_ON
#define AS7341_MODE AS7341_MODE_TEST
#elif defined CONFIG_AS7341_TEST_OFF
#define AS7341_MODE AS7341_MODE_NORMAL
#endif

#define AS7341_REG_ID             0x92
#define AS7341_DEFAULT_ID         0b00001001
#define AS7341_ID_MASK            0b11111100
#define AS7341_ID_SHIFT           2

struct as7341_data {
	const struct device *dev;
	const struct device *bus;
	const struct device *gpio;
	struct gpio_callback gpio_cb;
	struct k_work work;
	sensor_trigger_handler_t trigger_handler;
	struct sensor_trigger trigger;
	uint8_t gpio_pin;
	bool interrupt_enabled;
};

struct as7341_config {
	const char *bus_label;
	const struct as7341_reg_io *reg_io;
	uint16_t i2c_addr;
	const char* gpio_label;
	uint8_t gpio_pin;
	uint32_t gpio_flags;
};

static inline struct as7341_data *to_data(const struct device *dev) { return (struct as7341_data*) dev->data; }
static inline const struct as7341_config *to_config(const struct device *dev) { return (const struct as7341_config*) dev->config; }

typedef int (*as7341_reg_read_fn)(const struct device *dev, uint8_t start, uint8_t *buf, int size);
typedef int (*as7341_reg_write_fn)(const struct device *dev, uint8_t start, uint8_t *buf, int size);

struct as7341_reg_io {
	as7341_reg_read_fn read;
	as7341_reg_write_fn write;
};

static inline int as7341_setup_interrupt(struct as7341_data* drv_data, bool enable) {
	drv_data->interrupt_enabled = enable;
	uint32_t flags = enable ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
	return gpio_pin_interrupt_configure(drv_data->gpio, drv_data->gpio_pin, flags);
}

void as7341_work_cb(struct k_work* work);

int as7341_attr_set(const struct device* dev,
	enum sensor_channel chan,
	enum sensor_attribute attr,
	const struct sensor_value * val);

int as7341_trigger_set(const struct device* dev,
	const struct sensor_trigger* trig,
	sensor_trigger_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_AS7341_AS7341_H_ */
