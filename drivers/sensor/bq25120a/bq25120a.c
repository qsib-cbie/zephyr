// Butchered from TI forum: https://e2e.ti.com/support/power-management/f/196/t/495496
// * BQ25120 Battery Charger Driver
// * Copyright (C) 2015,  Texas Instruments Corporation

#include <init.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <kernel.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "bq25120a.h"

/* Device Tree Bus Inst */
#define DT_DRV_COMPAT ti_bq25120a
#define BQ25120A_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#if !BQ25120A_BUS_I2C
#error "Failure: BQ25120A not configured as okay on I2C bus"
#endif

/* Enable Logging from Driver */
LOG_MODULE_REGISTER(BQ25120A, CONFIG_SENSOR_LOG_LEVEL);

/* I2C Bus Interaction Interface */
static inline struct bq25120a_data_t *to_data(const struct device *dev) { return (struct bq25120a_data_t*) dev->data; }
static inline const struct bq25120a_config_t *to_config(const struct device *dev) { return (const struct bq25120a_config_t*) dev->config; }


static int bq25120a_reg_write_i2c(const struct device *dev, uint8_t start, uint8_t *buf, int size) {
	return i2c_burst_write(to_data(dev)->bus, to_config(dev)->i2c_addr, start, buf, size);
}

static int bq25120a_reg_read_i2c(const struct device *dev, uint8_t start, uint8_t *buf, int size) {
	return i2c_burst_read(to_data(dev)->bus, to_config(dev)->i2c_addr, start, buf, size);
}

int bq25120a_transceive(const struct device *dev, uint8_t reg, bool write, void *buf, size_t length) {
	if(write) {
		return bq25120a_reg_write_i2c(dev, reg, buf, length);
	} else {
		return bq25120a_reg_read_i2c(dev, reg, buf, length);
	}
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

/**
 * bq25120_update_ps_status - refreshes the power source status
 * @bq25120: pointer to bq25120 charger instance
 *
 * Function checks whether any power source is connected to the charger and
 * updates internal state accordingly. If there is a change to previous state
 * function returns %1, otherwise %0 and negative errno in case of errror.
 */
static int bq25120_update_ps_status(struct bq25120_charger *bq25120)
{
	bool usb = false;
	bool dc = false;
	unsigned int val;
	int ret;
	int vinDpm; 

	ret = bq25120a_transceive(bq25120, BQ25120_BATT_VOLT_MONITOR_REG, false, &val, 1);
	if (ret < 0)
		return ret;

	/*
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if (bq25120->pdata->use_mains)
		dc = !( (val >> 7) BQ25120_VINDPM_OFF) );

	ret = bq25120->mains_online != dc;

	bq25120->mains_online = dc;
	
	return ret;
}

enum power_supply_property bq25120_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};


/**
 * bq25120_charging_status - returns status of charging
 * @bq25120: pointer to bq25120 charger instance
 *
 * Function returns charging status.
 * %0 means ready, %1 charging is in progress,
 * %2charging done and %3 fault.
 */
static int bq25120_charging_status(struct bq25120_charger *bq25120)
{
	unsigned int val;
	int ret;

	if (!bq25120->charger_online)
		return 0;

	ret = bq25120a_transceive(bq25120, BQ25120_STATUS_SHIPMODE_REG, false, &val, 1);
	if (ret < 0)
		return 0;

	return  ((val & BQ25120_STAT_MASK) >> BQ25120_STAT_SHIFT);
}

static int bq25120_charging_type(struct bq25120_charger *bq25120)
{
	unsigned int val;
	int ret;

	if (!bq25120->charger_online)
		return 0;

	ret = bq25120a_transceive(bq25120, BQ25120_STATUS_SHIPMODE_REG, false, &val, 1);
	if (ret < 0)
		return 0;

	return  ((val >> BQ25120_STAT_SHIFT) & BQ25120_STAT_MASK);
}

static int bq25120_timer_suspend(struct bq25120_charger *bq25120)
{
	int ret = 0;

	if (bq25120->pdata->enable_control != BQ25120_CE) {
		dev_dbg(bq25120->dev, "vsys enable/disable in SW disabled\n");
		return 0;
	}

	ret = regmap_update_bits(bq25120->regmap, BQ25120_VIN_DPM_TIMER_REG, BQ25120_TMR, BQ25120_TMR_DISABLE);
	if (!ret)
		dev_err(bq25120->dev, "failed to suspend timer\n");

	return ret;
}

static int bq25120_timer_reset(struct bq25120_charger *bq25120)
{
	int ret;
	unsigned int val;

	/* Reset with previous time value, when fault occur */
	ret = bq25120a_transceive(bq25120, BQ25120_VIN_DPM_TIMER_REG, false, &val, 1);

	val = (val & BQ25120_TMR) >> BQ25120_TMR_SHIFT;

	ret = regmap_update_bits(bq25120->regmap, BQ25120_VIN_DPM_TIMER_REG, BQ25120_TMR, val);
	if (!ret)
		dev_err(bq25120->dev, "failed to reset timer\n");

	return ret;

}

static int bq25120_charging_set(struct bq25120_charger *bq25120, bool enable)
{
	int ret = 0;

	if (bq25120->charging_enabled != enable) {
		ret = regmap_update_bits(bq25120->regmap, BQ25120_FASTCHARGE_CTL_REG, BQ25120_CE, enable ? 0 : 1);
		if (!ret)
			bq25120->charging_enabled = enable;
	}
	return ret;
}

static inline int bq25120_charging_enable(struct bq25120_charger *bq25120)
{
	return bq25120_charging_set(bq25120, true);
}

static inline int bq25120_charging_disable(struct bq25120_charger *bq25120)
{
	return bq25120_charging_set(bq25120, false);
}

static int bq25120_start_stop_charging(struct bq25120_charger *bq25120)
{
	int ret;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (bq25120->charger_online) {
		ret = bq25120_charging_enable(bq25120);
		if (ret < 0)
			dev_err(bq25120->dev, "failed to enable charging\n");
	} else {
		ret = bq25120_charging_disable(bq25120);
		if (ret < 0)
			dev_err(bq25120->dev, "failed to disable charging\n");
	}

	return ret;
}

static int bq25120_set_charge_current(struct bq25120_charger *bq25120)
{
	int ret;
	unsigned int max_chrg_curr = bq25120->pdata->max_charge_current;
	unsigned int term_curr = bq25120->pdata->termination_current;

	if (max_chrg_curr) {
		if (max_chrg_curr > 300) {
			dev_warn(bq25120->dev, "Platform max charge current %d. \
				 Set to 300mA for bq25120\n", max_chrg_curr);
		}

		ret = current_to_hw(bq25120a_cc_tbl, ARRAY_SIZE(bq25120a_cc_tbl), max_chrg_curr);

		if (ret < 0)
			return ret;

		ret = regmap_update_bits(bq25120->regmap, BQ25120_CHARGETERM_I2CADDR_REG, BQ25120_IPRETERM, ret);

		if (ret < 0)
			return ret;
	}

	if (term_curr) {
		if (term_curr < 5) {
			dev_warn(bq25120->dev, "Platform termination current %d. \
				 Set to 5mA for bq25120\n", term_curr);
		}

		ret = current_to_hw(tc_tbl, ARRAY_SIZE(tc_tbl), term_curr);
		if (ret < 0)
			return ret;

		ret = regmap_update_bits(bq25120->regmap,
					 BQ25120_CHARGETERM_I2CADDR_REG,
					 BQ25120_IPRETERM, ret);

		if (ret < 0)
			return ret;
	}
	return 0;
}


static int bq25120_set_current_limits(struct bq25120_charger *bq25120)
{
	int ret;

	if (bq25120->pdata->mains_current_limit) {
		ret = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    bq25120->pdata->mains_current_limit);
		if (ret < 0)
			return ret;

		if (ret > 400)
			dev_warn(bq25120->dev, "Invalid mains input current limit\n"); 
		
		else {
			ret = regmap_update_bits(bq25120->regmap, BQ25120_ILIMIT_UVLO_CTL_REG, BQ25120_INLIM, ret);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static int bq25120_set_voltage_limits(struct bq25120_charger *bq25120)
{
	int ret;

	if (bq25120->pdata->max_charge_voltage) {
		ret = bq25120->pdata->max_charge_voltage;

		if (ret < 0)
			return ret;

		ret = regmap_update_bits(bq25120->regmap,
					 BQ25120_BATT_VOLTAGE_CTL_REG,
					 BQ25120_VBREG, ret);
		if (ret < 0)
			return ret;
	}

	return 0;
}


static int bq25120_hw_init(struct bq25120_charger *bq25120)
{
	int ret;

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */
	ret = bq25120_set_charge_current(bq25120);
	if (ret < 0)
		goto fail;

	ret = bq25120_set_current_limits(bq25120);
	if (ret < 0)
		goto fail;

	ret = bq25120_set_voltage_limits(bq25120);
	if (ret < 0)
		goto fail;


	ret = bq25120_update_ps_status(bq25120);
	if (ret < 0)
		goto fail;

	ret = bq25120_start_stop_charging(bq25120);

fail:
	return ret;
}

static irqreturn_t bq25120_interrupt(int irq, void *data)
{
	struct bq25120_charger *bq25120 = data;
	unsigned int irqstat;
	bool handled = false;
	int ret;

	ret = regmap_read(bq25120->regmap, BQ25120_FAULTS_FAULTMASKS_REG,
			  &irqstat);
	if (ret < 0) {
		dev_warn(bq25120->dev, "reading BQ25120_FAULTS_FAULTMASKS_REG failed\n");
		return IRQ_NONE;
	}

	irqstat &= 0xF0;  //Mask mask bits

	/*
	 * If we get a fault report the error. 
	 */
	switch (irqstat)  {
	case BQ25120_NORMAL:	/* Normal  - No fault */
		handled = true;
		break;
	case BQ25120_VIN_OV:	
		dev_info(bq25120->dev,"Charger Fault: Vin Over Voltage fault.\n");
		handled = false;
		break;
	case BQ25120_VIN_UV:
		dev_info(bq25120->dev,"Charger Fault: VIN Under Voltage fault.\n");
		handled = false;
		break;
	case BQ25120_BAT_UVLO:	
		dev_info(bq25120->dev,
			"Battery Fault: BAT_UVLO fault.\n");
		handled = false;
		break;
	case BQ25120_BAT_OCP:
		dev_info(bq25120->dev,
			"Battery Fault: BAT_OCP fault.\n");
		handled = false;
		break;
	
	default:
		dev_err(bq25120->dev,
			"Fault: unknown fault# %d - not handled. \n",
			 irqstat);
		handled = false;
		break;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static int bq25120_irq_set(struct bq25120_charger *bq25120, bool enable)
{
	int ret;

	/*
	* Enable/disable interrupts for:
	*/
	ret = regmap_update_bits(bq25120->regmap, BQ25120_TSCONTROL_STATUS_REG,
				 BQ25120_EN_INT, enable ? 1 : 0);

	if (ret < 0)
		return ret;

	return 0;
}

static inline int bq25120_irq_enable(struct bq25120_charger *bq25120)
{
	return bq25120_irq_set(bq25120, true);
}

static inline int bq25120_irq_disable(struct bq25120_charger *bq25120)
{
	return bq25120_irq_set(bq25120, false);
}

static int bq25120_irq_init(struct bq25120_charger *bq25120, int irq)
{
	int ret;

	if (irq <= 0) {
		dev_info(bq25120->dev, "invalid irq number: %d\n", irq);
		goto out;
	}

	
	regmap_update_bits(bq25120->regmap, BQ25120_TSCONTROL_STATUS_REG,
			   BQ25120_EN_INT, 1);

	ret = request_threaded_irq(irq, NULL, bq25120_interrupt,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "bq25120_irq", bq25120);
	if (ret)
		return ret;

	bq25120->irq = irq;
	bq25120_irq_enable(bq25120);
out:
	return 0;
}

static ssize_t bq25120_show_charger_fault(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	unsigned int val;
	int ret;
	struct bq25120_charger *bq25120 = dev_get_drvdata(dev);

	ret = regmap_read(bq25120->regmap, BQ25120_FAULTS_FAULTMASKS_REG, &val);
	if (ret < 0)
		return 0;

	val &= 0xC0; 
	switch (val) 
	{
	case BQ25120_VIN_OV:
	    return scnprintf(buf, PAGE_SIZE, "%s\n", "Charge VIN overvoltage fault.");
	    break;
	case BQ25120_VIN_UV:
	     return scnprintf(buf, PAGE_SIZE, "%s\n", "Charge VIN undervoltage fault.");
	     break; 
	}
	return 0; 
}	


static ssize_t bq25120_show_temp_fault(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	unsigned int val;
	int ret;
	struct bq25120_charger *bq25120 = dev_get_drvdata(dev);

	ret = regmap_read(bq25120->regmap, BQ25120_TSCONTROL_STATUS_REG, &val);
	if (ret < 0)
		return 0;

	val = (val >> BQ25120_TS_FAULT_SHIFT) & BQ25120_TS_FAULT_MASK;

	return scnprintf(buf, PAGE_SIZE, "%s\n", temp_status_desc[val]);
}

static DEVICE_ATTR(charger_fault, S_IRUSR, bq25120_show_charger_fault, NULL);
static DEVICE_ATTR(temp_fault, S_IRUSR, bq25120_show_temp_fault, NULL);

static struct attribute *bq25120_charger_attr[] = {
	&dev_attr_charger_fault.attr,
	&dev_attr_temp_fault.attr,
	NULL,
};

static const struct attribute_group bq25120_attr_group = {
	.attrs = bq25120_charger_attr,
};

/*
 * Returns the constant charge current programmed
 * into the charger in mA.
 */
static int get_const_charge_current(struct bq25120_charger *bq25120)
{
	int ret;
	unsigned int val, intval;

	if (!bq25120->charger_online)
		return -ENODATA;

	ret = regmap_read(bq25120->regmap, bq25120_CHARGE_CURR_TERM_REG, &val);
	if (ret < 0)
		return ret;

	val = (val & bq25120_CHARGE_CURR) >> bq25120_CHARGE_CURR_SHIFT;

	intval = 500 + val * 100;

	if (intval >= 3000)
		intval = 3000; /* set Max as 3A */

	dev_info(bq25120->dev, "const charge current is %d mA\n", intval);
	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in mV.
 */
static int get_const_charge_voltage(struct bq25120_charger *bq25120)
{
	int ret;
	unsigned int val;
	int intval;

	if (!bq25120->charger_online)
		return -ENODATA;

	ret = regmap_read(bq25120->regmap, bq25120_BATTERY_VOLTAGE_REG, &val);
	if (ret < 0)
		return ret;

	val = (val & bq25120_BATTERY_VOLTAGE) >> bq25120_BATTERY_VOLTAGE_SHIFT;

	intval = 3500 + val * 20;
	if (intval >= 4440)	/* set MAX as 4.4V */
		intval = 4440;

	dev_info(bq25120->dev, "const charge voltage is=%d mV\n", intval);
	return intval;
}

static int bq25120_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct bq25120_charger *bq25120 =
		container_of(psy, struct bq25120_charger, mains);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq25120->mains_online;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage(bq25120);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(bq25120);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}


static int bq25120_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct bq25120_charger *bq25120 =
			container_of(psy, struct bq25120_charger, battery);
	const struct bq25120_charger_platform_data *pdata = bq25120->pdata;
	int ret;

	ret = bq25120_update_ps_status(bq25120);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!bq25120->charger_online) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}

		switch (bq25120_charging_status(bq25120)) {
		case 0:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		case 1:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 2:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case 3:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN; /* FAULT */
			break;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!bq25120->charger_online)
			return -ENODATA;

		/* We handle charger mode and boost mode.  */
		switch (bq25120_charging_type(bq25120)) {
		case 0:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->battery_info.voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->battery_info.voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property bq25120_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int bq25120_power_supply_register(struct bq25120_charger *bq25120)
{
	static char *battery[] = { "bq25120-battery" };
	int ret;

	if (bq25120->pdata->use_mains) {
		bq25120->mains.name = "bq25120-mains";
		bq25120->mains.type = POWER_SUPPLY_TYPE_MAINS;
		bq25120->mains.get_property = bq25120_mains_get_property;
		bq25120->mains.properties = bq25120_charger_properties;
		bq25120->mains.num_properties =
					ARRAY_SIZE(bq25120_charger_properties);
		bq25120->mains.supplied_to = battery;
		bq25120->mains.num_supplicants = ARRAY_SIZE(battery);

		ret = power_supply_register(bq25120->dev, &bq25120->mains);
		if (ret < 0)
			return ret;
	}


	bq25120->battery.name = "bq25120-battery";
	bq25120->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	bq25120->battery.get_property = bq25120_battery_get_property;
	bq25120->battery.properties = bq25120_battery_properties;
	bq25120->battery.num_properties =
				ARRAY_SIZE(bq25120_battery_properties);

	ret = power_supply_register(bq25120->dev, &bq25120->battery);
	if (ret < 0) {
		if (bq25120->pdata->use_mains)
			power_supply_unregister(&bq25120->mains);
		return ret;
	}

	return 0;
}

static void bq25120_power_supply_unregister(struct bq25120_charger *bq25120)
{
	power_supply_unregister(&bq25120->battery);
	if (bq25120->pdata->use_usb)
		power_supply_unregister(&bq25120->usb);
	if (bq25120->pdata->use_mains)
		power_supply_unregister(&bq25120->mains);
}

static const struct regmap_config bq25120_regmap = {
	.reg_bits         = 8,
	.val_bits         = 8,

	.max_register     = bq25120_MAX_REGISTER,
	.reg_defaults     = bq25120_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25120_reg_defs),
	.cache_type	  = REGCACHE_RBTREE,
};

static int bq25120_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct bq25120_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct bq25120_charger *bq25120;
	int ret;

	pdata = dev->platform_data;
	if (!pdata) {
		printk(KERN_NOTICE "In probe - failed with NO PDATA\n");
		return -EINVAL;
	}

	bq25120 = devm_kzalloc(dev, sizeof(*bq25120), GFP_KERNEL);
	if (!bq25120)
		return -ENOMEM;

	bq25120->dev = &client->dev;
	bq25120->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, bq25120);

	bq25120->regmap = devm_regmap_init_i2c(client, &bq25120_regmap);
	if (IS_ERR(bq25120->regmap))
		return PTR_ERR(bq25120->regmap);


	ret = bq25120_hw_init(bq25120);
	if (ret < 0) {
		dev_err(dev, "failed to initialize bq25120 device: %d\n", ret);
		goto err_dev;
	}

	ret = bq25120_power_supply_register(bq25120);
	if (ret < 0) {
		dev_err(dev, "failed to register power supply: %d\n", ret);
		goto err_dev;
	}

	ret = sysfs_create_group(&dev->kobj, &bq25120_attr_group);
	if (ret < 0) {
		dev_err(dev, "failed to add charge sysfs: %d\n", ret);
		goto err_sysfs;
	}

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	ret = bq25120_irq_init(bq25120, client->irq);
	if (ret < 0) {
		dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
		dev_warn(dev, "disabling IRQ support\n");
	}

	return 0;

err_sysfs:
	bq25120_power_supply_unregister(bq25120);
err_dev:
	return ret;
}

static int bq25120_remove(struct i2c_client *client)
{
	struct bq25120_charger *bq25120 = i2c_get_clientdata(client);

	if (bq25120->irq) {
		bq25120_irq_disable(bq25120);
		free_irq(bq25120->irq, bq25120);
	}

	sysfs_remove_group(&bq25120->dev->kobj, &bq25120_attr_group);
	bq25120_power_supply_unregister(bq25120);

	return 0;
}

static const struct i2c_device_id bq25120_id[] = {
	{ "bq25120", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bq25120_id);

static struct i2c_driver bq25120_driver = {
	.driver = {
		.name = "bq25120",
		.owner = THIS_MODULE,
	},
	.probe        = bq25120_probe,
	.remove       = bq25120_remove,
	.id_table     = bq25120_id,
};

module_i2c_driver(bq25120_driver);

MODULE_DESCRIPTION("bq25120 battery charger driver");
MODULE_AUTHOR("TI BMS software tools team");
MODULE_LICENSE("TI BSD");
