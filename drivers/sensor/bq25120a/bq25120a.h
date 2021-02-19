#ifndef ZEPHYR_DRIVERS_SENSOR_BQ25120A_BQ25120A_H
#define ZEPHYR_DRIVERS_SENSOR_BQ25120A_BQ25120A_H

#include <init.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <kernel.h>
#include <sys/util.h>
#include <zephyr/types.h

/* Registers */
#define BQ25120_STATUS_SHIPMODE_REG		0x00
#define BQ25120_FAULTS_FAULTMASKS_REG		0x01
#define BQ25120_TSCONTROL_STATUS_REG		0x02
#define BQ25120_FASTCHARGE_CTL_REG		0x03
#define BQ25120_CHARGETERM_I2CADDR_REG		0x04
#define BQ25120_BATT_VOLTAGE_CTL_REG		0x05
#define BQ25120_SYSTEM_VOUT_CTL_REG		0x06
#define BQ25120_LOADSW_LDO_CTL_REG		0x07
#define BQ25120_PUSH_BTN_CTL_REG		0x08
#define BQ25120_ILIMIT_UVLO_CTL_REG		0x09
#define BQ25120_BATT_VOLT_MONITOR_REG		0x0A
#define BQ25120_VIN_DPM_TIMER_REG		0x0B

/* Status and ShipMode Control Register (0x0) */
#define BQ25120_STAT		(BIT(7) | BIT(6))
#define BQ25120_STAT_SHIFT	0x06
#define BQ25120_STAT_MASK	0x03
#define BQ25120_EN_SHIPMODE	BIT(5)
#define BQ25120_RESET		BIT(4)
#define BQ25120_TIMER		BIT(3)
#define BQ25120_VINDPM_STAT	BIT(2)
#define BQ25120_NOT_CD_STAT	BIT(1)
#define BQ25120_SYS_EN_STAT	BIT(0)

/* Faults and Fault Masks Register (Ox1) */
#define BQ25120_VIN_OV		BIT(7)
#define BQ25120_VIN_UV		BIT(6)
#define BQ25120_BAT_UVLO	BIT(5)
#define BQ25120_BAT_OCP		BIT(4)
#define BQ25120_FAULTS		(BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define BQ25120_FAULTS_SHIFT    0x04
#define BQ25120_FAULTS_MASK	0x0F
#define BQ25120_VIN_OV_M	BIT(3)
#define BQ25120_VIN_OV_M_SHIFT	0x03
#define BQ25120_VIN_OV_M_MASK	0x01
#define BQ25120_VIN_UV_M	BIT(2)
#define BQ25120_VIN_UV_M_SHIFT	0x02
#define BQ25120_VIN_UV_M_MASK	0x01
#define BQ25120_BAT_UVLO_M	BIT(1)
#define BQ25120_BAT_UVLO_M_SHIFT	0x01
#define BQ25120_BAT_UVLO_M_MASK	0x01
#define BQ25120_BAT_OCP_M	BIT(0)
#define BQ25120_BAT_OCP_M_SHIFT	0x0
#define BQ25120_BAT_OCP_M_MASK	0x01

/* TS Control and Status Mask Register (0x2) */
#define BQ25120_TS_EN		BIT(7)
#define BQ25120_TS_FAULT	(BIT(6) | BIT(5))
#define BQ25120_TS_FAULT_SHIFT  0x05
#define BQ25120_TS_FAULT_MASK	0X03
#define BQ25120_TS_FAULT_OPEN	BIT(4)
#define BQ25120_EN_INT		BIT(3)
#define BQ25120_WAKE_M		BIT(2)
#define BQ25120_WAKE_M_SHIFT	0x02
#define BQ25120_WAKE_M_MASK	0x01
#define BQ25120_RESET_M		BIT(1)
#define BQ25120_RESET_SHIFT	0x01
#define BQ25120_RESET_MASK	0x01
#define BQ25120_TIMER_M		BIT(0)
#define BQ25120_TIMER_M_SHIFT	0
#define BQ25120_TIMER_M_MASK	0x01

/* Fast Charge Control Register (0x03) */
#define BQ25120_ICHRG_RANGE	BIT(7)			
#define BQ25120_ICHARG		(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2)) 
#define BQ25120_ICHARG_SHIFT	0x02
#define BQ25120_ICHARG_MASK	0x1F
#define BQ25120_CE		BIT(1)
#define BQ25120_HZ_MODE		BIT(0)

/* Termination/Pre-Charge and I2C Address Register (0x4) */
#define BQ25120_IPRETERM_RANGE	BIT(7)
#define BQ25120_IPRETERM	(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define BQ25120_IPRETERM_SHIFT  0x02
#define BQ25120_IPRETERM_MASK   0x1F
#define BQ25120_TE		BIT(1)
// Bit 0 Reserved

/* Battery Voltage Control Register (0x05) */
#define BQ25120_VBREG		(BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define BQ25120_VBREG_SHIFT	0x1
#define BQ25120_VBREG_MASK	0x7F
// Bit 0 Reserved

/* SYS VOUT Control Register (0x06) */
#define BQ25120_EN_SYS_OUT	BIT(7)
#define BQ25120_SYS_SEL		(BIT(6) | BIT(5))
// TODO Double check this value
#define BQ25120_SYS_SEL_SHIFT	0x4 
#define BQ25120_SYS_MASK	0x3
#define BQ25120_SYS_VOUT	(BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define BQ25120_SYS_VOUT_SHIFT	0x01
#define BQ25120_SYS_VOUT_MASK	0x0F
//Bit 0 Reserved

/* Load Switch and LDO Control Register (0x07) */
#define BQ25120_EN_LS_LDO	BIT(7)
#define BQ25120_LS_LDO		(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2))
#define BQ25120_LS_LDO_SHIFT	0x2
#define BQ25120_LS_LDO_MASK	0x1F
//Bit 1 Reserved
#define BQ25120_MRRESET_VIN	BIT(0)

/* Pushbutton Control Register (0x08) */
#define BQ25120_MRWAKE1	    	BIT(7)
#define BQ25120_MRWAKE2		BIT(6)
#define BQ25120_MRREC		BIT(5)
#define BQ25120_MRRESET		(BIT(4) | BIT(3))
#define BQ25120_MRRESET_SHIFT	0x03
#define BQ25120_MRRESET_MASK	0x03
#define BQ25120_PGB_MRS		BIT(2)
#define BQ25120_WAKE1		BIT(1)
#define BQ25120_WAKE2		BIT(0)

/* ILIM and Battery UVLO Control Register (0x09) */
#define BQ25120_RESET_REG	BIT(7)
//Bit 6 Reserved
#define BQ25120_INLIM		(BIT(5) | BIT(4) | BIT(3))
#define BQ25120_INLIM_SHIFT	0x03
#define BQ25120_INLIM_MASK	0x07
#define BQ25120_BUVLO		(BIT(2) | BIT(1) | BIT(0))
#define BQ25120_BUVLO_SHIFT	0x0
#define BQ25120_BUVLO_MASK	0x7

/* Voltage Based Battery Monitor Register (0x0A) */
#define BQ25120_VBMON_READ	BIT(7)
#define BQ25120_VBMON_RANGE	(BIT(6) | BIT(5))
#define BQ25120_VBMON_RANGE_SHIFT 0x05
#define BQ25120_VBMON_RANGE_MASK  0x03
#define BQ25120_VBMON_TH	(BIT(4) | BIT(3) | BIT(2))
#define BQ25120_VBMON_TH_SHIFT	0x02
#define BQ25120_VBMON_TH_MASK	0x07
//Bit 1 and 0 Reserved

/* VIN_DPM and Timers Register (0x0B) */
#define BQ25120_VINDPM_OFF	BIT(7)
#define BQ25120_VINDPM		(BIT(6) | BIT(5) | BIT(4))
#define BQ25120_VINDPM_SHIFT	0x04
#define BQ25120_VINDPM_MASK	0x07
#define BQ25120_2XTMR_EN	BIT(3)
#define BQ25120_TMR		(BIT(2) | BIT(1))
#define BQ25120_TMR_SHIFT	0x01
#define BQ25120_TMR_MASK	0x03
#define BQ25120_TMR_DISABLE	0x03
//Bit 0 Reserved

#define BQ25120_MAX_CHG_FAULT	8

/* BQ25120A Driver Structure Definitions */

struct reg_default {
    uint8_t reg;
    uint8_t dflt_val;
};

struct power_supply {

};

typedef int (*bq25120a_interrupt_handler_t)(const struct device *dev);

struct bq25120a_config_t {
    const char *bus_label;
	uint16_t i2c_addr;
	const char* gpio_label;
	uint8_t gpio_pin;
	uint32_t gpio_flags;
};

struct bq25120a_data_t {
    const struct device* bus;
	const struct device* gpio;
	struct gpio_callback gpio_cb;
	struct k_sem sem;

    struct power_supply charger;
    struct power_supply battery;

    bool charger_online;
    bool charging_enabled;

	bool interrupt_enabled;
	bq25120a_interrupt_handler_t handler;
	void* owner;
};

extern int bq25120a_setup_interrupt(const struct device* dev, bool enable);

extern int bq25120a_transceive(const struct device *dev, uint8_t reg, bool write, void *buf, size_t length);

/* BQ25120A Register/Enum Mappings */

static struct reg_default BQ25120_reg_defs[] = {
	{BQ25120_STATUS_SHIPMODE_REG, 0x01},
	{BQ25120_FAULTS_FAULTMASKS_REG, 0x00},
	{BQ25120_TSCONTROL_STATUS_REG, 0x88},
	{BQ25120_FASTCHARGE_CTL_REG, 0x14},
	{BQ25120_CHARGETERM_I2CADDR_REG, 0x0E},
	{BQ25120_BATT_VOLTAGE_CTL_REG, 0x78},
	{BQ25120_SYSTEM_VOUT_CTL_REG, 0xAA},
	{BQ25120_LOADSW_LDO_CTL_REG, 0x7C},
	{BQ25120_PUSH_BTN_CTL_REG, 0x68},
	{BQ25120_ILIMIT_UVLO_CTL_REG, 0x0A},
	{BQ25120_BATT_VOLT_MONITOR_REG, 0x00},
	{BQ25120_VIN_DPM_TIMER_REG, 0x4A},
};

enum BQ25120_charge_fault {
	BQ25120_NORMAL = 0,
	BQ25120_CH_VIN_OV,
	BQ25120_CH_VIN_UV,
	BQ25120_BATTERY_UVLO,
	BQ25120_BATTERY_OCP,
};

enum BQ25120_temp_status {
	BQ25120_TEMP_NORMAL = 0,
	BQ25120_TEMP_TNTC,
	BQ25120_TEMP_TCOOL,
	BQ25120_TEMP_TWARM,
};

static const char * temp_status_desc[] = {
	[BQ25120_NORMAL] = "Normal",
	[BQ25120_TEMP_TNTC] = "TS temp < Tcold or TS temp > Thot",
	[BQ25120_TEMP_TCOOL] = "Tcool > TS temp or TS temp > Tcold",
	[BQ25120_TEMP_TWARM] = "Twarm < TS temp or TS temp < Tcold",
};

/* Charge current in mA */
static const unsigned int bq25120a_cc_tbl[] = {
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	31,
	32,
	33,
	34,
	35,
	40,
	50,
	60,
	70,
	80,
	90,
	100,
	110,
	120,
	130,
	140,
	150,
	160,
	170,
	180,
	190,
	200,
	210,
	220,
	230,
	240,
	250,
	260,
	270,
	280,
	290,
	300
};

/* Termination current in mA */
static const unsigned int bq25120a_tc_tbl[] = {
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,
	10,
	11,
	12,
	13,
	14,
	15,
	16,
	17,
	18,
	19,
	20,
	21,
	22,
	23,
	24,
	25,
	26,
	27,
	28,
	29,
	30,
	31,
	32,
	33,
	34,
	35,
	36,
	37
};

/* Input current limit in mA */
static const unsigned int bq25120a_icl_tbl[] = {
	50,
	100,
	150,
	200,
	250,
	300,
	350,
	400
};

/* Safety timer time limit in minutes */
static const unsigned int bq25120a_stl_tbl[] = {
	30,
	180,
	540,
	0, /* Disable */
}


#endif /* ZEPHYR_DRIVERS_SENSOR_BQ25120A_BQ25120A_H */