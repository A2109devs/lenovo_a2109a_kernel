/*
 * BQ27541 battery driver
 *
 * Modified by Johnny Qiu <joqiu@nvidia.com>, Chandler Zhang <chazhang@nvidia.com>
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/smb349-charger.h>

#include <mach/gpio.h>
//&*&*&*HC1_20120525
#ifdef CONFIG_CL2N_FAKE_BATTERY
#include <linux/io.h>
#include <mach/iomap.h>
#endif
//&*&*&*HC2_20120525

#define DRIVER_VERSION			"2.0.0"
#define ADD_ATTRIBUTES_FOR_DEBUG 1

#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_FAC			0x0E
#define BQ27541_REG_FCC			0x12

#define BQ27541_REG_SOC			0x2c
#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BATTERY_POLL_PERIOD		10000

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};

enum bq27541_chip { BQ27541 };

struct bq27541_device_info {
	struct device 		*dev;
	int			id;
	struct bq27541_access_methods	*bus;
	enum bq27541_chip	chip;
	int			irq;
	struct i2c_client	*client;
	struct power_supply	bat;
	struct power_supply	ac;
	struct power_supply	usb;
	/* State Of Connect */	
	int lifesoc;
	int status;
	int ac_online;	
	int usb_online;
	int battery_online;
	//struct timer_list	battery_poll_timer;
	struct delayed_work		battery_polling_delay_work;
	struct delayed_work		battery_updata_delay_work;
	int fake_soc;
//&*&*&*HC1_20120525
	#ifdef CONFIG_CL2N_FAKE_BATTERY
	bool fake_battery;
	#endif
//&*&*&*HC2_20120525
} *bq27541_device;

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

static enum power_supply_property bq27541_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property bq27541_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *power_supplied_to[] = {
	"battery",
};

int bq27541_get_battery_rsoc();
extern int bq275xx_init_development_mode(struct i2c_client *client);
extern int bq275xx_parameter_update(struct i2c_client *client);
//&*&*&*HC1_20120525
#ifdef CONFIG_CL2N_FAKE_BATTERY
#define PMC_SCRATCH20		0xa0
#define FAKE_BATTERY_MODE	BIT(25)
bool is_fake_battery()
{
	void __iomem *reset = IO_ADDRESS(TEGRA_PMC_BASE + 0x00);
	u32 reg;
	bool ret = false;

	reg = readl_relaxed(reset + PMC_SCRATCH20);

	if (reg & FAKE_BATTERY_MODE) {
		reg &= ~(FAKE_BATTERY_MODE);
		writel_relaxed(reg, reset + PMC_SCRATCH20);
		ret = true;
	}

	printk("%s, fack_battery=%d \n", __func__, ret);
	
	return ret;
}
#endif
//&*&*&*HC2_20120525

/*
 * Common code for BQ27541 devices
 */

static int bq27541_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;

	ret = bq27541_read(BQ27541_REG_TEMP, &temp, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	return temp - 2731;
}

/*
 * Return the battery full charge capacity in mAh
 */
static int bq27541_full_charge_capacity(struct bq27541_device_info *di)
{
	int tval = 0;
	int ret;

	ret = bq27541_read(BQ27541_REG_FCC, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading full charge capacity");
		return ret;
	}

	return tval;
}

/*
 * Return the battery available charge capacity in mAh
 */
static int bq27541_full_available_capacity(struct bq27541_device_info *di)
{
	int tval = 0;
	int ret;

	ret = bq27541_read(BQ27541_REG_FAC, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading full available capacity");
		return ret;
	}

	return tval;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bq27541_read(BQ27541_REG_VOLT, &volt, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}
	//return volt * 1000;
	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_battery_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;
	//int flags = 0;

	ret = bq27541_read(BQ27541_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = (int)(s16)curr;
	return curr * 1000;
	//return curr;
	
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
int bq27541_get_battery_rsoc()
{
	int ret;
	int rsoc = 0;
	if(bq27541_device == NULL)
		return -1;

	ret = bq27541_read(BQ27541_REG_SOC, &rsoc, 0, bq27541_device);
	return rsoc;
}
EXPORT_SYMBOL_GPL(bq27541_get_battery_rsoc);

static int bq27541_battery_rsoc(struct bq27541_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bq27541_read(BQ27541_REG_SOC, &rsoc, 0, di);
	di->lifesoc = rsoc;
	return rsoc;
}

/*
static int bq27541_get_ac_status(void)
{
	int charger_gpio = irq_to_gpio(bq27541_device->irq);
	return !gpio_get_value(charger_gpio);
}
*/

static int bq27541_get_charging_status(void)
{
	int ret;
	int curr = 0;

	struct bq27541_device_info *di = bq27541_device;

	ret = bq27541_read(BQ27541_REG_AI, &curr, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}

	curr = (int)(s16)curr;
	return curr * 1000;
}

static int bq27541_battery_status(struct bq27541_device_info *di,
				  union power_supply_propval *val)
{
	val->intval = di->status;
	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27541_battery_time(struct bq27541_device_info *di, int reg,
				union power_supply_propval *val)
{
	int tval = 0;
	int ret;

	ret = bq27541_read(reg, &tval, 0, di);
	if (ret) {
		dev_err(di->dev, "error reading register %02x\n", reg);
		return ret;
	}

	if (tval == 65535)
		return -ENODATA;

	val->intval = tval * 60;
	return 0;
}

//&*&*&*HC1_20120525
#ifdef CONFIG_CL2N_FAKE_BATTERY
int bq27541_fake_battery_get_property(enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
			val->intval = 1; // not charging
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:	
			val->intval = 4000;

		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:		
			val->intval = 100000000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 6000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:		
			val->intval = 6000;	
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_TEMP:
			val->intval = 250;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
			val->intval = 60000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			val->intval = 60000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
			val->intval = 60000;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
#endif
//&*&*&*HC2_20120525

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = bq27541_device;

//&*&*&*HC1_20120525
	#ifdef CONFIG_CL2N_FAKE_BATTERY
	if (di->fake_battery) {
		return  bq27541_fake_battery_get_property(psp, val);
	}
	#endif
//&*&*&*HC2_20120525

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->battery_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq27541_battery_status(di, val);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27541_battery_voltage(di);
		if (psp == POWER_SUPPLY_PROP_PRESENT)
			val->intval = val->intval <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27541_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq27541_full_charge_capacity(di);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq27541_full_available_capacity(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if(di->fake_soc != 1)
			val->intval = di->fake_soc;
		else
			val->intval = bq27541_battery_rsoc(di);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27541_battery_temperature(di);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = bq27541_battery_time(di, BQ27541_REG_TTE, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = bq27541_battery_time(di, BQ27541_REG_TTECP, val);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = bq27541_battery_time(di, BQ27541_REG_TTF, val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq27541_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = bq27541_device;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->ac_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq27541_usb_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = bq27541_device;
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(di->usb_online)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static void bq27541_charger_status(enum charging_states status, enum charger_type chrg_type, void *data)
{	
	struct bq27541_device_info *di = (struct bq27541_device_info *)data;	
	
	mutex_lock(&battery_mutex);
	di->ac_online = 0;	
	di->usb_online = 0;	
	di->battery_online = 0;	
	if (chrg_type == SMB349_AC)	
		di->ac_online = 1;	
	else if (chrg_type == SMB349_USB)	
		di->usb_online = 1;
	else
		di->battery_online = 1;
	
	if (status == smb349_progress)	
		di->status = POWER_SUPPLY_STATUS_CHARGING;	
	else if (status == smb349_completed)	
		di->status = POWER_SUPPLY_STATUS_FULL;	
	else if (status >= smb349_temp_hot_fault)	
		di->status = POWER_SUPPLY_STATUS_NOT_CHARGING;	
	else
	{
		if((chrg_type >= SMB349_AC) && (di->lifesoc >= 100))
			di->status = POWER_SUPPLY_STATUS_FULL;
		else
			di->status = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	mutex_unlock(&battery_mutex);
	
	power_supply_changed(&di->bat);	
}

static int bq27541_powersupply_init(struct bq27541_device_info *di)
{
	int retval;
	struct power_supply *battery, *ac, *usb;
	
	ac = &di->ac;
	ac->type = POWER_SUPPLY_TYPE_MAINS;
	ac->properties = bq27541_ac_props;
	ac->num_properties = ARRAY_SIZE(bq27541_ac_props);
	ac->get_property = bq27541_ac_get_property;
	ac->external_power_changed = NULL;
	retval = power_supply_register(&di->client->dev, ac);
	if (retval) {
		dev_err(&di->client->dev, "failed to register ac\n");
		goto ac_failed;
	}

	usb = &di->usb;
	usb->type = POWER_SUPPLY_TYPE_USB;
	usb->properties = bq27541_usb_props;
	usb->num_properties = ARRAY_SIZE(bq27541_usb_props);
	usb->get_property = bq27541_usb_get_property;
	usb->external_power_changed = NULL;
	retval = power_supply_register(&di->client->dev, usb);
	if (retval) {
		dev_err(&di->client->dev, "failed to register usb\n");
		goto usb_failed;
	}

	battery = &di->bat;
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = bq27541_battery_props;
	battery->num_properties = ARRAY_SIZE(bq27541_battery_props);
	battery->get_property = bq27541_battery_get_property;
	battery->external_power_changed = NULL;
	retval = power_supply_register(&di->client->dev, battery);
	if (retval) {
		dev_err(&di->client->dev, "failed to register battery\n");
		goto batt_failed;
	}
	
	return 0;

batt_failed:
	power_supply_unregister(&di->usb);
usb_failed:
	power_supply_unregister(&di->ac);
ac_failed:
	return retval;
}

/*
 * i2c specific code
 */

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	s32 data;
	struct i2c_client *client = di->client;

	if (!b_single)
		data = i2c_smbus_read_word_data(client, reg);
	else
		data = i2c_smbus_read_byte_data(client, reg);

	if (data < 0)
		return data;
	else
	{
		*rt_value = *((s16 *)(&data));
		return 0;
	}
}

#if 0
static irqreturn_t ac_present_irq(int irq, void *data)
{
	power_supply_changed(&bq27541_device->ac);
	power_supply_changed(&bq27541_device->bat);
	return IRQ_HANDLED;
}
#endif

#ifdef ADD_ATTRIBUTES_FOR_DEBUG
static ssize_t bq27541_fake_soc_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bq27541_device_info *di = dev_get_drvdata(dev);
	
	return sprintf(buf, "%u\n", di->fake_soc);
}

static ssize_t bq27541_fake_soc_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bq27541_device_info *di = dev_get_drvdata(dev);
	int val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	if(val == 28239)
	{
		bq275xx_init_development_mode(di->client);
		printk("[bq27541 gauge]Enable data flash mode : %d...\n", val);
	}
	else if(val>=0 && val<105)
	{
		di->fake_soc = val;
		if(di->fake_soc == 1)
			printk("[bq27541 gauge]Disable fake soc : %d...\n", di->fake_soc);
	}

	return count;
}

static DEVICE_ATTR(fake_soc, 0664, bq27541_fake_soc_show, bq27541_fake_soc_store);

static struct attribute *bq27541_attributes[] = {
	&dev_attr_fake_soc.attr,
	NULL
};

static const struct attribute_group bq27541_attr_group = {
	.attrs = bq27541_attributes,
};
#endif

static void battery_poll_worker_func(struct work_struct *work)
{
	struct bq27541_device_info *di = container_of(work, struct bq27541_device_info, battery_polling_delay_work.work);
	int val;
	
	power_supply_changed(&di->bat);
//&*&*&*HC1_20120525
	#ifdef CONFIG_CL2N_FAKE_BATTERY
	if (!di->fake_battery) {
	#endif	
	val = bq27541_battery_voltage(di);
	if(val%1000 <100)
		printk("Battery Voltage: %d.0%d V\n",val/1000, val%1000);
	else
		printk("Battery Voltage: %d.%d V\n",val/1000, val%1000);
	val = bq27541_battery_current(di);
	printk("Battery Current: %d mA\n",val/1000);
	val = bq27541_battery_rsoc(di);
	printk("Battery Soc: %d %%\n", val);
	val = bq27541_battery_temperature(di);
	printk("Battery Temperature: %d.%d C\n",val/10, val%10);
	schedule_delayed_work(&di->battery_polling_delay_work, 10*HZ);
	#ifdef CONFIG_CL2N_FAKE_BATTERY	
	} else {
		printk("Fake battery \n");
	}
	#endif
//&*&*&*HC2_20120525	

}

static void battery_updata_worker_func(struct work_struct *work)
{
	struct bq27541_device_info *di = container_of(work, struct bq27541_device_info, battery_updata_delay_work.work);
	
	bq275xx_init_development_mode(di->client);
	bq275xx_parameter_update(di->client);
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_access_methods *bus;
	int num;
	int retval = 0;

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	bq27541_device = kzalloc(sizeof(*bq27541_device), GFP_KERNEL);
	if (!bq27541_device) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	bq27541_device->id = num;
	bq27541_device->chip = id->driver_data;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method "
					"data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, bq27541_device);
	bq27541_device->dev = &client->dev;
	bq27541_device->bat.name = "battery";
	bq27541_device->ac.name = "ac";
	bq27541_device->usb.name = "usb";
	bus->read = &bq27541_read_i2c;
	bq27541_device->bus = bus;
	bq27541_device->client = client;
	bq27541_device->irq = client->irq;
	bq27541_device->fake_soc = 1;
//&*&*&*HC1_20120525
	#ifdef CONFIG_CL2N_FAKE_BATTERY
	bq27541_device->fake_battery = is_fake_battery();
	#endif
//&*&*&*HC2_20120525

	retval = bq27541_powersupply_init(bq27541_device);
	if (retval) {
		dev_err(&client->dev, "failed to register power_suuply\n");
		goto batt_failed_5;
	}

	/*
	retval = request_threaded_irq(bq27541_device->irq, NULL,
		ac_present_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"ac_present", bq27541_device);
	if (retval < 0) {
		dev_err(&client->dev, "failed to request irq for battery\n");
		goto batt_failed_4;
	}
	*/
	retval = register_callback(bq27541_charger_status, bq27541_device);
	if (retval < 0)		
		dev_info(&client->dev, "register smb349 callback error\n");

#ifdef ADD_ATTRIBUTES_FOR_DEBUG
	retval = sysfs_create_group(&client->dev.kobj, &bq27541_attr_group);
	if (retval)
		dev_err(&client->dev, "%s() error in creating sysfs attribute" , __func__);
#endif

	INIT_DELAYED_WORK(&bq27541_device->battery_polling_delay_work, battery_poll_worker_func);
	schedule_delayed_work(&bq27541_device->battery_polling_delay_work, 10*HZ);

	INIT_DELAYED_WORK(&bq27541_device->battery_updata_delay_work, battery_updata_worker_func);
	schedule_delayed_work(&bq27541_device->battery_updata_delay_work, 1*HZ);
	
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	retval = update_charger_status();	
	if (retval) {		
		dev_err(&client->dev, "failed: update_charger_status\n");		
	}

	return 0;
batt_failed_5:
	//free_irq(bq27541_device->irq, bq27541_device);
	kfree(bus);
batt_failed_3:
	kfree(bq27541_device);
batt_failed_2:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	cancel_delayed_work(&di->battery_polling_delay_work);
	
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->usb);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);

	return 0;
}

#if defined (CONFIG_PM)
static int bq27541_battery_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	cancel_delayed_work(&di->battery_polling_delay_work);
	
	return 0;
}

static int bq27541_battery_resume(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	//power_supply_changed(&bq27541_device->bat);
	schedule_delayed_work(&di->battery_polling_delay_work, 10*HZ);
	
	return 0;
}
#endif

/*
 * Module stuff
 */

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541", BQ27541 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.driver = {
		.name = "bq27541-battery",
	},
	.probe = bq27541_battery_probe,
	.remove = bq27541_battery_remove,
#if defined (CONFIG_PM)
	.suspend = bq27541_battery_suspend,
	.resume = bq27541_battery_resume,
#endif
	.id_table = bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
MODULE_LICENSE("GPL");
