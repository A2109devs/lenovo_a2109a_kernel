/*
 * drivers/power/smb349-charger.c
 *
 * Battery charger driver for smb349 from summit microelectronics
 *
 * Modify on 2012.05, Foxconn Corporation, by Aimar Liu.
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb349-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

#define SMB349_CHARGE		0x00
#define SMB349_CHRG_CRNTS	0x01
#define SMB349_VRS_FUNC		0x02
#define SMB349_FLOAT_VLTG	0x03
#define SMB349_CHRG_CTRL	0x04
#define SMB349_STAT_TIME_CTRL	0x05
#define SMB349_PIN_CTRL		0x06
#define SMB349_THERM_CTRL	0x07
#define SMB349_CTRL_REG		0x09

#define SMB349_OTG_TLIM_REG	0x0A
#define SMB349_HRD_SFT_TEMP	0x0B
#define SMB349_FAULT_INTR	0x0C
#define SMB349_STS_INTR_1	0x0D
#define SMB349_SYSOK_USB3	0x0E
#define SMB349_IN_CLTG_DET	0x10
#define SMB349_STS_INTR_2	0x11

#define SMB349_CMD_REG		0x30
#define SMB349_CMD_REG_B	0x31
#define SMB349_CMD_REG_c	0x33

#define SMB349_INTR_STS_A	0x35
#define SMB349_INTR_STS_B	0x36
#define SMB349_INTR_STS_C	0x37
#define SMB349_INTR_STS_D	0x38
#define SMB349_INTR_STS_E	0x39
#define SMB349_INTR_STS_F	0x3A

#define SMB349_STS_REG_A	0x3B
#define SMB349_STS_REG_B	0x3C
#define SMB349_STS_REG_C	0x3D
#define SMB349_STS_REG_D	0x3E
#define SMB349_STS_REG_E	0x3F

#define SMB349_ENABLE_WRITE	1
#define SMB349_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define THERM_CTRL		0x10
#define BATTERY_MISSING		0x10
#define CHARGING		0x06
#define DEDICATED_CHARGER	0x04
#define CHRG_DOWNSTRM_PORT	0x08
#define ENABLE_CHARGE		0x02
#define TERNIBATION_CHARGING 0x01
#define FAST_CHARGING 0x01
#define TAPER_CHARGING 0x04
#define CHARGINGTIMEOUT 0x01
#define TEMPERATURE_HOT 0x40
#define TEMPERATURE_COLD 0x10
#define POWER_OK 0x01
#define ENABLE_HC_MODE 0x01
#define LOW_BATTERY_ALERT 0x04

#define USB_WAKE_LOCK

#define ARRAYSIZE(a)		(sizeof(a)/sizeof(a[0]))
static struct smb349_charger *charger;
#ifdef USB_WAKE_LOCK
static struct wake_lock usb_lock;
#endif

#ifdef MAX77663_CALLBACK_FUNCTION
#define SHUTDOWN_SOC 5
#endif

static int smb349_configure_charger(struct i2c_client *client, int value, int max_uA);
static void smb349_early_suspend(struct early_suspend *h);
static void smb349_late_resume(struct early_suspend *h);
extern int bq27541_get_battery_rsoc();
extern int tegra_get_host(int suspend);
extern void max77663_OTG_alert(void* data);

static int smb349_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb349_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb349_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb349_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb349_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb349_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == SMB349_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb349_update_reg(client, SMB349_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, SMB349_CMD_REG);
			return ret;
		}
	} else {
		ret = smb349_read(client, SMB349_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb349_write(client, SMB349_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

static void smb349_set_low_battery(int low)
{
	struct i2c_client *client = charger->client;
	int ret;
	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
									__func__);
	}
	/* Configure Low battery alert */		
	//ret = smb349_read(client, SMB349_STS_INTR_1);		
	//if (ret < 0) {			
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	//}		
	ret = smb349_write(client, SMB349_STS_INTR_1, 0x99);
	if (ret < 0) {			
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}
	/* Configure Low battery to 3.58v */		
	//ret = smb349_read(client, SMB349_IN_CLTG_DET);		
	//if (ret < 0) {			
		//dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	//}		
	ret = smb349_write(client, SMB349_IN_CLTG_DET, low);
	if (ret < 0) {			
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
									__func__);
	}
	printk("[smb349 charger]set_low_battery :%d.\n", low);
}

static void smb349_set_configuration(void)
{
	struct i2c_client *client = charger->client;
	struct smb349_charger_platform_data *pdata = charger->client->dev.platform_data;
	int ret,i;
	/* configurate charger */
	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
									__func__);
	}
	for(i=0; i<ARRAYSIZE(pdata->configuration_data); i++)
	{
		if(pdata->configuration_data[i] == 0xFF)
			continue;
		ret = smb349_write(client, i, pdata->configuration_data[i]);
	}
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
									__func__);
	}
	/* end of configurate charger */
	return;
}

static int smb349_configure_otg(struct i2c_client *client, int enable)
{
	int ret = 0;

	/*Enable volatile writes to registers*/
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring otg..\n",
								__func__);
		goto error;
	}

	if (enable) {
		/* Configure PGOOD to be active low */
		ret = smb349_read(client, SMB349_SYSOK_USB3);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb349_write(client, SMB349_SYSOK_USB3, (ret & (~(1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		/* Enable OTG */
		ret = smb349_update_reg(client, SMB349_CMD_REG, 0x10);
		if (ret < 0) {
			dev_err(&client->dev, "%s: Failed in writing register"
				"0x%02x\n", __func__, SMB349_CMD_REG);
			goto error;
		}
	} else {
		/* Disable OTG */
		ret = smb349_read(client, SMB349_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb349_write(client, SMB349_CMD_REG, (ret & (~(1<<4))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		/* Configure PGOOD to be active high */
		ret = smb349_update_reg(client, SMB349_SYSOK_USB3, 0x01);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}

	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
error:
	return ret;
}

static int smb349_configure_charger(struct i2c_client *client, int value, int max_uA)
{
	int ret = 0;
	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		 /* Enable charging */
		ret = smb349_read(client, SMB349_PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		ret = smb349_write(client, SMB349_PIN_CTRL, (ret | ((6<<4))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		/* use 0x31 command to enhance AC power */
		if(max_uA == 1800*1000)
		{
			ret = smb349_write(client, SMB349_CMD_REG_B, ENABLE_HC_MODE);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
			ret = smb349_read(client, SMB349_PIN_CTRL);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
			ret = smb349_write(client, SMB349_PIN_CTRL, (ret & (~(1<<4))));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}
		}
		/* end of enhanced AC power*/
		
	} else {
		ret = smb349_read(client, SMB349_PIN_CTRL);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
		ret = ret & (~(6<<4));
		ret = smb349_write(client, SMB349_PIN_CTRL, (ret | ((4<<4))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	
error : 
	return ret;

}

int update_charger_status(void)
{
	struct i2c_client *client = charger->client;

	if(client == NULL)
		return -1;

	if(charger->chrg_type == SMB349_USB)
		printk("[smb349 charger]cable type is %s\n", "USB");
	else if(charger->chrg_type == SMB349_AC)
		printk("[smb349 charger]cable type is %s\n", "AC adapter");
	else if(charger->chrg_type == SMB349_BATTERY)
	{
		//if(charger->state == smb349_completed)
		charger->state = smb349_idle;
		printk("[smb349 charger]cable type is %s\n", "Battery");
	}
	
	if (charger->charger_cb)
		charger->charger_cb(charger->state, charger->chrg_type,
						charger->charger_cb_data);
	return 0;
}
EXPORT_SYMBOL_GPL(update_charger_status);

#ifdef MAX77663_CALLBACK_FUNCTION
int register_max77663_callback(max77663_callback_t cb, void *args, enum max77663_callback_type type)
{
	struct smb349_charger *charger_data = charger;
	if (!charger_data)
		return -ENODEV;

	charger_data->max77663_cb[type] = cb;
	charger_data->max77663_cb_data[type] = args;
	
	return 0;
}
EXPORT_SYMBOL_GPL(register_max77663_callback);
#endif

int register_callback(charging_callback_t cb, void *args)
{
	struct smb349_charger *charger_data = charger;
	if (!charger_data)
		return -ENODEV;

	charger_data->charger_cb = cb;
	charger_data->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_callback);

int smb349_battery_online(void)
{
	int val;
	struct i2c_client *client = charger->client;

	val = smb349_read(charger->client, SMB349_INTR_STS_B);
	if (val < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, SMB349_INTR_STS_B);
		return val;
	}
	if (val & BATTERY_MISSING)
		return 0;
	else
		return 1;
}

static void smb349_otg_status(enum usb_otg_state to, enum usb_otg_state from, void *data)
{
	struct i2c_client *client = charger->client;
	int ret = 0;
	int val = 0;

	if ((from == OTG_STATE_A_SUSPEND) && (to == OTG_STATE_A_HOST)) {
		mutex_lock(&charger->mutex);
		if(charger->otg_suspend == 1)
		{
			mutex_unlock(&charger->mutex);
			return;
		}

		if (charger->max77663_cb[SMB349_OTG_CABLE])
		{
			*((int *)charger->max77663_cb_data[SMB349_OTG_CABLE]) = 1;
			charger->max77663_cb[SMB349_OTG_CABLE](charger->max77663_cb_data[SMB349_OTG_CABLE]);
		}
		else
		{
			val = 1;
			max77663_OTG_alert(&val);
		}
		/*Enable volatile writes to registers*/
		ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
		if (ret < 0) {
			dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
			goto error;
		}
		/* Configure PGOOD to be active low */		
		ret = smb349_read(client, SMB349_SYSOK_USB3);		
		if (ret < 0) {			
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		}		
		ret = smb349_write(client, SMB349_SYSOK_USB3, (ret & (~(1))));
		if (ret < 0) {			
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		}
		/* Disable volatile writes to registers */
		ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
		if (ret < 0) {
			dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
			goto error;
		}
		/* configure charger */
		ret = smb349_configure_charger(client, 0, 0);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);

		/* ENABLE OTG */
		ret = smb349_configure_otg(client, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
		charger->chrg_type = SMB349_ID_BOOST;
		mutex_unlock(&charger->mutex);
		printk("Enter %s, to OTG_STATE_A_HOST.\n", __func__);
		
#ifdef USB_WAKE_LOCK
		if(wake_lock_active(&usb_lock))
		{
			wake_unlock(&usb_lock);
		}
#endif

	} else if ((from == OTG_STATE_A_HOST) && (to == OTG_STATE_A_SUSPEND)) {

		/* Disable OTG */
		ret = smb349_configure_otg(client, 0);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
		/* configure charger */
		ret = smb349_configure_charger(client, 1, 0);
		if (ret < 0)
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
		
		charger->chrg_type = SMB349_BATTERY;
		//charger->otg_suspend = 0;

		/*Enable volatile writes to registers*/
		ret = smb349_volatile_writes(client, SMB349_ENABLE_WRITE);
		if (ret < 0) {
			dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
			goto error;
		}
		/* Configure PGOOD to be active high */		
		ret = smb349_update_reg(client, SMB349_SYSOK_USB3, 0x01);	
		if (ret < 0) {		
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		}
		/* Disable volatile writes to registers */
		ret = smb349_volatile_writes(client, SMB349_DISABLE_WRITE);
		if (ret < 0) {
			dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
			goto error;
		}
		#if 0
		msleep(100); /* Let AC_OK irq doing first before setting the flag*/
		if (charger->max77663_cb[SMB349_OTG_CABLE])
		{
			*((int *)charger->max77663_cb_data[SMB349_OTG_CABLE]) = 0;
			charger->max77663_cb[SMB349_OTG_CABLE](charger->max77663_cb_data[SMB349_OTG_CABLE]);
		}
		else
		{
			val = 0;
			max77663_OTG_alert(&val);
		}
		#endif
		printk("Enter %s, to OTG_STATE_A_SUSPEND.\n", __func__);
	}
	else if((from == OTG_STATE_A_SUSPEND) && (to == OTG_STATE_B_PERIPHERAL) && !charger->cable_exist)
	{
		charger->chrg_type = SMB349_VBUS_VALID;
		smb349_set_configuration(); /* re-config charger */
#ifdef USB_WAKE_LOCK
		if(!wake_lock_active(&usb_lock))
			wake_lock(&usb_lock);
#endif
	}
#ifdef USB_WAKE_LOCK
	else if((from == OTG_STATE_B_PERIPHERAL) && (to == OTG_STATE_A_SUSPEND) && !charger->cable_exist)
	{
		if(wake_lock_active(&usb_lock))
			wake_unlock(&usb_lock);
	}
#endif
error:
	return;
}

static int smb349_enable_charging(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct i2c_client *client = charger->client;
	int ret;
	
	if (!max_uA) {
		/* Disable charger */
		charger->chrg_type = SMB349_BATTERY;
#if 0   /* Don't disbale charger when usb suspend */
		ret = smb349_configure_charger(client, 0);
		if (ret < 0) {
			dev_err(&client->dev, "%s() error in configuring"
				"charger..\n", __func__);
			return ret;
		}
		charger->state = smb349_stopped;
#endif
		printk("[smb349 charger]set charging current to 0mA(Disable charging)\n");
	} else {
		if(max_uA == 500 * 1000)
			charger->chrg_type = SMB349_USB;
		else if(max_uA == 1800 * 1000)
		{
			charger->chrg_type = SMB349_AC;
			if(!charger->cable_exist)
			{
				/* configure charger */
				ret = smb349_configure_charger(client, 1, max_uA);
				if (ret < 0) {
					dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
					return ret;
				}
			}
		}
		printk("[smb349 charger]set charging current to %dmA(Enable charging)\n", max_uA);
	}
	if(!charger->suspend)
		update_charger_status();
	return 0;
}

#ifdef ADD_ATTRIBUTES_FOR_DIAG
static ssize_t smb349_dump_reg_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	struct smb349_charger_platform_data *pdata = chip->client->dev.platform_data;
	int i, val;
	/* configurate charger */
	printk("**** Dump Charger Registers ****\n");
	for(i=0; i<ARRAYSIZE(pdata->configuration_data)+2; i++)
	{
		val = smb349_read(chip->client, i);
		printk("[0x%02X] : [0x%02X]\n", i, val);
	}
	for(i=0x30; i<0x32;i++)
	{
		val = smb349_read(chip->client, i);
		printk("[0x%02X] : [0x%02X]\n", i, val);
	}
	for(i=0x35; i<=0x3A;i++)
	{
		val = smb349_read(chip->client, i);
		printk("[0x%02X] : [0x%02X]\n", i, val);
	}
	for(i=0x3B; i<0x40;i++)
	{
		val = smb349_read(chip->client, i);
		printk("[0x%02X] : [0x%02X]\n", i, val);
	}
	printk("**** End Dump Charger Registers **** \n");
	return 1;
}

static ssize_t smb349_dump_reg_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	unsigned long val;
	int error, ret;
	char *tok;
	int first = 1;
	int addr = 0x00;

	while (true) {
		tok = strsep(&buf, ",");
		if (!tok)
			break;
		error = strict_strtoul(tok, 10, &val);
		if (error)
			return error;
		if(first){
			addr = val;
			first = 0;
		}
	}
	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(chip->client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s() error in configuring charger..\n",
								__func__);
	}
	ret = smb349_write(chip->client, addr, val);
	if (ret < 0)
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(chip->client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s() error in configuring charger..\n",
								__func__);
	}
	ret = smb349_read(chip->client, addr);
	printk("[smb349 charger]The addr:0x%02X is 0x%02X\n", addr, ret);

	return count;
}

static DEVICE_ATTR(dump_reg, 0664, smb349_dump_reg_show, smb349_dump_reg_store);

static ssize_t smb349_suspend_charger_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	int ret;
	ret = smb349_read(chip->client, SMB349_CMD_REG);
	printk("[SMB349 charger]command register 0x30, %d.\n", ret);

	return sprintf(buf, "%u\n", ret);
}

static ssize_t smb349_suspend_charger_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = strict_strtoul(buf, 10, &val);
	if (ret)
		return ret;

	/* Enable volatile writes to registers */
	ret = smb349_volatile_writes(chip->client, SMB349_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	ret = smb349_read(chip->client, SMB349_CMD_REG);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
		goto error;
	}
	if(val)
	{
		ret = smb349_write(chip->client, SMB349_CMD_REG, (ret | (1<<2)));
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	else
	{
		ret = smb349_write(chip->client, SMB349_CMD_REG, (ret & (~(1<<2))));
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb349_volatile_writes(chip->client, SMB349_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	
error : 
	return count;

}

static DEVICE_ATTR(suspend_charger, 0664, smb349_suspend_charger_show, smb349_suspend_charger_store);

static ssize_t smb349_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	u8 val;
	val = smb349_read(chip->client, SMB349_PIN_CTRL);
	printk("[smb349 charger]Pin Control Register(06), 0x%02x.\n", val);
	return sprintf(buf, "%u\n", val);
}

static ssize_t smb349_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct smb349_charger *chip = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	mutex_lock(&chip->mutex);
	if (val) {
		printk("[smb349 charger]Enter %s, enable charging....\n", __func__);
		if(charger->chrg_type == SMB349_AC)
			smb349_configure_charger(chip->client, 1, 1800*1000);
		else if(charger->chrg_type == SMB349_USB)
			smb349_configure_charger(chip->client, 1, 500*1000);
		
	} else {
		printk("[smb349 charger]Enter %s, disable charging....\n", __func__);
		smb349_configure_charger(chip->client, 0, 0);
	}
	mutex_unlock(&chip->mutex);

	return count;
}

static DEVICE_ATTR(enable, 0664, smb349_enable_show, smb349_enable_store);

static struct attribute *smb349_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_suspend_charger.attr,
	&dev_attr_dump_reg.attr,
	NULL
};

static const struct attribute_group amb349_attr_group = {
	.attrs = smb349_attributes,
};
#endif

static void smb349_irq_delay_worker_interrupt(struct work_struct *work)
{
	struct smb349_charger *chip = container_of(work, struct smb349_charger, smb349_interrupt_delay_work.work);
	struct smb349_charger_platform_data *pdata = chip->client->dev.platform_data;
	int i, val, soc;

	for(i=SMB349_INTR_STS_A; i<=SMB349_INTR_STS_F; i++)
	{
		val = smb349_read(chip->client, i);
		if(((i==SMB349_INTR_STS_B) && (val&FAST_CHARGING)) 
			|| ((i==SMB349_INTR_STS_C) && (val&TAPER_CHARGING)))
			chip->state = smb349_progress;
		if((i==SMB349_INTR_STS_C) && (val&TERNIBATION_CHARGING))
			chip->state = smb349_completed;
		if((i==SMB349_INTR_STS_A) && (val&TEMPERATURE_HOT))
			chip->state = smb349_temp_hot_fault;
		if((i==SMB349_INTR_STS_A) && (val&TEMPERATURE_COLD))
			chip->state = smb349_temp_cold_fault;
		if((i==SMB349_INTR_STS_D) && (val&CHARGINGTIMEOUT))
			chip->state = smb349_temp_timeout;
		if((i==SMB349_INTR_STS_F) && !(val&POWER_OK))
			chip->state = smb349_idle;
		if((i==SMB349_INTR_STS_B) && (val&LOW_BATTERY_ALERT))
		{
			printk("[smb349 charger]Low battery alert\n");
			#ifdef MAX77663_CALLBACK_FUNCTION
			soc = bq27541_get_battery_rsoc();
			printk("[bq27541 gauge]low battery soc is %d\n", soc);
			if(soc <= SHUTDOWN_SOC)
			{
				if (chip->max77663_cb[SMB349_LOW_BATTERY])
					chip->max77663_cb[SMB349_LOW_BATTERY](chip->max77663_cb_data[SMB349_LOW_BATTERY]);
			}
			else
			{
				chip->low_battery = chip->low_battery-1;
				smb349_set_low_battery(chip->low_battery);
			}
			#endif
		}
		printk("[0x%02X] : [0x%02X]\n", i, val);
	}
	if(!chip->suspend)
	{
		printk("[smb349 charger]charger status is %d\n", chip->state);
		update_charger_status();
	}
	enable_irq(gpio_to_irq(pdata->stat_gpio));
}

static irqreturn_t smb349_stat_irq_handler(int irq, void *data)
{
	struct smb349_charger *chip = (struct smb349_charger *)data;

	disable_irq_nosync(irq);
	schedule_delayed_work(&chip->smb349_interrupt_delay_work, 10);

	return IRQ_HANDLED;
}

static int smb349_gpio_and_irq_init(struct smb349_charger *chip)
{
	int irq_num, ret;
	struct smb349_charger_platform_data *pdata = chip->client->dev.platform_data;
	irq_num = gpio_to_irq(pdata->stat_gpio);
		
	ret = gpio_request(pdata->stat_gpio, "charge_int");
	if (ret) {
		dev_err(&chip->client->dev, "couldn't request smb349 charger STAT gpio: %d\n",
			pdata->stat_gpio);
		goto err_request;
	}
	ret = gpio_direction_input(pdata->stat_gpio);
	
	ret = request_threaded_irq(pdata->irq_gpio, NULL, smb349_stat_irq_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "smb349_charger", chip);
	if (ret) {			
		dev_err(&chip->client->dev, "can't allocate smb349 state pin's irq %d\n", pdata->stat_gpio);
		goto err_request;		
	}
	
	enable_irq_wake(irq_num);

err_request:
	return ret;

}

static struct regulator_ops smb349_tegra_regulator_ops = {
	.set_current_limit = smb349_enable_charging,
};

static int __devinit smb349_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb349_charger_platform_data *pdata;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	charger->low_battery = 0x0F;

	pdata = client->dev.platform_data;
	i2c_set_clientdata(client, charger);

	/* Check battery presence */
	if (!smb349_battery_online()) {
		dev_err(&client->dev, "%s() No Battery present, exiting..\n",
					__func__);
		ret = -ENODEV;
		goto regulator_error;
	}
	
	charger->reg_desc.name  = "vbus_charger";
	charger->reg_desc.ops   = &smb349_tegra_regulator_ops;
	charger->reg_desc.type  = REGULATOR_CURRENT;
	charger->reg_desc.id    = pdata->regulator_id;
	charger->reg_desc.type  = REGULATOR_CURRENT;
	charger->reg_desc.owner = THIS_MODULE;

	charger->reg_init_data.supply_regulator         = NULL;
	charger->reg_init_data.num_consumer_supplies    =
				       pdata->num_consumer_supplies;
	charger->reg_init_data.regulator_init           = NULL;
	charger->reg_init_data.consumer_supplies        =
				       pdata->consumer_supplies;
	charger->reg_init_data.driver_data              = charger;
	charger->reg_init_data.constraints.name         = "vbus_charger";
	charger->reg_init_data.constraints.min_uA       = 0;
	charger->reg_init_data.constraints.max_uA       =
					pdata->max_charge_current_mA * 1000;

	charger->reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;

	charger->reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_CURRENT;

	charger->rdev = regulator_register(&charger->reg_desc, charger->dev,
					&charger->reg_init_data, charger);
	if (IS_ERR(charger->rdev)) {
		dev_err(&client->dev, "failed to register %s\n",
				charger->reg_desc.name);
		ret = PTR_ERR(charger->rdev);
		goto regulator_error;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	charger->early_suspend.suspend = smb349_early_suspend;
	charger->early_suspend.resume = smb349_late_resume;
	register_early_suspend(&charger->early_suspend);
#endif
	/* disable OTG */
	ret = smb349_configure_otg(client, 0);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring"
			"charger..\n", __func__);
		goto error;
	}
	/*avoid to bootup mis-detection */
	ret = smb349_read(client, SMB349_INTR_STS_B);
	if(ret & FAST_CHARGING)
		charger->state = smb349_progress;
	ret = smb349_read(client, SMB349_INTR_STS_C);
	if(ret & TAPER_CHARGING)
		charger->state = smb349_progress;
	else if(ret & TERNIBATION_CHARGING)
		charger->state = smb349_completed;
	ret = smb349_read(client, SMB349_INTR_STS_F);
	if((ret & 0x01) || charger->state == smb349_progress || charger->state == smb349_completed)
		charger->chrg_type = SMB349_USB;
		
	ret = register_otg_callback(smb349_otg_status, charger);
	if (ret < 0)
		goto error;

#ifdef ADD_ATTRIBUTES_FOR_DIAG
	mutex_init(&charger->mutex);
	ret = sysfs_create_group(&client->dev.kobj, &amb349_attr_group);
	if (ret)
		dev_err(&client->dev, "%s() error in creating sysfs attribute" , __func__);
#endif
#ifdef USB_WAKE_LOCK
	wake_lock_init(&usb_lock, WAKE_LOCK_SUSPEND, "smb349_charger_wake_lock");
#endif
	INIT_DELAYED_WORK(&charger->smb349_interrupt_delay_work, smb349_irq_delay_worker_interrupt);
	smb349_gpio_and_irq_init(charger);

	return 0;
error:
	regulator_unregister(charger->rdev);
regulator_error:
	kfree(charger);
	return ret;
}

static int __devexit smb349_remove(struct i2c_client *client)
{
	struct smb349_charger *charger = i2c_get_clientdata(client);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&charger->early_suspend);
	#endif
	regulator_unregister(charger->rdev);
	#ifdef USB_WAKE_LOCK
	wake_lock_destroy(&usb_lock);
	#endif
	kfree(charger);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void smb349_early_suspend(struct early_suspend *h)
{
	struct smb349_charger *chip = container_of(h, struct smb349_charger, early_suspend);
	mutex_lock(&chip->mutex);
	if(chip->chrg_type >= SMB349_AC)
	{
		chip->cable_exist = 1;
		chip->low_battery = 0x0F;
	}
	else if(chip->chrg_type == SMB349_BATTERY)
	{
		smb349_set_low_battery(chip->low_battery);
	}
	//else if(chip->chrg_type == SMB349_ID_BOOST)
	{
	//else if(tegra_get_host(1))
		//smb349_otg_status(OTG_STATE_A_SUSPEND, OTG_STATE_A_HOST, NULL);
		tegra_get_host(1);
		chip->otg_suspend = 1;
	}
	chip->suspend = 1;
	mutex_unlock(&chip->mutex);
		
	printk("[smb349 charger]Enter early suspend, cable type is %d\n", chip->chrg_type);
}

void smb349_late_resume(struct early_suspend *h)
{
	struct smb349_charger *chip = container_of(h, struct smb349_charger, early_suspend);
	chip->suspend = 0;
	chip->otg_suspend = 0;
	chip->cable_exist = 0;
	update_charger_status();
	#ifdef USB_WAKE_LOCK
	if(chip->chrg_type == SMB349_BATTERY)
	{
		if(wake_lock_active(&usb_lock))
			wake_unlock(&usb_lock);
	}
	#endif
	if(tegra_get_host(0))
		smb349_otg_status(OTG_STATE_A_HOST, OTG_STATE_A_SUSPEND, NULL);
		
	printk("[smb349 charger]Enter late resume, chip->otg_suspend is %d\n", chip->suspend);
}
#endif

#if defined (CONFIG_PM)
static int smb349_charger_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct smb349_charger *chip = i2c_get_clientdata(client);
	struct smb349_charger_platform_data *pdata = chip->client->dev.platform_data;
	int ret;
	ret = cancel_delayed_work_sync(&chip->smb349_interrupt_delay_work);
	printk("[smb349 charger]cancel work is %d\n", ret);
	if(ret)
		enable_irq(pdata->irq_gpio);
	if(chip->chrg_type == SMB349_ID_BOOST)
		smb349_otg_status(OTG_STATE_A_SUSPEND, OTG_STATE_A_HOST, NULL);
	return 0;
}

static int smb349_charger_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static const struct i2c_device_id smb349_id[] = {
	{ "smb349", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb349_id);

static struct i2c_driver smb349_i2c_driver = {
	.driver	= {
		.name	= "smb349",
	},
	.probe		= smb349_probe,
	.remove		= __devexit_p(smb349_remove),
#if defined (CONFIG_PM)
	.suspend = smb349_charger_suspend,
	.resume = smb349_charger_resume,
#endif
	.id_table	= smb349_id,
};

static int __init smb349_init(void)
{
	return i2c_add_driver(&smb349_i2c_driver);
}
subsys_initcall(smb349_init);

static void __exit smb349_exit(void)
{
	i2c_del_driver(&smb349_i2c_driver);
}
module_exit(smb349_exit);

MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("SMB349 Battery-Charger");
MODULE_LICENSE("GPL");
