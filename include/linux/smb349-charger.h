/*
 * include/linux/smb349-charger.h
 *
 * Battery charger driver interface for Summit SMB349
 *
 * Copyright (C) 2012 NVIDIA Corporation
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
 *
 */

#ifndef __LINUX_SMB349_CHARGER_H
#define __LINUX_SMB349_CHARGER_H

#include <linux/regulator/machine.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/usb/otg.h>
#include <linux/power_supply.h>
#include <linux/earlysuspend.h>

#define ADD_ATTRIBUTES_FOR_DIAG
#define MAX77663_CALLBACK_FUNCTION

struct smb349_charger_platform_data {
	int regulator_id;
	int max_charge_volt_mV;
	int max_charge_current_mA;
	int charging_term_current_mA;
	int num_consumer_supplies;
	struct regulator_consumer_supply *consumer_supplies;
	int stat_gpio;
	int irq_gpio;
	u8 configuration_data[17];
};

enum charging_states {
	smb349_idle =0,
	smb349_progress,
	smb349_completed,
	smb349_temp_hot_fault,
	smb349_temp_cold_fault,
	smb349_temp_timeout,
	smb349_battery_fault,
	smb349_stopped,
};

enum charger_type {
	SMB349_BATTERY = 0,
	SMB349_VBUS_VALID,
	SMB349_ID_BOOST,
	SMB349_AC,
	SMB349_USB,
};

typedef void (*charging_callback_t)(enum charging_states state,
enum charger_type chrg_type, void *args);

#ifdef MAX77663_CALLBACK_FUNCTION
enum max77663_callback_type {
	SMB349_LOW_BATTERY = 0,
	SMB349_OTG_CABLE,
};

typedef void (*max77663_callback_t)(void *args);
#endif

struct smb349_charger {
	struct i2c_client	*client;
	struct device	*dev;
	void	*charger_cb_data;
	int state;
	int chrg_type;
	charging_callback_t	charger_cb;
	#ifdef MAX77663_CALLBACK_FUNCTION
	max77663_callback_t max77663_cb[2];
	void	*max77663_cb_data[2];
	#endif
	struct regulator_dev    *rdev;
	struct regulator_desc   reg_desc;
	struct regulator_init_data      reg_init_data;
	#ifdef ADD_ATTRIBUTES_FOR_DIAG
	struct mutex mutex;	/* reentrant protection for struct */
	#endif
	struct power_supply		ac;
	struct power_supply		usb;
	struct power_supply		battery;
	struct delayed_work		smb349_interrupt_delay_work;
	struct early_suspend early_suspend;
	bool suspend;
	bool cable_exist;
	int low_battery;
	bool otg_suspend;
};

int smb349_battery_online(void);
typedef void (*callback_t)(enum usb_otg_state to,
		enum usb_otg_state from, void *args);

/*
 * Register callback function for the client.
 * Used by fuel-gauge driver to get battery charging properties.
 */
#ifdef MAX77663_CALLBACK_FUNCTION
extern int register_max77663_callback(max77663_callback_t cb, void *args, enum max77663_callback_type);
#endif
extern int register_callback(charging_callback_t cb, void *args);
extern int register_otg_callback(callback_t cb, void *args);
extern int update_charger_status(void);

#endif /*__LINUX_SMB349_CHARGER_H */
