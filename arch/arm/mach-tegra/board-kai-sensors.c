
/*
 * arch/arm/mach-tegra/board-kai-sensors.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/lis3dh.h>
#include <linux/regulator/consumer.h>
#include <linux/jsa1127.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <media/s5k5cag.h>
#include <media/sensor_yuv.h>
#include "board.h"
#include "board-kai.h"

/* rear camera */
#define CAM1_RST_GPIO			TEGRA_GPIO_PBB3
#define CAM1_POWER_DWN_GPIO		TEGRA_GPIO_PBB5
/* front camera */
#define CAM2_RST_GPIO			TEGRA_GPIO_PBB4
#define CAM2_POWER_DWN_GPIO		TEGRA_GPIO_PBB6

#define CAM_MCLK_EN_GPIO		TEGRA_GPIO_PCC0 //&*&*&*cj_20120612:add for MCLK power sequence

//&*&*&sam0702
#define CAM1_LDO_GPIO	TEGRA_GPIO_PR6
#define CAM2_LDO_GPIO	TEGRA_GPIO_PR7
#define CAM_VDD_GPIO	TEGRA_GPIO_PS0
//&*&*&sam0702


struct kai_cam_gpio {
	int gpio;
	const char *label;
	int value;
	int active_high;
};

#define TEGRA_CAM_GPIO(_gpio, _label, _value, _active_high) \
	{					\
		.gpio = _gpio,			\
		.label = _label,		\
		.value = _value,		\
		.active_high = _active_high	\
	}

struct kai_cam_power_rail {
	struct regulator *cam_1v8_reg;
	struct regulator *cam_2v8_reg;
	struct kai_cam_gpio *gpio_pwdn;
	struct kai_cam_gpio *gpio_rst;
};

enum CAM_INDEX {
	CAM_REAR,
	CAM_FRONT
};

static struct kai_cam_gpio kai_cam_gpio_data[] = {
	[0] = TEGRA_CAM_GPIO(CAM1_POWER_DWN_GPIO, "cam1_power_en", 1, 1),
	[1] = TEGRA_CAM_GPIO(CAM1_RST_GPIO, "cam1_reset", 0, 0),
	[2] = TEGRA_CAM_GPIO(CAM2_POWER_DWN_GPIO, "cam2_power_en", 1, 1),
	[3] = TEGRA_CAM_GPIO(CAM2_RST_GPIO, "cam2_reset", 0, 0),
};

static struct kai_cam_power_rail kai_cam_pwr[] = {
	[0] = {
		.cam_1v8_reg = NULL,
		.cam_2v8_reg = NULL,
		.gpio_pwdn   = &kai_cam_gpio_data[0],
		.gpio_rst    = &kai_cam_gpio_data[1],
	},
	[1] = {
		.cam_1v8_reg = NULL,
		.cam_2v8_reg = NULL,
		.gpio_pwdn   = &kai_cam_gpio_data[2],
		.gpio_rst    = &kai_cam_gpio_data[3],
	},
};

static struct lis3dh_acc_platform_data kai_lis3dh_acc_pdata = {
	.poll_interval	= 200,
	.min_interval	= 1,

	.g_range	= LIS3DH_ACC_G_4G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 1,
	.negate_y	= 1,
	.negate_z	= 0,

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	.gpio_int1	= -EINVAL,
	.gpio_int2	= -EINVAL,
};

static void jsa1127_configure_platform(void)
{
	/* regulator_get regulator_enable*/
}

static struct jsa1127_platform_data kai_jsa1127_pdata = {
    .configure_platform = jsa1127_configure_platform,
};

static struct i2c_board_info kai_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("jsa1127", 0x39),
		.platform_data = &kai_jsa1127_pdata,
	},
	{
		I2C_BOARD_INFO(LIS3DH_ACC_DEV_NAME, 0x18),
		.platform_data = &kai_lis3dh_acc_pdata,
	},
};

static inline void kai_msleep(u32 t)
{
        /*
        If timer value is between ( 10us - 20ms),
        usleep_range() is recommended.
        Please read Documentation/timers/timers-howto.txt.
        */
        usleep_range(t*1000, t*1000 + 500);
}

static int kai_s5kcag_init(void)
{	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];

	printk("[camera](%s) \n",__FUNCTION__);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_3M low
	
	gpio_set_value(CAM_VDD_GPIO, 1);//1.8 high
	
	msleep(1); 
	
	gpio_set_value(CAM1_LDO_GPIO, 1);//2.8v_3M high

	msleep(1);
	
	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	
	msleep(20);		
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_3M high
	
	msleep(20);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_3M low
	
	msleep(20);
		
	gpio_set_value(cam_pwr->gpio_rst->gpio, !cam_pwr->gpio_rst->active_high);//rst_3M high
	
	msleep(20);
	
	
	return 0;
}

static int kai_camera_init(void)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(kai_cam_gpio_data); i++) {
		ret = gpio_request(kai_cam_gpio_data[i].gpio,
				   kai_cam_gpio_data[i].label);

		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, kai_cam_gpio_data[i].gpio);
			goto fail_free_gpio;
		}
		gpio_direction_output(kai_cam_gpio_data[i].gpio,
				      kai_cam_gpio_data[i].value);
		gpio_export(kai_cam_gpio_data[i].gpio, false);
		tegra_gpio_enable(kai_cam_gpio_data[i].gpio);
	}

//&*&*&sam0702
	tegra_gpio_enable(CAM1_LDO_GPIO);
	ret = gpio_request(CAM1_LDO_GPIO, "cam1_ldo_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM1_LDO_GPIO");
			goto fail_cam1_gpio;
	}
	
	tegra_gpio_enable(CAM2_LDO_GPIO);
	ret = gpio_request(CAM2_LDO_GPIO, "cam2_ldo_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM2_LDO_GPIO");
			goto fail_cam2_gpio;
	}
	
	tegra_gpio_enable(CAM_VDD_GPIO);
	ret = gpio_request(CAM_VDD_GPIO, "cam_vdd_en");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "CAM_VDD_GPIO");
			goto fail_vdd_gpio;
	}
	gpio_direction_output(CAM1_LDO_GPIO,0);
//	gpio_set_value(CAM1_LDO_GPIO, 1);
	gpio_direction_output(CAM2_LDO_GPIO,0);
//	gpio_set_value(CAM2_LDO_GPIO, 1);
	gpio_direction_output(CAM_VDD_GPIO,0);
//	gpio_set_value(CAM_VDD_GPIO, 1);
//&*&*&sam0702


//&*&*&*cj1_20120612:add for MCLK power sequence
	ret = gpio_request(CAM_MCLK_EN_GPIO, "mclk_disable");
	if (ret < 0)
	{
			pr_err("CAM_MCLK_EN_GPIO gpio_request failed\n");
			goto fail_mclk_gpio;
	}
	gpio_direction_output(CAM_MCLK_EN_GPIO,0);
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);
//&*&*&*cj2_20120612:add for MCLK power sequence
//&*&*&*cj1_20120528,mod to avoid power on leak current in camera 1.8v pin
	gpio_direction_output(CAM1_POWER_DWN_GPIO,0);
	gpio_set_value(CAM1_POWER_DWN_GPIO, 0);
//	gpio_direction_output(CAM2_POWER_DWN_GPIO,0);
//	gpio_set_value(CAM2_POWER_DWN_GPIO, 0);
//&*&*&*cj2_20120528,mod to avoid power on leak current in camera 1.8v pin
	return 0;

fail_free_gpio:
	pr_err("%s failed!", __func__);
	while(i--)
		gpio_free(kai_cam_gpio_data[i].gpio);
	return ret;

//&*&*&*cj1_20120612:add for MCLK power sequence
fail_mclk_gpio:
	gpio_free(CAM_MCLK_EN_GPIO);
	return ret;
//&*&*&*cj2_20120612:add for MCLK power sequence

//&*&*&sam0702
fail_vdd_gpio:
	gpio_free(CAM_VDD_GPIO);
	return ret;
fail_cam1_gpio:
	gpio_free(CAM1_LDO_GPIO);
	return ret;
fail_cam2_gpio:
	gpio_free(CAM2_LDO_GPIO);
	return ret;
//&*&*&sam0702
		
}

static int kai_cam_power_on(enum CAM_INDEX idx, int delay_time)
{
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[idx];

	printk("[camera](%s)idx=%d, delay_time=%d \n",__FUNCTION__,idx,delay_time);
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);	//&*&*&*cj_20120525,mod for power sequence
//&*&*&*cj1_20120601,mod for power sequence
	if(idx ==0)
	{
//		if (cam_pwr->cam_2v8_reg == NULL) {
//			if (idx == CAM_REAR)
//				cam_pwr->cam_2v8_reg = regulator_get(NULL, "vdd_cam1");
//			else if (idx == CAM_FRONT)
//				cam_pwr->cam_2v8_reg = regulator_get(NULL, "vdd_cam2");
//	
//			if (WARN_ON(IS_ERR(cam_pwr->cam_2v8_reg))) {
//				pr_err("%s: couldn't get regulator vdd_cam: %ld\n",
//					__func__, PTR_ERR(cam_pwr->cam_2v8_reg));
//				goto reg_get_vdd_cam_fail;
//			}
//		}
//		regulator_enable(cam_pwr->cam_2v8_reg);
//		//kai_msleep(1); 
//		msleep(10);	//&*&*&*cj_20120601,mod for rear facing camera CTS stability
//		if (cam_pwr->cam_1v8_reg == NULL) {
//			if (idx == CAM_REAR)
//				cam_pwr->cam_1v8_reg = regulator_get(NULL, "vdd_1v8_cam1");
//			else if (idx == CAM_FRONT)
//				cam_pwr->cam_1v8_reg = regulator_get(NULL, "vdd_1v8_cam2");
//	
//			if (WARN_ON(IS_ERR(cam_pwr->cam_1v8_reg))) {
//				pr_err("%s: couldn't get regulator vdd_1v8_cam: %ld\n",
//					__func__, PTR_ERR(cam_pwr->cam_1v8_reg));
//				goto reg_get_vdd_1v8_cam_fail;
//			}
//		}
//		regulator_enable(cam_pwr->cam_1v8_reg);
	gpio_set_value(CAM1_LDO_GPIO, 1);//2.8v_rear
	kai_msleep(1); 
	gpio_set_value(CAM_VDD_GPIO, 1);//1.8



	}
	else if(idx ==1)
	{
//		if (cam_pwr->cam_1v8_reg == NULL) {
//			if (idx == CAM_REAR)
//				cam_pwr->cam_1v8_reg = regulator_get(NULL, "vdd_1v8_cam1");
//			else if (idx == CAM_FRONT)
//				cam_pwr->cam_1v8_reg = regulator_get(NULL, "vdd_1v8_cam2");
//	
//			if (WARN_ON(IS_ERR(cam_pwr->cam_1v8_reg))) {
//				pr_err("%s: couldn't get regulator vdd_1v8_cam: %ld\n",
//					__func__, PTR_ERR(cam_pwr->cam_1v8_reg));
//				goto reg_get_vdd_1v8_cam_fail;
//			}
//		}
//		regulator_enable(cam_pwr->cam_1v8_reg);
//		//kai_msleep(1); 
//		msleep(10); 	//&*&*&*cj_20120601,mod for rear facing camera CTS stability
//		if (cam_pwr->cam_2v8_reg == NULL) {
//			if (idx == CAM_REAR)
//				cam_pwr->cam_2v8_reg = regulator_get(NULL, "vdd_cam1");
//			else if (idx == CAM_FRONT)
//				cam_pwr->cam_2v8_reg = regulator_get(NULL, "vdd_cam2");
//	
//			if (WARN_ON(IS_ERR(cam_pwr->cam_2v8_reg))) {
//				pr_err("%s: couldn't get regulator vdd_cam: %ld\n",
//					__func__, PTR_ERR(cam_pwr->cam_2v8_reg));
//				goto reg_get_vdd_cam_fail;
//			}
//		}
//		regulator_enable(cam_pwr->cam_2v8_reg);
	
	gpio_set_value(CAM_VDD_GPIO, 1);//1.8v
	kai_msleep(1); 
	gpio_set_value(CAM2_LDO_GPIO, 1);//2.8v_front
	
	}

//&*&*&*cj1_20120612:add for MCLK power sequence
	msleep(1);
	tegra_gpio_disable(CAM_MCLK_EN_GPIO); 
//&*&*&*cj2_20120612:add for MCLK power sequence
//&*&*&*cj2_20120601,mod for power sequence
	if(idx ==CAM_REAR)
		msleep(delay_time);
	else
		msleep(10);
//&*&*&*cj1_20120525,mod for power sequence
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);
	//kai_msleep(2); 
	if(idx ==CAM_REAR)
		msleep(delay_time);
	else
		msleep(10);

//&*&*&*cj1_20120601,mod for LH HW power sequence
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);
	//kai_msleep(2); 
	if(idx ==CAM_REAR)
		msleep(delay_time);
	else
		msleep(10);
		
	gpio_set_value(cam_pwr->gpio_rst->gpio, !cam_pwr->gpio_rst->active_high);
	//kai_msleep(2); 
	if(idx ==CAM_REAR)
		msleep(delay_time);
	else
		msleep(10);
	
//&*&*&*cj2_20120601,mod for LH HW power sequence
//&*&*&*cj2_20120525,mod for power sequence
	return 0;

//reg_get_vdd_1v8_cam_fail:
//	cam_pwr->cam_1v8_reg = NULL;
//	regulator_put(cam_pwr->cam_2v8_reg);
//
//reg_get_vdd_cam_fail:
//	cam_pwr->cam_2v8_reg = NULL;

	return -ENODEV;
}

static int kai_cam_power_off(enum CAM_INDEX idx)
{
	printk("[camera](%s)idx=%d \n",__FUNCTION__,idx);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[idx];

	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);
	//kai_msleep(2); //&*&*&*cj_20120525,mod for power sequence
	msleep(20);	//&*&*&*cj_20120601,mod for camera CTS stability
	gpio_set_value(cam_pwr->gpio_rst->gpio, cam_pwr->gpio_rst->active_high);

	msleep(1);	//&*&*&*cj_20120614,mod for camera power down sequence.
//&*&*&*cj1_20120612:add for MCLK power sequence
//	tegra_gpio_enable(CAM_MCLK_EN_GPIO);
//	gpio_set_value(CAM_MCLK_EN_GPIO, 0);
//&*&*&*cj2_20120612:add for MCLK power sequence
//	if (cam_pwr->cam_1v8_reg)
//		regulator_disable(cam_pwr->cam_1v8_reg);
	gpio_set_value(CAM_VDD_GPIO, 0);
//	//kai_msleep(110); //&*&*&*cj_20120601,mod for power sequence
//	msleep(200);	//&*&*&*cj_20120601,mod for camera CTS stability
//	if (cam_pwr->cam_2v8_reg)
//		regulator_disable(cam_pwr->cam_2v8_reg);
	gpio_set_value(CAM1_LDO_GPIO, 0);
	gpio_set_value(CAM2_LDO_GPIO, 0);

	//kai_msleep(2); //&*&*&*cj_20120525,mod for power sequence
	msleep(20);	//&*&*&*cj_20120601,mod for camera CTS stability
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);	//&*&*&*cj_20120525,mod for power sequence

	return 0;
}

#if 0
static int kai_s5k5cag_power_on(int delay_time)
{
	return kai_cam_power_on(CAM_REAR,delay_time);
}

static int kai_s5k5cag_power_off(void)
{
	return kai_cam_power_off(CAM_REAR);
}

static int kai_mt9m114_power_on(int delay_time)
{
	return kai_cam_power_on(CAM_FRONT,0);
}

static int kai_mt9m114_power_off(void)
{
	return kai_cam_power_off(CAM_FRONT);
}
#else
static int kai_s5k5cag_power_on(int delay_time)
{
	printk("[Jimmy][camera](%s) \n",__FUNCTION__);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];	
	
	msleep(1);
	
	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	
	msleep(10);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_3M low
	
	msleep(20);
	
	return 0;
}

static int kai_s5k5cag_power_off(void)
{
	printk("[Jimmy][camera](%s) \n",__FUNCTION__);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_3M high

	msleep(1);	
	
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low
	
	msleep(20);
	
	return 0;
}

static int kai_s5k5cag_suspend(void)
{
	printk("[Jimmy][camera](%s) \n",__FUNCTION__);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_REAR];	

	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_3M high
	
	msleep(20);
	
	gpio_set_value(cam_pwr->gpio_rst->gpio, cam_pwr->gpio_rst->active_high);//rst_3M low

	msleep(1);
	
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low

	gpio_set_value(CAM_VDD_GPIO, 0);//1.8v low
	
	gpio_set_value(CAM1_LDO_GPIO, 0);//2.8v_3M low

	msleep(20);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_3M low
	
	return 0;
}

static int kai_mt9m114_power_on(int delay_time)
{
	printk("[Jimmy][camera](%s) \n",__FUNCTION__);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_FRONT];
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_1.3M low

	gpio_set_value(CAM2_LDO_GPIO, 1);//2.8v_1.3M high
	
	msleep(1);
	
	tegra_gpio_disable(CAM_MCLK_EN_GPIO);//gpio->clk
	
	msleep(10);
		
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_1.3M high
	
	msleep(10);

	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_1.3M low
	
	msleep(10);
		
	gpio_set_value(cam_pwr->gpio_rst->gpio, !cam_pwr->gpio_rst->active_high);//rst_1.3M high
	
	msleep(10);
			
	return 0;
}

static int kai_mt9m114_power_off(void)
{
	printk("[Jimmy][camera](%s) \n",__FUNCTION__);
	
	struct kai_cam_power_rail *cam_pwr = &kai_cam_pwr[CAM_FRONT];	

	gpio_set_value(cam_pwr->gpio_pwdn->gpio, cam_pwr->gpio_pwdn->active_high);//pwdn_1.3M high
	
	msleep(20);
	
	gpio_set_value(cam_pwr->gpio_rst->gpio, cam_pwr->gpio_rst->active_high);//rst_1.3M low

	msleep(1);
	
	tegra_gpio_enable(CAM_MCLK_EN_GPIO);//clk->gpio
	
	gpio_set_value(CAM_MCLK_EN_GPIO, 0);//gpio -> low

	gpio_set_value(CAM2_LDO_GPIO, 0);//2.8v_1.3M low

	msleep(20);
	
	gpio_set_value(cam_pwr->gpio_pwdn->gpio, !cam_pwr->gpio_pwdn->active_high);//pwdn_1.3M low
	
	return 0;
}
#endif

struct yuv_sensor_platform_data kai_s5k5cag_data = {
	.init = kai_s5kcag_init,
	.suspend = kai_s5k5cag_suspend,
	.power_on = kai_s5k5cag_power_on,
	.power_off = kai_s5k5cag_power_off,
};

struct yuv_sensor_platform_data kai_mt9m114_data = {
	.power_on = kai_mt9m114_power_on,
	.power_off = kai_mt9m114_power_off,
};

static struct i2c_board_info kai_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("s5k5cag", 0x2d),
		.platform_data = &kai_s5k5cag_data,
	},
	{
		I2C_BOARD_INFO("mt9m114", 0x48),
		.platform_data = &kai_mt9m114_data,
	},
};

int __init kai_sensors_init(void)
{
	kai_camera_init();
	kai_s5kcag_init();
	kai_s5k5cag_power_off();
	
	i2c_register_board_info(2, kai_i2c2_board_info,
		ARRAY_SIZE(kai_i2c2_board_info));

	i2c_register_board_info(0, kai_i2c0_board_info,
		ARRAY_SIZE(kai_i2c0_board_info));

	return 0;
}
