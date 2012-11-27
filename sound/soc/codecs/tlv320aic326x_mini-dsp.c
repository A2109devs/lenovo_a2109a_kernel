/*
 * linux/sound/soc/codecs/tlv320aic326x_mini-dsp.c
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * The TLV320AIC3262 is a flexible, low-power, low-voltage stereo audio
 * codec with digital microphone inputs and programmable outputs.
 *
 * History:
 *
 * Rev 0.1   Added the miniDSP Support	 01-03-2011
 *
 * Rev 0.2   Updated the code-base for miniDSP switching and
 *	 mux control update.	21-03-2011
 *
 * Rev 0.3   Updated the code-base to support Multi-Configuration feature
 *		   of PPS GDE
 */

/*
 *****************************************************************************
 * INCLUDES
 *****************************************************************************
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/soc-dapm.h>
#include <sound/control.h>
#include <linux/time.h>		/* For timing computations */
#include <sound/tlv320aic326x.h>
#include "tlv320aic326x.h"
#include "tlv320aic326x_mini-dsp.h"


#include "base_main_Rate48_pps_driver.h"
#include "one_mic_aec_nc_latest.h"
#include "rec_2mic_nc_latest.h"

//#include "simple_process.h"

#ifdef CONFIG_MINI_DSP

/*
 *****************************************************************************
 * LOCAL STATIC DECLARATIONS
 *****************************************************************************
 */
static int m_control_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo);
static int m_control_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol);
static int m_control_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol);

/*
 *****************************************************************************
 * MINIDSP RELATED GLOBALS
 *****************************************************************************
 */
/* The below variable is used to maintain the I2C Transactions
 * to be carried out during miniDSP switching.
 */
 #if 1
minidsp_parser_data dsp_parse_data[MINIDSP_PARSER_ARRAY_SIZE*2];

struct i2c_msg i2c_transaction[MINIDSP_PARSER_ARRAY_SIZE * 2];

spi_buf spi_data[MINIDSP_PARSER_ARRAY_SIZE*2];

/* Total count of I2C Messages are stored in the i2c_count */
int i2c_count;

/* The below array is used to store the burst array for I2C Multibyte
 * Operations
 */
minidsp_i2c_page i2c_page_array[MINIDSP_PARSER_ARRAY_SIZE];
int i2c_page_count;
#else
minidsp_parser_data dsp_parse_data;

struct i2c_msg i2c_transaction;
/* Total count of I2C Messages are stored in the i2c_count */
int i2c_count;

/* The below array is used to store the burst array for I2C Multibyte
 * Operations
 */
minidsp_i2c_page i2c_page_array;
int i2c_page_count;
#endif

/* kcontrol structure used to register with ALSA Core layer */
static struct snd_kcontrol_new snd_mux_controls[MAX_MUX_CONTROLS];

/* mode variables */
static int amode;
static int dmode;

/* k-control macros used for miniDSP related Kcontrols */
#define SOC_SINGLE_VALUE_M(xmax, xinvert) \
	((unsigned long)&(struct soc_mixer_control) \
	{.max = xmax, \
	.invert = xinvert})
#define SOC_SINGLE_M(xname, max, invert) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = m_control_info, .get = m_control_get,\
	.put = m_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.private_value = SOC_SINGLE_VALUE_M(max, invert) }
#define SOC_SINGLE_AIC3262_M(xname) \
{\
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = m_control_info, .get = m_control_get,\
	.put = m_control_put, \
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
}

/*
 * aic3262_minidsp_controls
 *
 * Contains the list of the Kcontrol macros required for modifying the
 * miniDSP behavior at run-time.
 */
static const struct snd_kcontrol_new aic3262_minidsp_controls[] = {
	SOC_SINGLE_AIC3262_M("Minidsp mode") ,
	SOC_SINGLE_AIC3262_M("ADC Adaptive mode Enable") ,
	SOC_SINGLE_AIC3262_M("DAC Adaptive mode Enable") ,
	SOC_SINGLE_AIC3262_M("Dump Regs Book0") ,
	SOC_SINGLE_AIC3262_M("Verify minidsp program") ,
	SOC_SINGLE_AIC3262_M("Minidsp patch") ,

};

int dsp_reg_write (struct snd_soc_codec *codec , u8 book, unsigned int reg, unsigned int val)
{
	u8 buf[2];
	int ret=0;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
	buf[AIC3262_REG_OFFSET_INDEX] = reg;
	buf[AIC3262_REG_DATA_INDEX] = val;
	if(aic3262->bus_type == SND_SOC_SPI) {
		if (codec->hw_write(codec, buf, 2) < 0) {
			printk("error in Hw write \n");
			mutex_unlock(&codec_io_mutex);
			ret= -EIO;
		}
	} else {
		if(codec->hw_write(codec, buf, 2) != 2) {
			printk(KERN_ERR"error in Hw write \n");
			mutex_unlock(&codec_io_mutex);
			ret= -EIO;
		}
	}

	aic3262_change_book(codec, 0);
	mutex_unlock(&codec_io_mutex);
	return ret;

}



unsigned int dsp_reg_read (struct snd_soc_codec *codec , u8 book, unsigned int reg)
{

	u8 val;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
	if(aic3262->bus_type == SND_SOC_SPI) {
		val = codec->hw_read(codec,reg);
	}else{
		val = i2c_smbus_read_byte_data(codec->control_data, reg);
	}

	aic3262_change_book(codec, 0);
	mutex_unlock(&codec_io_mutex);
	return val;
}

int dsp_reg_update_bits(struct snd_soc_codec *codec , u8 book, unsigned int reg,
										unsigned int mask,unsigned int value)
{
	int change_bit;
	unsigned int  old_bit, new_bit;
	int ret =0;
	u8 buf[2];
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	mutex_lock(&codec_io_mutex);
	aic3262_change_book(codec, book);
	if(aic3262->bus_type == SND_SOC_SPI) {
		ret=codec->hw_read(codec,reg);
	}else{
		ret = i2c_smbus_read_byte_data(codec->control_data, reg);
	}

	if(ret <0)
		goto out;
	old_bit=ret;
	new_bit=(old_bit&~ mask) | value;
	change_bit = old_bit != new_bit;
	if(change_bit){
		buf[AIC3262_REG_OFFSET_INDEX] = reg;
		buf[AIC3262_REG_DATA_INDEX] = new_bit;
		if(aic3262->bus_type == SND_SOC_SPI) {
			if (codec->hw_write(codec, buf, 2) < 0) {
			printk(KERN_ERR"error in Hw write \n");
			mutex_unlock(&codec_io_mutex);
			ret= -EIO;
				}
		}else{
			if(codec->hw_write(codec, buf, 2) != 2) {
			printk(KERN_ERR"error in Hw write \n");
			mutex_unlock(&codec_io_mutex);
			ret= -EIO;
				}
			}

		}
out:
	aic3262_change_book(codec,0);
	mutex_unlock(&codec_io_mutex);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3262_dump_page
 * Purpose  : Read and display one codec register page, for debugging purpose
 *----------------------------------------------------------------------------
 */
void aic3262_dump_page_reg(struct snd_soc_codec *codec, u8 page)
{
	int i;
	u8 data;
	u8 test_page_array[256];
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	mutex_lock(&codec_io_mutex);
	aic3262_change_page(codec, page);

	data = 0x0;
	if(aic3262->bus_type == SND_SOC_SPI) {
		for (i=0;i<128;i++) {

			test_page_array[i] = codec->hw_read(codec,i);

		}
	} else {

		i2c_master_send(codec->control_data, &data, 1);
		i2c_master_recv(codec->control_data, test_page_array, 128);

	}
	mutex_unlock(&codec_io_mutex);
	DBG("\n------- MINI_DSP PAGE %d DUMP --------\n", page);
	for (i = 0; i < 128; i++)
		DBG(KERN_INFO " [ %d ] = 0x%x\n", i, test_page_array[i]);

}



/*
 *----------------------------------------------------------------------------
 * Function : byte_i2c_array_transfer
 * Purpose  : Function used only for debugging purpose. This function will
 *			be used while switching miniDSP Modes register by register.
 *			This needs to be used only during development.
 *-----------------------------------------------------------------------------
 */

int byte_i2c_array_transfer(struct snd_soc_codec *codec,
				reg_value *program_ptr,
				int size)
{
	int j;
	u8 buf[3];
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	/* lock added */
	mutex_lock(&codec_io_mutex);
	for (j = 0; j < size; j++) {
		/* Check if current Reg offset is zero */
		if (program_ptr[j].reg_off == 0) {
			/* Check for the Book Change Request */
			//printk(KERN_INFO "inside if 1 j =%d\n", j);
			if ((j < (size - 1)) &&
				(program_ptr[j+1].reg_off == 127)) {
				aic3262_change_book(codec,
					program_ptr[j+1].reg_val);
			/* Increment for loop counter across Book Change */
				j++;
				continue;
		}
		/* Check for the Page Change Request in Current book */
		aic3262_change_page(codec, program_ptr[j].reg_val);
		continue;
		}

		buf[AIC3262_REG_OFFSET_INDEX] = program_ptr[j].reg_off % 128;
		buf[AIC3262_REG_DATA_INDEX] =
				program_ptr[j].reg_val & AIC3262_8BITS_MASK;
		if(aic3262->bus_type == SND_SOC_SPI) {
			if (codec->hw_write(codec, buf, 2) < 0) {
				mutex_unlock(&codec_io_mutex);
				printk(KERN_ERR "Error in spi write\n");
				return -EIO;
			}
		} else {
			if (codec->hw_write(codec, buf, 2) != 2) {
				mutex_unlock(&codec_io_mutex);
				printk(KERN_ERR "Error in i2c write\n");
				return -EIO;
			}
		}
	}
	aic3262_change_book(codec, 0);
	/*added unlock*/
	mutex_unlock(&codec_io_mutex);
	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : byte_i2c_array_read
 * Purpose  : This function is used to perform Byte I2C Read. This is used
 *			only for debugging purposes to read back the Codec Page
 *			Registers after miniDSP Configuration.
 *----------------------------------------------------------------------------
 */
int byte_i2c_array_read(struct snd_soc_codec *codec, reg_value *program_ptr, int size)
{
	int j;
	u8 val1;
	u8 cur_page = 0;
	u8 cur_book = 0;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	/* lock added */
	mutex_lock(&codec_io_mutex);
	for (j = 0; j < size; j++) {
		/* Check if current Reg offset is zero */
		if (program_ptr[j].reg_off == 0) {
			/* Check for the Book Change Request */
			if ((j < (size - 1)) &&
				(program_ptr[j+1].reg_off == 127)) {
				aic3262_change_book(codec,
					program_ptr[j+1].reg_val);
				cur_book = program_ptr[j+1].reg_val;
			/* Increment for loop counter across Book Change */
				j++;
				continue;
			}
			/* Check for the Page Change Request in Current book */
			aic3262_change_page(codec, program_ptr[j].reg_val);
			cur_page = program_ptr[j].reg_val;
			continue;
		}

		if(aic3262->bus_type == SND_SOC_SPI) {
			val1 = codec->hw_read(codec, program_ptr[j].reg_off);
		} else {
			val1 = i2c_smbus_read_byte_data(codec->control_data,
					program_ptr[j].reg_off);
		}
		if (val1 < 0)
			printk(KERN_ERR "Error in codec read\n");

		if(val1 != program_ptr[j].reg_val)
			/*printk(KERN_INFO "mismatch [%d][%d][%d] = %x %x\n",
			cur_book, cur_page, program_ptr[j].reg_off, val1, program_ptr[j].reg_val);*/
		DBG(KERN_INFO "[%d][%d][%d]= %x\n",
			cur_book, cur_page, program_ptr[j].reg_off, val1);
	}
	aic3262_change_book(codec, 0);
	/*unlocking*/
	mutex_unlock(&codec_io_mutex);
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : minidsp_get_burst
 * Purpose  : Format one I2C burst for transfer from mini dsp program array.
 *			This function will parse the program array and get next burst
 *			data for doing an I2C bulk transfer.
 *----------------------------------------------------------------------------
 */
static void
minidsp_get_burst(struct snd_soc_codec *codec, reg_value *program_ptr,
				int program_size,
				minidsp_parser_data *parse_data)
{
	int index = parse_data->current_loc;
	int burst_write_count = 0;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	/*DBG("GET_BURST: start\n");*/
	/* check if first location is page register, and populate page addr */
	if (program_ptr[index].reg_off == 0) {
		if ((index < (program_size - 1)) &&
			(program_ptr[index+1].reg_off == 127)) {
			parse_data->book_change = 1;
			parse_data->book_no = program_ptr[index+1].reg_val;
			index += 2;
			goto finish_out;

		}
		parse_data->page_num = program_ptr[index].reg_val;

		if(aic3262->bus_type == SND_SOC_SPI) {
			parse_data->burst_array[burst_write_count++] =
				(program_ptr[index].reg_off << 1);
		} else {
			parse_data->burst_array[burst_write_count++] =
				program_ptr[index].reg_off;
		}

		parse_data->burst_array[burst_write_count++] =
			program_ptr[index].reg_val;
		index++;
		goto finish_out;
	}

	if(aic3262->bus_type == SND_SOC_SPI) {
		parse_data->burst_array[burst_write_count++] =
			(program_ptr[index].reg_off << 1);
	} else {
		parse_data->burst_array[burst_write_count++] =
			program_ptr[index].reg_off;
	}

	parse_data->burst_array[burst_write_count++] =
			program_ptr[index].reg_val;
	index++;

	for (; index < program_size; index++) {
		if (program_ptr[index].reg_off !=
				(program_ptr[index - 1].reg_off + 1))
			break;
		else
			parse_data->burst_array[burst_write_count++] =
				program_ptr[index].reg_val;

	}
finish_out:
	parse_data->burst_size = burst_write_count;
	if (index == program_size)
		/* parsing completed */
		parse_data->current_loc = MINIDSP_PARSING_END;
	else
		parse_data->current_loc = index;
	/*DBG("GET_BURST: end\n");*/
}
/*
 *----------------------------------------------------------------------------
 * Function : minidsp_i2c_multibyte_transfer
 * Purpose  : Function used to perform multi-byte I2C Writes. Used to configure
 *			the miniDSP Pages.
 *----------------------------------------------------------------------------
 */
int
minidsp_i2c_multibyte_transfer(struct snd_soc_codec *codec,
					reg_value *program_ptr,
					int program_size)
{
	struct i2c_client *client = codec->control_data;
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	minidsp_parser_data parse_data;
	int count = 0;

#ifdef DEBUG_MINIDSP_LOADING
	int i = 0, j = 0;
#endif
	/* point the current location to start of program array */
	parse_data.current_loc = 0;
	parse_data.page_num = 0;
	parse_data.book_change = 0;
	parse_data.book_no = 0;
	/*lock added*/
	mutex_lock(&codec_io_mutex);

	DBG(KERN_INFO "size is : %d", program_size);
	do {
		do {
			/* Get first burst data */
			minidsp_get_burst(codec,program_ptr, program_size,
					&parse_data);
			if (parse_data.book_change == 1)
				break;
			dsp_parse_data[count] = parse_data;
			if(aic3262->bus_type == SND_SOC_SPI) {
				spi_data[count].buf = dsp_parse_data[count].burst_array;
				spi_data[count].len = dsp_parse_data[count].burst_size;
				spi_cs_en(0, aic3262->pdata->cspin);
				//codec->hw_write(codec, spi_data[count].buf, spi_data[count].len);
				spi_write(codec->control_data, spi_data[count].buf, spi_data[count].len);
				spi_cs_en(1, aic3262->pdata->cspin);
			} else {
			i2c_transaction[count].addr = client->addr;
			i2c_transaction[count].flags =
				client->flags & I2C_M_TEN;
			i2c_transaction[count].len =
				dsp_parse_data[count].burst_size;
			i2c_transaction[count].buf =
				dsp_parse_data[count].burst_array;
			}

#ifdef DEBUG_MINIDSP_LOADING
			DBG(KERN_INFO
			"i: %d\taddr: %d\tflags: %d\tlen: %d\tbuf:",
			i, client->addr, client->flags & I2C_M_TEN,
			dsp_parse_data[count].burst_size);

			for (j = 0; j <= dsp_parse_data[count].burst_size; j++)
				DBG(KERN_INFO "%x ",
					dsp_parse_data[i].burst_array[j]);

			DBG(KERN_INFO "\n\n");
			i++;
#endif

			count++;
			/* Proceed to the next burst reg_addr_incruence */
		} while (parse_data.current_loc != MINIDSP_PARSING_END);
		if(aic3262->bus_type == SND_SOC_I2C) {
			if (count > 0) {
				if (i2c_transfer(client->adapter,
					i2c_transaction, count) != count) {
					printk(KERN_ERR "Write burst i2c data error!\n");
				}
			}
		}
		if (parse_data.book_change == 1) {
			aic3262_change_book(codec, parse_data.book_no);
			parse_data.book_change = 0;
		}
	} while (parse_data.current_loc != MINIDSP_PARSING_END);
	aic3262_change_book(codec, 0);
	/*unlock added*/
	mutex_unlock(&codec_io_mutex);
	return 0;
}

/*
* Process_Flow Structure
* Structure used to maintain the mapping of each PFW like the miniDSP_A
* miniDSP_D array values and sizes. It also contains information about
* the patches required for each patch.
*/
struct process_flow{
	int init_size;
	reg_value *miniDSP_init;
	int A_size;
	reg_value *miniDSP_A_values;
	int D_size;
	reg_value *miniDSP_D_values;
	int post_size;
	reg_value *miniDSP_post;
	struct minidsp_config {
		int a_patch_size;
		reg_value *a_patch;
		int d_patch_size;
		reg_value *d_patch;
	} configs[MAXCONFIG];

} miniDSP_programs[]  = {
  	{

	ARRAY_SIZE(main44_REG_Section_init_program), main44_REG_Section_init_program,
  	ARRAY_SIZE(main44_miniDSP_A_reg_values),main44_miniDSP_A_reg_values,
  	ARRAY_SIZE(main44_miniDSP_D_reg_values),main44_miniDSP_D_reg_values,
  	ARRAY_SIZE(main44_REG_Section_post_program),main44_REG_Section_post_program,


  	{

  		{ ARRAY_SIZE(handset_miniDSP_A_reg_values), handset_miniDSP_A_reg_values,
			ARRAY_SIZE(handset_miniDSP_D_reg_values),handset_miniDSP_D_reg_values },
		{ ARRAY_SIZE(headphone_miniDSP_A_reg_values),headphone_miniDSP_A_reg_values ,
			ARRAY_SIZE(headphone_miniDSP_D_reg_values),headphone_miniDSP_D_reg_values },
		{  ARRAY_SIZE(speaker_miniDSP_A_reg_values),speaker_miniDSP_A_reg_values,
			ARRAY_SIZE(speaker_miniDSP_D_reg_values),speaker_miniDSP_D_reg_values },
		{ 0, 0, 0, 0},

	},
},

	{
	ARRAY_SIZE(base_speaker_SRS_REG_init_Section_program),base_speaker_SRS_REG_init_Section_program,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_A_reg_values),base_speaker_SRS_miniDSP_A_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_miniDSP_D_reg_values),base_speaker_SRS_miniDSP_D_reg_values,
  	ARRAY_SIZE(base_speaker_SRS_REG_post_Section_program),base_speaker_SRS_REG_post_Section_program,




	{
			{0,	0,ARRAY_SIZE(SRS_ON_miniDSP_D_reg_values),SRS_ON_miniDSP_D_reg_values},
			{0,	0,ARRAY_SIZE(SRS_OFF_miniDSP_D_reg_values),SRS_OFF_miniDSP_D_reg_values},
			{0,0,0,0},
			{0,0,0,0},
	},
},

	{

	ARRAY_SIZE(rec_2nc_REG_Section_init_program),rec_2nc_REG_Section_init_program,
  	ARRAY_SIZE(rec_2nc_miniDSP_A_reg_values),rec_2nc_miniDSP_A_reg_values,
  	ARRAY_SIZE(rec_2nc_miniDSP_D_reg_values),rec_2nc_miniDSP_D_reg_values,
  	ARRAY_SIZE(rec_2nc_REG_Section_post_program),rec_2nc_REG_Section_post_program,

	{
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},

		},
	},

};







int change_codec_power_status(struct snd_soc_codec *codec,
						int off_restore, int power_mask)
{
	int minidsp_power_mask;
	u8 dac_status, adc_status;


	minidsp_power_mask=0;

	switch(off_restore) {

		case 0:/*for powering off*/

				dac_status = snd_soc_read(codec, DAC_FLAG_R1);
				/*power down dac only if dac is already on*/
				/*left dac*/
				if(dac_status & 0x80){
					minidsp_power_mask |= 0x01;
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x80, 0x00);
				}
				/*right dac*/
				if(dac_status & 0x08){
					minidsp_power_mask |= 0x02;
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x40, 0x00);
				}

				/*Switching off Both ADC and Disabling the Soft stepping*/
				codec->write(codec,ADC_CHANNEL_POW,0x02);
				mdelay(5);

				/*check power status*/
				if(dac_status & 0x80){
					poll_dac(codec, 0, 0);
				}
				if(dac_status & 0x08){
					poll_dac(codec, 1, 0);
				}
				if(adc_status & 0x40) {
					poll_adc(codec, 0, 0);
				}
				if(adc_status & 0x04) {
					poll_adc(codec, 1, 0);
		}
				break;
		case 1:/*for restoring minidsp to former state as per bit mask*/
				if(power_mask & 0x01) {
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x80, 0x80);
				}
				if(power_mask & 0x02) {
					snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0x40, 0x40);
				}
				if(power_mask & 0x04) {
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x80);
				}
				if(power_mask & 0x08) {
					snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x40, 0x40);
				}
				/*check power satatus*/
				if(power_mask & 0x01) {
					poll_dac(codec, 0, 1);
				}
				if(power_mask & 0x02) {
					poll_dac(codec, 1, 1);
		}
				if(power_mask & 0x04) {
					poll_adc(codec, 0, 1);
				}
				if(power_mask & 0x08) {
					poll_adc(codec, 1, 1);
	}
				break;
		default:
				printk(KERN_ERR "%s:Unknown power state requested\n", __func__);

	};

	return minidsp_power_mask;

		}
/*
 *----------------------------------------------------------------------------
 * Function  : boot_minidsp
 * Purpose  : for laoding the default minidsp mode for the first time .
 *----------------------------------------------------------------------------
 */
int
boot_minidsp(struct snd_soc_codec *codec, int new_mode, int new_config)
{
	struct aic3262_priv *aic326x = snd_soc_codec_get_drvdata(codec);
	struct process_flow *  pflows = &miniDSP_programs[new_mode];
	int minidsp_stat;
	u8 pfw_changed;
	int (*ptransfer)(struct snd_soc_codec *codec,
				reg_value *program_ptr,
				int size);

	DBG("%s: switch  mode start\n", __func__);
	if (new_mode >= ARRAY_SIZE(miniDSP_programs))
		return 0; //  error condition
	//if (new_mode == aic326x->process_flow)
		//return 0;

	printk("=====changing processflow from %d to %d=====",aic326x->process_flow,new_mode);
	mutex_lock(&codec_io_mutex);
	aic3262_change_book( codec,0);
	mutex_unlock(&codec_io_mutex);

#ifndef MULTIBYTE_I2C
	ptransfer = byte_i2c_array_transfer;
#else
	ptransfer = minidsp_i2c_multibyte_transfer;
#endif

if (new_mode !=  aic326x->process_flow) {

	minidsp_stat = change_codec_power_status (codec, 0x0, 0x3);

	ptransfer(codec, pflows->miniDSP_init,		   pflows->init_size);
	ptransfer(codec, pflows->miniDSP_A_values,  pflows->A_size);
	ptransfer(codec, pflows->miniDSP_D_values,  pflows->D_size);
	ptransfer(codec, pflows->miniDSP_post,		 pflows->post_size);

	if(new_mode == 1) {
		snd_soc_update_bits(codec,MINIDSP_REG, 0xF0, 0xF0);
	}

	aic326x->process_flow = new_mode;
	pfw_changed=1;

	if (new_mode == 0 || new_mode == 2) {
		snd_soc_update_bits(codec,ASI3_ADC_CTRL_REG, 0x77, 0x66);
		snd_soc_update_bits(codec,MINIDSP_REG, 0xF0, 0xA0);
		mdelay(5);
	}
if(new_mode == 1)
	change_codec_power_status(codec, 1, minidsp_stat);

if((new_mode == 2) || (new_mode == 0)){
	if(new_mode == 2) {
		snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xc0, 0xc0);
		snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0xc0, 0xc0);

	}
	if(new_mode == 0) {
		snd_soc_update_bits(codec, PASI_DAC_DP_SETUP, 0xc0, 0xc0);
		snd_soc_update_bits(codec, ADC_CHANNEL_POW, 0x80, 0x80);

	}
	mdelay(10);
	snd_soc_update_bits(codec,MINIDSP_REG,0xF0,0x50);
	snd_soc_update_bits(codec,ASI3_ADC_CTRL_REG,0x77,0x00);
		}
	}

#ifdef MULTICONFIG_SUPPORT

	if (new_config < 0 )
		return 0; // No configs supported in this pfw
	if (!pfw_changed && (new_config == aic326x->current_config))
		return 0;
	if (pflows->configs[new_config].a_patch_size || pflows->configs[new_config].d_patch_size)
		minidsp_multiconfig(codec,
			pflows->configs[new_config].a_patch, pflows->configs[new_config].a_patch_size,
			pflows->configs[new_config].d_patch,  pflows->configs[new_config].d_patch_size);
#endif


	return 0;
}


/*
 *----------------------------------------------------------------------------
 * Function : m_control_info
 * Purpose  : This function is to initialize data for new control required to
 *			program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_info(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_info *uinfo)
{
	uinfo->count = 1;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : m_control_get
 * Purpose  : This function is to read data of new control for
 *			program the AIC3262 registers.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);
	u32 val;
	u8 val1;

	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		val = aic3262->process_flow;
		ucontrol->value.integer.value[0] = val;
		DBG(KERN_INFO "control get : mode=%d\n", aic3262->process_flow);
	}
	if (!strcmp(kcontrol->id.name, "DAC Adaptive mode Enable")) {


		//val1 = codec->read(codec, 1);
		val1=dsp_reg_read(codec,80,1);
		ucontrol->value.integer.value[0] = ((val1>>1)&0x01);
		DBG(KERN_INFO "control get : mode=%d\n", aic3262->process_flow);

	}
	if (!strcmp(kcontrol->id.name, "ADC Adaptive mode Enable")) {


		//val1 = codec->read(codec, 1);
		val1=dsp_reg_read(codec,40,1);
		ucontrol->value.integer.value[0] = ((val1>>1)&0x01);
		DBG(KERN_INFO "control get : mode=%d\n", dmode);

	}

	if (!strcmp(kcontrol->id.name, "Minidsp patch")) {
		ucontrol->value.integer.value[0] = aic3262->current_config;
	}

	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : m_new_control_put
 * Purpose  : new_control_put is called to pass data from user/application to
 *			the driver.
 *
 *----------------------------------------------------------------------------
 */
static int m_control_put(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic3262_priv *aic3262 = snd_soc_codec_get_drvdata(codec);

	u32 val;
	u8 val1;
	int mode = aic3262->process_flow;

	DBG("n_control_put\n");
	val = ucontrol->value.integer.value[0];
	mutex_lock(&codec->mutex);
	if (!strcmp(kcontrol->id.name, "Minidsp mode")) {
		DBG(KERN_INFO "\nMini dsp put\n mode = %d, val=%d\n",
			aic3262->process_flow, val);
		if (val != mode) {
			if (aic3262->mute_codec == 1) {
				boot_minidsp(codec, val, -1);
			} else {
				printk(KERN_ERR
			" Cant Switch Processflows, Playback in progress");
			}
		}
	}

	#if 1
	if (!strcmp(kcontrol->id.name, "DAC Adaptive mode Enable")) {
		DBG(KERN_INFO "\nMini dsp put\n mode = %d, val=%d\n",
			aic3262->process_flow, val);
		if (val != amode) {

			//val1 = codec->read(codec, 1);
			//aic3262_write(codec, 1, (val1&0xfb)|(val<<1));
			dsp_reg_update_bits(codec, 80, 1, 0xfb, 0x04);
		}
		amode = val;
	}
	#endif

	#if 1
	if (!strcmp(kcontrol->id.name, "ADC Adaptive mode Enable")) {
		DBG(KERN_INFO "\nMini dsp put\n mode = %d, val=%d\n",
			aic3262->process_flow, val);
		if (val != dmode) {

			//val1 = codec->read(codec, 1);
			//aic3262_write(codec, 1, (val1&0xfb)|(val<<1));
			dsp_reg_update_bits(codec, 40, 1, 0xfb, 0x04);

		}
		dmode = val;
	}
	#endif

	//if (!strcmp(kcontrol->id.name, "Dump Regs Book0"))
		//i2c_verify_book0(codec);
	if (!strcmp(kcontrol->id.name, "Minidsp patch"))
		boot_minidsp(codec, aic3262->process_flow, val);

#if 0
	if (!strcmp(kcontrol->id.name, "Verify minidsp program")) {

		if (mode == 0) {
			DBG("Current mod=%d\nVerifying minidsp_D_regs", mode);
			byte_i2c_array_read(codec,  main44_miniDSP_D_reg_values,
				(main44_miniDSP_D_reg_values_COEFF_SIZE +
				main44_miniDSP_D_reg_values_INST_SIZE));
		} else {
			byte_i2c_array_read(codec,
				Second_Rate_miniDSP_A_reg_values,
				(Second_Rate_miniDSP_A_reg_values_COEFF_SIZE +
				Second_Rate_miniDSP_A_reg_values_INST_SIZE));
			byte_i2c_array_read(codec,
				Second_Rate_miniDSP_D_reg_values,
				(Second_Rate_miniDSP_D_reg_values_COEFF_SIZE +
				Second_Rate_miniDSP_D_reg_values_INST_SIZE));
		}
	}
#endif
	DBG("\nmode = %d\n", mode);
	mutex_unlock(&codec->mutex);
	return mode;
}

/************************** MUX CONTROL section *****************************/
/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_minidsp_mux
 * Purpose  : info routine for mini dsp mux control amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int __new_control_info_minidsp_mux(struct snd_kcontrol *kcontrol,
					  struct snd_ctl_elem_info *uinfo)
{
	int index,index2;
	int ret_val = -1;


	for (index = 0; index < ARRAY_SIZE(main44_MUX_controls); index++) {
		if (strstr(kcontrol->id.name, main44_MUX_control_names[index]))
			break;
	}
	if (index < ARRAY_SIZE(main44_MUX_controls))
		{
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
	}

	#if 1
	else{
		printk(" The second rate kcontrol id name is====== %s\n",kcontrol->id.name);


	for (index2 = 0; index < ARRAY_SIZE(base_speaker_SRS_MUX_controls); index2++) {
		if (strstr(kcontrol->id.name, base_speaker_SRS_MUX_control_names[index2]))
			break;
		}
		if (index < ARRAY_SIZE(base_speaker_SRS_MUX_controls))
		{
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
		uinfo->count = 1;
		uinfo->value.integer.min = MIN_MUX_CTRL;
		uinfo->value.integer.max = MAX_MUX_CTRL;
		ret_val = 0;
		}
	}

	#endif

	return ret_val;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_minidsp_mux
 *
 * Purpose  : get routine for  mux control amixer kcontrols,
 *   read current register values to user.
 *   Used for for mini dsp 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int __new_control_get_minidsp_mux(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{

	ucontrol->value.integer.value[0] = kcontrol->private_value;
	return 0;
}


/*
 *--------------------------------------------------------------------------
 * Function : aic3262_add_minidsp_controls
 * Purpose :  Configures the AMIXER Control Interfaces that can be exercised by
 *			the user at run-time. Utilizes the  the snd_adaptive_controls[]
 *			array to specify two run-time controls.
 *---------------------------------------------------------------------------
 */

int aic3262_add_minidsp_controls(struct snd_soc_codec *codec)
{
#ifdef ADD_MINI_DSP_CONTROLS
	int i, err, no_mux_controls,no_mux_controls1;
	/* add mode k control */
	for (i = 0; i < ARRAY_SIZE(aic3262_minidsp_controls); i++) {
		err = snd_ctl_add(codec->card->snd_card,
		snd_ctl_new1(&aic3262_minidsp_controls[i], codec));
		if (err < 0) {
			printk(KERN_ERR "Invalid control\n");
			return err;
		}
	}
#endif /* ADD_MINI_DSP_CONTROLS */
	return 0;
}

MODULE_DESCRIPTION("ASoC TLV320AIC3262 miniDSP driver");
MODULE_AUTHOR("Y Preetam Sashank Reddy <preetam@mistralsolutions.com>");
MODULE_LICENSE("GPL");
#endif /* End of CONFIG_MINI_DSP */
