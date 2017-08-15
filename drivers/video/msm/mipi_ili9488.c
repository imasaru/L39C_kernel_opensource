/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_ili9488.h"
#include <asm/gpio.h>
#include <mach/vreg.h>
#include CONFIG_LGE_BOARD_HEADER_FILE
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
#include <linux/hrtimer.h>
#endif

#define ILI9488_CMD_DELAY 	0
#define ILI9488_DEBUG 0

// sd card file end point must finalize necessory "$"
#define ILI9488_TUNING_SET	0 // 0 : normal version, 1 : tuning version

// esd all register read
#define CONFIG_LGE_LCD_ESD_ALL_REGISTER_READ	0

// esd log display disable
#define CONFIG_LGE_LCD_ESD_LOG					0

#if ILI9488_TUNING_SET
#define ILI9488_TUNING_LOG  1
#define ILI9488_TUNING_FROM_SDCARD  1 // 0 : /data/ili9486  1: /sdcard/external_sd/ili9488 , /sdcard2/
#endif

#define ILI9488_ESD_COM_REGISTER 	0xB0

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
enum {
	BOOSTER_VOLTAGE_STATUS = 1U << 2,
	SLEEP_IN_OUT = 1U << 4,
	DISPLAY_ON_OFF = 1U << 7,
};

#if ILI9488_TUNING_SET
#define ILI9488_ESD_FIRST_BOOTING_TIME 		35000	// 35 sec
#define ILI9488_ESD_NON_FIRST_BOOTING_TIME 	3000	// 3 sec
#else
#define ILI9488_ESD_FIRST_BOOTING_TIME 		240000	// 4 min
#define ILI9488_ESD_NON_FIRST_BOOTING_TIME 	10000	// 10 sec
#endif

#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
#define BACKLIGHT_CHECK_TIME 		2000
extern int lm3530_get_state(void);
extern int late_resume_value;
//extern int bl_status;
extern void lm3530_bl_control(int status);
static struct work_struct work_bl;
static struct hrtimer bl_timer;
static int lcd_status;
#endif

static int esd_start = 0;
static int lcd_on_off = 0;
static int esd_init = 0;
static int esd_exception_case = 0;
static int esd_exception_value = 0;
static struct work_struct work_esd;
static struct hrtimer esd_timer;

//extern void request_suspend_resume(void);
extern int lge_get_esd_flag(void);
extern int lge_set_esd_flag(int flag);
#endif

#if ILI9488_TUNING_SET
extern void mipi_timing_reg_init(void);
struct ili9488_register_value_pair {
	char register_num;
	char register_value[20];	
};

#define EXT_BUF_SIZE		100
static struct ili9488_register_value_pair ext_reg_settings[EXT_BUF_SIZE];
static int filesystem_read = 0;
static int size_length = 0;
#endif

static void mipi_ldp_lcd_panel_poweroff(void);
static struct msm_panel_common_pdata *mipi_ili9488_pdata;
static struct platform_device *platform_data;

static int mipi_ili9488_status(struct platform_device *pdev);
#define GPIO_LCD_MAKER_ID 126   //LCD maker_id pin number is 126
static int maker_id_result = 0;



static struct dsi_buf ili9488_tx_buf;
static struct dsi_buf ili9488_rx_buf;
static int init_boot = 0;

static struct dsi_buf *ili9488_tx_buf_msg;
static char *ili9488_tmp;

/* --------------------start configuration---------------------*/
//--write start
#if ILI9488_TUNING_SET
static char config_F2[1] = {0x00}; //Gen-long 
static char config_F8[1]={0x00}; //Gen-2param

static char config_memory_access_control[1]={0x00}; //DCS-1param
static char config_display_inversion_cont[1]={0x00}; //Gen-1param
static char config_display_func[1]={0x00};  //Gen-long

static char config_power_ctrl_1[1]={0x00}; //Gen-2param
static char config_power_ctrl_2[1]={0x00}; //Gen-1param
static char config_power_ctrl_3[1]={0x00}; //Gen-1param
static char config_power_vcom_control[1]={0x00}; //Gen-long

static char config_positive_gamma_correc[1]={0x00}; //Gen-long
static char config_negative_gamma_correc[1]={0x00}; //Gen-long

static char config_column_address_set[1]={0x00}; //DCS-Long
static char config_page_address_set[1]={0x00}; //DCS-Long

static char config_pixel_format[1]={0x00}; //DCS-1param
static char config_entry_mode_set[1]={0x00}; //Gen-1param
static char config_frame_rate[1]={0x00}; //Gen-2param

static char config_F7[1]={0x00}; //Gen
static char config_blacnking_porch_control[1]={0x00}; //Gen-long
#else

static char ili9488_config_power_ctrl_2[2]={
	0xC1, 0x41}; //DTYPE_GEN_WRITE2

static char ili9488_config_power_ctrl_1[4]={
	0xC0, 0x0C, 0x0C, 0xff}; //DTYPE_GEN_LWRITE

static char ili9488_config_positive_gamma_correc[16]={
	0xE0, 0x00, 0x11, 0x12,
	0x04, 0x10, 0x06, 0x3C,
	0x89, 0x54, 0x0A, 0x0D,
	0x0B, 0x30, 0x36, 0x0F};

static char ili9488_config_negative_gamma_correc[16]={
	0xE1, 0x00, 0x07, 0x0D,
	0x04, 0x12, 0x04, 0x29,
	0x34, 0x42, 0x09, 0x0F,
	0x0B, 0x2C, 0x2D, 0x0F};

static char ili9488_config_power_vcom_control[4]={
	0xC5, 0x00, 0x60, 0x80};

static char ili9488_config_frame_rate[3]={
	0xB1, 0xC0, 0x15}; //DTYPE_DCS_LWRITE

static char ili9488_config_blanking_porch_ctrl[5]={
	0xB5, 0x0F, 0x0F, 0x0C, 0x2A};

static char ili9488_config_display_inversion_cont[2]={
	0xB4, 0x00}; //DTYPE_GEN_WRITE2

static char ili9488_config_display_func[4]={
	0xB6, 0x02, 0x42, 0x3B};  //DTYPE_GEN_LWRITE

static char ili9488_set_image_func[2]={
	0xE9, 0x00}; //DTYPE_GEN_WRITE2

static char ili9488_config_F2[6]={
	0xF2, 0x58, 0x10, 0x12, 0x02, 0x92};

static char ili9488_config_F7[5]={
	0xF7, 0xA9, 0x51, 0x2C, 0x82}; //DTYPE_DCS_WRITE

static char ili9488_config_pixel_format[2]={
	0x3A, 0x66}; //DTYPE_DCS_WRITE1

static char display_inversion_on[2] = {
	0x21, 0x00}; //DTYPE_DCS_WRITE

static char ili9488_config_memory_access_control[2]={
	0x36, 0x08}; //DTYPE_DCS_WRITE1



#endif

//this command should be only used at power on sequence
static char config_sleep_out[1]={0x11}; //DCS-0param
//wait 120 ms
static char config_memory_write[1]={0x2C}; //DCS-0param
//wait 80ms
static char config_display_on[1]={0x29}; //DCS-0param

//--display off
static char config_display_off[1]={0x28}; //DCS-0param
//wait 50ms
static char config_sleep_in[1]={0x10}; //DCS-0-param

#ifdef CONFIG_LGE_LCD_TUNING
#define LUT_SIZE 			256
#define GAMMNA_SIZE 		16
#define TUNING_REGSIZE  	50
#define TUNING_REGNUM 		16
#define TUNING_BUFSIZE 		4096
typedef struct tuning_buff{
	char buf[TUNING_REGNUM][TUNING_REGSIZE];
	int size;
	int idx;
}TUNING_BUFF_;
#endif

//--read start
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
static char config_esd_status_F2[1] = {0xF2};
static char config_esd_status_F8[1] = {0xF8};
static char config_esd_status_36[1] = {0x36};
static char config_esd_status_B4[1] = {0xB4};
static char config_esd_status_B6[1] = {0xB6};
static char config_esd_status_C0[1] = {0xC0};
static char config_esd_status_C1[1] = {0xC1};
static char config_esd_status_C2[1] = {0xC2};
static char config_esd_status_C5[1] = {0xC5};
static char config_esd_status_2A[1] = {0x2A};
static char config_esd_status_2B[1] = {0x2B};
static char config_esd_status_3A[1] = {0x3A};
static char config_esd_status_B7[1] = {0xB7};
static char config_esd_status_B1[1] = {0xB1};
static char config_esd_status_F7[1] = {0xF7};
static char config_esd_status_B5[1] = {0xB5};
static char config_esd_status_0A[1] = {0x0A};

static char config_tear_on[2] = {0x35, 0x00};
static char config_tear_off[2] = {0x34, 0x00};
static struct dsi_cmd_desc ili9488_tear_on_cmd = {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(config_tear_on), config_tear_on};
static struct dsi_cmd_desc ili9488_tear_off_cmd = {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(config_tear_off), config_tear_off};
#endif

/* --------------------end configuration-----------------------*/

/*----------------------start setting cmd----------------------*/
/*----------------------start write cmd----------------------*/
//display on
static struct dsi_cmd_desc ili9488_init_on_cmds[] = {
#if 1
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_ctrl_2),ili9488_config_power_ctrl_2 },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_ctrl_1),ili9488_config_power_ctrl_1 },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_positive_gamma_correc),ili9488_config_positive_gamma_correc },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_negative_gamma_correc),ili9488_config_negative_gamma_correc },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_vcom_control),ili9488_config_power_vcom_control },
//	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_interface_mode_ctrl),ili9488_interface_mode_ctrl },
	{DTYPE_DCS_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_frame_rate),ili9488_config_frame_rate },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_blanking_porch_ctrl),ili9488_config_blanking_porch_ctrl},
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_display_inversion_cont),ili9488_config_display_inversion_cont },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_display_func),ili9488_config_display_func },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_set_image_func),ili9488_set_image_func },
	{DTYPE_DCS_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_F2),ili9488_config_F2 },
	{DTYPE_DCS_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_F7),ili9488_config_F7 },
	{DTYPE_DCS_WRITE1, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_pixel_format),ili9488_config_pixel_format },
	{DTYPE_DCS_WRITE,  1, 0, 0, ILI9488_CMD_DELAY,		sizeof(display_inversion_on),display_inversion_on },
	{DTYPE_DCS_WRITE1, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_memory_access_control),ili9488_config_memory_access_control },
#else
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_ctrl_2),ili9488_config_power_ctrl_2 },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_ctrl_1),ili9488_config_power_ctrl_1 },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_positive_gamma_correc),ili9488_config_positive_gamma_correc },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_negative_gamma_correc),ili9488_config_negative_gamma_correc },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_power_vcom_control),ili9488_config_power_vcom_control },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_interface_mode_ctrl),ili9488_interface_mode_ctrl },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_frame_rate),ili9488_config_frame_rate },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_display_inversion_cont),ili9488_config_display_inversion_cont },
	{DTYPE_GEN_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_display_func),ili9488_config_display_func },
	{DTYPE_GEN_WRITE2, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_set_image_func),ili9488_set_image_func },
	{DTYPE_DCS_LWRITE, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_F7),ili9488_config_F7 },
	{DTYPE_DCS_WRITE1, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_pixel_format),ili9488_config_pixel_format },
	{DTYPE_DCS_WRITE,  1, 0, 0, ILI9488_CMD_DELAY,		sizeof(display_inversion_on),display_inversion_on },
	{DTYPE_DCS_WRITE1, 1, 0, 0, ILI9488_CMD_DELAY,		sizeof(ili9488_config_memory_access_control),ili9488_config_memory_access_control },
#endif
};

static struct dsi_cmd_desc ili9488_sleep_out_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, 
		sizeof(config_sleep_out), config_sleep_out}
};
static struct dsi_cmd_desc ili9488_memory_write_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 80,
		sizeof(config_memory_write), config_memory_write},
};
static struct dsi_cmd_desc ili9488_disp_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 0,
		sizeof(config_display_on), config_display_on}
};
//display off
static struct dsi_cmd_desc ili9488_disp_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(config_display_off),config_display_off },
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(config_sleep_in), config_sleep_in}	
};
/*----------------------end write cmd----------------------*/

/*----------------------start read cmd----------------------*/
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
static struct dsi_cmd_desc ili9488_esd_status_F2_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_F2), config_esd_status_F2};
static struct dsi_cmd_desc ili9488_esd_status_F8_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_F8), config_esd_status_F8};
static struct dsi_cmd_desc ili9488_esd_status_36_read_cmds = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(config_esd_status_36), config_esd_status_36};
static struct dsi_cmd_desc ili9488_esd_status_B4_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_B4), config_esd_status_B4};
static struct dsi_cmd_desc ili9488_esd_status_B6_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_B6), config_esd_status_B6};
static struct dsi_cmd_desc ili9488_esd_status_C0_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_C0), config_esd_status_C0};
static struct dsi_cmd_desc ili9488_esd_status_C1_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_C1), config_esd_status_C1};
static struct dsi_cmd_desc ili9488_esd_status_C2_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_C2), config_esd_status_C2};
static struct dsi_cmd_desc ili9488_esd_status_C5_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_C5), config_esd_status_C5};
static struct dsi_cmd_desc ili9488_esd_status_2A_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_2A), config_esd_status_2A};
static struct dsi_cmd_desc ili9488_esd_status_2B_read_cmds = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(config_esd_status_2B), config_esd_status_2B};
static struct dsi_cmd_desc ili9488_esd_status_3A_read_cmds = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(config_esd_status_3A), config_esd_status_3A};
static struct dsi_cmd_desc ili9488_esd_status_B7_read_cmds = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(config_esd_status_B7), config_esd_status_B7};
static struct dsi_cmd_desc ili9488_esd_status_B1_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_B1), config_esd_status_B1};
static struct dsi_cmd_desc ili9488_esd_status_F7_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_F7), config_esd_status_F7};
static struct dsi_cmd_desc ili9488_esd_status_B5_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_B5), config_esd_status_B5};
static struct dsi_cmd_desc ili9488_esd_status_0A_read_cmds = {
	DTYPE_GEN_READ1, 1, 0, 1, 5, sizeof(config_esd_status_0A), config_esd_status_0A};
#endif

/*----------------------end read cmd----------------------*/
/*----------------------end setting cmd----------------------*/


#if ILI9488_TUNING_SET
#define LOOP_INTERVAL		100
#define IS_NUM(c)			((0x30<=c)&&(c<=0x39))
#define IS_CHAR_C(c)		((0x41<=c)&&(c<=0x46))						// Capital Letter
#define IS_CHAR_S(c)		((0x61<=c)&&(c<=0x66))						// Small Letter
#define IS_VALID(c)			(IS_NUM(c)||IS_CHAR_C(c)||IS_CHAR_S(c))		// NUM or CHAR
#define TO_BE_NUM_OFFSET(c)	(IS_NUM(c) ? 0x30 : (IS_CHAR_C(c) ? 0x37 : 0x57))	
#define TO_BE_READ_SIZE		 EXT_BUF_SIZE*40							// 8pages (4000x8)

static char *file_buf_alloc_pages=NULL;

static long ili9488_read_ext_reg(char *filename)
{	
	int value=0, read_idx=0, i=0, j=0, k=0;
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs=get_fs();
	phMscd_Filp = filp_open(filename, O_RDONLY |O_LARGEFILE, 0);

	printk("%s : enter this function!\n", __func__);

	if (IS_ERR(phMscd_Filp)) {
		printk("%s : open error!\n", __func__);
		return 0;
	}

	file_buf_alloc_pages = kmalloc(TO_BE_READ_SIZE, GFP_KERNEL);	

	if(!file_buf_alloc_pages) {
		printk("%s : mem alloc error!\n", __func__);
		return 0;
	}

	set_fs(get_ds());
	phMscd_Filp->f_op->read(phMscd_Filp, file_buf_alloc_pages, TO_BE_READ_SIZE-1, &phMscd_Filp->f_pos);
	set_fs(old_fs);

	do
	{		
		if(file_buf_alloc_pages[read_idx]=='['){
			if (file_buf_alloc_pages[read_idx]=='[' && file_buf_alloc_pages[read_idx+2]==']'){ // one value
				read_idx += 1;

				value = file_buf_alloc_pages[read_idx]-TO_BE_NUM_OFFSET(file_buf_alloc_pages[read_idx]);
				read_idx += 1;
				ext_reg_settings[i].register_num = value;
#if ILI9488_TUNING_LOG
				printk("%s : case 1, i = %d, value = %d\n", __func__, i, value);
#endif
			}
			else if (file_buf_alloc_pages[read_idx]=='[' && file_buf_alloc_pages[read_idx+3]==']'){ // two value
				read_idx += 1;

				value = (file_buf_alloc_pages[read_idx]-TO_BE_NUM_OFFSET(file_buf_alloc_pages[read_idx]))*10 \
						+ (file_buf_alloc_pages[read_idx+1]-TO_BE_NUM_OFFSET(file_buf_alloc_pages[read_idx+1]));

				read_idx += 2;
				ext_reg_settings[i].register_num = value;
#if ILI9488_TUNING_LOG
				printk("%s : case 2, i = %d, value = %d\n", __func__, i, value);
#endif
			}



			for(j=0; j < LOOP_INTERVAL; j++){
				if ((file_buf_alloc_pages[read_idx]=='0' && file_buf_alloc_pages[read_idx+1]=='x' 
							&& file_buf_alloc_pages[read_idx + 2] != ' ') || (file_buf_alloc_pages[read_idx]=='0' && file_buf_alloc_pages[read_idx+1]=='X' 
								&& file_buf_alloc_pages[read_idx + 2] != ' ')){	// skip : 0x, 0X
					read_idx += 2;

					value = (file_buf_alloc_pages[read_idx]-TO_BE_NUM_OFFSET(file_buf_alloc_pages[read_idx]))*0x10 \
							+ (file_buf_alloc_pages[read_idx+1]-TO_BE_NUM_OFFSET(file_buf_alloc_pages[read_idx+1]));

					read_idx += 2;
					ext_reg_settings[i].register_value[k++] = value;

#if ILI9488_TUNING_LOG
					printk("%s : case 3, value = %x\n", __func__, value);
#endif
				}	
				else
				{
					if(file_buf_alloc_pages[read_idx]=='['){
						read_idx -= 5;
						break;
					}

					read_idx += 1;
				}
			}	
			k = 0;
			read_idx += 1;
			i++;
		}		
		else
		{
			++read_idx;
		}
	}while(file_buf_alloc_pages[read_idx] != '$');

	kfree(file_buf_alloc_pages);
	file_buf_alloc_pages=NULL;
	filp_close(phMscd_Filp,NULL);

	return i;
}

static long ili9488_reg_init_ext(void)
{	
	uint16_t length = 0;

	printk("%s\n", __func__);
#if ILI9488_TUNING_FROM_SDCARD
	length = ili9488_read_ext_reg("/sdcard/external_sd/ili9488");
#else
	length = ili9488_read_ext_reg("/data/ili9488");
#endif	
	printk("%s : length = %d!\n", __func__, length);	

	if (!length)
		return 0;
	else
		return length;
}
#endif

/*LGE_CHANGE_S,hyungjoon.jeon,13-02-06, for M4 lcd backlight timning code*/
#if defined(CONFIG_MACH_MSM7X25A_M4) //need to modify feature to L4II
extern int lcd_on_completed;
#endif

static int mipi_ili9488_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	int bl_value = BACKLIGHT_CHECK_TIME;
#endif
#if ILI9488_TUNING_SET
	int loop_num = 0;	
#endif
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	int value = 0;
#endif

	mfd = platform_get_drvdata(pdev);	

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;


	if( 0){
		// when panic mode, lcd init no need.
#if 0 //def CONFIG_LGE_HANDLE_PANIC
		if(get_kernel_panicmode() == 2)
			return 0;
#endif

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
		value = ILI9488_ESD_FIRST_BOOTING_TIME;
#endif
		platform_data = pdev;		
		printk("mipi_ili9488_lcd_on init booting\n");	

		//check ili9485 old or new
		mipi_ili9488_status(pdev);
		printk("%s, maker_id_result:%d\n",__func__, maker_id_result );
	}
	else{
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
		value = ILI9488_ESD_NON_FIRST_BOOTING_TIME;
#endif		
	}

#if ILI9488_TUNING_SET
	if(init_boot == 1)
#endif
	{
		printk("mipi_ili9488_lcd_on START\n");		
		mipi_set_tx_power_mode(1);
#if ILI9488_TUNING_SET
		for(loop_num = 0; loop_num < size_length; loop_num++)
		{
			ili9488_init_on_cmds[loop_num].dlen = ext_reg_settings[loop_num].register_num;
			ili9488_init_on_cmds[loop_num].payload= ext_reg_settings[loop_num].register_value;	
			if(ext_reg_settings[loop_num].register_value[0] == 0x11)
				break;
		}
#endif

		mipi_dsi_cmds_tx(&ili9488_tx_buf, ili9488_init_on_cmds,
				ARRAY_SIZE(ili9488_init_on_cmds));
		printk("%s,mipi_dsi_cmds_tx ili9488_init_on_cmds..\n",__func__ );
		printk("%s,PANEL_ID_OLD_ILI9488\n",__func__ );

#if ILI9488_DEBUG
		printk("mipi_ili9488_init_on_cmd ..\n");
#endif
		ili9488_tx_buf_msg=&ili9488_tx_buf;
		ili9488_tmp=ili9488_tx_buf_msg->data;

#if ILI9488_DEBUG
		printk("mipi_ili9488_init_on_cmd %s\n", ili9488_tmp);
#endif	

		mipi_dsi_cmds_tx( &ili9488_tx_buf, ili9488_sleep_out_cmds,
				ARRAY_SIZE(ili9488_sleep_out_cmds));

		msleep(120);

#if ILI9488_DEBUG
		printk("mipi_ili9488_sleep_out_cmds..\n");
		printk("mipi_ili9488_init_on_cmd %s\n", ili9488_tmp);
#endif

#if 0 //For Power Sqeunce
		mipi_dsi_cmds_tx( &ili9488_tx_buf, ili9488_memory_write_cmds,
				ARRAY_SIZE(ili9488_memory_write_cmds));

#if ILI9488_DEBUG
		printk("mipi_ili9488_memory_write_cmds..\n");
		printk("mipi_ili9488_init_on_cmd %s\n", ili9488_tmp);
#endif
#endif

		mipi_dsi_cmds_tx( &ili9488_tx_buf, ili9488_disp_on_cmds,
				ARRAY_SIZE(ili9488_disp_on_cmds));

#if ILI9488_DEBUG
		printk("mipi_ili9488_disp_on_cmd..\n");
		printk("mipi_ili9488_disp_on_cmd %s\n", ili9488_tmp);
#endif


		msleep(20);

		mipi_dsi_cmds_tx( &ili9488_tx_buf, ili9488_memory_write_cmds,
				ARRAY_SIZE(ili9488_memory_write_cmds));
#if ILI9488_DEBUG
		printk("mipi_ili9488_memory_write_cmds..\n");
		printk("mipi_ili9488_init_on_cmd %s\n", ili9488_tmp);
#endif
		mipi_set_tx_power_mode(0);
		printk("mipi_ili9488_lcd_on FINISH\n");
	}

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	hrtimer_start(&esd_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
#endif

	if(init_boot == 0){		
		init_boot = 1;		
	}    

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	lcd_on_off = 1;
#endif

	/*LGE_CHANGE_S,hyungjoon.jeon,13-02-06, for M4 lcd backlight timning code*/
#if defined(CONFIG_MACH_MSM7X25A_M4)
	lcd_on_completed = 1;
#endif
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	lcd_status = 1;
	hrtimer_start(&bl_timer,
			ktime_set(bl_value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
#endif
	return 0;
}

static int mipi_ili9488_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	esd_start = 0;
	lcd_on_off = 0;
	esd_init = 0;
#endif
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	lcd_status = 0;
#endif
	printk("mipi_ili9488_lcd_off START\n");

	mipi_set_tx_power_mode(1);
	mipi_dsi_cmds_tx( &ili9488_tx_buf, ili9488_disp_off_cmds,
			ARRAY_SIZE(ili9488_disp_off_cmds));
	mipi_set_tx_power_mode(0);

	mipi_ldp_lcd_panel_poweroff();
#if ILI9488_TUNING_SET
	//filesystem_read = 0;
#endif
	printk("mipi_ili9488_lcd_off END\n");


#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	if(lm3530_get_state() != 2)
	{
		printk("%s(), lm3530_get_state() : %d, late_resume_value:%d\n",__func__, lm3530_get_state(), late_resume_value );
		lm3530_bl_control(0);
	}
#endif
	return 0;
}

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
uint8 mipi_ili9488_esd_check_log(int nNum, int Count)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;	
	uint8  lp_8_int_1, lp_8_int_2, lp_8_int_3,lp_8_int_4;
	uint32 lp_32_int;
	uint64 *lp_64;
	int	reg_type=0;

	struct msm_fb_data_type *mfd;

	if (!platform_data)
		return -ENODEV;	

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(platform_data);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	tp = &ili9488_tx_buf;
	rp = &ili9488_rx_buf;
	switch(nNum){	
		case 1:		cmd = &ili9488_esd_status_F2_read_cmds;		reg_type = 0xF2;  	break;
		case 2:		cmd = &ili9488_esd_status_F8_read_cmds;		reg_type = 0xF8; 	break;			
		case 3:		cmd = &ili9488_esd_status_36_read_cmds;		reg_type = 0x36;	break;
		case 4:		cmd = &ili9488_esd_status_B4_read_cmds;		reg_type = 0xB4;	break;
		case 5:		cmd = &ili9488_esd_status_B6_read_cmds;		reg_type = 0xB6;	break;
		case 6:		cmd = &ili9488_esd_status_C0_read_cmds;		reg_type = 0xC0;	break;		
		case 7:		cmd = &ili9488_esd_status_C1_read_cmds;		reg_type = 0xC1;	break;
		case 8:		cmd = &ili9488_esd_status_C2_read_cmds;		reg_type = 0xC2;	break;
		case 9:		cmd = &ili9488_esd_status_C5_read_cmds;		reg_type = 0xC5;	break;
		case 10:	cmd = &ili9488_esd_status_2A_read_cmds;		reg_type = 0x2A;	break;
		case 11:	cmd = &ili9488_esd_status_2B_read_cmds;		reg_type = 0x2B;	break;
		case 12:	cmd = &ili9488_esd_status_3A_read_cmds;		reg_type = 0x3A;	break;
		case 13:	cmd = &ili9488_esd_status_B7_read_cmds;		reg_type = 0xB7;	break;
		case 14:	cmd = &ili9488_esd_status_B1_read_cmds;		reg_type = 0xB1;	break;
		case 15:	cmd = &ili9488_esd_status_F7_read_cmds;		reg_type = 0xF7;	break;
		case 16:	cmd = &ili9488_esd_status_B5_read_cmds;		reg_type = 0xB5;	break;
		case 17:	cmd = &ili9488_esd_status_0A_read_cmds;		reg_type = 0x0A;	break;
		default:	return 0;
	}
	mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 16);

	if(Count > 4){		
		lp_64 = (uint64 *)rp->data;
		lp_8_int_1 = (*lp_64)         & 0xff;
		lp_8_int_2 = ((*lp_64) >> 8)  & 0xff;
		lp_8_int_3 = ((*lp_64) >> 16) & 0xff;
		lp_8_int_4 = ((*lp_64) >> 24) & 0xff;
#if CONFIG_LGE_LCD_ESD_LOG
		printk("%s: Reg=(0x%x,0x%x,0x%x,0x%x,0x%x,", __func__, reg_type, lp_8_int_1, lp_8_int_2, lp_8_int_3, lp_8_int_4);
#endif

		lp_32_int = (uint32) (*lp_64 >> 32) ;
		lp_8_int_1 = (lp_32_int)       & 0xff;
		lp_8_int_2 = (lp_32_int >> 8)  & 0xff;
		lp_8_int_3 = (lp_32_int >> 16) & 0xff;
		lp_8_int_4 = (lp_32_int >> 24) & 0xff;
#if CONFIG_LGE_LCD_ESD_LOG
		if(Count == 5)
			printk("0x%x)\n", lp_8_int_1);
		else if(Count == 6)
			printk("0x%x,0x%x)\n", lp_8_int_1, lp_8_int_2);
		else if(Count == 7)
			printk("0x%x,0x%x,0x%x)\n", lp_8_int_1, lp_8_int_2, lp_8_int_3);
		else if(Count == 8)
			printk("0x%x,0x%x,0x%x,0x%x)\n", lp_8_int_1, lp_8_int_2, lp_8_int_3, lp_8_int_4);
		else
			printk("0x%x,0x%x,0x%x,0x%x)\n", lp_8_int_1, lp_8_int_2, lp_8_int_3, lp_8_int_4);
#endif
	}
	else{
		lp_64 = (uint64 *)rp->data;
		lp_8_int_1 = (*lp_64)         & 0xff;
		lp_8_int_2 = ((*lp_64) >> 8)  & 0xff;
		lp_8_int_3 = ((*lp_64) >> 16) & 0xff;
		lp_8_int_4 = ((*lp_64) >> 24) & 0xff;			

#if CONFIG_LGE_LCD_ESD_LOG
		if(Count == 1)
			printk("%s: Reg=(0x%x,0x%x)\n", __func__, reg_type, lp_8_int_1);
		else if(Count == 2)
			printk("%s: Reg=(0x%x,0x%x,0x%x)\n", __func__, reg_type, lp_8_int_1, lp_8_int_2);
		else if(Count == 3)
			printk("%s: Reg=(0x%x,0x%x,0x%x,0x%x)\n", __func__, reg_type, lp_8_int_1, lp_8_int_2, lp_8_int_3);
		else if(Count == 4)
			printk("%s: Reg=(0x%x,0x%x,0x%x,0x%x,0x%x)\n", __func__, reg_type, lp_8_int_1, lp_8_int_2, lp_8_int_3, lp_8_int_4);
		else
			printk("%s: Reg=(0x%x,0x%x,0x%x,0x%x,0x%x)\n", __func__, reg_type, lp_8_int_1, lp_8_int_2, lp_8_int_3, lp_8_int_4);
#endif
	}

	return lp_8_int_1;

}

int mipi_ili9488_esd_check(void)
{	
	uint8 lp_8_1, lp_8_2 = 0;	
	struct msm_fb_data_type *mfd;

//	if((!esd_start) || (lm3530_get_state() == 2 /* SLEEP_STATE */))
	if((!esd_start))  
		return 0;	

	if (!platform_data)
		return -ENODEV;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(platform_data);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	// [important] to read the correct read data, tearing effect must disable.
	mipi_dsi_cmds_tx( &ili9488_tx_buf, &ili9488_tear_off_cmd, 1);
#if CONFIG_LGE_LCD_ESD_ALL_REGISTER_READ    
	mipi_ili9488_esd_check_log(1, 9);
	mipi_ili9488_esd_check_log(2, 2);
	mipi_ili9488_esd_check_log(3, 1);
	mipi_ili9488_esd_check_log(4, 1);
	mipi_ili9488_esd_check_log(5, 3);
	mipi_ili9488_esd_check_log(6, 2);
	mipi_ili9488_esd_check_log(7, 1);
	mipi_ili9488_esd_check_log(8, 1);
	mipi_ili9488_esd_check_log(9, 4);
	mipi_ili9488_esd_check_log(10, 4);
	mipi_ili9488_esd_check_log(11, 4);
	mipi_ili9488_esd_check_log(12, 1);
	mipi_ili9488_esd_check_log(13, 1);
	mipi_ili9488_esd_check_log(14, 2);
	mipi_ili9488_esd_check_log(15, 5);
	mipi_ili9488_esd_check_log(16, 4);
	mipi_ili9488_esd_check_log(17, 1);	

	printk("%s: Register Read Completed !!! \n\n", __func__);	
#endif

	lp_8_1 = mipi_ili9488_esd_check_log(14, 2);
	lp_8_2 = mipi_ili9488_esd_check_log(17, 1);
	mipi_dsi_cmds_tx( &ili9488_tx_buf, &ili9488_tear_on_cmd, 1);	

	// sometimes not correct read the data at first time.
	if(esd_init == 0){
		esd_init = 1;
		return 0;	
	}

	// (- 15V) issue happend [register 0x00]
	//if(lp_8 == 0)
	//	return 0;

	//if(!(lp_8 & BOOSTER_VOLTAGE_STATUS) || !(lp_8 & SLEEP_IN_OUT) || !(lp_8 & DISPLAY_ON_OFF))		
	//	return -1;

	if((lp_8_1 != ILI9488_ESD_COM_REGISTER) || ((lp_8_2 != 0x9c) && (lp_8_2 != 0x00)))
	{
		printk("%s: Esd Working !!! \n\n", __func__);
		return -1;
	}

	return 0;
}

void lcd_esd_control(void)
{	
	int ret = 0;

#if CONFIG_LGE_LCD_ESD_LOG
	printk("%s\n", __func__);
#endif
	ret = mipi_ili9488_esd_check();
#if 0	
	if(ret != 0)
		request_suspend_resume();
#endif
}

EXPORT_SYMBOL(lcd_esd_control);
#endif

ssize_t mipi_ili9488_lcd_show_onoff(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("%s : start\n", __func__);
	return 0;
}

ssize_t mipi_ili9488_lcd_store_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/*struct platform_device dummy_pdev;*/
	int onoff;
	struct platform_device *pdev = to_platform_device(dev);

	sscanf(buf, "%d", &onoff);
	printk("%s: onoff : %d\n", __func__, onoff);
	if (onoff) {
		mipi_ili9488_lcd_on(pdev);
	} else {
		mipi_ili9488_lcd_off(pdev);
	}
	return count;
}

DEVICE_ATTR(lcd_onoff, 0664, mipi_ili9488_lcd_show_onoff, mipi_ili9488_lcd_store_onoff);
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
static void mipi_ili9488_lcd_bl_start(struct work_struct *work)
{
	printk("\n %s(), late_resume_value:%d, lm3530_get_state:%d, lcd_status:%d\n",
				__func__, late_resume_value, lm3530_get_state(), lcd_status);
	if((lm3530_get_state() == 2) && (late_resume_value > 0))
	{
		if(lcd_status)
			lm3530_bl_control(1);
	}
	else if((lm3530_get_state() == 2) && (late_resume_value == 0))
	{
		if(lcd_status)
			lm3530_bl_control(1);
	}
}
#endif
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
static void mipi_ili9488_lcd_esd_start(struct work_struct *work)
{	
	if(lcd_on_off){
		printk("%s\n",__func__);

		//after phone is booting, only one time check need.
		if(esd_exception_case == 0){
			esd_exception_value = lge_get_esd_flag();
			if(esd_exception_value != 8)
				lge_set_esd_flag(0); //wrong value is resetting to zero.
			esd_exception_case = 1;
		}

		if(esd_exception_value != 8){ //abnormal case : value = 8			
			esd_start = 1;

		}		

#if ILI9488_TUNING_SET
		esd_start = 0;
		if(filesystem_read == 0){
			mipi_timing_reg_init();
			size_length = ili9488_reg_init_ext();
			filesystem_read = 1;
		}
#endif
	}
}

static void esd_timer_callfunction(void)
{	
	schedule_work(&work_esd);	
}

static enum hrtimer_restart esd_timer_func(struct hrtimer *timer)
{	
	esd_timer_callfunction();	
	return HRTIMER_NORESTART;
}
#endif

#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
static void bl_timer_callfunction(void)
{
	schedule_work(&work_bl);
}

static enum hrtimer_restart bl_timer_func(struct hrtimer *timer)
{
	bl_timer_callfunction();
	return HRTIMER_NORESTART;
}
#endif


static int __devinit mipi_ili9488_lcd_probe(struct platform_device *pdev)
{
	int rc = 0;

	init_boot = 0;
#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	esd_start = 0;
#endif

	if (pdev->id == 0) {
		mipi_ili9488_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifdef CONFIG_LGE_LCD_ESD_DETECTION
	INIT_WORK(&work_esd, mipi_ili9488_lcd_esd_start);
	hrtimer_init(&esd_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	esd_timer.function = esd_timer_func;
#endif

#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	INIT_WORK(&work_bl, mipi_ili9488_lcd_bl_start);
	hrtimer_init(&bl_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bl_timer.function = bl_timer_func;
#endif

	msm_fb_add_device(pdev);
	/*this for AT Command*/
	rc = device_create_file(&pdev->dev, &dev_attr_lcd_onoff);

	//check ili9485 old or new
	mipi_ili9488_status(pdev);
	printk("%s, maker_id_result:%d\n",__func__, maker_id_result );

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_ili9488_lcd_probe,
	.driver = {
		.name   = "mipi_ili9488",
	},
};

static struct msm_fb_panel_data ili9488_panel_data = {
	.on		= mipi_ili9488_lcd_on,
	.off	= mipi_ili9488_lcd_off,
};

static int ch_used[3];

int mipi_ili9488_device_register(struct msm_panel_info *pinfo,
		u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;
	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_ili9488", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	ili9488_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &ili9488_panel_data,
			sizeof(ili9488_panel_data));
	if (ret) {
		pr_err("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_ili9488_status(struct platform_device *pdev)
{
	maker_id_result = gpio_get_value(GPIO_LCD_MAKER_ID);
	printk("%s, maker_id_result:%d\n",__func__,maker_id_result );

	return maker_id_result;

}


#ifdef CONFIG_LGE_LCD_TUNING
int tuning_read_regset(unsigned long tmp)
{
	int i;
	int size;
	TUNING_BUFF_ *rbuf;

	rbuf = (TUNING_BUFF_ *)tmp;

	size = ARRAY_SIZE(ili9488_init_on_cmds);

	for (i = 0; i < size; i++) {
		if (copy_to_user(rbuf->buf[i], ili9488_init_on_cmds[i].payload,
					ili9488_init_on_cmds[i].dlen))
		{
			printk(KERN_ERR "read_file : error of copy_to_user_buff\n");
			return -EFAULT;
		}
	}
	if (put_user(size, &(rbuf->size)))
	{
		printk(KERN_ERR "read_file : error of copy_to_user_buffsize\n");
		return -EFAULT;
	}
	return 0;
}
EXPORT_SYMBOL(tuning_read_regset);
char set_buff[TUNING_REGNUM][TUNING_REGSIZE];
int tuning_write_regset(unsigned long tmp)
{
	char *buff;
	TUNING_BUFF_ *wbuf = (TUNING_BUFF_ *)tmp;
	int i = wbuf->idx;

	printk(KERN_INFO "write file\n");
	buff = kmalloc(TUNING_BUFSIZE, GFP_KERNEL);
	if (copy_from_user(buff, wbuf->buf, wbuf->size))
	{
		printk(KERN_ERR "write_file : error of copy_from_user\n");
		return -EFAULT;
	}

	memset(set_buff[i], 0x00, TUNING_REGSIZE);
	memcpy(set_buff[i], buff, wbuf->size);
	ili9488_init_on_cmds[i].payload = set_buff[i];
	ili9488_init_on_cmds[i].dlen = wbuf->size;

	kfree(buff);
	return 0;
}
EXPORT_SYMBOL(tuning_write_regset);
#endif

static int __init mipi_ili9488_lcd_init(void)
{
	mipi_dsi_buf_alloc(&ili9488_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&ili9488_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

static void mipi_ldp_lcd_panel_poweroff(void)
{
	gpio_set_value(GPIO_LCD_RESET, 0);
	msleep(10);
}
module_init(mipi_ili9488_lcd_init);
