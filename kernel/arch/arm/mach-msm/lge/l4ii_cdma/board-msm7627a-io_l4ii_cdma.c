/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio_event.h>
#include <linux/leds.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/input/rmi_platformdata.h>
#include <linux/input/rmi_i2c.h>
#include <linux/delay.h>
#include <linux/atmel_maxtouch.h>
#include <linux/input/ft5x06_ts.h>
#include <linux/leds-msm-tricolor.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>
#include <mach/rpc_server_handset.h>
#include <mach/pmic.h>
#include <mach/vreg.h>
#include <mach/rpc_pmapp.h>
#include <linux/init.h>

#include CONFIG_LGE_BOARD_HEADER_FILE
#include "devices.h"
#include "board-msm7627a.h"
#include "devices-msm7x2xa.h"

#ifdef CONFIG_LGE_NFC
#include <linux/nfc/pn544_lge.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_IST30XX)
static int iTSCount = 0;
#endif

#define ATMEL_TS_I2C_NAME "maXTouch"
#define ATMEL_X_OFFSET 13
#define ATMEL_Y_OFFSET 0

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) || \
defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C_MODULE)

#ifndef CLEARPAD3000_ATTEN_GPIO
#define CLEARPAD3000_ATTEN_GPIO (48)
#endif

#ifndef CLEARPAD3000_RESET_GPIO
#define CLEARPAD3000_RESET_GPIO (26)
#endif

#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))

static unsigned int kp_row_gpios[] = {36, 37};
static unsigned int kp_col_gpios[] = {32};

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	[KP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv64 = 40 * NSEC_PER_USEC,
	.poll_time.tv64 = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};

/* 8625 keypad device information */
static unsigned int kp_row_gpios_8625[] = {31};
static unsigned int kp_col_gpios_8625[] = {36, 37};

static const unsigned short keymap_8625[] = {
	KEY_VOLUMEUP,
	KEY_VOLUMEDOWN,
};

static const unsigned short keymap_8625_evt[] = {
	KEY_VOLUMEDOWN,
	KEY_VOLUMEUP,
};

static struct gpio_event_matrix_info kp_matrix_info_8625 = {
	.info.func      = gpio_event_matrix_func,
	.keymap         = keymap_8625,
	.output_gpios   = kp_row_gpios_8625,
	.input_gpios    = kp_col_gpios_8625,
	.noutputs       = ARRAY_SIZE(kp_row_gpios_8625),
	.ninputs        = ARRAY_SIZE(kp_col_gpios_8625),
	.settle_time.tv64 = 40 * NSEC_PER_USEC,
	.poll_time.tv64 = 20 * NSEC_PER_MSEC,
	.flags          = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info_8625[] = {
	&kp_matrix_info_8625.info,
};

static struct gpio_event_platform_data kp_pdata_8625 = {
	.name           = "7x27a_kp",
	.info           = kp_info_8625,
	.info_count     = ARRAY_SIZE(kp_info_8625)
};

static struct platform_device kp_pdev_8625 = {
	.name   = GPIO_EVENT_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data  = &kp_pdata_8625,
	},
};

#define LED_GPIO_PDM 96

#define MXT_TS_IRQ_GPIO         48
#define MXT_TS_RESET_GPIO       26
#define MAX_VKEY_LEN		100

static ssize_t mxt_virtual_keys_register(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *virtual_keys = __stringify(EV_KEY) ":" __stringify(KEY_MENU) \
		":60:840:120:80" ":" __stringify(EV_KEY) \
		":" __stringify(KEY_HOME)   ":180:840:120:80" \
		":" __stringify(EV_KEY) ":" \
		__stringify(KEY_BACK) ":300:840:120:80" \
		":" __stringify(EV_KEY) ":" \
		__stringify(KEY_SEARCH)   ":420:840:120:80" "\n";

	return snprintf(buf, strnlen(virtual_keys, MAX_VKEY_LEN) + 1 , "%s",
			virtual_keys);
}

static struct kobj_attribute mxt_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel_mxt_ts",
		.mode = S_IRUGO,
	},
	.show = &mxt_virtual_keys_register,
};

static struct attribute *mxt_virtual_key_properties_attrs[] = {
	&mxt_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mxt_virtual_key_properties_attr_group = {
	.attrs = mxt_virtual_key_properties_attrs,
};

struct kobject *mxt_virtual_key_properties_kobj;

static int mxt_vkey_setup(void)
{
	int retval = 0;

	mxt_virtual_key_properties_kobj =
		kobject_create_and_add("board_properties", NULL);
	if (mxt_virtual_key_properties_kobj)
		retval = sysfs_create_group(mxt_virtual_key_properties_kobj,
				&mxt_virtual_key_properties_attr_group);
	if (!mxt_virtual_key_properties_kobj || retval)
		pr_err("failed to create mxt board_properties\n");

	return retval;
}

static const u8 mxt_config_data[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	16, 1, 0, 0, 0, 0, 0, 0,
	/* T7 Object */
	32, 16, 50,
	/* T8 Object */
	30, 0, 20, 20, 0, 0, 20, 0, 50, 0,
	/* T9 Object */
	3, 0, 0, 18, 11, 0, 32, 75, 3, 3,
	0, 1, 1, 0, 10, 10, 10, 10, 31, 3,
	223, 1, 11, 11, 15, 15, 151, 43, 145, 80,
	100, 15, 0, 0, 0,
	/* T15 Object */
	131, 0, 11, 11, 1, 1, 0, 45, 3, 0,
	0,
	/* T18 Object */
	0, 0,
	/* T19 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0,
	/* T23 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	/* T25 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T40 Object */
	0, 0, 0, 0, 0,
	/* T42 Object */
	0, 0, 0, 0, 0, 0, 0, 0,
	/* T46 Object */
	0, 2, 32, 48, 0, 0, 0, 0, 0,
	/* T47 Object */
	1, 20, 60, 5, 2, 50, 40, 0, 0, 40,
	/* T48 Object */
	1, 12, 80, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 6, 6, 0, 0, 100, 4, 64,
	10, 0, 20, 5, 0, 38, 0, 20, 0, 0,
	0, 0, 0, 0, 16, 65, 3, 1, 1, 0,
	10, 10, 10, 0, 0, 15, 15, 154, 58, 145,
	80, 100, 15, 3,
};

static const u8 mxt_config_data_evt[] = {
	/* T6 Object */
	0, 0, 0, 0, 0, 0,
	/* T38 Object */
	20, 0, 0, 0, 0, 0, 0, 0,
	/* T7 Object */
	24, 12, 10,
	/* T8 Object */
	30, 0, 20, 20, 0, 0, 9, 45, 10, 192,
	/* T9 Object */
	3, 0, 0, 18, 11, 0, 16, 60, 3, 1,
	0, 1, 1, 0, 10, 10, 10, 10, 107, 3,
	223, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	20, 15, 0, 0, 2,
	/* T15 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0,
	/* T18 Object */
	0, 0,
	/* T19 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0,
	/* T23 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0,
	/* T25 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
	/* T40 Object */
	17, 0, 0, 30, 30,
	/* T42 Object */
	3, 20, 45, 40, 128, 0, 0, 0,
	/* T46 Object */
	0, 2, 16, 16, 0, 0, 0, 0, 0,
	/* T47 Object */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	/* T48 Object */
	1, 128, 96, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 6, 6, 0, 0, 63, 4, 64,
	10, 0, 32, 5, 0, 38, 0, 8, 0, 0,
	0, 0, 0, 0, 16, 65, 3, 1, 1, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0,
};

static struct mxt_config_info mxt_config_array[] = {
	{
		.config		= mxt_config_data,
		.config_length	= ARRAY_SIZE(mxt_config_data),
		.family_id	= 0x81,
		.variant_id	= 0x01,
		.version	= 0x10,
		.build		= 0xAA,
	},
};

static int mxt_key_codes[MXT_KEYARRAY_MAX_KEYS] = {
	[0] = KEY_HOME,
	[1] = KEY_MENU,
	[9] = KEY_BACK,
	[10] = KEY_SEARCH,
};

static struct mxt_platform_data mxt_platform_data = {
	.config_array		= mxt_config_array,
	.config_array_size	= ARRAY_SIZE(mxt_config_array),
	.panel_minx		= 0,
	.panel_maxx		= 479,
	.panel_miny		= 0,
	.panel_maxy		= 799,
	.disp_minx		= 0,
	.disp_maxx		= 479,
	.disp_miny		= 0,
	.disp_maxy		= 799,
	.irqflags		= IRQF_TRIGGER_FALLING,
	.i2c_pull_up		= true,
	.reset_gpio		= MXT_TS_RESET_GPIO,
	.irq_gpio		= MXT_TS_IRQ_GPIO,
	.key_codes		= mxt_key_codes,
};

static struct i2c_board_info mxt_device_info[] __initdata = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", 0x4a),
		.platform_data = &mxt_platform_data,
		.irq = MSM_GPIO_TO_INT(MXT_TS_IRQ_GPIO),
	},
};

static int synaptics_touchpad_setup(void);

static struct msm_gpio clearpad3000_cfg_data[] = {
	{GPIO_CFG(CLEARPAD3000_ATTEN_GPIO, 0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "rmi4_attn"},
	{GPIO_CFG(CLEARPAD3000_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "rmi4_reset"},
};

static struct rmi_XY_pair rmi_offset = {.x = 0, .y = 0};
static struct rmi_range rmi_clipx = {.min = 48, .max = 980};
static struct rmi_range rmi_clipy = {.min = 7, .max = 1647};
static struct rmi_f11_functiondata synaptics_f11_data = {
	.swap_axes = false,
	.flipX = false,
	.flipY = false,
	.offset = &rmi_offset,
	.button_height = 113,
	.clipX = &rmi_clipx,
	.clipY = &rmi_clipy,
};

#define MAX_LEN		100

static ssize_t clearpad3000_virtual_keys_register(struct kobject *kobj,
		     struct kobj_attribute *attr, char *buf)
{
	char *virtual_keys = __stringify(EV_KEY) ":" __stringify(KEY_MENU) \
			     ":60:830:120:60" ":" __stringify(EV_KEY) \
			     ":" __stringify(KEY_HOME)   ":180:830:120:60" \
				":" __stringify(EV_KEY) ":" \
				__stringify(KEY_SEARCH) ":300:830:120:60" \
				":" __stringify(EV_KEY) ":" \
			__stringify(KEY_BACK)   ":420:830:120:60" "\n";

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",
			virtual_keys);
}

static struct kobj_attribute clearpad3000_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.sensor00fn11",
		.mode = S_IRUGO,
	},
	.show = &clearpad3000_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&clearpad3000_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
	.attrs = virtual_key_properties_attrs,
};

struct kobject *virtual_key_properties_kobj;

static struct rmi_functiondata synaptics_functiondata[] = {
	{
		.function_index = RMI_F11_INDEX,
		.data = &synaptics_f11_data,
	},
};

static struct rmi_functiondata_list synaptics_perfunctiondata = {
	.count = ARRAY_SIZE(synaptics_functiondata),
	.functiondata = synaptics_functiondata,
};

static struct rmi_sensordata synaptics_sensordata = {
	.perfunctiondata = &synaptics_perfunctiondata,
	.rmi_sensor_setup	= synaptics_touchpad_setup,
};

static struct rmi_i2c_platformdata synaptics_platformdata = {
	.i2c_address = 0x2c,
	.irq_type = IORESOURCE_IRQ_LOWLEVEL,
	.sensordata = &synaptics_sensordata,
};

static struct i2c_board_info synaptic_i2c_clearpad3k[] = {
	{
	I2C_BOARD_INFO("rmi4_ts", 0x2c),
	.platform_data = &synaptics_platformdata,
	},
};

static int synaptics_touchpad_setup(void)
{
	int retval = 0;

	virtual_key_properties_kobj =
		kobject_create_and_add("board_properties", NULL);
	if (virtual_key_properties_kobj)
		retval = sysfs_create_group(virtual_key_properties_kobj,
				&virtual_key_properties_attr_group);
	if (!virtual_key_properties_kobj || retval)
		pr_err("failed to create ft5202 board_properties\n");

	retval = msm_gpios_request_enable(clearpad3000_cfg_data,
		    sizeof(clearpad3000_cfg_data)/sizeof(struct msm_gpio));
	if (retval) {
		pr_err("%s:Failed to obtain touchpad GPIO %d. Code: %d.",
				__func__, CLEARPAD3000_ATTEN_GPIO, retval);
		retval = 0; /* ignore the err */
	}
	synaptics_platformdata.irq = gpio_to_irq(CLEARPAD3000_ATTEN_GPIO);

	gpio_set_value(CLEARPAD3000_RESET_GPIO, 0);
	usleep(10000);
	gpio_set_value(CLEARPAD3000_RESET_GPIO, 1);
	usleep(50000);

	return retval;
}
#endif
/* handset device */
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};
 
/* LGE_CHANGE  [yoonsoo.kim@lge.com]  20111006  : U0 RevB Key Configuration */
static unsigned int keypad_row_gpios[] = {36, 37};
static unsigned int keypad_col_gpios[] = {32};

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(keypad_row_gpios) + (row))

static const unsigned short keypad_keymap_l4ii_cdma[] = {
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
};
/* LGE_CHANGE_E  [yoonsoo.kim@lge.com]  20111006  : U0 RevB Key Configuration */

int l4ii_cdma_matrix_info_wrapper(struct gpio_event_input_devs *input_dev,
			   struct gpio_event_info *info, void **data, int func)
{
	int ret;
	if (func == GPIO_EVENT_FUNC_RESUME) {
		gpio_tlmm_config(GPIO_CFG(keypad_row_gpios[0], 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(keypad_row_gpios[1], 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
	/* LGE_CHANGE [yoonsoo.kim@lge.com] 20120303  : Revert Patch Code */
	
	ret = gpio_event_matrix_func(input_dev, info, data, func);
	return ret ;
}

static int l4ii_cdma_gpio_matrix_power(const struct gpio_event_platform_data *pdata, bool on)
{
	/* this is dummy function
	 * to make gpio_event driver register suspend function
	 * 2010-01-29, cleaneye.kim@lge.com
	 * copy from ALOHA code
	 * 2010-04-22 younchan.kim@lge.com
	 */
	return 0;
}

static struct gpio_event_matrix_info l4ii_cdma_keypad_matrix_info = {
	.info.func			= l4ii_cdma_matrix_info_wrapper,
	.keymap				= keypad_keymap_l4ii_cdma,
	.output_gpios		= keypad_col_gpios,
	.input_gpios		= keypad_row_gpios,
	.noutputs			= ARRAY_SIZE(keypad_col_gpios),
	.ninputs			= ARRAY_SIZE(keypad_row_gpios),	
/*LGE_CHANGE_S : seven.kim@lge.com kernel3.0 proting
 * gpio_event_matrix_info structure member was changed, ktime_t -> struct timespec */	
	.settle_time.tv64 	= 40 * NSEC_PER_USEC,
	.poll_time.tv64 	= 20 * NSEC_PER_MSEC,
/*LGE_CHANGE_E : seven.kim@lge.com kernel3.0 proting*/
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_PRINT_UNMAPPED_KEYS | GPIOKPF_DRIVE_INACTIVE
};

static struct gpio_event_info *l4ii_cdma_keypad_info[] = {
	&l4ii_cdma_keypad_matrix_info.info
};

static struct gpio_event_platform_data l4ii_cdma_keypad_data = {
	.name		= "l4ii_cdma_keypad",
	.info		= l4ii_cdma_keypad_info,
	.info_count	= ARRAY_SIZE(l4ii_cdma_keypad_info),
	.power     	= l4ii_cdma_gpio_matrix_power,
};

struct platform_device keypad_device_l4ii_cdma = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &l4ii_cdma_keypad_data,
	},
};

/* input platform device */
static struct platform_device *l4ii_cdma_input_devices[] __initdata = {
	&hs_pdev,
};

static struct platform_device *l4ii_cdma_gpio_input_devices[] __initdata = {
	&keypad_device_l4ii_cdma,
};

#ifdef CONFIG_LGE_DIAGTEST
static struct platform_device lg_diag_input_device = {
	.name = "ats_input",
	.id = -1,
	.dev = { .platform_data = 0, },
};

static struct platform_device *l4ii_cdma_ats_input_devices[] __initdata = {
       &lg_diag_input_device,
};
#endif


/* LGE_CHANGE_S [seven.kim@lge.com] 20110922 New Bosch compass+accel Sensor Porting*/ 
#if defined (CONFIG_SENSORS_BMM050) || defined (CONFIG_SENSORS_BMA250)
static struct gpio_i2c_pin accel_i2c_pin[] = {
	[0] = {
		.sda_pin	= SENSOR_GPIO_I2C_SDA,
		.scl_pin	= SENSOR_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= ACCEL_GPIO_INT,
	},
};

static struct gpio_i2c_pin ecom_i2c_pin[] = {
	[0] = {
		.sda_pin	= SENSOR_GPIO_I2C_SDA,
		.scl_pin	= SENSOR_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= ECOM_GPIO_INT,
	},
};

static struct i2c_gpio_platform_data sensor_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device sensor_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &sensor_i2c_pdata,
};

static struct i2c_board_info sensor_i2c_bdinfo[] = {
	[0] = {
#if defined (CONFIG_SENSORS_BMA2X2)
/*#LGE_CHANGE : 2012-10-24 Sanghun,Lee(eee3114.@lge.com) sensor change from bmc150 to bmc050
*/
		I2C_BOARD_INFO("bma2x2", ACCEL_I2C_ADDRESS),
		.type = "bma2x2",
#else
		I2C_BOARD_INFO("bma250", ACCEL_I2C_ADDRESS),
		.type = "bma250",
#endif		
	},
	[1] = {
		I2C_BOARD_INFO("bmm050", ECOM_I2C_ADDRESS),
		.type = "bmm050",
	},
};

#if defined (CONFIG_MACH_MSM7X27A_L4II_CDMA)
static void __init l4ii_cdma_init_i2c_sensor(int bus_num)
{
	sensor_i2c_device.id = bus_num;
	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, accel_i2c_pin[0], &sensor_i2c_bdinfo[0]);
	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, ecom_i2c_pin[0], &sensor_i2c_bdinfo[1]);
	i2c_register_board_info(bus_num, sensor_i2c_bdinfo, ARRAY_SIZE(sensor_i2c_bdinfo));
	platform_device_register(&sensor_i2c_device);
}
#else
static void __init l4ii_cdma_init_i2c_sensor(int bus_num)
{
	sensor_i2c_device.id = bus_num;

	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, accel_i2c_pin[0], &sensor_i2c_bdinfo[0]);
	lge_init_gpio_i2c_pin(&sensor_i2c_pdata, ecom_i2c_pin[0], &sensor_i2c_bdinfo[1]);

	i2c_register_board_info(bus_num, sensor_i2c_bdinfo, ARRAY_SIZE(sensor_i2c_bdinfo));

	platform_device_register(&sensor_i2c_device);
}
#endif
#endif /* LGE_CHANGE_E [seven.kim@lge.com 20110922 New Bosch compass+accel Sensor Porting*/ 



/* proximity */
#define PROX_LDO_EN 12
static int prox_power_set(unsigned char onoff)
{

#if defined (CONFIG_SENSOR_APDS9190)

	if(onoff == 1)
	{
		gpio_tlmm_config(GPIO_CFG(PROX_LDO_EN, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(PROX_LDO_EN, 1);
		printk(KERN_INFO "%s,***********Proximity probe enter when power on*****\n",__func__);
	} else
	{
		gpio_tlmm_config(GPIO_CFG(PROX_LDO_EN, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(PROX_LDO_EN, 0);
		printk(KERN_INFO "%s,***********Proximity probe enter when power off*****\n",__func__);
	}

#endif
	//Need to implement Power Control : PMIC L12 Block use
	return 0;
}

#if defined (CONFIG_SENSOR_APDS9190)

static struct proximity_platform_data proxi_pdata = {
	.irq_num	= PROXI_GPIO_DOUT,
	.power		= prox_power_set,
	.methods		= 1,
	.operation_mode		= 0,
};

static struct i2c_board_info prox_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("proximity_apds9190", PROXI_I2C_ADDRESS),
		.type = "proximity_apds9190",
		.platform_data = &proxi_pdata,
	},
};

#else
static struct proximity_platform_data proxi_pdata = {
	.irq_num = PROXI_GPIO_DOUT,
	.power = prox_power_set,
	.methods = 0,
	.operation_mode = 0,
	.debounce = 0,
	.cycle = 2,
};

static struct i2c_board_info prox_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("proximity_gp2ap", PROXI_I2C_ADDRESS),
		.type = "proximity_gp2ap",
		.platform_data = &proxi_pdata,
	},
};
#endif

/* sukkyoon.hong@lge.com */
#if defined(CONFIG_TOUCHSCREEN_IST30XX)
/* REV_C */
static struct gpio_i2c_pin ts_i2c_pin[] = {
	[0] = {
		.reset_pin	= 0,
		.irq_pin	= TS_GPIO_IRQ,
	},
};

static struct i2c_gpio_platform_data ts_i2c_pdata = {
	.udelay			= 1,
};/* REV_C */

/* REV_B */
static struct gpio_i2c_pin ts_i2c_pin_rev_a[] = {
	[0] = {
		.sda_pin	= TS_GPIO_I2C_SDA,
		.scl_pin	= TS_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= TS_GPIO_IRQ,
	},
};

static struct i2c_gpio_platform_data ts_i2c_pdata_rev_a = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 1,
};

static struct platform_device ts_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &ts_i2c_pdata_rev_a,
};/* REV_B */

/*static*/ int ts_set_vreg(unsigned char onoff) //for touch download
{

	int rc = 0;
	struct vreg *vreg_l15 = NULL;

	vreg_l15 = vreg_get(NULL, "usim2");
	if (IS_ERR(vreg_l15)) {
		pr_err("%s: vreg_get failed (%ld)\n", __func__, PTR_ERR(vreg_l15));
		return PTR_ERR(vreg_l15);
	}

	/* LGE_CHANGE_S: E1 eungjin.kim@lge.com [2012-02-16] 
	: For Touch screen non response after wakeup*/	
	printk(KERN_INFO "ts_set_vreg : %d\n", onoff);
		if(onoff){
		
			if (iTSCount == 0)
			{
				rc = vreg_set_level(vreg_l15, 3000);
				printk("[ TSP ] : vreg_set_level\n");
			}
	
			if (rc < 0) {
				pr_err("%s: vreg_set_level failed (%d)\n", __func__, rc);
				goto vreg_touch_fail;
			}

			if (iTSCount == 0)
			{
				if(lge_get_board_revno() > LGE_REV_A) {
					gpio_set_value(TS_GPIO_IRQ, 1);
				}
				else {
					gpio_set_value(TS_GPIO_I2C_SCL, 1);
					gpio_set_value(TS_GPIO_I2C_SDA, 1);
					gpio_set_value(TS_GPIO_IRQ, 1);
				}

				rc = vreg_enable(vreg_l15);

				printk("[ TSP ] : vreg_enable\n");
				if(rc) {
					printk(KERN_ERR "%s:vreg enable failed (%d)\n", __func__, rc);
					return -EIO;
				}
				iTSCount++;
			}

			if (rc < 0) {
				pr_err("%s: vreg_enable failed (%d)\n", __func__, rc);
				goto vreg_touch_fail;
			}
		}

		else {
			if (iTSCount == 1)
			{
				if(lge_get_board_revno() > LGE_REV_A) {
					gpio_set_value(TS_GPIO_IRQ, 0);
				}
				else {
					gpio_set_value(TS_GPIO_I2C_SCL, 0);
					gpio_set_value(TS_GPIO_I2C_SDA, 0);
					gpio_set_value(TS_GPIO_IRQ, 0);
				}

				rc = vreg_disable(vreg_l15);
				iTSCount--;
			}

			if (rc < 0) {
				pr_err("%s: vreg_disable failed (%d)\n", __func__, rc);
				goto vreg_touch_fail;
			}
	}
	/* LGE_CHANGE_S: E1 eungjin.kim@lge.com [2012-02-16] 
	: For Touch screen non response after wakeup*/	

/*	rc = regulator_set_voltage(vreg_l15, 3000000, 3000000);
	if (rc) {
		pr_err("%s: vreg_set_level failed for vreg_l15\n", __func__);
		goto vreg_touch_fail;
	}*/

	printk("into vreg function.\n");

	return rc;

vreg_touch_fail:
	vreg_put(vreg_l15);
	return rc;
}

static struct touch_platform_data ts_pdata = {
	.ts_x_min = TS_X_MIN,
	.ts_x_max = TS_X_MAX,
	.ts_y_min = TS_Y_MIN,
	.ts_y_max = TS_Y_MAX,
	.power 	  = ts_set_vreg,
	.irq 	  = TS_GPIO_IRQ,
};

static struct touch_platform_data ts_pdata_rev_a = {
	.ts_x_min = TS_X_MIN,
	.ts_x_max = TS_X_MAX,
	.ts_y_min = TS_Y_MIN,
	.ts_y_max = TS_Y_MAX,
	.power    = ts_set_vreg,
	.irq      = TS_GPIO_IRQ,
	.scl      = TS_GPIO_I2C_SCL,
	.sda      = TS_GPIO_I2C_SDA,
};

static struct i2c_board_info ts_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("IST30XX", 0xA0>>1),
		.type = "IST30XX",
		.platform_data = &ts_pdata,
	},
};

static struct i2c_board_info ts_i2c_bdinfo_rev_a[] = {
	[0] = {
		I2C_BOARD_INFO("IST30XX", 0xA0>>1),
		.type = "IST30XX",
		.platform_data = &ts_pdata_rev_a,
	},
};

/* this routine should be checked for nessarry */
/* REV_C */
static int init_gpio_i2c_pin_touch(
	struct i2c_gpio_platform_data *i2c_adap_pdata,
	struct gpio_i2c_pin gpio_i2c_pin,
	struct i2c_board_info *i2c_board_info_data)
{

	gpio_request(TS_GPIO_IRQ, 	"IST30XX_I2C_INT");


	if (gpio_i2c_pin.irq_pin) {
		//gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
		//			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}
	gpio_free(TS_GPIO_IRQ);

/*	gpio_request(TS_GPIO_I2C_SDA, 	"IST30XX_I2C_SDA");
	gpio_request(TS_GPIO_I2C_SCL, 	"IST30XX_I2C_SCL");

	i2c_adap_pdata->sda_pin = 0;
	i2c_adap_pdata->scl_pin = 0;

	gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SDA, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SCL, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_free(TS_GPIO_I2C_SDA);
	gpio_free(TS_GPIO_I2C_SCL);*/

	return 0;
}/* REV_C */

/* REV_B */
static int init_gpio_i2c_pin_touch_rev_a(
	struct i2c_gpio_platform_data *i2c_adap_pdata,
	struct gpio_i2c_pin gpio_i2c_pin,
	struct i2c_board_info *i2c_board_info_data)
{
	i2c_adap_pdata->sda_pin = gpio_i2c_pin.sda_pin;
	i2c_adap_pdata->scl_pin = gpio_i2c_pin.scl_pin;

	/* LGE_CHANGE_S: E1 eungjin.kim@lge.com [2012-03-06] : Fix Touch GPIO Warning Message*/
	gpio_request(TS_GPIO_I2C_SDA, 	"IST30XX_I2C_SDA");
	gpio_request(TS_GPIO_I2C_SCL, 	"IST30XX_I2C_SCL");
	gpio_request(TS_GPIO_IRQ, 	"IST30XX_I2C_INT");

	printk("int : %d, sda : %d, scl : %d\n",gpio_i2c_pin.irq_pin, gpio_i2c_pin.sda_pin, gpio_i2c_pin.scl_pin);

	/* LGE_CHANGE_E: E1 eungjin.kim@lge.com [2012-03-06] : Fix Touch GPIO Warning Message*/
	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.sda_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.scl_pin, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_set_value(gpio_i2c_pin.sda_pin, 1);
	gpio_set_value(gpio_i2c_pin.scl_pin, 1);

/*	if (gpio_i2c_pin.reset_pin) {
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.reset_pin, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(gpio_i2c_pin.reset_pin, 1);
	}*/

	if (gpio_i2c_pin.irq_pin) {
		//gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
		//			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_i2c_pin.irq_pin, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		i2c_board_info_data->irq =
			MSM_GPIO_TO_INT(gpio_i2c_pin.irq_pin);
	}

	gpio_free(TS_GPIO_I2C_SDA);
	gpio_free(TS_GPIO_I2C_SCL);
	gpio_free(TS_GPIO_IRQ);

	return 0;
}/* REV_B */

static void __init ist30xx_init_i2c_touch(int bus_num)
{
	printk("[ TSP ] bus_num : %d\n",bus_num);

	ts_i2c_device.id = bus_num;

	init_gpio_i2c_pin_touch_rev_a(&ts_i2c_pdata_rev_a, ts_i2c_pin_rev_a[0], &ts_i2c_bdinfo_rev_a[0]);
	i2c_register_board_info(bus_num, &ts_i2c_bdinfo_rev_a[0], 1);
	platform_device_register(&ts_i2c_device);
	printk("into ist30xx_init_i2c_touch\n");
}
#endif /* CONFIG_TOUCHSCREEN_IST30XX */

static struct gpio_i2c_pin proxi_i2c_pin[] = {
	[0] = {
		.sda_pin	= PROXI_GPIO_I2C_SDA,
		.scl_pin	= PROXI_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= PROXI_GPIO_DOUT,
	},
};

static struct i2c_gpio_platform_data proxi_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device proxi_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &proxi_i2c_pdata,
};


#if defined (CONFIG_SENSOR_APDS9190)
static void __init l4ii_cdma_init_i2c_prox(int bus_num)
{
	proxi_i2c_device.id = bus_num;
	//proxi_i2c_device.id=sensor_i2c_device.id;
	lge_init_gpio_i2c_pin(&proxi_i2c_pdata, proxi_i2c_pin[0], &prox_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &prox_i2c_bdinfo[0], 1);
	platform_device_register(&proxi_i2c_device);
}

#else
static void __init l4ii_cdma_init_i2c_prox(int bus_num)
{
	proxi_i2c_device.id = bus_num;

	lge_init_gpio_i2c_pin(
		&proxi_i2c_pdata, proxi_i2c_pin[0], &prox_i2c_bdinfo[0]);

	i2c_register_board_info(bus_num, &prox_i2c_bdinfo[0], 1);
	platform_device_register(&proxi_i2c_device);
}
#endif

#if defined(CONFIG_TOUCHSCREEN_MELFAS_MMS136)
static struct gpio_i2c_pin ts_i2c_pin[] = {
	[0] = {
		.sda_pin	= TS_GPIO_I2C_SDA,
		.scl_pin	= TS_GPIO_I2C_SCL,
		.reset_pin	= 0,
		.irq_pin	= TS_GPIO_IRQ,
	},
};


static struct i2c_gpio_platform_data ts_i2c_pdata = {
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.udelay			= 1,
};

static struct platform_device ts_i2c_device = {
	.name	= "i2c-gpio",
	.dev.platform_data = &ts_i2c_pdata,
};

static int ts_config_gpio(int config)
{
	if(config) {
		/* for wake state */
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SDA, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SCL, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_TOUCH_ID, 0, GPIO_CFG_INPUT,
				 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}
	else {
		/* for sleep state */
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SDA, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_GPIO_I2C_SCL, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(TS_TOUCH_ID, 0, GPIO_CFG_INPUT,
				 GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	}

	return 0;
}

int ts_set_vreg_01(int onoff, bool log_en)
{
	int rc = 0;

	if (log_en)
		printk(KERN_INFO "[Touch D] %s: power %s\n",
			__func__, onoff ? "On" : "Off");

	if (onoff) {
		ts_config_gpio(1);
		gpio_tlmm_config(GPIO_CFG(TS_TOUCH_LDO_EN, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(TS_TOUCH_LDO_EN, 1);
	}
	else {
		ts_config_gpio(0);
		gpio_tlmm_config(GPIO_CFG(TS_TOUCH_LDO_EN, 0, GPIO_CFG_OUTPUT,
				 GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		gpio_set_value(TS_TOUCH_LDO_EN, 0);
	}

	return rc;
}

static struct melfas_tsi_platform_data melfas_ts_pdata = {
	.max_x = TS_X_MAX,
	.max_y = TS_Y_MAX,
	.max_pressure = TS_Y_MIN,
	.max_width = TS_Y_MAX,
	.gpio_scl = TS_GPIO_I2C_SCL,
	.gpio_sda = TS_GPIO_I2C_SDA,
	.i2c_int_gpio = TS_GPIO_IRQ,
	.power_enable = ts_set_vreg_01,
};

static struct i2c_board_info ts_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO(MELFAS_TS_NAME, TS_I2C_SLAVE_ADDR),
		.platform_data = &melfas_ts_pdata,
	},
};


static void __init l4ii_cdma_init_i2c_touch(int bus_num)
{
	ts_i2c_device.id = bus_num;

	lge_init_gpio_i2c_pin(&ts_i2c_pdata, ts_i2c_pin[0], &ts_i2c_bdinfo[0]);
	i2c_register_board_info(bus_num, &ts_i2c_bdinfo[0], 1);
	platform_device_register(&ts_i2c_device);
}
#endif



#ifdef CONFIG_LGE_NFC
// 2012.09.26 garam.kim@lge.com NFC registration
static struct gpio_i2c_pin nfc_i2c_pin[] = {
	[0] = {
		.sda_pin	= NFC_GPIO_I2C_SDA,
		.scl_pin	= NFC_GPIO_I2C_SCL,
		.reset_pin	= NFC_GPIO_VEN,
		.irq_pin	= NFC_GPIO_IRQ,
	},
};

static struct i2c_gpio_platform_data nfc_i2c_pdata = {
	.sda_is_open_drain = 0,
	.scl_is_open_drain = 0,
	.udelay = 2,
};

static struct platform_device nfc_i2c_device = {
	.name = "i2c-gpio",
	.dev.platform_data = &nfc_i2c_pdata,
};

static struct pn544_i2c_platform_data nfc_pdata = {
	.ven_gpio 		= NFC_GPIO_VEN,
	.irq_gpio 	 	= NFC_GPIO_IRQ,
	.scl_gpio		= NFC_GPIO_I2C_SCL,
	.sda_gpio		= NFC_GPIO_I2C_SDA,
	.firm_gpio		= NFC_GPIO_FIRM,
};

static struct i2c_board_info nfc_i2c_bdinfo[] = {
	[0] = {
		I2C_BOARD_INFO("pn544", NFC_I2C_SLAVE_ADDR),
		.platform_data = &nfc_pdata,
		.irq = MSM_GPIO_TO_INT(NFC_GPIO_IRQ),
	},
};

static void __init v7_init_i2c_nfc(int bus_num)
{
	int ret;
/*
 	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_FIRM, 0, GPIO_CFG_OUTPUT,
 				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_VEN, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_IRQ, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_I2C_SDA, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_I2C_SCL, 0, GPIO_CFG_OUTPUT,
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE); 
*/

	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_FIRM, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(NFC_GPIO_VEN, 1);
	gpio_tlmm_config(GPIO_CFG(NFC_GPIO_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
    nfc_i2c_bdinfo->irq = MSM_GPIO_TO_INT(NFC_GPIO_IRQ);
	nfc_i2c_device.id = bus_num;

	ret = lge_init_gpio_i2c_pin(&nfc_i2c_pdata, nfc_i2c_pin[0],	&nfc_i2c_bdinfo[0]);
	
	ret = i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID, &nfc_i2c_bdinfo[0], 1);
	
	platform_device_register(&nfc_i2c_device);	
}
#endif

#if 0
static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#define FT5X06_IRQ_GPIO		48
#define FT5X06_RESET_GPIO	26

static ssize_t
ft5x06_virtual_keys_register(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	return snprintf(buf, 200,
	__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":40:510:80:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":120:510:80:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":200:510:80:60"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":280:510:80:60"
	"\n");
}

static struct kobj_attribute ft5x06_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x06_ts",
		.mode = S_IRUGO,
	},
	.show = &ft5x06_virtual_keys_register,
};

static struct attribute *ft5x06_virtual_key_properties_attrs[] = {
	&ft5x06_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group ft5x06_virtual_key_properties_attr_group = {
	.attrs = ft5x06_virtual_key_properties_attrs,
};

struct kobject *ft5x06_virtual_key_properties_kobj;

static struct ft5x06_ts_platform_data ft5x06_platformdata = {
	.x_max		= 320,
	.y_max		= 480,
	.reset_gpio	= FT5X06_RESET_GPIO,
	.irq_gpio	= FT5X06_IRQ_GPIO,
	.irqflags	= IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
};

static struct i2c_board_info ft5x06_device_info[] __initdata = {
	{
		I2C_BOARD_INFO("ft5x06_ts", 0x38),
		.platform_data = &ft5x06_platformdata,
		.irq = MSM_GPIO_TO_INT(FT5X06_IRQ_GPIO),
	},
};

static void __init ft5x06_touchpad_setup(void)
{
	int rc;

	rc = gpio_tlmm_config(GPIO_CFG(FT5X06_IRQ_GPIO, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
			GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, FT5X06_IRQ_GPIO);

	rc = gpio_tlmm_config(GPIO_CFG(FT5X06_RESET_GPIO, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
			GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, FT5X06_RESET_GPIO);

	ft5x06_virtual_key_properties_kobj =
			kobject_create_and_add("board_properties", NULL);

	if (ft5x06_virtual_key_properties_kobj)
		rc = sysfs_create_group(ft5x06_virtual_key_properties_kobj,
				&ft5x06_virtual_key_properties_attr_group);

	if (!ft5x06_virtual_key_properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n", __func__);

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				ft5x06_device_info,
				ARRAY_SIZE(ft5x06_device_info));
}

/* SKU3/SKU7 keypad device information */
#define KP_INDEX_SKU3(row, col) ((row)*ARRAY_SIZE(kp_col_gpios_sku3) + (col))
static unsigned int kp_row_gpios_sku3[] = {31, 32};
static unsigned int kp_col_gpios_sku3[] = {36, 37};

static const unsigned short keymap_sku3[] = {
	[KP_INDEX_SKU3(0, 0)] = KEY_VOLUMEUP,
	[KP_INDEX_SKU3(0, 1)] = KEY_VOLUMEDOWN,
	[KP_INDEX_SKU3(1, 1)] = KEY_CAMERA,
};

static struct gpio_event_matrix_info kp_matrix_info_sku3 = {
	.info.func      = gpio_event_matrix_func,
	.keymap         = keymap_sku3,
	.output_gpios   = kp_row_gpios_sku3,
	.input_gpios    = kp_col_gpios_sku3,
	.noutputs       = ARRAY_SIZE(kp_row_gpios_sku3),
	.ninputs        = ARRAY_SIZE(kp_col_gpios_sku3),
	.settle_time.tv64 = 40 * NSEC_PER_USEC,
	.poll_time.tv64 = 20 * NSEC_PER_MSEC,
	.flags          = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
				GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info_sku3[] = {
	&kp_matrix_info_sku3.info,
};
static struct gpio_event_platform_data kp_pdata_sku3 = {
	.name           = "7x27a_kp",
	.info           = kp_info_sku3,
	.info_count     = ARRAY_SIZE(kp_info_sku3)
};

static struct platform_device kp_pdev_sku3 = {
	.name   = GPIO_EVENT_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data  = &kp_pdata_sku3,
	},
};

static struct led_info ctp_backlight_info = {
	.name           = "button-backlight",
	.flags          = PM_MPP__I_SINK__LEVEL_40mA << 16 | PM_MPP_7,
};

static struct led_platform_data ctp_backlight_pdata = {
	.leds = &ctp_backlight_info,
	.num_leds = 1,
};

static struct platform_device pmic_mpp_leds_pdev = {
	.name   = "pmic-mpp-leds",
	.id     = -1,
	.dev    = {
		.platform_data  = &ctp_backlight_pdata,
	},
};

static struct led_info tricolor_led_info[] = {
	[0] = {
		.name           = "red",
		.flags          = LED_COLOR_RED,
	},
	[1] = {
		.name           = "green",
		.flags          = LED_COLOR_GREEN,
	},
};

static struct led_platform_data tricolor_led_pdata = {
	.leds = tricolor_led_info,
	.num_leds = ARRAY_SIZE(tricolor_led_info),
};

static struct platform_device tricolor_leds_pdev = {
	.name   = "msm-tricolor-leds",
	.id     = -1,
	.dev    = {
		.platform_data  = &tricolor_led_pdata,
	},
};
#endif

void __init msm7627a_add_io_devices(void)
{
#if defined (CONFIG_MACH_MSM7X27A_L4II_CDMA)
	/*LGE_CHANGE_S : seven.kim@lge.com JB 2035.2B Migration*/
	/* ignore end key as this target doesn't need it */
	hs_platform_data.ignore_end_key = true;
	/*LGE_CHANGE_E : seven.kim@lge.com JB 2035.2B Migration*/
	
	platform_add_devices(
		l4ii_cdma_input_devices, ARRAY_SIZE(l4ii_cdma_input_devices));
	platform_add_devices(
		l4ii_cdma_gpio_input_devices, ARRAY_SIZE(l4ii_cdma_gpio_input_devices));

/* sukkyoon.hong@lge.com */
#if defined (CONFIG_TOUCHSCREEN_IST30XX)
	if(lge_get_board_revno() > LGE_REV_A) { /* REV_C */
		init_gpio_i2c_pin_touch(&ts_i2c_pdata, ts_i2c_pin[0], &ts_i2c_bdinfo[0]);

		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			ts_i2c_bdinfo,
			ARRAY_SIZE(ts_i2c_bdinfo));
	}
	else { /* REV_B */
		lge_add_gpio_i2c_device(ist30xx_init_i2c_touch);
	}
#else
	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_touch);
#endif

#ifdef CONFIG_LGE_DIAGTEST
	platform_add_devices(l4ii_cdma_ats_input_devices, ARRAY_SIZE(l4ii_cdma_ats_input_devices));
#endif
	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_prox);
#if defined (CONFIG_SENSORS_BMM050) || defined(CONFIG_SENSORS_BMA250)
	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_sensor);
#else
	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_acceleration);
	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_ecom);
#endif
	
//	lge_add_gpio_i2c_device(l4ii_cdma_init_i2c_prox);

#else
	/* touchscreen */
	if (machine_is_msm7625a_surf() || machine_is_msm7625a_ffa()) {
		atmel_ts_pdata.min_x = 0;
		atmel_ts_pdata.max_x = 480;
		atmel_ts_pdata.min_y = 0;
		atmel_ts_pdata.max_y = 320;
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				atmel_ts_i2c_info,
				ARRAY_SIZE(atmel_ts_i2c_info));
	/* keypad */
	platform_device_register(&kp_pdev);

	/* headset */
	platform_device_register(&hs_pdev);

	/* LED: configure it as a pdm function */
	if (gpio_tlmm_config(GPIO_CFG(LED_GPIO_PDM, 3,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config for %d failed\n",
			__func__, LED_GPIO_PDM);
	else
		platform_device_register(&led_pdev);
#endif	/* end of qct origin */

#ifdef CONFIG_LGE_NFC
	lge_add_gpio_i2c_device(v7_init_i2c_nfc);
#endif

	/* Vibrator */
	//if (machine_is_msm7x27a_ffa() || machine_is_msm7625a_ffa()
	//				|| machine_is_msm8625_ffa())
		msm_init_pmic_vibrator();
}

void __init qrd7627a_add_io_devices(void)
{
#if 0
	int rc;
	/* touchscreen */
	if (machine_is_msm7627a_qrd1()) {
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
					synaptic_i2c_clearpad3k,
					ARRAY_SIZE(synaptic_i2c_clearpad3k));
	} else if (machine_is_msm7627a_evb() || machine_is_msm8625_evb() ||
			machine_is_msm8625_evt()) {
		/* Use configuration data for EVT */
		if (machine_is_msm8625_evt()) {
			mxt_config_array[0].config = mxt_config_data_evt;
			mxt_config_array[0].config_length =
					ARRAY_SIZE(mxt_config_data_evt);
			mxt_platform_data.panel_maxy = 875;
			mxt_vkey_setup();
		}

		rc = gpio_tlmm_config(GPIO_CFG(MXT_TS_IRQ_GPIO, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config for %d failed\n",
				__func__, MXT_TS_IRQ_GPIO);
		}

		rc = gpio_tlmm_config(GPIO_CFG(MXT_TS_RESET_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_8MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config for %d failed\n",
				__func__, MXT_TS_RESET_GPIO);
		}

		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
					mxt_device_info,
					ARRAY_SIZE(mxt_device_info));
	} else if (machine_is_msm7627a_qrd3() || machine_is_msm8625_qrd7()) {
		ft5x06_touchpad_setup();
	}

	/* headset */
	platform_device_register(&hs_pdev);

	/* vibrator */
#ifdef CONFIG_MSM_RPC_VIBRATOR
	msm_init_pmic_vibrator();
#endif

	/* keypad */
	if (machine_is_msm8625_evt())
		kp_matrix_info_8625.keymap = keymap_8625_evt;

	if (machine_is_msm7627a_evb() || machine_is_msm8625_evb() ||
			machine_is_msm8625_evt())
		platform_device_register(&kp_pdev_8625);
	else if (machine_is_msm7627a_qrd3() || machine_is_msm8625_qrd7())
		platform_device_register(&kp_pdev_sku3);

	/* leds */
	if (machine_is_msm7627a_evb() || machine_is_msm8625_evb() ||
						machine_is_msm8625_evt()) {
		platform_device_register(&pmic_mpp_leds_pdev);
		platform_device_register(&tricolor_leds_pdev);
	}
#endif
}
