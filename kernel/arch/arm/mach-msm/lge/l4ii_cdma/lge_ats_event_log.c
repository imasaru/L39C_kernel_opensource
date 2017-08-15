/*
 *  arch/arm/mach-msm/lge/lge_ats_event_log.c
 *
 *  Copyright (c) 2010 LGE.
 *
 *  All source code in this file is licensed under the following license
 *  except where indicated.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/module.h> /* error: 'THIS_MODULE' undeclared here (not in a function) */
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/lge_at_cmd.h>
#include <linux/slab.h>

#define DRIVER_NAME "ats_event_log"
// BEGIN: 0010583 alan.park@lge.com 2010.11.06
// ADD 0010583: [ETA/MTC] ETA Capture, Key, Touch, Logging / MTC Key, Logging 
#define ATS_XMAX 480
#define ATS_YMAX 800
// END: 0010583 alan.park@lge.com 2010.11.06 

static struct input_dev *ats_event_log_dev;

/* add ETA  key event logging for vs660 [younchan.kim 2010-05-31]*/
static struct input_handler input_handler;
static struct work_struct event_log_work;
struct ats_mtc_key_log_type ats_mtc_key_log1;
static int ats_event_log_status = 0;


extern int ats_mtc_log_mask;
extern void ats_mtc_send_key_log_to_eta(struct ats_mtc_key_log_type *);

extern int touch_get_x_max (void);
extern int touch_get_y_max(void);

#ifdef CONFIG_LGE_DIAG_MTC
extern unsigned char g_diag_mtc_check;
extern void mtc_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);
#endif

#ifdef CONFIG_LGE_DIAG_ICD
extern unsigned char g_diag_icd_check;
//extern void LGT_Icd_send_key_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);
extern int slate_send_log_data(struct ats_mtc_key_log_type* p_ats_mtc_key_log);
#endif

/* TODO :  need to modify key map for each model */
#define ETA_KEY_MAX     8
#define ETA_ABS_MAX	10


/* key list for VS660 & LGP-500 */
int eta_key_list[ETA_KEY_MAX]={
	/* thunder keypad key */
	KEY_MENU,
	KEY_HOME,
	KEY_VOLUMEUP,
	KEY_SEARCH,
	KEY_BACK,
	KEY_VOLUMEDOWN,
	/* 7k_handset key */
	KEY_MEDIA,
	KEY_END,
};

int eta_abs_event[ETA_ABS_MAX]={
	ABS_X,
	ABS_Y,
	ABS_Z,
	ABS_MT_TRACKING_ID,
	ABS_MT_TOUCH_MAJOR,
	ABS_MT_TOUCH_MINOR,
	ABS_MT_POSITION_X,
	ABS_MT_POSITION_Y,
	SYN_MT_REPORT,
	SYN_REPORT
};

//ACTION filed information
typedef enum{
	ETA_TOUCH_MOVETO = 0, /*Move the pointer to the specified location*/
	ETA_TOUCH_MOVEBY = 1, /*Move the pointer by the specified values*/
	ETA_TOUCH_TAB = 2, /*Tab at the current location*/
	ETA_TOUCH_DOUBLETAB = 3, /*Double tab at the current location*/
	ETA_TOUCH_DOWN = 4, /*Touch down at the current location*/
	ETA_TOUCH_UP = 5, /*Touch up at the current location*/
	ETA_TOUCH_DEFAULT = 0xff,
}eta_touch_event_action_type;
static char eta_prev_action = ETA_TOUCH_DEFAULT;

typedef enum{
	TOUCH_STATUS_DEFAULT = 0, /*initial value*/
	TOUCH_STATUS_EVENT = 1, /*receiving events*/
	TOUCH_STATUS_READY = 2, /*ready to send events*/
}eta_touch_ststus_type;

int touch_status = TOUCH_STATUS_DEFAULT;


static int ats_event_log_connect(struct input_handler *handler,struct input_dev *dev,const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;

	if(strcmp(dev->name, "lge_ats_input") == 0)
		return 0;

	printk(" connect () %s \n\n",dev->name);

	for (i = 0 ; i < ETA_KEY_MAX - 1 ; i++){
		if (!test_bit(eta_key_list[i], dev->keybit))
			continue;
	}
	for (i = 0 ; i < ETA_ABS_MAX - 1 ; i++){
		if (!test_bit(eta_abs_event[i], dev->absbit))
			continue;
	}
	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if(!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "event_log";
	handle->private = NULL;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	return 0;
err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void ats_event_log_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id ats_event_log_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};


static void event_log_work_func(struct work_struct *work)
{
	int event_posted = 0;
	
#ifdef CONFIG_LGE_DIAG_MTC
	if((event_posted == 0)&&(g_diag_mtc_check==1))
	{
		mtc_send_key_log_data(&ats_mtc_key_log1);
		event_posted = 1;
	}
#endif

#ifdef CONFIG_LGE_DIAG_ICD
	if((event_posted == 0)&&(g_diag_icd_check==1))
	{
		slate_send_log_data(&ats_mtc_key_log1);
		//LGT_Icd_send_key_log_data(&ats_mtc_key_log1);
		event_posted = 1;
	}
#endif

	if(event_posted == 0)
	{
		ats_mtc_send_key_log_to_eta(&ats_mtc_key_log1);
		event_posted = 1;
	}
}

static void ats_event_log_event(struct input_handle *handle, unsigned int type,unsigned int code, int value)
{
	/*
	  * EV_SYN is followed by every key or touch events
	  * if EV_SYN for key event is not handled, the event will be recognized as touch event
	  */
	static unsigned int prev_action = EV_MAX;

	if ( (type == EV_KEY || prev_action == EV_KEY) && (0x00000001 & ats_mtc_log_mask) ){
		printk(KERN_INFO "%s, send key event, hold : %d, code : %d\n", __func__, value, code);
		prev_action = type;
		
		// if EV_SYN is sent, every key events will be handled as release event
		if(type == EV_SYN)
		{
			printk(KERN_INFO "%s, EV_SYN skip sending\n", __func__);
			return;
		}
		
		ats_mtc_key_log1.log_id = 1; /* LOG_ID, 1 key, 2 touch */
		ats_mtc_key_log1.log_len = 18; /* LOG_LEN */
		ats_mtc_key_log1.x_hold = value; /* hold */
		ats_mtc_key_log1.y_code = code;
		schedule_work(&event_log_work);
	}
	else if ( (type == EV_ABS || type == EV_SYN) && (0x00000002 & ats_mtc_log_mask) ){
		prev_action = type;
			
		switch(code){
			case ABS_MT_TOUCH_MAJOR:
			{
				touch_status = TOUCH_STATUS_EVENT;
#if defined(CONFIG_TOUCHSCREEN_MELFAS_MMS136)
				// mefas touch does not send these events for touch release
				// this will be handled in SYN_REPORT event
#else
				if(value == 1){ /* value = 1 is touch pressed case */
					if (eta_prev_action == ETA_TOUCH_DOWN)
						ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_MOVETO;
					else
						ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_DOWN;
					eta_prev_action = ETA_TOUCH_DOWN;
				}
				else {
					ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_UP;
					eta_prev_action = ETA_TOUCH_UP;
				}
#endif
				break;
			}
			case ABS_MT_POSITION_X :
			{
				//ats_mtc_key_log1.x_hold = value;
				ats_mtc_key_log1.x_hold = value * ATS_XMAX / touch_get_x_max();
				touch_status = TOUCH_STATUS_EVENT;
				break;
			}
			case ABS_MT_POSITION_Y:
			{
				//ats_mtc_key_log1.y_code = value;
				ats_mtc_key_log1.y_code = value * ATS_YMAX / touch_get_y_max();
				touch_status = TOUCH_STATUS_EVENT;
				break;
			}

			case SYN_MT_REPORT:
			{
				break;
			}
			
			case SYN_REPORT:
			{
#if defined(CONFIG_TOUCHSCREEN_MELFAS_MMS136)
				if(touch_status == TOUCH_STATUS_EVENT)
				{
					// received touch events, this means down or move events
					if (eta_prev_action == ETA_TOUCH_DOWN)
						ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_MOVETO;
					else
						ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_DOWN;
					eta_prev_action = ETA_TOUCH_DOWN;
				}
				else if(touch_status == TOUCH_STATUS_DEFAULT)
				{
					// received no touch events but just sync, this means up events
					ats_mtc_key_log1.action = (unsigned char)ETA_TOUCH_UP;
					eta_prev_action = ETA_TOUCH_UP;
				}
#endif

				touch_status = TOUCH_STATUS_READY;
				break;
			}
		}
		if(touch_status == TOUCH_STATUS_READY){
			ats_mtc_key_log1.log_id = 2; /* LOG_ID, 1 key, 2 touch */
			ats_mtc_key_log1.log_len = 22; /*LOG_LEN */
			touch_status = TOUCH_STATUS_DEFAULT;
			schedule_work(&event_log_work);
		}
	}
}

int event_log_start(void)
{
	int ret = 0;

	if(ats_event_log_status == 0)
	{
		input_handler.name = "key_log";
		input_handler.connect = ats_event_log_connect;
		input_handler.disconnect = ats_event_log_disconnect;
		input_handler.event = ats_event_log_event;
		input_handler.id_table = ats_event_log_ids;
		ret = input_register_handler(&input_handler);
		if (ret != 0)
			printk("%s:fail to registers input handler\n", __func__);

		INIT_WORK(&event_log_work,event_log_work_func);

		ats_event_log_status = 1;
	}
	return 0;
}
EXPORT_SYMBOL(event_log_start);

int event_log_end(void)
{
	if(ats_event_log_status == 1)
	{
		input_unregister_handler(&input_handler);
		ats_event_log_status = 0;
	}
	return 0 ;
}
EXPORT_SYMBOL(event_log_end);

/* [END] add ETA  key event logging for vs660 [younchan.kim 2010-05-31]*/

#ifndef BIGLAKE_TEST
static int __devinit ats_event_log_probe(struct platform_device *pdev)
{
	int rc = 0 ;
	return rc;
}

#else
static int  __devinit ats_event_log_probe(struct platform_device *pdev)
{
	int rc = 0 ;
	return rc;
}
#endif
static int __devexit ats_event_log_remove(struct platform_device *pdev)
{
	input_unregister_device(ats_event_log_dev);
	return 0;
}

static struct platform_driver ats_event_log_driver = {
	.probe	 = ats_event_log_probe,
	.remove = ats_event_log_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __devinit ats_event_log_init(void)
{
	return platform_driver_register(&ats_event_log_driver);
}


static void __devexit ats_event_log_exit(void)
{
	platform_driver_unregister(&ats_event_log_driver);
}

module_init(ats_event_log_init);
module_exit(ats_event_log_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("ATS_EVENT_LOG driver");
MODULE_LICENSE("GPL v2");
