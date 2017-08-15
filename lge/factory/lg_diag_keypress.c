#include <linux/module.h>
#include <linux/delay.h>
#include <lg_diagcmd.h>
#include <lg_diag_keypress.h>
#include <linux/input.h>
#include <mach/gpio.h>
/*==========================================================================*/
#define HS_RELEASE_K 0xFFFF
#define KEY_TRANS_MAP_SIZE 77

#define DEBUG_DIAG_KEYPRESS 1

/* Virtual Key */
#define V_KEY_DIAL					0x60
#define V_KEY_CALL_LOG				0x61
#define V_KEY_DELETE_ALL_CALL_LOG	0x62
#define V_KEY_OK_DELETE				0x63
#define V_KEY_UNLOCK	198
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
#define V_KEY_STAR	522
#define V_KEY_POUND	523
#define KEY_COORDINATES_SIZE 28
#else
#define V_KEY_STAR	227
#define V_KEY_POUND	228

#endif
#define DIAL_TOUCH_X  137	
#define DIAL_TOUCH_Y  1843
#define CALL_LOG_TOUCH_X  397
#define CALL_LOG_TOUCH_Y  222
#define DELET_ALL_TOUCH_X 978
#define DELET_ALL_TOUCH_Y 1916
#define DELET_OK_TOUCH_X 333
#define DELET_OK_TOUCH_Y 1231

#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA

typedef struct {
	unsigned int Android_key_code;	
	unsigned int x;
	unsigned int y;
}key_coordinates;

key_coordinates key_coordinates_table[KEY_COORDINATES_SIZE]={
	{ KEY_A, 27, 363},
	{ KEY_B, 194,424},	
	{ KEY_C, 131,419},
	{ KEY_D, 95, 367},
	{ KEY_E, 76, 320},
	{ KEY_F, 129,370},
	{ KEY_G, 163,371},
	{ KEY_H, 185,370},
	{ KEY_I, 239,325},
	{ KEY_J, 224,376},
	{ KEY_K, 252,370},
	{ KEY_L, 286,378},
	{ KEY_N, 229,418},
	{ KEY_M, 248,417},
	{ KEY_O, 268,337},
	{ KEY_P, 303,331},
	{ KEY_Q, 17, 328},
	{ KEY_R, 114,329},
	{ KEY_S, 63, 375},
	{ KEY_T, 145,332},
	{ KEY_U, 208,328},
	{ KEY_V, 159,426},
	{ KEY_W, 55, 322},
	{ KEY_X, 93, 416},
	{ KEY_Y, 181,332},
	{ KEY_Z, 66, 414},
	{ KEY_SPACE,137,469},
	{ KEY_DOT,  219,468},
};
#endif
extern struct input_dev* get_ats_input_dev(void);
typedef struct {
	  word LG_common_key_code;
	    unsigned int Android_key_code;
}keycode_trans_type;

keycode_trans_type keytrans_table[KEY_TRANS_MAP_SIZE]={
/* index = 0 */	{0x30, KEY_0},	
/* index = 1 */	{0x31, KEY_1},	
/* index = 2 */	{0x32, KEY_2},	
/* index = 3 */	{0x33, KEY_3},	
/* index = 4 */	{0x34, KEY_4},	
/* index = 5 */	{0x35, KEY_5},	
/* index = 6 */	{0x36, KEY_6},	
/* index = 7 */	{0x37, KEY_7},	
/* index = 8 */	{0x38, KEY_8},	
/* index = 9 */	{0x39, KEY_9},	
/* index = 10 */{0x2A, V_KEY_STAR},	
/* index = 11 */{0x23, V_KEY_POUND},
/* index = 12 */{0x50, KEY_SEND},
/* index = 13 */{0x51, KEY_END},
/* index = 14 */{0x52, V_KEY_UNLOCK},
/* index = 15 */{0x10, KEY_LEFT},	
/* index = 16 */{0x11, KEY_RIGHT},
/* index = 17 */{0x54, KEY_UP},
/* index = 18 */{0x55, KEY_DOWN},
/* index = 19 */{0x53, KEY_OK},
/* index = 20 */{0x96, KEY_VOLUMEUP},
/* index = 21 */{0x97, KEY_VOLUMEDOWN},
/* index = 22 */{0x51, KEY_POWER},
/* index = 23 */{0xA0, KEY_MENU},	
/* index = 24 */{0xA1, KEY_HOMEPAGE},		
/* index = 25 */{0xA2, KEY_BACK},			
/* index = 26 */{0xB0, V_KEY_DIAL}, 
/* index = 27 */{0xB1, V_KEY_CALL_LOG}, 
/* index = 28 */{0xB2, V_KEY_DELETE_ALL_CALL_LOG}, 
/* index = 29 */{0xB3, V_KEY_OK_DELETE},
/* index = 30 */{0xC0, KEY_A},
/* index = 31 */{0xC1, KEY_B},	
/* index = 32 */{0xC2, KEY_C},
/* index = 33 */{0xC3, KEY_D},
/* index = 34 */{0xC4, KEY_E},
/* index = 35 */{0xC5, KEY_F},
/* index = 36 */{0xC6, KEY_G},
/* index = 37 */{0xC7, KEY_H},
/* index = 38 */{0xC8, KEY_I},
/* index = 39 */{0xC9, KEY_J},
/* index = 40 */{0xCA, KEY_K},
/* index = 41 */{0xCB, KEY_L},
/* index = 42 */{0xCC, KEY_M},
/* index = 43 */{0xCD, KEY_N},
/* index = 44 */{0xCE, KEY_O},
/* index = 45 */{0xCF, KEY_P},
/* index = 46 */{0xD0, KEY_Q},
/* index = 47 */{0xD1, KEY_R},
/* index = 48 */{0xD2, KEY_S},
/* index = 49 */{0xD3, KEY_T},
/* index = 50 */{0xD4, KEY_U},
/* index = 51 */{0xD5, KEY_V},
/* index = 52 */{0xD6, KEY_W},
/* index = 53 */{0xD7, KEY_X},
/* index = 54 */{0xD8, KEY_Y},
/* index = 55 */{0xD9, KEY_Z},
/* index = 56 */{0xDA, KEY_SPACE},
/* index = 57 */{0xDB, KEY_DOT},
};

unsigned int LGF_KeycodeTrans(word input)
{
	int index = 0;
	unsigned int ret = (unsigned int)input;  // if we can not find, return the org value. 
 
	for( index = 0; index < KEY_TRANS_MAP_SIZE ; index++)
	{
		if( keytrans_table[index].LG_common_key_code == input)
		{
			ret = keytrans_table[index].Android_key_code;
			break;
		}
	}  
#ifdef DEBUG_DIAG_KEYPRESS
	printk(KERN_INFO "##DIAG_KEYPRESS## %s, input(%d), key_code(%d)\n", __func__, input, keytrans_table[index].Android_key_code);		
#endif
	return ret;
}

EXPORT_SYMBOL(LGF_KeycodeTrans);
/* ==========================================================================
===========================================================================*/
extern PACK(void *) diagpkt_alloc (diagpkt_cmd_code_type code, unsigned int length);
//extern unsigned int LGF_KeycodeTrans(word input);
/* 2012-10-15 JongWook-Park(blood9874@lge.com) [V3] DIAG Touch Key patch */
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
extern void Send_Touch( unsigned int x, unsigned int y, bool press);
#else
extern void Send_Touch( unsigned int x, unsigned int y);
#endif
/*==========================================================================*/

static unsigned saveKeycode =0 ;
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
extern void uts_input_key( unsigned int code, int value);
void uts_input_touch(unsigned int code, bool value)
{
	int i;
	printk("UTS_INPUT_TOUCH:: ENTER\n");
	if(KEY_Q <= code && code<=KEY_SPACE)
	{
		for(i=0;i<KEY_COORDINATES_SIZE;i++)
		{
			if(key_coordinates_table[i].Android_key_code==code)
			{
				printk("##UTS key_press: Match key code: %d, x: %d, y: %d \n",code,key_coordinates_table[i].x,key_coordinates_table[i].y);
				Send_Touch(key_coordinates_table[i].x,key_coordinates_table[i].y,value);
				break;
			}
		}
	}
}
#endif
void SendKey(unsigned int keycode, unsigned char bHold)
{
  struct input_dev *idev = get_ats_input_dev();

  if( keycode != HS_RELEASE_K)
  {
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
    if((keycode==KEY_MENU)||(keycode==KEY_HOMEPAGE)||(keycode==KEY_BACK))
	{
		uts_input_key(keycode, 1);
	}else
	uts_input_touch(keycode, 1);
#endif    
	input_report_key( idev,keycode , 1 ); // press event
	input_sync(idev);
  }

  if(bHold)
  {
    saveKeycode = keycode; 
  }
  else
  {
    if( keycode != HS_RELEASE_K)
    {
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
    if((keycode==KEY_MENU)||(keycode==KEY_HOMEPAGE)||(keycode==KEY_BACK))
    {
	uts_input_key(keycode, 0);
    }else
	  uts_input_touch(keycode, 0);
#endif    
      input_report_key( idev,keycode , 0 ); // release  event
      input_sync(idev);
    }
    else
    {
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
	if((saveKeycode==KEY_MENU)||(saveKeycode==KEY_HOMEPAGE)||(saveKeycode==KEY_BACK))
	{
	   uts_input_key(saveKeycode, 0);
	}else
	  uts_input_touch(saveKeycode, 0);
#endif	
      input_report_key( idev,saveKeycode , 0 ); // release  event
      input_sync(idev);
    }
  }
}

void LGF_SendKey(word keycode)
{
	struct input_dev* idev = NULL;

	idev = get_ats_input_dev();

	if(idev == NULL)
		printk("%s: input device addr is NULL\n",__func__);
	
	input_report_key(idev,(unsigned int)keycode, 1);
	input_sync(idev);
	input_report_key(idev,(unsigned int)keycode, 0);
	input_sync(idev);

}

EXPORT_SYMBOL(LGF_SendKey);

#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS100S
void Send_Touch( unsigned int x, unsigned int y)
{
	struct input_dev* idev = NULL;

	idev = get_ats_input_dev();

	if(idev == NULL){	//WBT #472461 Null Pointer Dereference : if idev == NULL return
		printk("%s: input device addr is NULL\n",__func__);
		return;
	}

	/* Press */
        input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
        input_report_abs(idev, ABS_MT_POSITION_X, x);
        input_report_abs(idev, ABS_MT_POSITION_Y, y);
        input_mt_sync(idev);
        input_sync(idev);
	/* Release */
        input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(idev, ABS_MT_POSITION_X, x);
        input_report_abs(idev, ABS_MT_POSITION_Y, y);
        input_mt_sync(idev);
        input_sync(idev);
}
#endif

PACK (void *)LGF_KeyPress (
        PACK (void	*)req_pkt_ptr,			/* pointer to request packet  */
        uint16		pkt_len )		      	/* length of request packet   */
{
  DIAG_HS_KEY_F_req_type *req_ptr = (DIAG_HS_KEY_F_req_type *) req_pkt_ptr;
  DIAG_HS_KEY_F_rsp_type *rsp_ptr;
  unsigned int keycode = 0;
  const int rsp_len = sizeof( DIAG_HS_KEY_F_rsp_type );

  rsp_ptr = (DIAG_HS_KEY_F_rsp_type *) diagpkt_alloc( DIAG_HS_KEY_F, rsp_len );
  if (!rsp_ptr)
  	return 0;

  if((req_ptr->magic1 == 0xEA2B7BC0) && (req_ptr->magic2 == 0xA5B7E0DF))
  {
    rsp_ptr->magic1 = req_ptr->magic1;
    rsp_ptr->magic2 = req_ptr->magic2;
    rsp_ptr->key = 0xff; //ignore byte key code
    rsp_ptr->ext_key = req_ptr->ext_key;

    keycode = LGF_KeycodeTrans((word) req_ptr->ext_key);
  }
  else
  {
    rsp_ptr->key = req_ptr->key;
    keycode = LGF_KeycodeTrans((word) req_ptr->key);

  }

  if( keycode == 0xff)
    keycode = HS_RELEASE_K;  // to mach the size
  
#ifdef DEBUG_DIAG_KEYPRESS		
	printk(KERN_INFO "##DIAG_KEYPRESS## %s, line(%d), keycode(%d), hold(%d)\n", __func__, __LINE__, keycode, req_ptr->hold);			  
#endif
/* 2012-10-15 JongWook-Park(blood9874@lge.com) [V3] DIAG Touch Key patch [START] */
#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
  switch (keycode){
  	case V_KEY_DIAL:
		Send_Touch(DIAL_TOUCH_X, DIAL_TOUCH_Y,true);
		Send_Touch(DIAL_TOUCH_X, DIAL_TOUCH_Y,false);
		break;
	case V_KEY_CALL_LOG:
	    Send_Touch(CALL_LOG_TOUCH_X, CALL_LOG_TOUCH_Y,true);
	    Send_Touch(CALL_LOG_TOUCH_X, CALL_LOG_TOUCH_Y,false);
		break;
	case V_KEY_DELETE_ALL_CALL_LOG:
		SendKey(KEY_MENU, req_ptr->hold);
		SendKey(KEY_MENU, !(req_ptr->hold));
		mdelay(1000);
		Send_Touch(DELET_ALL_TOUCH_X, DELET_ALL_TOUCH_Y,true);
		Send_Touch(DELET_ALL_TOUCH_X, DELET_ALL_TOUCH_Y,false);
	    break;
				
	case V_KEY_OK_DELETE:
	    Send_Touch(DELET_OK_TOUCH_X, DELET_OK_TOUCH_Y,true);
	    Send_Touch(DELET_OK_TOUCH_X, DELET_OK_TOUCH_Y,false);
	    break;

	default:
    	SendKey(keycode , req_ptr->hold);
		break;
  	}
#else
  switch (keycode){
  	case V_KEY_DIAL:
		Send_Touch(DIAL_TOUCH_X, DIAL_TOUCH_Y);
		break;
	case V_KEY_CALL_LOG:
	    Send_Touch(CALL_LOG_TOUCH_X, CALL_LOG_TOUCH_Y);
		break;
	case V_KEY_DELETE_ALL_CALL_LOG:
		SendKey(KEY_MENU, req_ptr->hold);
		SendKey(KEY_MENU, !(req_ptr->hold));
		mdelay(1000);
		Send_Touch(DELET_ALL_TOUCH_X, DELET_ALL_TOUCH_Y);
	    break;
				
	case V_KEY_OK_DELETE:
	    Send_Touch(DELET_OK_TOUCH_X, DELET_OK_TOUCH_Y);
	    break;

	default:
    	SendKey(keycode , req_ptr->hold);
		break;
  	}
#endif
/* 2012-10-15 JongWook-Park(blood9874@lge.com) [V3] DIAG Touch Key patch [END] */  	
  return (rsp_ptr);
}

EXPORT_SYMBOL(LGF_KeyPress);
