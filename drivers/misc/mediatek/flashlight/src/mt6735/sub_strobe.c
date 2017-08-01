
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"

#include <linux/i2c.h>
#include <linux/leds.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE

#define TAG_NAME "[strobe]-sub-"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

extern struct i2c_client *LM3560_i2c_client;


/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_substrobeSMPLock); /* cotta-- SMP proection */


static u32 substrobe_Res = 0;
static u32 substrobe_Timeus = 0;
static BOOL g_substrobe_On = 0;

static int g_duty_sub=-1;
static int g_subtimeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_substrobeSem);
#else
static DECLARE_MUTEX(g_substrobeSem);
#endif


#define STROBE_DEVICE_ID I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR


static struct work_struct subworkTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

struct LM3560_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3560_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3560_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};


/* i2c access*/

static int LM3560_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	int retry_fl = 3;
	struct LM3560_chip_data *chip = i2c_get_clientdata(client);

	PK_DBG("0x%02x 0x%02x\n",reg,val);
	mutex_lock(&chip->lock);
	while (retry_fl > 0)
	{
		ret =  i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0)
		{
			PK_ERR("failed writting at 0x%02x to 0x%02x,ret=%d, try_time=%d\n", reg,val,ret,retry_fl);
			retry_fl--;
		}
		else
			break;
	}
	mutex_unlock(&chip->lock);

	return ret;
}

/*
static int LM3560_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3560_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}
*/

static int sub_FL_Enable(void)
{
	char buf[2];
	int val;
	/*
		the sub led is LXCL-PY01, which support max torch current 100mA, max pulse current 200mA
		note that the LM3560 support max torch is 250mA, max flash current is 1000mA
	*/
    if(g_duty_sub<0)
        g_duty_sub=0;
    else if(g_duty_sub>4)
        g_duty_sub=4;

	PK_DBG("sub_FL_Enable duty=%d\n",g_duty_sub);

	if(g_duty_sub<=2)
	{
		//torch
		
		val=g_duty_sub; //012,the current is 31,62,93mA

		buf[0]=0xA0;
		buf[1]=val<<3;
		//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

		buf[0]=0x10;
		buf[1]=0x12;
		//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	}
	else
	{
		//flash

		val = g_duty_sub-2;	//34===>12,so the current is 125,187

		buf[0]=0xB0;
		buf[1]=val<<4;
		//writeReg(LM3560_i2c_client , 0xB0, val);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

		buf[0]=0x10;
		buf[1]=0x13;
		//writeReg(LM3560_i2c_client , 0x10, 0x0B);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	}
    return 0;
}



static int sub_FL_Disable(void)
{
	char buf[2];

	PK_DBG("FL_Disable\n");

	buf[0]=0x10;
	buf[1]=0x00;
	//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	return 0;
}


static int sub_FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("sub_FL_dim_duty duty=%d\n",duty);
	g_duty_sub = duty;
    return 0;
}




static int sub_FL_Init(void)
{
	//int regVal0;
	char buf[2];

	if(mt_set_gpio_mode(GPIO_CAMERA_FLASH_EN_PIN,GPIO_CAMERA_FLASH_EN_PIN_M_GPIO)) {PK_DBG("[flashlight] set gpio mode failed!!\n");}
	if(mt_set_gpio_dir(GPIO_CAMERA_FLASH_EN_PIN,GPIO_DIR_OUT)) {PK_DBG("[flashlight] set gpio dir failed!!\n");}
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO)) {PK_DBG("[flashlight] set gpio failed!!\n");}

	mdelay(5);
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ONE)) {PK_DBG("[flashlight] set gpio one failed!!\n");}

	mdelay(5);

	/*
	buf[0]=0x10;
	buf[1]=0x0;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

	buf[0]=0xA0;
	buf[1]=0x0;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

	buf[0]=0xB0;
	buf[1]=0x0;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

	buf[0]=0xC0;
	buf[1]=0x0;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	*/

    PK_DBG("sub_FL_Init\n");
    return 0;
}


static int sub_FL_Uninit(void)
{
	//sub_FL_Disable();

	mdelay(5);
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO)) {PK_DBG("[flashlight] uninit set gpio failed!!\n");}
	mdelay(5);

	PK_DBG("sub_FL_Uninit\n");

	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	PK_DBG("ledTimeOut_callback\n");
	sub_FL_Disable();
    //printk(KERN_ALERT "work handler function./n");
}


static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&subworkTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_subtimeOutTimer;
static void sub_timerInit(void)
{
  INIT_WORK(&subworkTimeOut, work_timeOutFunc);
	g_subtimeOutTimeMs=1000; //1s
	hrtimer_init( &g_subtimeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_subtimeOutTimer.function=ledTimeOutCallback;

}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("ior=%d, iow=%d iowr=%d arg=%d\n",ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_subtimeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		sub_FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{

    		    int s;
    		    int ms;
    		    if(g_subtimeOutTimeMs>1000)
            	{
            		s = g_subtimeOutTimeMs/1000;
            		ms = g_subtimeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_subtimeOutTimeMs;
            	}

				if(g_subtimeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_subtimeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			sub_FL_Enable();
    		}
    		else
    		{
    			sub_FL_Disable();
				hrtimer_cancel( &g_subtimeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;

}

static int sub_strobe_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("sub_strobe_open line=%d\n", __LINE__);

	if (0 == substrobe_Res)
	{
	    sub_FL_Init();
		sub_timerInit();
	}
	PK_DBG("sub_strobe_open line=%d\n", __LINE__);
	spin_lock_irq(&g_substrobeSMPLock);


    if(substrobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        substrobe_Res += 1;
    }


    spin_unlock_irq(&g_substrobeSMPLock);
    PK_DBG("sub_strobe_open line=%d\n", __LINE__);

    return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub_strobe_release\n");

    if (substrobe_Res)
    {
        spin_lock_irq(&g_substrobeSMPLock);

        substrobe_Res = 0;
        substrobe_Timeus = 0;

        /* LED On Status */
        g_substrobe_On = FALSE;

        spin_unlock_irq(&g_substrobeSMPLock);

    	sub_FL_Uninit();
    }

    PK_DBG(" Done\n");
    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





