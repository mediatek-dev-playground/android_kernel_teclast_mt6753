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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

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

#define TAG_NAME "[strobe]-main-"
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

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR


static struct work_struct workTimeOut;

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

struct i2c_client *LM3560_i2c_client = NULL;

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

static int LM3560_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3560_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	return val;
}

static int LM3560_chip_init(struct LM3560_chip_data *chip)
{
	return 0;
}

static int LM3560_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3560_chip_data *chip;
	struct LM3560_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3560_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3560 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3560_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("LM3560 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3560_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3560_chip_init(chip)<0)
		goto err_chip_init;

	LM3560_i2c_client = client;
	PK_DBG("LM3560 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("LM3560 probe is failed \n");
	return -ENODEV;
}

static int LM3560_remove(struct i2c_client *client)
{
	struct LM3560_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3560_NAME "leds-LM3560"
static const struct i2c_device_id LM3560_id[] = {
	{LM3560_NAME, 0},
	{}
};

static struct i2c_driver LM3560_i2c_driver = {
	.driver = {
		.name  = LM3560_NAME,
	},
	.probe	= LM3560_probe,
	.remove   = LM3560_remove,
	.id_table = LM3560_id,
};

struct LM3560_platform_data LM3560_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3560={ I2C_BOARD_INFO(LM3560_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), \
													.platform_data = &LM3560_pdata,};

static int __init LM3560_init(void)
{
	PK_DBG("LM3560_init\n");
	//i2c_register_board_info(2, &i2c_LM3560, 1);
	i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_LM3560, 1);


	return i2c_add_driver(&LM3560_i2c_driver);
}

static void __exit LM3560_exit(void)
{
	i2c_del_driver(&LM3560_i2c_driver);
}


module_init(LM3560_init);
module_exit(LM3560_exit);

MODULE_DESCRIPTION("Flash driver for LM3560");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");


static int FL_Enable(void)
{
	char buf[2];
	int val;

	/*
		the main led is LXCL-PWF4, which support max torch current 500mA, max pulse current 1500mA, 
		note that the LM3560 support max torch is 250mA, max flash current is 1000mA
	*/
	
    if(g_duty<0)
        g_duty=0;
    else if(g_duty>15)
        g_duty=15;

	PK_DBG("FL_Enable duty=%d\n",g_duty);

	if(g_duty<=3)
	{
		//torch
		val = g_duty<<1; //0123====>0246, so the current is 31,93,156,218mA

		buf[0]=0xA0;
		buf[1]=val;
		//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

		buf[0]=0x10;
		buf[1]=0x0A;
		//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	}
	else
	{
		//flash

		val = g_duty; //456789 10 11,12,13,14,15,so the current is 312,375,437...750,812,875,937,1000mA
		buf[0]=0xB0;
		buf[1]=val;
		//writeReg(LM3560_i2c_client , 0xB0, val);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

		buf[0]=0x10;
		buf[1]=0x0B;
		//writeReg(LM3560_i2c_client , 0x10, 0x0B);
		LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	}
    return 0;
}



static int FL_Disable(void)
{
	char buf[2];

	PK_DBG("FL_Disable\n");

	///////////////////////
	buf[0]=0x10;
	buf[1]=0x00;
	//iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("FL_dim_duty duty=%d\n",duty);
	g_duty = duty;
    return 0;
}




static int FL_Init(void)
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

	buf[0]=0xC0;
	buf[1]=0x78;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

	buf[0]=0xA0;
	buf[1]=0x12;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);

	buf[0]=0xB0;
	buf[1]=0x77;
	LM3560_write_reg(LM3560_i2c_client, buf[0], buf[1]);
	*/
    PK_DBG("FL_Init\n");
    return 0;
}


static int FL_Uninit(void)
{
	//FL_Disable();
	
	mdelay(5);
	if(mt_set_gpio_out(GPIO_CAMERA_FLASH_EN_PIN,GPIO_OUT_ZERO)) {PK_DBG("[flashlight] uninit set gpio failed!!\n");}
	mdelay(5);

	PK_DBG("FL_Uninit\n");

	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	PK_DBG("ledTimeOut_callback\n");
    FL_Disable();
    //printk(KERN_ALERT "work handler function./n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
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
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
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
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}

				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( s, ms*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG("constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


