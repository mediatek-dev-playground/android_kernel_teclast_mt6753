/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "S5K3L2XXTRULYAF.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define LENS_I2C_BUSNUM 0

#define AF_DRVNAME "S5K3L2XXTRULYAF"
#define I2C_SLAVE_ADDRESS        0x18
#define I2C_REGISTER_ID            0x18//0x3A
#define PLATFORM_DRIVER_NAME "lens_actuator_s5k3l2xxtruly"
#define AF_DRIVER_CLASS_NAME "actuatordrv_s5k3l2xxtruly"

static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("S5K3L2XXTRULYAF", I2C_REGISTER_ID)};


#define S5K3L2XXTRULYAF_DRVNAME "S5K3L2XXTRULYAF"
#define S5K3L2XXTRULYAF_VCM_WRITE_ID           0x18

#define S5K3L2XXTRULYAF_DEBUG
#ifdef S5K3L2XXTRULYAF_DEBUG
#define S5K3L2XXTRULYAFDB printk
#else
#define S5K3L2XXTRULYAFDB(x,...)
#endif

static spinlock_t g_S5K3L2XXTRULYAF_SpinLock;

static struct i2c_client * g_pstS5K3L2XXTRULYAF_I2Cclient = NULL;

static dev_t g_S5K3L2XXTRULYAF_devno;
static struct cdev * g_pS5K3L2XXTRULYAF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4S5K3L2XXTRULYAF_Opened = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4S5K3L2XXTRULYAF_INF = 0;
static unsigned long g_u4S5K3L2XXTRULYAF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;

static int g_sr = 3;

#if 0
extern s32 mt_set_gpio_mode(u32 u4Pin, u32 u4Mode);
extern s32 mt_set_gpio_out(u32 u4Pin, u32 u4PinOut);
extern s32 mt_set_gpio_dir(u32 u4Pin, u32 u4Dir);
#endif

static int s4S5K3L2XXTRULYAF_ReadReg(unsigned short * a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

    i4RetValue = i2c_master_recv(g_pstS5K3L2XXTRULYAF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] I2C read failed!! \n");
        return -1;
    }

		*a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];

    return 0;
}

static int s4S5K3L2XXTRULYAF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

		char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xc4), (char)(a_u2Data & 0xff)};

    //S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] g_sr %d, write %d \n", g_sr, a_u2Data);
    g_pstS5K3L2XXTRULYAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstS5K3L2XXTRULYAF_I2Cclient, puSendCmd, 2);
	
    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] I2C send failed!! \n");
        return -1;
    }

    return 0;
}

inline static int getS5K3L2XXTRULYAFInfo(__user stS5K3L2XXTRULYAF_MotorInfo * pstMotorInfo)
{
    stS5K3L2XXTRULYAF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4S5K3L2XXTRULYAF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4S5K3L2XXTRULYAF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

	if (g_i4MotorStatus == 1)	{stMotorInfo.bIsMotorMoving = 1;}
	else						{stMotorInfo.bIsMotorMoving = 0;}

	if (g_s4S5K3L2XXTRULYAF_Opened >= 1)	{stMotorInfo.bIsMotorOpen = 1;}
	else						{stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stS5K3L2XXTRULYAF_MotorInfo)))
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] copy to user failed when getting motor information \n");
    }

    return 0;
}

inline static int moveS5K3L2XXTRULYAF(unsigned long a_u4Position)
{
    int ret = 0;
    
    if((a_u4Position > g_u4S5K3L2XXTRULYAF_MACRO) || (a_u4Position < g_u4S5K3L2XXTRULYAF_INF))
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] out of range \n");
        return -EINVAL;
    }

    if (g_s4S5K3L2XXTRULYAF_Opened == 1)
    {
        unsigned short InitPos;
        ret = s4S5K3L2XXTRULYAF_ReadReg(&InitPos);
	    
        spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
        if(ret == 0)
        {
            S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Init Pos %6d \n", InitPos);
            g_u4CurrPosition = (unsigned long)InitPos;
        }
        else
        {		
            g_u4CurrPosition = 0;
        }
        g_s4S5K3L2XXTRULYAF_Opened = 2;
        spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);
    }

    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);	
        g_i4Dir = 1;
        spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);	
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);	
        g_i4Dir = -1;
        spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);			
    }
    else										{return 0;}

    spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);	

    //S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);

            spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
            g_sr = 3;
            g_i4MotorStatus = 0;
            spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);	
		
            if(s4S5K3L2XXTRULYAF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
            {
                spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);		
                g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
                spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);				
            }
            else
            {
                S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] set I2C failed when moving the motor \n");			
                spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
                g_i4MotorStatus = -1;
                spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);				
            }

    return 0;
}

inline static int setS5K3L2XXTRULYAFInf(unsigned long a_u4Position)
{
    spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
    g_u4S5K3L2XXTRULYAF_INF = a_u4Position;
    spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);	
    return 0;
}

inline static int setS5K3L2XXTRULYAFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
    g_u4S5K3L2XXTRULYAF_MACRO = a_u4Position;
    spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);	
    return 0;	
}

////////////////////////////////////////////////////////////////
static long AF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] CMD = 0x%x \n",a_u4Command);
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAFIOC_G_MOTORINFO = 0x%lx \n",/*(unsigned int)*/S5K3L2XXTRULYAFIOC_G_MOTORINFO);
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAFIOC_T_MOVETO = 0x%lx  \n",/*(unsigned int)*/S5K3L2XXTRULYAFIOC_T_MOVETO);
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAFIOC_T_SETINFPOS = 0x%lx  \n",/*(unsigned int)*/S5K3L2XXTRULYAFIOC_T_SETINFPOS);
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAFIOC_T_SETMACROPOS = 0x%lx  \n",/*(unsigned int)*/S5K3L2XXTRULYAFIOC_T_SETMACROPOS);

    switch(a_u4Command)
    {
        case S5K3L2XXTRULYAFIOC_G_MOTORINFO :
            i4RetValue = getS5K3L2XXTRULYAFInfo((__user stS5K3L2XXTRULYAF_MotorInfo *)(a_u4Param));
        break;

        case S5K3L2XXTRULYAFIOC_T_MOVETO :
            i4RetValue = moveS5K3L2XXTRULYAF(a_u4Param);
        break;
 
        case S5K3L2XXTRULYAFIOC_T_SETINFPOS :
            i4RetValue = setS5K3L2XXTRULYAFInf(a_u4Param);
        break;

        case S5K3L2XXTRULYAFIOC_T_SETMACROPOS :
            i4RetValue = setS5K3L2XXTRULYAFMacro(a_u4Param);
        break;
		
        default :
      	    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int S5K3L2XXTRULYAF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAF_Open - Start\n");
    int  i4RetValue = 0;
    spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);

    if(g_s4S5K3L2XXTRULYAF_Opened)
    {
        spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] the device is opened \n");
        return -EBUSY;
    }

    g_s4S5K3L2XXTRULYAF_Opened = 1;
		
    spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);
char puSendCmd1[2]={(char)(0xCC),(char)(0x5A)};///rf[4:0]=0x07[80Hz],slew_rate[1:0]=01;
char puSendCmd2[2]={(char)(0xD4),(char)(0x32)};//uncont1 = 80 code; 
char puSendCmd3[2]={(char)(0xDC),(char)(0x3C)};//uncont2 = 100 code;
char puSendCmd4[2]={(char)(0xE4),(char)(0x41)};//stt[4:0]=0x01[50us],str[2:0]=0x02[2LSB];

 i4RetValue = i2c_master_send(g_pstS5K3L2XXTRULYAF_I2Cclient, puSendCmd1, 2);	
    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[BU64241] I2C send failed!! \n");
        return -1;
    }
 i4RetValue = i2c_master_send(g_pstS5K3L2XXTRULYAF_I2Cclient, puSendCmd2, 2);
    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[BU64241] I2C send failed!! \n");
        return -1;
    }
 i4RetValue = i2c_master_send(g_pstS5K3L2XXTRULYAF_I2Cclient, puSendCmd3, 2);
    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[BU64241] I2C send failed!! \n");
        return -1;
    }
 i4RetValue = i2c_master_send(g_pstS5K3L2XXTRULYAF_I2Cclient, puSendCmd4, 2);
    if (i4RetValue < 0) 
    {
        S5K3L2XXTRULYAFDB("[BU64241] I2C send failed!! \n");
        return -1;
    }

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAF_Open - End\n");

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int S5K3L2XXTRULYAF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAF_Release - Start\n");

    if (g_s4S5K3L2XXTRULYAF_Opened == 2)
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] feee \n");
        g_sr = 5;
	    s4S5K3L2XXTRULYAF_WriteReg(200);
        msleep(10);
	    s4S5K3L2XXTRULYAF_WriteReg(100);
        msleep(10);
        }

    if (g_s4S5K3L2XXTRULYAF_Opened)
    {
        S5K3L2XXTRULYAFDB("Free \n");
        	            	    	    
        spin_lock(&g_S5K3L2XXTRULYAF_SpinLock);
        g_s4S5K3L2XXTRULYAF_Opened = 0;
        spin_unlock(&g_S5K3L2XXTRULYAF_SpinLock);

    }

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAF_Release - End\n");

    return 0;
}

static const struct file_operations g_stS5K3L2XXTRULYAF_fops = 
{
    .owner = THIS_MODULE,
    .open = S5K3L2XXTRULYAF_Open,
    .release = S5K3L2XXTRULYAF_Release,
    .unlocked_ioctl = AF_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = AF_Ioctl,
#endif
};

inline static int Register_S5K3L2XXTRULYAF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Register_S5K3L2XXTRULYAF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_S5K3L2XXTRULYAF_devno, 0, 1,AF_DRVNAME) )
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pS5K3L2XXTRULYAF_CharDrv = cdev_alloc();

    if(NULL == g_pS5K3L2XXTRULYAF_CharDrv)
    {
        unregister_chrdev_region(g_S5K3L2XXTRULYAF_devno, 1);

        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pS5K3L2XXTRULYAF_CharDrv, &g_stS5K3L2XXTRULYAF_fops);

    g_pS5K3L2XXTRULYAF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pS5K3L2XXTRULYAF_CharDrv, g_S5K3L2XXTRULYAF_devno, 1))
    {
        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Attatch file operation failed\n");

        unregister_chrdev_region(g_S5K3L2XXTRULYAF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, AF_DRIVER_CLASS_NAME);
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        S5K3L2XXTRULYAFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_S5K3L2XXTRULYAF_devno, NULL, AF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Register_S5K3L2XXTRULYAF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_S5K3L2XXTRULYAF_CharDrv(void)
{
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Unregister_S5K3L2XXTRULYAF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pS5K3L2XXTRULYAF_CharDrv);

    unregister_chrdev_region(g_S5K3L2XXTRULYAF_devno, 1);
    
    device_destroy(actuator_class, g_S5K3L2XXTRULYAF_devno);

    class_destroy(actuator_class);

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Unregister_S5K3L2XXTRULYAF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int S5K3L2XXTRULYAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int S5K3L2XXTRULYAF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id S5K3L2XXTRULYAF_i2c_id[] = {{AF_DRVNAME,0},{}};   
struct i2c_driver S5K3L2XXTRULYAF_i2c_driver = {                       
    .probe = S5K3L2XXTRULYAF_i2c_probe,                                   
    .remove = S5K3L2XXTRULYAF_i2c_remove,                           
    .driver.name = AF_DRVNAME,                 
    .id_table = S5K3L2XXTRULYAF_i2c_id,                             
};  

#if 0 
static int S5K3L2XXTRULYAF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, S5K3L2XXTRULYAF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int S5K3L2XXTRULYAF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int S5K3L2XXTRULYAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] S5K3L2XXTRULYAF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstS5K3L2XXTRULYAF_I2Cclient = client;
    
    g_pstS5K3L2XXTRULYAF_I2Cclient->addr = S5K3L2XXTRULYAF_VCM_WRITE_ID;
    g_pstS5K3L2XXTRULYAF_I2Cclient->addr = g_pstS5K3L2XXTRULYAF_I2Cclient->addr >> 1;
    
    //Register char driver
    i4RetValue = Register_S5K3L2XXTRULYAF_CharDrv();

    if(i4RetValue){

        S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_S5K3L2XXTRULYAF_SpinLock);

    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF] Attached!! \n");

    return 0;
}

static int S5K3L2XXTRULYAF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&S5K3L2XXTRULYAF_i2c_driver);
}

static int S5K3L2XXTRULYAF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&S5K3L2XXTRULYAF_i2c_driver);
    return 0;
}

static int S5K3L2XXTRULYAF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int S5K3L2XXTRULYAF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stS5K3L2XXTRULYAF_Driver = {
    .probe		= S5K3L2XXTRULYAF_probe,
    .remove	= S5K3L2XXTRULYAF_remove,
    .suspend	= S5K3L2XXTRULYAF_suspend,
    .resume	= S5K3L2XXTRULYAF_resume,
    .driver		= {
        .name	= PLATFORM_DRIVER_NAME,
        .owner	= THIS_MODULE,
    }
};
static struct platform_device g_stAF_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};

static int __init S5K3L2XXTRULYAF_i2C_init(void)
{
    S5K3L2XXTRULYAFDB("[S5K3L2XXTRULYAF_i2C_init]\n");
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
    if(platform_device_register(&g_stAF_device)){
        S5K3L2XXTRULYAFDB("failed to register S5K3L2XXTRULYAF driver\n");
        return -ENODEV;
    }
	
    if(platform_driver_register(&g_stS5K3L2XXTRULYAF_Driver)){
        S5K3L2XXTRULYAFDB("failed to register S5K3L2XXTRULYAF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit S5K3L2XXTRULYAF_i2C_exit(void)
{
	platform_driver_unregister(&g_stS5K3L2XXTRULYAF_Driver);
}

module_init(S5K3L2XXTRULYAF_i2C_init);
module_exit(S5K3L2XXTRULYAF_i2C_exit);

MODULE_DESCRIPTION("S5K3L2XXTRULYAF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");


