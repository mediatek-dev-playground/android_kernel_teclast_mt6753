#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(PFX	fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_debug(PFX  fmt, ##args); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);
extern void upmu_set_rg_vcama(kal_uint8 x);


int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };



    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

	upmu_set_rg_vcama(15);


    //power ON
    if (On) {

        PK_ERR("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

		// MAIN
        if(pinSetIdx == 0 ) {

			//===========================================
			if (currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))
			{
				ISP_MCLK1_EN(1);

                if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

	            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}

	            mdelay(1);

	            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}

	            mdelay(1);

	            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}

	            mdelay(1);

	            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}

	            mdelay(2);

                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

                if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}

				mdelay(5);

	        }

			//===========================================
			else if (currSensorName && (0 == strcmp(currSensorName, "s5k3l2mipiraw")))
			{
				ISP_MCLK1_EN(1);

                if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}

				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}

				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}

				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}

				mdelay(2);

				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}

				mdelay(5);

			}
			else if (currSensorName && (0 == strcmp("s5k3m2mipiraw", currSensorName)))	
			{
                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				mdelay(5);

				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(5);

				ISP_MCLK1_EN(1);
				mdelay(20);
			}
			else if (currSensorName && (0 == strcmp("s5k3p3mipiraw", currSensorName)))	
			{
				upmu_set_rg_vcama(8);

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}
				mdelay(5);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				mdelay(5);

				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(5);
				
				ISP_MCLK1_EN(1);
				mdelay(20);

			}
			else
			{
				goto _kdCISModulePowerOn_exit_;
			}
			
        }

		// SUB
        else if (pinSetIdx == 1) {

			//=============================================
			if ((currSensorName && (0 == strcmp(currSensorName,"ov5670mipiraw"))) ||
				(currSensorName && (0 == strcmp(currSensorName,"ov8858mipiraw"))))
			{

				#if 0 // L509 PR2 use case 2
				// case 1: dovdd short to pwrdn
                //if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
                //if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
                //if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}

                if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST1)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST1)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST1)\n");}

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_DPK_ERRBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}
				mdelay(1);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				//if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST1)\n");}
				mdelay(2);

				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1220,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}
				mdelay(5);

				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(10);

				ISP_MCLK1_EN(1);
				mdelay(10);

				#else
				// case 2: dovdd short to xshutdn(reset)
                if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
                if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(2);

                //if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST1)\n");}
                //if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST1)\n");}
                //if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST1)\n");}
                
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}
				mdelay(5);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}
				mdelay(5);

				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(10);

				ISP_MCLK1_EN(1);
				mdelay(30);

				#endif

			}
			else if (currSensorName && (0 == strcmp(currSensorName,"ov5648mipi")))
			{
                if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
                if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(2);
				
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);}
				mdelay(5);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);}
				mdelay(10);

				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(5);

				ISP_MCLK1_EN(1);
				mdelay(30);

			}
			else
			{
				goto _kdCISModulePowerOn_exit_;
			}
        }
    }
	
    else {//power OFF

        PK_ERR("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);

		//MAIN
        if(pinSetIdx == 0 ) {

			if((currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw"))) ||
				(currSensorName && (0 == strcmp(currSensorName, "s5k3l2mipiraw"))))
			{

				ISP_MCLK1_EN(0);

	            if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN)\n");}
	            if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN)\n");}
	            if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN)\n");}

	            if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
	            if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
	            if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
			}
			else if (currSensorName && (0 == strcmp("s5k3m2mipiraw", currSensorName)))
			{
			
				ISP_MCLK1_EN(0);
				mdelay(10);

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				mdelay(30);
			}
			else if (currSensorName && (0 == strcmp("s5k3p3mipiraw", currSensorName)))
			{
			
				ISP_MCLK1_EN(0);
				mdelay(10);

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				mdelay(20);
			}
			else if (currSensorName && (0 == strcmp("s5k3p3mipiraw", currSensorName)))
			{
			
				ISP_MCLK1_EN(0);
				mdelay(10);

                if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);}
				mdelay(20);
			}
        }

		//SUB
        else if (pinSetIdx == 1) {

			if ((currSensorName && (0 == strcmp(currSensorName,"ov5670mipiraw"))) ||
				(currSensorName && (0 == strcmp(currSensorName,"ov8858mipiraw"))))
			{
			
				#if 0 // L509 PR2 use case 2
				ISP_MCLK1_EN(0);
				mdelay(10);// case 1: dovdd short to pwrdn
	            if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMRST1)\n");}
	            if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMRST1)\n");}
	            if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMRST1)\n");}
				mdelay(1);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
	            mdelay(1);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(1);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(1);

				//if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
				//if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
	            //if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(10);
				#else
				// case 2: dovdd short to xshutdn(reset)
				ISP_MCLK1_EN(0);
				mdelay(10);

				if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
	            if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
	            mdelay(5);
				
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(20);
				#endif
				
			}
			else if (currSensorName && (0 == strcmp(currSensorName,"ov5648mipi")))
			{
				if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)) {PK_ERR("[CAMERA SENSOR] set gpio mode failed!! (CMPDN1)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)) {PK_ERR("[CAMERA SENSOR] set gpio dir failed!! (CMPDN1)\n");}
	            if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)) {PK_ERR("[CAMERA SENSOR] set gpio failed!! (CMPDN1)\n");}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n",CAMERA_POWER_VCAM_D);}
	            mdelay(5);
				
				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);}
				mdelay(5);

				if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {PK_ERR("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);}
				mdelay(5);

	            ISP_MCLK1_EN(0);
				mdelay(30);

			}
        }
    }

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


