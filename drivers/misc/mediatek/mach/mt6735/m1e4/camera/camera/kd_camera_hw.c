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
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
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

extern void ISP_MCLK1_EN(BOOL En);

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

	if (On) {//power ON

		ISP_MCLK1_EN(1);

		printk(KERN_INFO "[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

		//(GPIO68 | 0x80000000)
		mt_set_gpio_mode(GPIO_SPI_MOSI_PIN,GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_SPI_MOSI_PIN,GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ONE);

		if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC5005MIPI_RAW,currSensorName))) && (SensorIdx == DUAL_CAMERA_MAIN_SENSOR))
		{
			//(GPIO7 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			//(GPIO44 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}

			//VCAM_IO
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			if(TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			//VCAM_A
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			//AF_VCC
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(2);

			//(GPIO7 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			mdelay(1);

			//(GPIO44 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}
			
			mdelay(5);
		}
		else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2235MIPI_RAW,currSensorName))) && (SensorIdx == DUAL_CAMERA_SUB_SENSOR))
		{
			if ((GPIO_CAMERA_INVALID != CAMERA_CMRST_PIN) && (GPIO_CAMERA_INVALID != CAMERA_CMPDN_PIN)) {
				if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			//(GPIO12 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			//(GPIO11 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}
			
			mdelay(1);

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			if(TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable digital power (VCAM_D), power id = %d \n", CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(1);

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);

			//(GPIO12 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			mdelay(1);

			//(GPIO11 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}
			
			mdelay(5);
		}

	}
	else {//power OFF

		ISP_MCLK1_EN(0);

		printk(KERN_INFO "[PowerOFF]pinSetIdx:%d\n", pinSetIdx);

		//(GPIO68 | 0x80000000)
		mt_set_gpio_out(GPIO_SPI_MOSI_PIN,GPIO_OUT_ZERO);

		if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC5005MIPI_RAW,currSensorName))) && (SensorIdx == DUAL_CAMERA_MAIN_SENSOR))
		{
			//(GPIO7 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN_PIN,CAMERA_CMPDN_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			//(GPIO44 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST_PIN,CAMERA_CMRST_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}

			if(TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n", SUB_CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			//VCAM_A
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			//VCAM_IO
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			//AF_VCC
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
			}

		}
		else if ((currSensorName && (0 == strcmp(SENSOR_DRVNAME_GC2235MIPI_RAW,currSensorName))) && (SensorIdx == DUAL_CAMERA_SUB_SENSOR))
		{
			//(GPIO12 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMPDN1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMPDN1_PIN,CAMERA_CMPDN1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
				if(mt_set_gpio_dir(CAMERA_CMPDN1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
				if(mt_set_gpio_out(CAMERA_CMPDN1_PIN,GPIO_OUT_ONE)){printk(KERN_INFO "[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
			}

			//(GPIO11 | 0x80000000)
			if (GPIO_CAMERA_INVALID != CAMERA_CMRST1_PIN) {
				if(mt_set_gpio_mode(CAMERA_CMRST1_PIN,CAMERA_CMRST1_PIN_M_GPIO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
				if(mt_set_gpio_dir(CAMERA_CMRST1_PIN,GPIO_DIR_OUT)){printk(KERN_INFO "[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
				if(mt_set_gpio_out(CAMERA_CMRST1_PIN,GPIO_OUT_ZERO)){printk(KERN_INFO "[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
			}

			if(TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D,mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF core power (VCAM_D), power id = %d \n", SUB_CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}

			//VCAM_A
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
				goto _kdCISModulePowerOn_exit_;
			}

			//VCAM_IO
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
				goto _kdCISModulePowerOn_exit_;
			}

			//AF_VCC
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
			{
				printk(KERN_INFO "[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
				goto _kdCISModulePowerOn_exit_;
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
