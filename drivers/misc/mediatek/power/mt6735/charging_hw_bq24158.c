/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "bq24158.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>
  
#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/diso.h>
#include "cust_diso.h"
#include <linux/of.h>
#include <linux/of_irq.h>
#ifdef MTK_DISCRETE_SWITCH
#include <mach/eint.h>
#include <cust_eint.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif
#if !defined(MTK_AUXADC_IRQ_SUPPORT)
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#endif
#endif 


 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number   = (168 | 0x80000000); 
#endif

kal_bool charging_type_det_done = KAL_TRUE;

static kal_uint32 charging_error;

const kal_uint32 VBAT_CV_VTH[]=
{
	BATTERY_VOLT_03_500000_V,   BATTERY_VOLT_03_520000_V,	BATTERY_VOLT_03_540000_V,   BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V,   BATTERY_VOLT_03_600000_V,	BATTERY_VOLT_03_620000_V,   BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V,	BATTERY_VOLT_03_680000_V,	BATTERY_VOLT_03_700000_V,	BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V,	BATTERY_VOLT_03_760000_V,	BATTERY_VOLT_03_780000_V,	BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V,	BATTERY_VOLT_03_840000_V,	BATTERY_VOLT_03_860000_V,	BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V,	BATTERY_VOLT_03_920000_V,	BATTERY_VOLT_03_940000_V,	BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V,	BATTERY_VOLT_04_000000_V,	BATTERY_VOLT_04_020000_V,	BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V,	BATTERY_VOLT_04_080000_V,	BATTERY_VOLT_04_100000_V,	BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V,   BATTERY_VOLT_04_160000_V,	BATTERY_VOLT_04_180000_V,   BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V,   BATTERY_VOLT_04_240000_V,	BATTERY_VOLT_04_260000_V,   BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_320000_V,	BATTERY_VOLT_04_340000_V,   BATTERY_VOLT_04_360000_V,	
	BATTERY_VOLT_04_380000_V,   BATTERY_VOLT_04_400000_V,	BATTERY_VOLT_04_420000_V,   BATTERY_VOLT_04_440000_V	
	
};

const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_650_00_MA,	CHARGE_CURRENT_750_00_MA, CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1050_00_MA,	CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1250_00_MA
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,	 CHARGE_CURRENT_500_00_MA,	 CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_MAX
 }; 

 const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 // ============================================================ //
 // function prototype
 // ============================================================ //
 
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
 extern void mt_power_off(void);

 // ============================================================ //
 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match \r\n");
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }


static void hw_bc11_dump_register(void)
{
  	 kal_uint32 reg_val = 0;
 	 kal_uint32 reg_num = MT6328_CHR_CON20;
 	 kal_uint32 i = 0;

 	 for(i=reg_num ; i<=MT6328_CHR_CON21 ; i+=2)
 	 {
	     reg_val = upmu_get_reg_value(i);
	     battery_xlog_printk(BAT_LOG_FULL, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
 	 }
}


 static void hw_bc11_init(void)
 {
 	 msleep(300);
	 Charger_Detect_Init();
    //RG_bc11_BIAS_EN=1
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,1);
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0);
    //bc11_RST=1
    bc11_set_register_value(PMIC_RG_BC11_RST,1);
    //bc11_BB_CTRL=1
    bc11_set_register_value(PMIC_RG_BC11_BB_CTRL,1);     
        //modify by ysx for bug0027997[FAQ11545]
 	 msleep(50);
 	 //mdelay(50);

	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
    		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	 }	
	 
 }
 
 
 static U32 hw_bc11_DCD(void)
 {
     U32 wChargerAvail = 0;
     //RG_bc11_IPU_EN[1.0] = 10
     bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2);  
     //RG_bc11_IPD_EN[1.0] = 01
     bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
     //RG_bc11_VREF_VTH = [1:0]=01
     bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
     //RG_bc11_CMP_EN[1.0] = 10
     bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x2);
     //modify by ysx for bug0027997[FAQ11545]
	 msleep(80);
	 //mdelay(80);
     wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	 
	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	 }
     //RG_bc11_IPU_EN[1.0] = 00
     bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
     //RG_bc11_IPD_EN[1.0] = 00
     bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0); 
     //RG_bc11_CMP_EN[1.0] = 00
     bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
     //RG_bc11_VREF_VTH = [1:0]=00
     bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
	 return wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA1(void)
 {
	U32 wChargerAvail = 0;

    //RG_bc11_IPD_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);

    //modify by ysx for bug0027997[FAQ11545]
	msleep(80);
	//mdelay(80);
    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}

    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);  
 
	return  wChargerAvail;
 }
 
 #if 0
 static U32 hw_bc11_stepB1(void)
 {
	U32 wChargerAvail = 0;
	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepC1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = upmu_get_rgs_bc11_cmp_out();
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 #endif
 
 static U32 hw_bc11_stepA2(void)
 {
	U32 wChargerAvail = 0;
    //RG_bc11_VSRC_EN[1.0] = 10 
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2);
    //RG_bc11_IPD_EN[1:0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);


    //modify by ysx for bug0027997[FAQ11545]
	msleep(80);
	//mdelay(80);

    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);    
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB2(void)
 {
	U32 wChargerAvail = 0;
    //RG_bc11_IPU_EN[1:0]=10
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2); 
    //RG_bc11_VREF_VTH = [1:0]=01
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
    //RG_bc11_CMP_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);    
    //modify by ysx for bug0027997[FAQ11545]
	msleep(80);
	//mdelay(80);
	
    wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB2() \r\n");
		hw_bc11_dump_register();
	}
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0); 
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0); 
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0); 
	return  wChargerAvail;
 }
 
 
 static void hw_bc11_done(void)
 {
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
    //RG_bc11_VREF_VTH = [1:0]=0
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
    //RG_bc11_BIAS_EN=0
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,0x0);
	Charger_Detect_Release();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}
    
 }


 static kal_uint32 charging_hw_init(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number,0); // 0:GPIO mode
	mt_set_gpio_dir(wireless_charger_gpio_number,0); // 0: input, 1: output
#endif
    battery_xlog_printk(BAT_LOG_FULL,"%s",__FUNCTION__);
    bc11_set_register_value(PMIC_RG_USBDL_SET,0x0);//force leave USBDL mode
    bc11_set_register_value(PMIC_RG_USBDL_RST,0x1);//force leave USBDL mode
   
	#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
        battery_xlog_printk(BAT_LOG_CRTI, "high voltage battery supported.\n");
        bq24158_config_interface_liao(0x06,0x77); // ISAFE = 1250mA, VSAFE = 4.34V
    #else
        battery_xlog_printk(BAT_LOG_CRTI, "high voltage battery not supported.\n");
        bq24158_config_interface_liao(0x06,0x70);
	#endif

	bq24158_config_interface_liao(0x02,0xAA);  //4.34V
    bq24158_config_interface_liao(0x00,0xC0);	//kick chip watch dog
    bq24158_config_interface_liao(0x01,0xb8/*0xbc*/);	//TE=1, CE=1, HZ_MODE=0, OPA_MODE=0
    bq24158_config_interface_liao(0x05,0x04);
    
    bq24158_config_interface_liao(0x04,0x4A); //186mA
    if ( !charging_init_flag ) {   
        bq24158_config_interface_liao(0x04,0x4A); //186mA
        charging_init_flag = KAL_TRUE;
    }	
 
	mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_SWCHARGER_EN_PIN_M_GPIO);  
    mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ZERO);	
 	return status;
 }


 static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	bq24158_dump_register();
   	
	return status;
 }	


 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);
	battery_xlog_printk(BAT_LOG_FULL, "enable=%d\r\n", enable);
	if(KAL_TRUE == enable)
	{
		bq24158_set_ce(0);
		bq24158_set_hz_mode(0);
		bq24158_set_opa_mode(0);
	}
	else
	{

#if defined(CONFIG_USB_MTK_HDRC_HCD)
   		if(mt_usb_is_device())
#endif 			
    	{
	        mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN,GPIO_SWCHARGER_EN_PIN_M_GPIO);  
	        mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN,GPIO_DIR_OUT);
	        mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN,GPIO_OUT_ONE);

	       // bq24158_set_ce(1);
    	}
	}
		
	return status;
 }


 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);
    battery_xlog_printk(BAT_LOG_FULL, "cv_value=%d\r\n", cv_value);
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH) ,*(kal_uint32 *)(data));
	bq24158_set_oreg(register_value); 

	return status;
 } 	


 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size;
    kal_uint8 reg_value;
	
    //Get current level
    array_size = GETARRAYNUM(CS_VTH);
    bq24158_read_interface(0x1, &reg_value, 0x3, 0x6);	//IINLIM
    *(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
    return status;
 }  
  


 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;
    battery_xlog_printk(BAT_LOG_FULL, "current_value=%d\r\n", current_value);
	if(current_value <= CHARGE_CURRENT_350_00_MA)
	{
		bq24158_set_io_level(1);
	}
	else
	{
		bq24158_set_io_level(0);
		array_size = GETARRAYNUM(CS_VTH);
		set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
		battery_xlog_printk(BAT_LOG_FULL, "register_value=%d\r\n",register_value);		
		bq24158_set_iocharge(register_value);
	}
	return status;
 } 	
 

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
    battery_xlog_printk(BAT_LOG_FULL, "input current=%d\r\n",*(kal_uint32 *)data);
    if(*(kal_uint32 *)data > CHARGE_CURRENT_500_00_MA)
    {
        register_value = 0x3;
    }
    else
    {
    	array_size = GETARRAYNUM(INPUT_CS_VTH);
    	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(kal_uint32 *)data);
    	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	
    }
    battery_xlog_printk(BAT_LOG_FULL, "register_value=%d\r\n",register_value);
    bq24158_set_input_charging_current(register_value);

	return status;
 } 	


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq24158_get_chip_status();
	
	if(ret_val == 0x2)
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;
	return status;
 } 	


 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
 
	 bq24158_set_tmr_rst(1);
	 
	 return status;
 }
 
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 
	 kal_uint32 set_hv_voltage;
	 kal_uint32 array_size;
	 kal_uint16 register_value;
	 kal_uint32 voltage = *(kal_uint32*)(data);
	 battery_xlog_printk(BAT_LOG_FULL, "voltage=%d\r\n",voltage);
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
     bc11_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);//upmu_set_rg_vcdt_hv_vth(register_value);
 
	 return status;
  }
 
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
	   kal_uint32 status = STATUS_OK;
       *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_VCDT_HV_DET);//upmu_get_rgs_vcdt_hv_det();
 	   
	   return status;
  }


 static kal_uint32 charging_get_battery_status(void *data)
 {
	   kal_uint32 status = STATUS_OK;
 
       bc11_set_register_value(PMIC_BATON_TDET_EN,1);
       bc11_set_register_value(PMIC_RG_BATON_EN,1); 
    
       *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_BATON_UNDET);//upmu_get_rgs_baton_undet();
	   return status;
 }


 static kal_uint32 charging_get_charger_det_status(void *data)
 {
	   kal_uint32 status = STATUS_OK;
	   *(kal_bool*)(data) = bc11_get_register_value(PMIC_RGS_CHRDET);//upmu_get_rgs_chrdet();
	
       if( bc11_get_register_value(PMIC_RGS_CHRDET)== 0 )
           g_charger_type = CHARGER_UNKNOWN;
	   return status;
 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}


 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
    int wireless_state = 0;
    wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
    if(wireless_state == WIRELESS_CHARGER_EXIST_STATE)
    {
        *(CHARGER_TYPE*)(data) = WIRELESS_CHARGER;
        battery_xlog_printk(BAT_LOG_CRTI, "WIRELESS_CHARGER!\r\n");
        return status;
    }
#endif
    if(g_charger_type!=CHARGER_UNKNOWN && g_charger_type!=WIRELESS_CHARGER)
    {
        *(CHARGER_TYPE*)(data) = g_charger_type;
        battery_xlog_printk(BAT_LOG_CRTI, "return %d!\r\n", g_charger_type);
        return status;
    }

	charging_type_det_done = KAL_FALSE;

	/********* Step initial  ***************/		 
	hw_bc11_init();
 
	/********* Step DCD ***************/  
	if(1 == hw_bc11_DCD())
	{
		/********* Step A1 ***************/
		if(1 == hw_bc11_stepA1())
		{
			*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
			battery_xlog_printk(BAT_LOG_CRTI, "step A1 : Apple 2.1A CHARGER!\r\n");
		}	 
		else
		{
			*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
			battery_xlog_printk(BAT_LOG_CRTI, "step A1 : Non STANDARD CHARGER!\r\n");
		}
	}
	else
	{
		 /********* Step A2 ***************/
		 if(1 == hw_bc11_stepA2())
		 {
			 /********* Step B2 ***************/
			 if(1 == hw_bc11_stepB2())
			 {
				 *(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			 }
			 else
			 {
				 *(CHARGER_TYPE*)(data) = CHARGING_HOST;
				 battery_xlog_printk(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			 }
		 }
		 else
		 {
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			 battery_xlog_printk(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		 }
 
	}
 
	 /********* Finally setting *******************************/
	 hw_bc11_done();

 	charging_type_det_done = KAL_TRUE;

	g_charger_type = *(CHARGER_TYPE*)(data);
#endif
	 return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
    kal_uint32 status = STATUS_OK;

    if(slp_get_wake_reason() == WR_PCM_TIMER)
        *(kal_bool*)(data) = KAL_TRUE;
    else
        *(kal_bool*)(data) = KAL_FALSE;

    battery_xlog_printk(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
       
    return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    *(kal_uint32*)(data) = get_boot_mode();

    battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
    kal_uint32 status = STATUS_OK;

#if 0 //#if defined(MTK_POWER_EXT_DETECT)
    if (MT_BOARD_PHONE == mt_get_board_type())
        *(kal_bool *)data = KAL_FALSE;
    else
        *(kal_bool *)data = KAL_TRUE;
#else
        *(kal_bool *)data = KAL_FALSE;
#endif

    return status;
}
static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}
static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 increase = *(kal_uint32*)(data);

	if(increase == KAL_TRUE)
	{
		
	}
	else
	{
		
	}

	return status;
}
static kal_uint32 charging_get_error_state(void)
{
	return charging_error;
}
static kal_uint32 charging_set_error_state(void *data)
{
	kal_uint32 status = STATUS_OK;
	charging_error = *(kal_uint32*)(data);

	return status;
}
static kal_uint32 charging_diso_init(void *data)
{	 
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	/* Initialization DISO Struct */
	pDISO_data->diso_state.cur_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.cur_vdc_state	 = DISO_OFFLINE;

	pDISO_data->diso_state.pre_otg_state	 = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vusb_state = DISO_OFFLINE;
	pDISO_data->diso_state.pre_vdc_state	 = DISO_OFFLINE;

	pDISO_data->chr_get_diso_state = KAL_FALSE;
	pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;

#if !defined(MTK_AUXADC_IRQ_SUPPORT)
	hrtimer_init(&diso_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	diso_kthread_timer.function = diso_kthread_hrtimer_func;
	INIT_DELAYED_WORK(&diso_polling_work, diso_polling_handler);

	kthread_run(diso_thread_kthread, NULL, "diso_thread_kthread");
	battery_xlog_printk(BAT_LOG_CRTI, "[%s] done\n", __func__);
#else
	struct device_node *node;
	int ret;

	//Initial AuxADC IRQ
	DISO_IRQ.vdc_measure_channel.number   = AP_AUXADC_DISO_VDC_CHANNEL;
	DISO_IRQ.vusb_measure_channel.number  = AP_AUXADC_DISO_VUSB_CHANNEL;
	DISO_IRQ.vdc_measure_channel.period   = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vusb_measure_channel.period  = AUXADC_CHANNEL_DELAY_PERIOD;
	DISO_IRQ.vdc_measure_channel.debounce	  = AUXADC_CHANNEL_DEBOUNCE;
	DISO_IRQ.vusb_measure_channel.debounce	= AUXADC_CHANNEL_DEBOUNCE;

	/* use default threshold voltage, if use high voltage,maybe refine*/
	DISO_IRQ.vusb_measure_channel.falling_threshold  = VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.falling_threshold	 = VDC_MIN_VOLTAGE/1000;
	DISO_IRQ.vusb_measure_channel.rising_threshold	= VBUS_MIN_VOLTAGE/1000;
	DISO_IRQ.vdc_measure_channel.rising_threshold	  = VDC_MIN_VOLTAGE/1000;

	node = of_find_compatible_node(NULL, NULL, "mediatek,AUXADC");
	if (!node) {
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: of_find_compatible_node failed!!\n");
	} else {
		pDISO_data->irq_line_number = irq_of_parse_and_map(node, 0);
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: IRQ Number: 0x%x\n", pDISO_data->irq_line_number);
	}

	mt_irq_set_sens(pDISO_data->irq_line_number, MT_EDGE_SENSITIVE);
	mt_irq_set_polarity(pDISO_data->irq_line_number, MT_POLARITY_LOW);

	ret = request_threaded_irq(pDISO_data->irq_line_number, diso_auxadc_irq_handler, \
	pDISO_data->irq_callback_func, IRQF_ONESHOT  , "DISO_ADC_IRQ", NULL);

	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: request_irq failed.\n");
	} else {
		set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
		set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
		battery_xlog_printk(BAT_LOG_CRTI, "[diso_adc]: diso_init success.\n");
	}
#endif

#if defined(MTK_DISCRETE_SWITCH) && defined(MTK_DSC_USE_EINT)
	battery_xlog_printk(BAT_LOG_CRTI, "[diso_eint]vdc eint irq registitation\n");
	mt_eint_set_hw_debounce(CUST_EINT_VDC_NUM, CUST_EINT_VDC_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_VDC_NUM, CUST_EINTF_TRIGGER_LOW, vdc_eint_handler, 0);
	mt_eint_mask(CUST_EINT_VDC_NUM); 
#endif
#endif

	return status;
}

static kal_uint32 charging_get_diso_state(void *data)
{
	kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	int diso_state = 0x0;
	DISO_ChargerStruct *pDISO_data = (DISO_ChargerStruct *)data;

	_get_diso_interrupt_state();
	diso_state = g_diso_state;
	battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] current diso state is %s!\n", DISO_state_s[diso_state]);
	if(((diso_state >> 1) & 0x3) != 0x0)
	{
		switch (diso_state){
			case USB_ONLY:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
#ifdef MTK_DISCRETE_SWITCH
#ifdef MTK_DSC_USE_EINT
				mt_eint_unmask(CUST_EINT_VDC_NUM); 
#else
				set_vdc_auxadc_irq(DISO_IRQ_ENABLE, 1);
#endif
#endif
				pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_ONLY:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_ENABLE, DISO_IRQ_RISING);
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_WITH_USB:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_ENABLE,DISO_IRQ_FALLING);
				pDISO_data->diso_state.cur_vusb_state  = DISO_ONLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_OFFLINE;
				break;
			case DC_WITH_OTG:
				set_vdc_auxadc_irq(DISO_IRQ_DISABLE, 0);
				set_vusb_auxadc_irq(DISO_IRQ_DISABLE, 0);
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_ONLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_ONLINE;
				break;
			default: // OTG only also can trigger vcdt IRQ
				pDISO_data->diso_state.cur_vusb_state  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_vdc_state	  = DISO_OFFLINE;
				pDISO_data->diso_state.cur_otg_state	  = DISO_ONLINE;
				battery_xlog_printk(BAT_LOG_CRTI, " switch load vcdt irq triggerd by OTG Boost!\n");
				break; // OTG plugin no need battery sync action
		}
	}

	if (DISO_ONLINE == pDISO_data->diso_state.cur_vdc_state)
		pDISO_data->hv_voltage = VDC_MAX_VOLTAGE;
	else
		pDISO_data->hv_voltage = VBUS_MAX_VOLTAGE;
#endif

	return status;
}
 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off			//watt.feng add 2015/5/18 11:27:43
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
	,charging_diso_init
	,charging_get_diso_state
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
	 return status;
 }


