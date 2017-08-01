#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
	#include <platform/upmu_common.h>
	#include <string.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <mach/upmu_common.h>
	#include <mach/upmu_sw.h>
	#include <mach/upmu_hw.h>
	#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY 0xFFFC
#define REGFLAG_UDELAY 0xFFFB

#define REGFLAG_END_OF_TABLE 0xFFFD  // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE  0

#define L503_LCD_ID_GPIO	(GPIO68 | 0x80000000)
#define HX8394A_HD720_ID    (0x94)
#define LCM_NAME 			"hx8394a_tcl"

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       
struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[128];
};

static struct LCM_setting_table lcm_init_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	//init_lcm_registers
};

static void init_lcm_registers(void)
{
	unsigned int data_array[20];

   	data_array[0] = 0x00043902;
    data_array[1] = 0x9483ffB9;
    dsi_set_cmdq(data_array, 2, 1);
  
    data_array[0] = 0x00023902;
    data_array[1] = 0x000053Ba;
    dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00113902;
    data_array[1] = 0x070001B1;
	data_array[2] = 0x11110189;
	data_array[3] = 0x3F3F3230;
	data_array[4] = 0xE6011247;
	data_array[5] = 0x000000E2;
	dsi_set_cmdq(data_array, 6, 1);

	data_array[0] = 0x00073902;
    data_array[1] = 0x08C800B2; 
	data_array[2] = 0x00A20004;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00203902;
    data_array[1] = 0x320680B4;
	data_array[2] = 0x15320310;
	data_array[3] = 0x08103208;
    data_array[4] = 0x05330433; 
    data_array[5] = 0x06330437;
	data_array[6] = 0x06066464;
	data_array[7] = 0x0A5F0644;
	data_array[8] = 0x0005706B;
	dsi_set_cmdq(data_array, 9, 1);
	
	data_array[0] = 0x00213902;                          
	data_array[1] = 0x0C0000D5;
	data_array[2] = 0x01000A00;
	data_array[3] = 0x0000CC00;
	data_array[4] = 0x88888800;
	data_array[5] = 0x88888888;
	data_array[6] = 0x01888888;
	data_array[7] = 0x01234567;
	data_array[8] = 0x88888823;
	data_array[9] = 0x00000088;
	dsi_set_cmdq(data_array, 10, 1); 

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x00000CB6; 
    dsi_set_cmdq(data_array, 2, 1);
	 
	data_array[0] = 0x00053902;                          
    data_array[1] = 0x001000C7; 
	data_array[2] = 0x00000010;
    dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00023902;                          
    data_array[1] = 0x000009CC; 
    dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x002B3902;
	data_array[1] = 0x070300E0;
	data_array[2]= 0x143D2F29;
	data_array[3]= 0x0E0C0636;
	data_array[4]= 0x13121412;
	data_array[5]= 0x03001712;
	data_array[6]= 0x3D2F2907;
	data_array[7]= 0x0C063614;
	data_array[8]= 0x1214120E;
	data_array[9]= 0x09171213;
	data_array[10]= 0x09120717;
	data_array[11]= 0x00120717;
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0] = 0x00053902;                          
    data_array[1] = 0x100206BF;
	data_array[2] = 0x00000004;
    dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00023902; 						 
	data_array[1] = 0x000032D4; 
	dsi_set_cmdq(data_array, 2, 1);

	// data_array[0] = 0x00023902;
	// data_array[1] = 0x00000035;
	// dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
  	MDELAY(150);
  	
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);	
}


static struct LCM_setting_table lcm_sleep_out_setting[] =
{
    // Sleep Out
	{0x11, 0, {}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {}},
    {REGFLAG_DELAY, 60, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;
				
			case REGFLAG_UDELAY :
				UDELAY(table[i].count);
				break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */	
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Video mode setting		

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 6;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 40;
	params->dsi.horizontal_backporch				= 60;
	params->dsi.horizontal_frontporch				= 60;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	//params->dsi.pll_select=1;	//0: MIPI_PLL; 1: LVDS_PLL
	//params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_234;//LCM_DSI_6589_PLL_CLOCK_221;//LCM_DSI_6589_PLL_CLOCK_208;///LCM_DSI_6589_PLL_CLOCK_201_5;//;//this value must be in MTK suggested table

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 220;
	params->dsi.esd_check_enable=1;

	params->dsi.customization_esd_check_enable=1;
	params->dsi.lcm_esd_check_table[0].cmd=0x0a;
	params->dsi.lcm_esd_check_table[0].count=1;
	params->dsi.lcm_esd_check_table[0].para_list[0]=0x1c;
	params->dsi.lcm_esd_check_table[1].cmd=0x09;
	params->dsi.lcm_esd_check_table[1].count=4;
	params->dsi.lcm_esd_check_table[1].para_list[0]=0x80;
	params->dsi.lcm_esd_check_table[1].para_list[1]=0x73;
	params->dsi.lcm_esd_check_table[1].para_list[2]=0x04;//te disable
	//params->dsi.lcm_esd_check_table[1].para_list[2]=0x06;//te enable
	params->dsi.lcm_esd_check_table[1].para_list[3]=0x00;

	params->dsi.HS_TRAIL=6;	//watt.feng add
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(180);

	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

   	SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	char  buffer;
	unsigned int data_array[2];
	int ret = 0;

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00023902;
	data_array[1]= (0x33<<8)|0xba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00043902;
	data_array[1]= (0x94<<24)|(0x83<<16)|(0xff<<8)|0xb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00013700;
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xf4, &buffer, 1);

#ifdef BUILD_LK
	printf("%s-%s LK debug: hx8394a id = 0x%08x\n", __func__, LCM_NAME, buffer);
#endif

	if (buffer == HX8394A_HD720_ID) 
	{
		char id_buf;
		data_array[0] = 0x00013700;
		dsi_set_cmdq(data_array, 1, 1);

		read_reg_v2(0xdc, &id_buf, 1);
		
#ifdef BUILD_LK
		printf("%s-%s LK debug: hx8394a idinfo = 0x%08x\n", __func__, LCM_NAME, id_buf);
#endif
		if (id_buf == 0x1A)
			return 1;

		/*
		below for lcd id diff by gpio
		mt_set_gpio_mode(L503_LCD_ID_GPIO,0); // 0:GPIO mode
		mt_set_gpio_dir(L503_LCD_ID_GPIO,GPIO_DIR_IN); // 0: input, 1: output
		mt_set_gpio_pull_enable(L503_LCD_ID_GPIO, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(L503_LCD_ID_GPIO, GPIO_PULL_DOWN);
		MDELAY(1);
		ret = mt_get_gpio_in(L503_LCD_ID_GPIO);

		return (ret == 1)?1:0;
		*/

	}

	return 0;

}



#ifndef BULID_LK
//use dcs read more then 3 values
extern unsigned int fbconfig_dsi_dcs_read_lcm_reg_v2(unsigned char cmd, unsigned char type,unsigned char *buffer, unsigned char buffer_size);
#endif
static unsigned int lcm_esd_check(void)
{
       unsigned int ret = FALSE;
#ifndef BUILD_LK
	char  buffer1[3];
	char buffer2[1];
	int   array[4];
       static unsigned char err_times = 0;
       
	array[0] = 0x00033700;
	dsi_set_cmdq(array, 1, 1);
	
	//read_reg_v2(0x09, buffer1, 3); //we need set dcs type
       fbconfig_dsi_dcs_read_lcm_reg_v2(0x09,0,buffer1,3);//second para 0-> dcs DSI_DCS_READ_PACKET_ID
	
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xd9, buffer2, 1); 


	//printk("lcm_esd_check read 0x09= 0x%x,0x%x,0x%x,0xd9=0x%x\n ", buffer1[0],buffer1[1],buffer1[2],buffer2[0]);
	//buffer[2]=4 te close  buffer[2]=6 te open
	if(buffer1[0]==0x80 && buffer1[1]==0x73 && (buffer1[2]==0x04 || buffer1[2]==0x06) \
	    && buffer2[0] ==0x80)
	{
		err_times = 0;
	}
	else
	{
		err_times++; //error
	}

	if(err_times == 2)
	{
	       err_times = 0;
		ret = TRUE;
	}
#endif

	return ret;
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
	printk("%s-%s\n", __func__, LCM_NAME);
	lcm_init();
#endif
	return TRUE;
}


static void lcm_init_power(void)
{
#ifndef FPGA_EARLY_PORTING
#ifdef BUILD_LK
	printf("%s-%s\n", __func__, LCM_NAME);
	pmic_set_register_value(PMIC_RG_VIO28_EN,1);
#else
	printk("%s-%s\n", __func__, LCM_NAME);
	hwPowerOn(MT6328_POWER_LDO_VIO28, VOL_DEFAULT, "LCM_DRV");
#endif
#endif
}


static void lcm_suspend_power(void)
{
#ifndef FPGA_EARLY_PORTING
#ifdef BUILD_LK
	printf("%s-%s\n", __func__, LCM_NAME);
	pmic_set_register_value(PMIC_RG_VIO28_EN,0);
#else
	printk("%s-%s\n", __func__, LCM_NAME);
	hwPowerDown(MT6328_POWER_LDO_VIO28, "LCM_DRV");	
#endif
#endif
}

static void lcm_resume_power(void)
{
#ifndef FPGA_EARLY_PORTING
#ifdef BUILD_LK
	printf("%s-%s\n", __func__, LCM_NAME);
	pmic_set_register_value(PMIC_RG_VIO28_EN,1);
#else
	printk("%s-%s\n", __func__, LCM_NAME);
	hwPowerOn(MT6328_POWER_LDO_VIO28, VOL_DEFAULT, "LCM_DRV");
#endif
#endif
}


LCM_DRIVER hx8394a_hd720_dsi_vdo_tcl_lcm_drv = 
{
    .name			= "hx8394a_hd720_dsi_vdo_tcl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
	//.esd_check 	= lcm_esd_check,
	//.esd_recover	= lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
