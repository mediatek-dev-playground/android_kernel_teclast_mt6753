#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1200)
#define FRAME_HEIGHT (1920)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static void lcd_power_en_part0()
{
    mt_set_gpio_mode(55, 0);
    mt_set_gpio_dir(55, 1);
    mt_set_gpio_out(55, 1);
    pmic_config_interface(0xa72, 7, 7, 4);
    pmic_config_interface(0xa2c, 1, 1, 1);
};

static void lcd_power_en(unsigned char enabled)
{
	if (enabled) {
		lcd_power_en_part0();
	} else {
		pmic_config_interface(0xA72, 0, 7, 4);
		pmic_config_interface(0xA2C, 0, 1, 1);
		mt_set_gpio_mode(55, 0);
		mt_set_gpio_dir(55, 1);
		mt_set_gpio_out(55, 0);
	}
};

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

	params->type = 2;
	params->width = FRAME_WIDTH;
	params->height  = FRAME_HEIGHT;
	params->dsi.mode = 3;

	params->dsi.data_format.format = 2;
	params->dsi.PS = 2;
	params->dsi.vertical_backporch = 14;
	params->dsi.vertical_frontporch = 11;
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 110;
	params->dsi.LANE_NUM = 4;
	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active = 1;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.PLL_CLOCK = 490;
	params->dsi.ssc_disable = 1;
};

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x10500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x831500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x841500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x971500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0xAA831500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x11841500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x4BA91500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x4851500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x8861500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x109C1500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x6911500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

	data_array[0] = 0x110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x290500;
	dsi_set_cmdq(data_array, 1, 1);
};

static void lcm_init(void)
{
	lcd_power_en(0);
	MDELAY(30);
	lcd_power_en_part0();
	MDELAY(30);
	init_lcm_registers();
	MDELAY(120);
};

static void lcm_suspend(void)
{
	lcd_power_en(0);
	MDELAY(10);
};

static void lcm_resume(void)
{
	lcd_power_en(0);
	MDELAY(30);
	lcd_power_en_part0();
	MDELAY(30);
	init_lcm_registers();
	MDELAY(120);
};

LCM_DRIVER otm9608_qhd_dsi_vdo_drv = 
{
    .name			= "otm9608a_qhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};
