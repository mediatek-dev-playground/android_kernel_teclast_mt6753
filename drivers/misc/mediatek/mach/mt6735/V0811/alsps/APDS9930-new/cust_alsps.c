#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,//1,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level	= {2, 8,  30,   80,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    .als_value	= {1, 8,30,  50, 120, 280,  600,  1000, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    .ps_threshold_high = 210,//82,
    .ps_threshold_low = 190,//72,
    .is_batch_supported_ps = false,
    .is_batch_supported_als = false,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

