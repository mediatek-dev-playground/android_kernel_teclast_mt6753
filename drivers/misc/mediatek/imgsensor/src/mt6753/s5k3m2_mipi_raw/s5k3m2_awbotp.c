#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/xlog.h>


#define PFX "S5K3M2_awbotp"
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOG_ERR(format, args...)	xlog_printk(ANDROID_LOG_ERROR, PFX, "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)


// =====  EEPROM =====
#define S5K3M2_EEPROM_READ_ID   0xA0
#define S5K3M2_EEPROM_I2C_SPEED 100  
#define S5K3M2_MAX_OFFSET		0xFFFF

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3M2_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3M2_EEPROM_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3M2_EEPROM_READ_ID)<0)
		return false;
	LOG_INF("read 0x%x=0x%x\n",addr,*data);
    return true;
}



// =====  moduler  =====

#define S5K3M2_CMOS_I2C_SPEED 300  
static kal_uint16 s5k3m2_cmos_slave_addr = 0x5A;

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kdSetI2CSpeed(S5K3M2_CMOS_I2C_SPEED); // Add this func to set i2c speed by each sensor
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, s5k3m2_cmos_slave_addr);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    kdSetI2CSpeed(S5K3M2_CMOS_I2C_SPEED); // Add this func to set i2c speed by each sensor
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, s5k3m2_cmos_slave_addr);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kdSetI2CSpeed(S5K3M2_CMOS_I2C_SPEED); // Add this func to set i2c speed by each sensor
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,s5k3m2_cmos_slave_addr);
    return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    kdSetI2CSpeed(S5K3M2_CMOS_I2C_SPEED); // Add this func to set i2c speed by each sensor
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, s5k3m2_cmos_slave_addr);
}


/*AWB OTP Part. Start*/

#define Sunny_SUPPORT_OTP
#ifdef Sunny_SUPPORT_OTP

#define SUNNY_ID           0x01

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_RED_ADDR      0x0210
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_GREEN2_ADDR   0x0214


kal_uint32 sunny_r_ratio = 0;
kal_uint32 sunny_b_ratio = 0;
kal_uint32 GOLDEN_RG_RATIO = 504;	// R/G Typical value
kal_uint32 GOLDEN_BG_RATIO = 536;	// B/G Typical value


/*
	[get_data_checksum]
	
			SUM(firstdata,endata)%0xff
*/
static unsigned char get_data_checksum(unsigned char* pData, unsigned int dataLen)
{
	int i = 0;
	int data_sum = 0;
	unsigned char chs = 0;

	if ( (0==dataLen) || (NULL==pData) )
		return 0;

	for (i=0; i<dataLen; i++)
		data_sum += pData[i];

	chs = (unsigned char)(data_sum%0xff);

	LOG_INF("len=%d,checksum=%d",dataLen,chs);

	return chs;
}


/*************************************************************************************************
* Function    :  sunny_wb_gain_set
* Description :  Set WB ratio to register gain setting  512x
* Parameters  :  [int] sunny_r_ratio : R ratio data compared with golden module R
                       sunny_b_ratio : B ratio data compared with golden module B
* Return      :  [bool] 0 : set wb fail 
                        1 : WB set success            
**************************************************************************************************/

static bool sunny_wb_gain_set()
{
	USHORT R_GAIN, R_GAIN_H, R_GAIN_L;
	USHORT B_GAIN, B_GAIN_H, B_GAIN_L;
	USHORT Gr_GAIN;
	USHORT Gb_GAIN;
	USHORT G_GAIN, G_GAIN_H, G_GAIN_L;
		
   if(!sunny_r_ratio || !sunny_b_ratio)
   {
            LOG_INF("otp_sunny WB ratio Data Err!");
            return 0;
   }


	if(sunny_b_ratio < GOLDEN_BG_RATIO)                                    
	{                                                                                        
		if (sunny_r_ratio < GOLDEN_RG_RATIO)                                      
		{                                                                         
			G_GAIN = GAIN_DEFAULT;                                                  
			B_GAIN = GAIN_DEFAULT * GOLDEN_BG_RATIO / sunny_b_ratio;                
			R_GAIN = GAIN_DEFAULT * GOLDEN_RG_RATIO / sunny_r_ratio;                
		}                                                                         
		else                                                                      
		{                                                                         
			R_GAIN = GAIN_DEFAULT;                                                  
			G_GAIN = GAIN_DEFAULT * sunny_r_ratio / GOLDEN_RG_RATIO;                
			B_GAIN = G_GAIN * GOLDEN_BG_RATIO / sunny_b_ratio;                      
		}                                                                         
	}                                                                           
	else                                                                        
	{                                                                           
		if (sunny_r_ratio < GOLDEN_RG_RATIO)                                      
		{                                                                         
			B_GAIN = GAIN_DEFAULT;                                                  
			G_GAIN = GAIN_DEFAULT * sunny_b_ratio / GOLDEN_BG_RATIO;                
			R_GAIN = G_GAIN * GOLDEN_RG_RATIO / sunny_r_ratio;                      
		}                                                                         
		else                                                                      
		{                                                                         
			Gb_GAIN = GAIN_DEFAULT * sunny_b_ratio / GOLDEN_BG_RATIO;              
			Gr_GAIN = GAIN_DEFAULT * sunny_r_ratio / GOLDEN_RG_RATIO;              
			if(Gb_GAIN > Gr_GAIN )                                                  
			{                                                                       
				B_GAIN = GAIN_DEFAULT;                                                
				G_GAIN = Gb_GAIN;                                                     
				R_GAIN = G_GAIN * GOLDEN_RG_RATIO / sunny_r_ratio;                    
			}                                                                       
			else                                                                    
			{                                                                       
				R_GAIN = GAIN_DEFAULT;                                                
				G_GAIN = Gr_GAIN;                                                     
				B_GAIN = G_GAIN * GOLDEN_BG_RATIO / sunny_b_ratio;                    
			}                                                                       
		}                                                                         
	}                                                                           
	//LOG_INF("otp_sunny_sunny_r_ratio=%d,sunny_b_ratio=%d \n",sunny_r_ratio,sunny_b_ratio);

	if(R_GAIN < GAIN_DEFAULT)
		R_GAIN = GAIN_DEFAULT;
	if(G_GAIN < GAIN_DEFAULT)
		G_GAIN = GAIN_DEFAULT;
	if(B_GAIN < GAIN_DEFAULT)
		B_GAIN = GAIN_DEFAULT;
		
	R_GAIN_H = (R_GAIN >> 8) & 0xFF;
	R_GAIN_L = R_GAIN & 0xFF;
	B_GAIN_H = (B_GAIN >> 8) & 0xFF;
	B_GAIN_L = B_GAIN & 0xFF;
	G_GAIN_H = (G_GAIN >> 8) & 0xFF;
	G_GAIN_L = G_GAIN & 0xFF;

	write_cmos_sensor_8(GAIN_RED_ADDR, R_GAIN_H);		
	write_cmos_sensor_8(GAIN_RED_ADDR+1, R_GAIN_L);
	write_cmos_sensor_8(GAIN_BLUE_ADDR, B_GAIN_H);
	write_cmos_sensor_8(GAIN_BLUE_ADDR+1, B_GAIN_L);     
	write_cmos_sensor_8(GAIN_GREEN1_ADDR, G_GAIN_H);     
	write_cmos_sensor_8(GAIN_GREEN1_ADDR+1, G_GAIN_L); //Green 1 default gain 1x		
	write_cmos_sensor_8(GAIN_GREEN2_ADDR, G_GAIN_H); //Green 2 default gain 1x
	write_cmos_sensor_8(GAIN_GREEN2_ADDR+1, G_GAIN_L);
	LOG_INF("otp_sunny WB Update Finished! \n");
	return 1;
}



/*************************************************************************************************
* Function    :  get_eeprom_sunny_wb
* Description :  Get WB data    
**************************************************************************************************/
static bool get_eeprom_sunny_wb(void)
{
	BYTE temph = 0;
	BYTE templ = 0;

	if (false == selective_read_eeprom(0x001D, &temph)) {
		LOG_ERR("read r_ratio high byte failed\n");
		return 0;
	}
	if (false == selective_read_eeprom(0x001E, &templ)) {
		LOG_ERR("read r_ratio low byte failed\n");
		return 0;
	}
    sunny_r_ratio=(temph<<8)+templ;

	if (false == selective_read_eeprom(0x001F, &temph)) {
		LOG_ERR("read b_ratio high byte failed\n");
		return 0;
	}
	if (false == selective_read_eeprom(0x0020, &templ)) {
		LOG_ERR("read b_ratio low byte failed\n");
		return 0;
	}
    sunny_b_ratio=(temph<<8)+templ;
    LOG_INF("sunny_r_ratio=%d,sunny_b_ratio=%d \n",sunny_r_ratio,sunny_b_ratio);
	return 1;
}


/*************************************************************************************************
* Function    :  sunny_wb_update
* Description :  Update WB correction 
* Return      :  [bool] 0 : otp_sunny data fail 
                        1 : otp_sunny_WB update success            
**************************************************************************************************/
static bool sunny_wb_update(void)
{
	//get
	if(1 != get_eeprom_sunny_wb())
		return 0;


	if(!sunny_r_ratio || !sunny_b_ratio ) {
		LOG_ERR("WB update Err!");
		return 0;
	}

	//set
	if (1 != sunny_wb_gain_set())
		return 0;

	LOG_INF("WB update finished! \n");
	return 1;
}

static unsigned char info_data[32] = {0};
static unsigned char awb_data[32] = {0};

/*************************************************************************************************
* Function    :  update_3m2_awb_eeprom()
* Description :  update otp data from eeprom , if otp data is valid, 
                 it include get ID and WB update function  
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
bool update_3m2_awb_eeprom(void)
{
	unsigned char info_flag = 0;
	int i = 0;

	// check module information
	if (false == selective_read_eeprom(0x0000, &info_flag)) {
		LOG_ERR("read module information flag failed\n");
		return 0;
	}
	if (0x01 != info_flag) {
		LOG_ERR("module information flag invalid, %d\n", info_flag);
		return 0;
	}

	for (i=0; i<27; i++) {
		if(false == selective_read_eeprom(0x0001+i, &info_data[i]))
			break;
		//LOG_INF("read_eeprom 0x%0x %d\n",i, awb_data[i]);
	}
	if (i != 27) {
		LOG_ERR("read module information failed\n");
		return 0;
	}

	if (info_data[26] != get_data_checksum(info_data,26)){
		LOG_ERR("module information checksum failed, read %d\n", info_data[26]);
		return 0;
	}

	//dump module info
	LOG_INF("id=%d,calibration version=%d,date=%2d-%2d-%2d",info_data[0],info_data[1],info_data[2],info_data[3],info_data[4]);


	// update awb
	return sunny_wb_update();

}




#define LSC_SIZE  (1868)
u8 S5K3M2_LSCData[LSC_SIZE]={0};
u8 S5K3M2_AFData[6]={0};
int s5k3m2_prepare_lsc_ok = 0;
int s5k3m2_prepare_af_ok = 0;

bool prepare_3m2_lsc_eeprom(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short lsc_eeprom_addr = 0x3b;

	if (0 == s5k3m2_prepare_lsc_ok) {
		
		if (false == selective_read_eeprom(0x003a,&ival)) {
			LOG_ERR("read lsc flag failed\n");
			return 0;
		}
		if (1 != ival){
			LOG_ERR("lsc flag is invalid\n");
			return 0;
		}

		for (i=0; i<LSC_SIZE; i++) {
			if (false == selective_read_eeprom(lsc_eeprom_addr+i,&ival)) {
				LOG_ERR("read lsc data failed\n");
				memset(S5K3M2_LSCData, 0, LSC_SIZE);
				return 0;
			}
			S5K3M2_LSCData[i] = ival;
		}

		//checksum
		if (false == selective_read_eeprom(0x0787,&ival)) {
			LOG_ERR("read lsc checksum failed\n");
			memset(S5K3M2_LSCData, 0, LSC_SIZE);
			return 0;
		}
		if (ival != get_data_checksum(S5K3M2_LSCData, LSC_SIZE)){
			LOG_ERR("lsc checksum failed, read %d\n", ival);
			memset(S5K3M2_LSCData, 0, LSC_SIZE);
			return 0;
		}

		s5k3m2_prepare_lsc_ok = 1;
	}

	return 1;
}


bool prepare_3m2_af_eeprom(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short af_eeprom_addr = 0x0789;

	if (0 == s5k3m2_prepare_af_ok) {
		if (false == selective_read_eeprom(0x0788,&ival)) {
			LOG_ERR("read af flag failed\n");
			return 0;
		}
		if (1 != ival){
			LOG_ERR("af flag is invalid\n");
			return 0;
		}

		for (i=0; i<6; i++) {
			if (false == selective_read_eeprom(af_eeprom_addr+i,&ival)) {
				LOG_ERR("read af data failed\n");
				memset(S5K3M2_AFData, 0, 6);
				return 0;
			}
			S5K3M2_AFData[i] = ival;
		}

		//checksum
		if (false == selective_read_eeprom(0x078F,&ival)) {
			LOG_ERR("read af checksum failed\n");
			memset(S5K3M2_AFData, 0, 6);
			return 0;
		}
		if (ival != get_data_checksum(S5K3M2_AFData, 6)){
			LOG_ERR("af checksum failed, read %d\n", ival);
			memset(S5K3M2_AFData, 0, 6);
			return 0;
		}

		s5k3m2_prepare_af_ok = 1;
	}

	return 1;
}


#endif

