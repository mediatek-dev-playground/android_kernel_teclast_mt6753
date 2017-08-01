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


#define PFX "S5K3P3_pdafotp"
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

#define S5K3P3_EEPROM_READ_ID  0xA2
#define S5K3P3_EEPROM_WRITE_ID   0xA3
#define S5K3P3_I2C_SPEED        100  
#define S5K3P3_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE s5k3p3_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3P3_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3P3_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3P3_EEPROM_READ_ID)<0)
		return false;
	LOG_INF("read 0x%x=0x%x\n",addr,*data);
    return true;
}

static bool _read_3p3_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	int offset = addr;
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_3p3_pdaf_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	unsigned char flag = 0;
	addr = 0x0801;
	size = 1404;
	
	LOG_INF("read 3p3 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {

		//check flag
		if ((false==selective_read_eeprom(0x0800,&flag)) || (0x07!=flag)) {
			LOG_ERR("pdaf data invalid, [0x0800] is %d\n", flag);
			get_done = 0;
			last_size = 0;
			last_offset = 0;
			return false;
		}
		
		if(!_read_3p3_eeprom(addr, s5k3p3_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	memcpy(data, s5k3p3_eeprom_data, size);
    return true;
}

bool print_3p3_eeprom_info(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short iaddr=0;
	unsigned char module_info[8]={0};
	unsigned char calibration_ver[4]={0};

	//program flag
	if (false == selective_read_eeprom(0x00,&ival)) {
		LOG_ERR("read program flag failed\n");
		return 0;
	}
	if (0x1 != ival){
		LOG_ERR("program flag is invalid 0x%x\n", ival);
		return 0;
	}

	//module information
	iaddr = 0x01;
	for (i=0; i<8; i++) {
		if (false == selective_read_eeprom(iaddr+i,&ival)) {
			LOG_ERR("read module information failed\n");
			return 0;
		}
		module_info[i] = ival;
	}
	LOG_INF("date:%d-%d-%d,mid:%d,color-temp:%d\n",module_info[0],module_info[1],module_info[2],module_info[3],module_info[7]);

	//Calibration Version
	iaddr = 0x0A;
	for (i=0; i<4; i++) {
		if (false == selective_read_eeprom(iaddr+i,&ival)) {
			LOG_ERR("read Calibration Version failed\n");
			return 0;
		}
		calibration_ver[i] = ival;
	}
	LOG_INF("calibration_ver:0x%x-0x%x-0x%x-0x%x\n",calibration_ver[0],calibration_ver[1],calibration_ver[2],calibration_ver[3]);

	return 1;
}


#define LSC_SIZE	(1868)
#define AF_SIZE		(4)
#define AWB_SIZE	(8)
u8 S5K3P3_LSCData[LSC_SIZE]={0};
u8 S5K3P3_AFData[AF_SIZE]={0};
u8 S5K3P3_AWBData[AWB_SIZE]={0};

int s5k3p3_prepare_lsc_ok = 0;
int s5k3p3_prepare_af_ok = 0;
int s5k3p3_prepare_awb_ok = 0;

bool prepare_3p3_lsc_eeprom(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short lsc_eeprom_addr = 0x20;

	LOG_INF("\n");

	if (0 == s5k3p3_prepare_lsc_ok) {
		// the LSC Table Size should be 0x074C(1868)
		if (false == selective_read_eeprom(0x001E,&ival)) {
			LOG_ERR("read LSC Table Size LSB failed\n");
			return 0;
		}
		if (0x4C != ival){
			LOG_ERR("LSC Table Size LSB is invalid 0x%x\n", ival);
			return 0;
		}
		if (false == selective_read_eeprom(0x001F,&ival)) {
			LOG_ERR("read LSC Table Size MSB failed\n");
			return 0;
		}
		if (0x07 != ival){
			LOG_ERR("LSC Table Size MSB is invalid 0x%x\n", ival);
			return 0;
		}

		for (i=0; i<LSC_SIZE; i++) {
			if (false == selective_read_eeprom(lsc_eeprom_addr+i,&ival)) {
				LOG_ERR("read lsc data failed\n");
				memset(S5K3P3_LSCData, 0, LSC_SIZE);
				return 0;
			}
			S5K3P3_LSCData[i] = ival;
		}

		s5k3p3_prepare_lsc_ok = 1;
	}

	return 1;
}


bool prepare_3p3_af_eeprom(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short af_eeprom_addr = 0x001A;

	LOG_INF("\n");

	if (0 == s5k3p3_prepare_af_ok) {
		for (i=0; i<AF_SIZE; i++) {
			if (false == selective_read_eeprom(af_eeprom_addr+i,&ival)) {
				LOG_ERR("read af data failed\n");
				memset(S5K3P3_AFData, 0, 6);
				return 0;
			}
			S5K3P3_AFData[i] = ival;
		}

		s5k3p3_prepare_af_ok = 1;
		
		LOG_INF("0x%x,0x%x,0x%x,0x%x\n",
			S5K3P3_AFData[0],S5K3P3_AFData[1],S5K3P3_AFData[2],S5K3P3_AFData[3]);
	}

	return 1;
}

bool prepare_3p3_awb_eeprom(void)
{
	int i=0;
	unsigned char ival=0;
	unsigned short awb_eeprom_addr = 0x0012;

	LOG_INF("\n");

	if (0 == s5k3p3_prepare_awb_ok) {
		for (i=0; i<AWB_SIZE; i++) {
			if (false == selective_read_eeprom(awb_eeprom_addr+i,&ival)) {
				LOG_ERR("read af data failed\n");
				memset(S5K3P3_AWBData, 0, 6);
				return 0;
			}
			S5K3P3_AWBData[i] = ival;
		}

		s5k3p3_prepare_awb_ok = 1;
		LOG_INF("0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
			S5K3P3_AWBData[0],S5K3P3_AWBData[1],S5K3P3_AWBData[2],S5K3P3_AWBData[3],S5K3P3_AWBData[4],S5K3P3_AWBData[5],S5K3P3_AWBData[6],S5K3P3_AWBData[7]);
	}

	return 1;
}

