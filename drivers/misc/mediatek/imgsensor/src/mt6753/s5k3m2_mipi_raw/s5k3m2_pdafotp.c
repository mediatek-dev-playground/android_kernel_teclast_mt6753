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


#define PFX "S5K3M2_pdafotp"
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

#define S5K3M2_EEPROM_READ_ID  0xA0
#define S5K3M2_EEPROM_WRITE_ID   0xA1
#define S5K3M2_I2C_SPEED        100  
#define S5K3M2_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE s5k3m2_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

#if 1 // Jane modify for pdaf map which include flag and checksum
/*
	data structure

	FLAG [1 byte] : 1-valid, others-invalid
	DATA [many bytes]
	CHECKSUM [1 byte] : sum(DATA)%0xFF
*/

#define PROC1_ADDR (0x0790)
#define PROC2_ADDR (0x0982)
#define PROC3_ADDR (0x0CAA)

#define PROC1_DATA_SIZE (496)
#define PROC2_DATA_SIZE (806)
#define PROC3_DATA_SIZE (102)

#define PROC_VALID_FLAG (1)

static unsigned char proc_data[1024] = {0};

static unsigned char get_procdata_checksum(unsigned char* pData, unsigned int dataLen)
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


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3M2_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3M2_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3M2_EEPROM_READ_ID)<0)
		return false;
    return true;
}


bool read_3m2_pdaf_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	int i = 0;
	int procdata_offset = 0;
	unsigned char flag_r = 0;
	unsigned char chs_r = 0;

	LOG_INF("size = %d\n", size);

	if ( get_done==true && last_size==size && last_offset==addr ) {
		memcpy(data, s5k3m2_eeprom_data, size);
	    return true;
	}

	//==========  proc1  ==========
	if (false == selective_read_eeprom(PROC1_ADDR, &flag_r)) {
		LOG_ERR("read proc1 flag failed\n");
		goto READ_EEPROM_FAIL;
	}
	if (PROC_VALID_FLAG != flag_r) {
		LOG_ERR("proc1 flag is invalid\n");
		goto READ_EEPROM_FAIL;
	}

	procdata_offset = PROC1_ADDR + 1;

	for (i=0; i<PROC1_DATA_SIZE; i++) {
		if(false == selective_read_eeprom(procdata_offset+i, &proc_data[i]))
			break;
		//LOG_INF("read_eeprom 0x%0x %d\n",offset+i, proc_data[i]);
	}
	if (i != PROC1_DATA_SIZE) {
		LOG_ERR("read proc1 data failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (false == selective_read_eeprom(procdata_offset+PROC1_DATA_SIZE, &chs_r)) {
		LOG_ERR("read proc1 flag failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (chs_r != get_procdata_checksum(proc_data,PROC1_DATA_SIZE)){
		LOG_ERR("proc1 checksum failed, read %d\n", chs_r);
		goto READ_EEPROM_FAIL;
	}

	//data ok
	memcpy(s5k3m2_eeprom_data, proc_data, PROC1_DATA_SIZE);

	//==========  proc2  ==========
	memset(proc_data,0,1024);
	flag_r = 0;
	chs_r = 0;

	if (false == selective_read_eeprom(PROC2_ADDR, &flag_r)) {
		LOG_ERR("read proc2 flag failed\n");
		goto READ_EEPROM_FAIL;
	}
	if (PROC_VALID_FLAG != flag_r) {
		LOG_ERR("proc2 flag is invalid\n");
		goto READ_EEPROM_FAIL;
	}

	procdata_offset = PROC2_ADDR + 1;

	for (i=0; i<PROC2_DATA_SIZE; i++) {
		if(false == selective_read_eeprom(procdata_offset+i, &proc_data[i]))
			break;
		//LOG_INF("read_eeprom 0x%0x %d\n",offset+i, proc_data[i]);
	}
	if (i != PROC2_DATA_SIZE) {
		LOG_ERR("read proc2 data failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (false == selective_read_eeprom(procdata_offset+PROC2_DATA_SIZE, &chs_r)) {
		LOG_ERR("read proc2 flag failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (chs_r != get_procdata_checksum(proc_data,PROC2_DATA_SIZE)){
		LOG_ERR("proc2 checksum failed, read %d\n", chs_r);
		goto READ_EEPROM_FAIL;
	}

	//data ok
	memcpy(s5k3m2_eeprom_data+PROC1_DATA_SIZE, proc_data, PROC2_DATA_SIZE);


	//==========  proc3  ==========
	memset(proc_data,0,1024);
	flag_r = 0;
	chs_r = 0;

	if (false == selective_read_eeprom(PROC3_ADDR, &flag_r)) {
		LOG_ERR("read proc3 flag failed\n");
		goto READ_EEPROM_FAIL;
	}
	if (PROC_VALID_FLAG != flag_r) {
		LOG_ERR("proc3 flag is invalid\n");
		goto READ_EEPROM_FAIL;
	}

	procdata_offset = PROC3_ADDR + 1;

	for (i=0; i<PROC3_DATA_SIZE; i++) {
		if(false == selective_read_eeprom(procdata_offset+i, &proc_data[i]))
			break;
		//LOG_INF("read_eeprom 0x%0x %d\n",offset+i, proc_data[i]);
	}
	if (i != PROC3_DATA_SIZE) {
		LOG_ERR("read proc3 data failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (false == selective_read_eeprom(procdata_offset+PROC3_DATA_SIZE, &chs_r)) {
		LOG_ERR("read proc3 flag failed\n");
		goto READ_EEPROM_FAIL;
	}

	if (chs_r != get_procdata_checksum(proc_data,PROC3_DATA_SIZE)){
		LOG_ERR("proc3 checksum failed, read %d\n", chs_r);
		goto READ_EEPROM_FAIL;
	}

	//data ok
	memcpy(s5k3m2_eeprom_data+PROC1_DATA_SIZE+PROC2_DATA_SIZE, proc_data, PROC3_DATA_SIZE);


	//all ok
	LOG_INF("3m2 get pdaf data from eeprom ok\n");
	memcpy(data, s5k3m2_eeprom_data, PROC1_DATA_SIZE+PROC2_DATA_SIZE+PROC3_DATA_SIZE);
    return true;

READ_EEPROM_FAIL:
	get_done = false;
    last_size = 0;
    last_offset = 0;
	return false;
}


#else
static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > S5K3M2_MAX_OFFSET)
        return false;
	kdSetI2CSpeed(S5K3M2_I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3M2_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_3m2_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
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

bool read_3m2_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size){
	int i;
	addr = 0x0800;
	size = 1404;
	
	LOG_INF("read 3m2 eeprom, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_3m2_eeprom(addr, s5k3m2_eeprom_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			return false;
		}
	}
	
	memcpy(data, s5k3m2_eeprom_data, size);
    return true;
}
#endif
