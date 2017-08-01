#ifndef _S5K3L2XXTRULYAF_H
#define _S5K3L2XXTRULYAF_H

#include <linux/ioctl.h>
//#include "kd_imgsensor.h"

#define S5K3L2XXTRULYAF_MAGIC 'A'
//IOCTRL(inode * ,file * ,cmd ,arg )


//Structures
typedef struct {
/* current position */
	u32 u4CurrentPosition;
/* macro position */
	u32 u4MacroPosition;
/* Infiniti position */
	u32 u4InfPosition;
/* Motor Status */
bool          bIsMotorMoving;
//Motor Open?
bool          bIsMotorOpen;
//Support SR?
bool          bIsSupportSR;
} stS5K3L2XXTRULYAF_MotorInfo;

//Control commnad
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"             
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"
#define S5K3L2XXTRULYAFIOC_G_MOTORINFO _IOR(S5K3L2XXTRULYAF_MAGIC,0,stS5K3L2XXTRULYAF_MotorInfo)

#define S5K3L2XXTRULYAFIOC_T_MOVETO _IOW(S5K3L2XXTRULYAF_MAGIC,1,u32)

#define S5K3L2XXTRULYAFIOC_T_SETINFPOS _IOW(S5K3L2XXTRULYAF_MAGIC,2,u32)

#define S5K3L2XXTRULYAFIOC_T_SETMACROPOS _IOW(S5K3L2XXTRULYAF_MAGIC,3,u32)

#else
#endif
