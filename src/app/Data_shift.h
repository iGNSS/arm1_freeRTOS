#ifndef ____DATA_SHIFT_H____
#define ____DATA_SHIFT_H____

#include "gd32f4xx.h"

#define	GYRO_MAX_COUNT		1800.0f		//1966080000 //º§π‚Õ”¬›∞Ê 4000
#define	ACCM_MAX_COUNT		30000.0f	//1966080000 //º§π‚Õ”¬›∞Ê 1000


int ProcessIMUDataUserAxis(float*GyroUserAxis, float*AccUserAxis, float* tGyro, float* tAcc);



#endif //____DATA_SHIFT_H____

