#include "string.h"
#include "drv_usart.h"
#include "frame_analysis.h"
#include "UartAdapter.h"

uint8_t get_decode_data[]=
{	
	0x23,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00
};

typedef struct
{
	float data1;
	float data2;
	float data3;
	float data4;
	float data5;
	float data6;
	float data7;
	float data8;
	float data9;
	float data10;
	float data11;
	float data12;
	float data13;
	float data14;
	float data15;
	float data16;	
}_debug_data;

float acc_m;
float theta = 0.0f;
_debug_data Debug_Data;
uint32_t Oldman_i = 0;
uint32_t Oldman_J = 0;
#include "nav_task.h"
extern CombineDataTypeDef combineData;
void Oscilloscope(void)
{
#define	Accel_Scale 	20.0
#define	Rate_Scale 		1260.0
#define	Angle_Scale 	360.0
#define	Temp_Scale		200.0
#define	Sensor_Scale 	65536.0

	uint8_t i;

//	Oldman_i++;
  
	//theta = theta+sensor.gyro.x*ts;
	//if(++Oldman_i >= 5ul)
	{
		Oldman_i = 0;
		

		//IMU
		Debug_Data.data1 = combineData.gnssInfo.Lat;		
		Debug_Data.data2 = combineData.gnssInfo.Lon;			
		Debug_Data.data3 = combineData.gnssInfo.Altitude;//att_air_G.x;
		Debug_Data.data4 = combineData.gnssInfo.ve;//att_air_G.y;
		Debug_Data.data5 = combineData.gnssInfo.vn;//tao;
		Debug_Data.data6 = combineData.gnssInfo.vu;	
		
		Debug_Data.data7 = combineData.gnssInfo.Pitch; //вкл╛╫г//Angle.x;
		Debug_Data.data8 = combineData.gnssInfo.Roll;//Angle.y;
		Debug_Data.data9 = combineData.gnssInfo.Heading;//Angle.z; 

		//Debug_Data.data10  = att_air.x;//att_air.x;//Encoder.Angle_P; 
		//Debug_Data.data11  = att_air.y;//att_air.y;//Encoder.Angle_R; 
		//Debug_Data.data12  = att_air.z
		
		;//att_air.z;//Encoder.Angle_Y; 
//Ouler_Point_Air.z = att_air_G.z;		
		//Debug_Data.data13 = Ouler_Point.x;//att_air.x;//gyro_air_n.x;//gyro_air.x;/KF_w[1];//Encoder.Angle_P;  gyro_air_n.x
		//Debug_Data.data14 = Ouler_Point.y;//att_air.y;//gyro_air_n.y;//gyro_air.y;//KF_w[1];//Fd_InsData.rev;   gyro_air_n.y
		//Debug_Data.data15 = Ouler_Point.z;//att_air.z;//gyro_air_n.z;//gyro_air.z;//KF_w[2];//Fd_InsData_State; gyro_air_n.z
		
		Debug_Data.data16  = combineData.gnssInfo.timestamp;//.Source;	 

			
	  get_decode_data[0] = 0x23;
		memcpy(get_decode_data+1,&Debug_Data,sizeof(Debug_Data));
		
		get_decode_data[65]=0;
		for(i=1;i<65;i++)
		{	  				    
			get_decode_data[65] += get_decode_data[i];
		}	
		gd32_usart_write(get_decode_data,66);
		//Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, 66, get_decode_data);
	}
	#undef	Accel_Scale
#undef	Rate_Scale
#undef	Angle_Scale
#undef	Temp_Scale
#undef	Sensor_Scale
}

