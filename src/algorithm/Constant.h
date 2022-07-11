/***********************************************************************************
This file DEFINED all header files of Constant parmameter
Application using constant parmameter should include this file first.
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-2          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/
/*****************************************************************
代码风格:
1,结构体均使用大写字母+下划线组成
2. 结构体变量均使用小写字母
3. 全局变量均以g_XXX开始
4. 函数使用首字母大写方式
******************************************************************/


#ifndef _CONSTANT_H
#define _CONSTANT_H



#ifndef PI
#define PI        								(3.1415926535897932384626433832795) 
#endif 

#ifndef WIE
#define WIE									7.29211506e-5
#endif

#ifndef WGS84G0
#define WGS84G0								9.780318
#endif

#ifndef eps
#define eps                      						2.2204e-16
#endif

#define DEG2RAD								(PI/180.0)          /* deg to rad */
#define RAD2DEG     							(180.0/PI)          /* rad to deg */
//---------------------------------------------------------------------------------------------
#define F1									(   2.0 * 1)						// 2
#define F2									((F1)*2 * 2)						// 8
#define F3									((F2)*2 * 3)						// 48
#define F4									((F3)*2 * 4)						// 384
#define F5									((F4)*2 * 5)						// 3840
/****************************************************************************************/



#define ZIHR									1							//0:关闭航向约束；1：打开航向约束

#define Re									6378137.0						// WGS84坐标系长半轴
#define F									1/298.257							// 扁率
#define wie									7.2921151467e-5					// 自转角速度（rad）
#define Rp									((1-(F))*(Re))						// 短半轴
#define G0									9.7803267714						// 地心引力
#define cs_2									(2.0/3.0)						// 双子样采样系数
#define e									sqrt(2*(F)-(F)*(F))						// 第一偏心率
#define ep									(sqrt((Re)*(Re)-(Rp)*(Rp))/(Rp))			// 第二偏心率

#define DEG									((PI)/180.0)						// 角度转弧度
#define DPH									((DEG)/3600.0)
#define UG									((G0)/1.0e6)

#define DPSH									((DEG)/60)						// rad/(sqrt(hour))
#define UGPSHZ								((UG)/1.0)						// ug/(sqrt(Hz))

//-----------------------------------------------------------------------------
#define NN									3						// 3×3矩阵的维度

#define SAMPLE_FREQ						100						//jiang  ，请根据实际情况设置采样时间
#define SAMPLE_FREQ_GNSS					1									//GNSS频率
#define  SAMPLE_Ratio						SAMPLE_FREQ/SAMPLE_FREQ_GNSS //jiang,需要考虑，SAMPLE_Ratio ? SAMPLE_FREQ or SAMPLE_FREQ/SAMPLE_FREQ_GNSS
#define TS									1.0/SAMPLE_FREQ			// 数据采样

#define NA									15						// 状态向量的维度  +杆臂
#define NB									6						// 量测方程的维度

#define ALN									7

//--------------ZIHR----------------------------------------------
#define SAMPLE_SIZE							SAMPLE_FREQ							//静态探测样本数量
#define VAR_THRESHSHOLD					5e-9								//方差阈值
#define ACCL_THRESHSHOLD					9.8								//加速度计的阈值
#define VN_3D_THRESHSHOLD					0.1								//速度的阈值=5*sqrt(sumsq(v));（注：效果不明显）

//--------------------------------------jiang 针对P矩阵在量测更新处理进行改进，需要测试----------------------------
#define  MeaUpdatePSetting					0//定义为1则使用公式中P, 否则使用原程序算法
//----------设置Q阵 jiang 需要根据实际惯导设备实际情况设置--------------------------------------------------------------
#define GYROBIAS							(30*(DPH))				// 陀螺零偏(deg/h)  10
#define GYROBIAS2 							(GYROBIAS*GYROBIAS)
#define ACCBIAS								(1000*(UG))				// 加速度计零偏(ug)  1000
#define ACCBIAS2 							(ACCBIAS*ACCBIAS)
#define ANGNRANDOMWALK					(0.6*(DPSH))			// 角度随机游走(deg/sqrt(h))//0.6
#define ANGNRANDOMWALK2 					(ANGNRANDOMWALK*ANGNRANDOMWALK)
#define VELRANDOMWALK						(10*(UGPSHZ))			// 速度随机游走(ug/sqrt(hz))
#define VELRANDOMWALK2 					(VELRANDOMWALK*VELRANDOMWALK)
#define ANGNR_RATERANDOMWALK			0.0						// 角速率随机游走
#define ANGNR_RATERANDOMWALK2 			(ANGNR_RATERANDOMWALK*ANGNR_RATERANDOMWALK)
#define ACC_RANDOMWALK					0.0						// 加速度随机游走
#define ACC_RANDOMWALK2 					(ACC_RANDOMWALK*ACC_RANDOMWALK)
#define LEVER_VAR							1.0						// 杆臂误差方差
//状态更新位置误差，目前认为位置误差远大于GNSS单点定位误差，以后可以建模估计
#define  INS_POS_VAR						((5.0/Re)*(5.0/Re)) *9.0//jiang: 需要测试考虑3倍误差是否合理
#define  INS_HEAD_VAR						5.5*5.5*9.0//jiang:
//-----------设置P阵-------------------------------------------------------------
#define DATT									(1*(DEG))
#define DATT_VAR							(DATT*DATT)
#define DVEL									1.0
#define DVEL_VAR							(DVEL*DVEL)
#define DPOS								(1/Re)
#define DPOS_VAR							(DPOS*DPOS)
//----------设置R阵--------------------------------------------------------------
#define VEL_VAR								(0.2*0.2)
#define VEL_VAR_F							(0.2/SAMPLE_Ratio)*(0.2/SAMPLE_Ratio)			//速度误差与时间有关系，0.2m/s
//#define POS_VAR								((0.1/Re)*(0.1/Re))
#define HEAD_VAR							(5.5)
#define SPP_POS_VAR						((5.0/Re)*(5.0/Re))   //单点定位位置误差
#define RTK_POS_VAR						((0.1/Re)*(0.1/Re))   //RTK定位位置误差





//#define WEEK2SECOND						604800.0;//一周秒				
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef  __cplusplus
}
#endif

#endif	/*_CONSTANT_H*/
/*End***********************************************************************************/



