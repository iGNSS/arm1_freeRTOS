#include "data_shift.h"
#include "ins_data.h"

float GyroCountInc[3];/*Inc Gyro Count of Gyro 1~3 */
float AccmCountIncDiff[3];/*different Inc Count of Accm 1~3 signal ClkP and ClkN*/

/**********************************************************************************
Function:process Origin Adis16460 FPGA Data
Input:OriginFpgaData
Output:
Use:none
Modify:
Note: ret !=0---error
***********************************************************************************/
int ProcessIMUDataUserAxis(float*GyroUserAxis, float*AccUserAxis, float* tGyro, float* tAcc)
{
    int	i,k,ret;
    //陀螺角增量数据不正常  用角速度乘以时间代替
    /*calculate Gyro pulse count inc */
    ret = 0;
    k=0;

    switch( hSetting.imuAxis )
    {
    case 1://x   y   z
    {
        for(i=0; i<3; i++)
        {
            GyroUserAxis[i] =tGyro[i];
            AccUserAxis[i] =tAcc[i];
        }
    }
    break;
    case 2:// x  -y   -z
    {
        GyroUserAxis[0]=tGyro[0];
        AccUserAxis[0]=tAcc[0];
        GyroUserAxis[1]=tGyro[1]*-1;
        AccUserAxis[1]=tAcc[1]*-1;
        GyroUserAxis[2]=tGyro[2]*-1;
        AccUserAxis[2]=tAcc[2]*-1;

    }
    break;
    case 3:// x  z   -y
    {
        GyroUserAxis[0]=tGyro[0];
        AccUserAxis[0]=tAcc[0];
        GyroUserAxis[1]=tGyro[2];
        AccUserAxis[1]=tAcc[2];
        GyroUserAxis[2]=tGyro[1]*-1;
        AccUserAxis[2]=tAcc[1]*-1;

    }
    break;
    case 4:// x  -z   y
    {
        GyroUserAxis[0]=tGyro[0];
        AccUserAxis[0]=tAcc[0];
        GyroUserAxis[1]=tGyro[2]*-1;
        AccUserAxis[1]=tAcc[2]*-1;
        GyroUserAxis[2]=tGyro[1];
        AccUserAxis[2]=tAcc[1];
    }
    break;
    case 5:// -x  y  -z
    {
        GyroUserAxis[0]=tGyro[0]*-1;
        AccUserAxis[0]=tAcc[0]*-1;
        GyroUserAxis[1]=tGyro[1];
        AccUserAxis[1]=tAcc[1];
        GyroUserAxis[2]=tGyro[2]*-1;
        AccUserAxis[2]=tAcc[2]*-1;
    }
    break;
    case 6:// -x  -y   z
    {
        GyroUserAxis[0]=tGyro[0]*-1;
        AccUserAxis[0]=tAcc[0]*-1;
        GyroUserAxis[1]=tGyro[1]*-1;
        AccUserAxis[1]=tAcc[1]*-1;
        GyroUserAxis[2]=tGyro[2];
        AccUserAxis[2]=tAcc[2];
    }
    break;
    case 7:// -x  z  y
    {
        GyroUserAxis[0]=tGyro[0]*-1;
        AccUserAxis[0]=tAcc[0]*-1;
        GyroUserAxis[1]=tGyro[2];
        AccUserAxis[1]=tAcc[2];
        GyroUserAxis[2]=tGyro[1];
        AccUserAxis[2]=tAcc[1];
    }
    break;
    case 8:// -x  -z   -y
    {
        GyroUserAxis[0]=tGyro[0]*-1;
        AccUserAxis[0]=tAcc[0]*-1;
        GyroUserAxis[1]=tGyro[2]*-1;
        AccUserAxis[1]=tAcc[2]*-1;
        GyroUserAxis[2]=tGyro[1]*-1;
        AccUserAxis[2]=tAcc[1]*-1;
    }
    break;
    case 9:// y  x   -z
    {
        GyroUserAxis[0]=tGyro[1];
        AccUserAxis[0]=tAcc[1];
        GyroUserAxis[1]=tGyro[0];
        AccUserAxis[1]=tAcc[0];
        GyroUserAxis[2]=tGyro[2]*-1;
        AccUserAxis[2]=tAcc[2]*-1;
    }
    break;
    case 10:// y -x  z
    {
        GyroUserAxis[0]=tGyro[1];
        AccUserAxis[0]=tAcc[1];
        GyroUserAxis[1]=tGyro[0]*-1;
        AccUserAxis[1]=tAcc[0]*-1;
        GyroUserAxis[2]=tGyro[2];
        AccUserAxis[2]=tAcc[2];
    }
    break;

    case 11:// y  z   x
    {
        GyroUserAxis[0]=tGyro[1];
        AccUserAxis[0]=tAcc[1];
        GyroUserAxis[1]=tGyro[2];
        AccUserAxis[1]=tAcc[2];
        GyroUserAxis[2]=tGyro[0];
        AccUserAxis[2]=tAcc[0];
    }
    break;
    case 12:// y -z  -x
    {
        GyroUserAxis[0]=tGyro[1];
        AccUserAxis[0]=tAcc[1];
        GyroUserAxis[1]=tGyro[2]*-1;
        AccUserAxis[1]=tAcc[2]*-1;
        GyroUserAxis[2]=tGyro[0]*-1;
        AccUserAxis[2]=tAcc[0]*-1;
    }
    break;
    case 13://  -y  x  z
    {
        GyroUserAxis[0]=tGyro[1]*-1;
        AccUserAxis[0]=tAcc[1]*-1;
        GyroUserAxis[1]=tGyro[0];
        AccUserAxis[1]=tAcc[0];
        GyroUserAxis[2]=tGyro[2];
        AccUserAxis[2]=tAcc[2];

    }
    break;
    case 14://  -y  -x  -z
    {
        GyroUserAxis[0]=tGyro[1]*-1;
        AccUserAxis[0]=tAcc[1]*-1;
        GyroUserAxis[1]=tGyro[0]*-1;
        AccUserAxis[1]=tAcc[0]*-1;
        GyroUserAxis[2]=tGyro[2]*-1;
        AccUserAxis[2]=tAcc[2]*-1;
    }
    break;
    case 15:// -y   z  -x
    {
        GyroUserAxis[0]=tGyro[1]*-1;
        AccUserAxis[0]=tAcc[1]*-1;
        GyroUserAxis[1]=tGyro[2];
        AccUserAxis[1]=tAcc[2];
        GyroUserAxis[2]=tGyro[0]*-1;
        AccUserAxis[2]=tAcc[0]*-1;
    }
    break;
    case 16://  -y  -z  x
    {
        GyroUserAxis[0]=tGyro[1]*-1;
        AccUserAxis[0]=tAcc[1]*-1;
        GyroUserAxis[1]=tGyro[2]*-1;
        AccUserAxis[1]=tAcc[2]*-1;
        GyroUserAxis[2]=tGyro[0];
        AccUserAxis[2]=tAcc[0];
    }
    break;
    case 17://  z  x  y
    {
        GyroUserAxis[0]=tGyro[2];
        AccUserAxis[0]=tAcc[2];
        GyroUserAxis[1]=tGyro[0];
        AccUserAxis[1]=tAcc[0];
        GyroUserAxis[2]=tGyro[1];
        AccUserAxis[2]=tAcc[1];
    }
    break;
    case 18://  z  -x  -y
    {
        GyroUserAxis[0]=tGyro[2];
        AccUserAxis[0]=tAcc[2];
        GyroUserAxis[1]=tGyro[0]*-1;
        AccUserAxis[1]=tAcc[0]*-1;
        GyroUserAxis[2]=tGyro[1]*-1;
        AccUserAxis[2]=tAcc[1]*-1;
    }
    break;
    case 19://  z  y  -x
    {
        GyroUserAxis[0]=tGyro[2];
        AccUserAxis[0]=tAcc[2];
        GyroUserAxis[1]=tGyro[1];
        AccUserAxis[1]=tAcc[1];
        GyroUserAxis[2]=tGyro[0]*-1;
        AccUserAxis[2]=tAcc[0]*-1;
    }
    break;
    case 20://z  -y  x
    {
        GyroUserAxis[0]=tGyro[2];
        AccUserAxis[0]=tAcc[2];
        GyroUserAxis[1]=tGyro[1]*-1;
        AccUserAxis[1]=tAcc[1]*-1;
        GyroUserAxis[2]=tGyro[0];
        AccUserAxis[2]=tAcc[0];
    }
    break;

    case 21://  -z  x  -y
    {
        GyroUserAxis[0]=tGyro[2]*-1;
        AccUserAxis[0]=tAcc[2]*-1;
        GyroUserAxis[1]=tGyro[0];
        AccUserAxis[1]=tAcc[0];
        GyroUserAxis[2]=tGyro[1]*-1;
        AccUserAxis[2]=tAcc[1]*-1;
    }
    break;

    case 22://  -z  -x  y
    {
        GyroUserAxis[0]=tGyro[2]*-1;
        AccUserAxis[0]=tAcc[2]*-1;
        GyroUserAxis[1]=tGyro[0]*-1;
        AccUserAxis[1]=tAcc[0]*-1;
        GyroUserAxis[2]=tGyro[1];
        AccUserAxis[2]=tAcc[1];
    }
    break;

    case 23://  -z  y  x
    {
        GyroUserAxis[0]=tGyro[2]*-1;
        AccUserAxis[0]=tAcc[2]*-1;
        GyroUserAxis[1]=tGyro[1];
        AccUserAxis[1]=tAcc[1];
        GyroUserAxis[2]=tGyro[0];
        AccUserAxis[2]=tAcc[0];
    }
    break;

    case 24://  -z  -y  -x
    {
        GyroUserAxis[0]=tGyro[2]*-1;
        AccUserAxis[0]=tAcc[2]*-1;
        GyroUserAxis[1]=tGyro[1]*-1;
        AccUserAxis[1]=tAcc[1]*-1;
        GyroUserAxis[2]=tGyro[0]*-1;
        AccUserAxis[2]=tAcc[0]*-1;
    }
    break;

    default:
        break;

    }

    for(i=0; i<3; i++)
    {
        k++;

        if( GyroUserAxis[i]>GYRO_MAX_COUNT )	/*Error:too large*/
        {
            GyroUserAxis[i] = GYRO_MAX_COUNT;
            ret |= 1<<(k-1);
        }
        if( GyroUserAxis[i]<(-GYRO_MAX_COUNT))	/*Error:too large*/
        {
            GyroUserAxis[i] = -GYRO_MAX_COUNT;
            ret |= 1<<(k-1);
        }
    }

    /*calculate Accm count inc*/
    for(i=0; i<3; i++)
    {
        k++;
        if( AccUserAxis[i]>ACCM_MAX_COUNT )	/*Error:too large*/
        {
            AccUserAxis[i] = ACCM_MAX_COUNT;
            ret |= 1<<(k-1);
        }
        if( AccUserAxis[i]<(-ACCM_MAX_COUNT) )	/*Error:too large*/
        {
            AccUserAxis[i] = -ACCM_MAX_COUNT;
            ret |= 1<<(k-1);
        }
    }
    return ret;
}


void ProcessINSWorkMode(uint16_t mode)
{
    int j;
    switch(mode)
    {
    case INS_DATA_MODE_1:
    case INS_DATA_MODE_6:
//			sprintf(ins_gnss_data.InsGnssDataChar, "$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n",ins_week,ins_ms*0.001,HeadingTemp,ins.att[0]*180/PI,ins.att[1]*180/PI,ins.pos[0]*180/PI,ins.pos[1]*180/PI,ins.pos[2],ins.vn[0],ins.vn[1],ins.vn[2],db_ana_length_for_fusion,Svs,ukf_r_mode,gnss_quality-48,66);
        break;
    case INS_DATA_MODE_2:
    case INS_DATA_MODE_7:
//			sprintf(ins_gnss_data.InsGnssDataChar, "$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n",ins_week,ins_ms*0.001,HeadingTemp,ins.att[0]*180/PI,ins.att[1]*180/PI,ins.pos[0]*180/PI,ins.pos[1]*180/PI,ins.pos[2],ins.vn[0],ins.vn[1],ins.vn[2],db_ana_length_for_fusion,Svs,ukf_r_mode,gnss_quality-48,66);
        break;
    case INS_DATA_MODE_3:
//			sprintf(ins_gnss_data.InsGnssDataChar, "$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f*%d\r\n",ins_week,ins_ms*0.001,Wib_b[0]*180/PI,Wib_b[1]*180/PI,Wib_b[2]*180/PI,fx,fy,fz,imuTempFloat,66);
        break;
    case INS_DATA_MODE_4:
    case INS_DATA_MODE_8:
//			if(NavDataSendedCnt%2==1)
//			{
//				for(j=0;j<3;j++){sumVm4[j]=vm[j];sumWm4[j]=wm[j];}
//				sprintf(ins_gnss_data.InsGnssDataChar, "$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n",ins_week,ins_ms*0.001,HeadingTemp,ins.att[0]*180/PI,ins.att[1]*180/PI,ins.pos[0]*180/PI,ins.pos[1]*180/PI,ins.pos[2],ins.vn[0],ins.vn[1],ins.vn[2],db_ana_length_for_fusion,Svs,ukf_r_mode,gnss_quality-48,66);
//			}
//			else
//			{
//				for(j=0;j<3;j++){sumVm4[j]+=vm[j];sumWm4[j]+=wm[j];}
//				sprintf(ins_gnss_data.InsGnssDataChar, "$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f*%d\r\n",ins_week,ins_ms*0.001,Wib_b[0]*180/PI,Wib_b[1]*180/PI,Wib_b[2]*180/PI,fx,fy,fz,imuTempFloat,66);
//			}
        break;
    case INS_DATA_MODE_5:
    case INS_DATA_MODE_9:
//			if(NavDataSendedCnt%2==1)
//			{
//				sprintf(ins_gnss_data.InsGnssDataChar, "$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n",ins_week,ins_ms*0.001,HeadingTemp,ins.att[0]*180/PI,ins.att[1]*180/PI,ins.pos[0]*180/PI,ins.pos[1]*180/PI,ins.pos[2],ins.vn[0],ins.vn[1],ins.vn[2],db_ana_length_for_fusion,Svs,ukf_r_mode,gnss_quality-48,66);
//			}
//			else
//			{
//				sprintf(ins_gnss_data.InsGnssDataChar, "$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f*%d\r\n",ins_week,ins_ms*0.001,Wib_b[0]*180/PI,Wib_b[1]*180/PI,Wib_b[2]*180/PI,fx,fy,fz,imuTempFloat,66);
//			}
        break;
    default:
        break;
    }
}


void ProcessINSBootMode(uint16_t bootMode)
{

}

void ProcessINS()
{

}

