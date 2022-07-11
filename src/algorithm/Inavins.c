/***********************************
INS operation module

************************************/
#include "inavins.h"
#include "constant.h"
#include "inavgnss.h"
#include "inavcan.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "inavlog.h"
#include "nav_task.h"

#include <string.h>

extern I_NAV_GNSS_RESULT g_NAV_GNSS_RESULT;
extern I_NAV_CAN_RESULT 	g_NAV_CAN_RESULT;

I_NAV_INS g_NAV_INS;

STATIC_DETECTION g_STATIC_DETECTION;

COMPENSATE_PARAMS g_Compensate_Params;

extern char g_PrintBuff[1024*4];
extern char	g_MatrixBuff[1024*4];

//double GnssArmLength[3];
	
#ifdef __cplusplus
extern "C" {
#endif

void Align2Att(double* gn, double* wnie, double* aveVm, double* aveWm, double* qnb0)
{
	int i,j;
	double traMat[9] = { 0,0,-1,1,0,0,0,1,0 };// Ë«Ê¸Á¿¶¨×ËµÄ×ª»»¾ØÕó
	double gnMat[9];// ÒÔgnÎªÖ÷²Î¿¼Ê¸Á¿£¬¹¹½¨µÄÓÒ¾ØÕó
	double vmWm[3],vmWmVm[3];
	double Cnb[9];

	cross3(aveVm, aveWm, vmWm);
	cross3(vmWm, aveVm, vmWmVm);

	//¹¹½¨ gnMat
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			switch (i)
			{
			case 0:gnMat[i + j * 3] = aveVm[j] / norm(aveVm, 3);
				break;
			case 1:gnMat[i + j * 3] = vmWm[j] / norm(vmWm, 3);
				break;
			case 2:gnMat[i + j * 3] = vmWmVm[j] / norm(vmWmVm, 3);
				break;
			}
		}
	}
	matmul("NN", 3, 3, 3, 1.0, traMat, gnMat, 0.0, Cnb);
	// ×ËÌ¬¾ØÕó¼ÆËãËÄÔªÊı
	m2qnb(Cnb, qnb0);
}

void CoarseAlign(double* sumWm, double* sumVm, double* pos, int count, double* qnb0)
{
	int i;
	double aveWm[3], aveVm[3];
	double gn[3], wnie[3];
	double sl,cl;

	sl = sin(pos[0]);
	cl = cos(pos[0]);

	aveWm[0] = sumWm[0] / count; aveWm[1] = sumWm[1] / count; aveWm[2] = sumWm[2] / count;
	aveVm[0] = -sumVm[0] / count; aveVm[1] = -sumVm[1] / count; aveVm[2] = -sumVm[2] / count;//
	//aveVm[0] = -sumVm[0] / i; aveVm[1] = -sumVm[1] / i; aveVm[2] = -sumVm[2] / i;//

	for (i = 0; i < 3; i++)gn[i] = wnie[i]= 0.0;//³õÊ¼»¯Îª0
	gn[2] = -(G0 * (1 + 5.27094e-3 * sl * sl + 2.32718e-5 * sl * sl * sl * sl) - 3.086e-6 * pos[2]);
	wnie[1] = wie * cl; wnie[2] = wie * sl;

	Align2Att(gn, wnie, aveVm, aveWm, qnb0);
}

void InitEarth(EARTH* eth)
{
	int i;
	eth->cl = eth->sl = eth->tl = eth->RMh = eth->RNh = 0.0;
	eth->g = G0;

	for (i = 0; i < 3; i++)
	{
		eth->wnen[i] = eth->wnie[i] = eth->wnien[i] = eth->wnin[i] =
				eth->gcc[i] = eth->gn[i] = 0.0;

		if (i == 2)
			eth->gn[i] = G0;
	}
}

void UpdateEarth(EARTH* eth, double* vn, double* pos)  //¸üĞÂ´óµØÊı¾İ
{
	double sq, sq2;
	double temp[3];
	eth->sl = sin(pos[0]), eth->cl = cos(pos[0]), eth->tl = eth->sl / eth->cl;
	sq = 1 - e * e * eth->sl * eth->sl;
	sq2 = sqrt(sq);
	eth->RMh = Re * (1 - e * e) / sq / sq2 + pos[2];
	eth->RNh = Re / sq2 + pos[2]; eth->clRNh = eth->cl * eth->RNh;

	eth->wnie[0] = 0.0, eth->wnie[1] = wie * eth->cl, eth->wnie[2] = wie * eth->sl;
	eth->wnen[0] = -vn[1] / eth->RMh; eth->wnen[1] = vn[0] / eth->RNh; eth->wnen[2] = eth->wnen[1] * eth->tl;
	matrixSum(eth->wnie, eth->wnen, 3, 1, 1, eth->wnin);
	eth->gn[2] = -(G0 * (1 + 5.27094e-3 * eth->sl * eth->sl + 2.32718e-5 * eth->sl * eth->sl * eth->sl * eth->sl) - 3.086e-6 * pos[2]);
	matrixSum(eth->wnie, eth->wnin, 3, 1, 1, eth->wnien);
	cross3(eth->wnien, vn, temp);
	matrixSum(eth->gn, temp, 3, 1, -1, eth->gcc);
}
void Lever(double* qnb, double* Mpv, double* pos,double *GnssArmLength, double* pos_)// ³õÊ¼»¯¸Ë±Û²¹³¥£¬½«ÌìÏßÏàÎ»ÖĞĞÄµÄËÙ¶ÈºÍÎ»ÖÃ¹éËãÖÁÔØÌå×ø±êÏµ
{
	double Cnb[NN * NN],MpvCnb[NN * NN];
	double dpos[NN];

	q2mat(qnb, Cnb);

	matmul("NN", NN, NN, NN, 1.0, Mpv, Cnb, 0.0, MpvCnb);//MpvCnb = Mpv*Cnb
	matmul("NN", NN, 1, NN, 1.0, MpvCnb, GnssArmLength, 0.0, dpos);//dpos=MpvCnb*GnssArmLength

	matrixSum(pos, dpos, 3, 1, 1, pos_);
}
void InitPQ(KALMAN* kf, double x,double P, double  Q, int i)
{
	int j = 0;
	kf->Xk[i] = x;
	for (j = 0; j < NA; j++) {
		kf->Pxk[i + j * NA] = kf->Pxk[j + i * NA] = i == j ? P : 0.0;
		kf->Qk[i + j * NA] = kf->Qk[j + i * NA] = i == j ? Q : 0.0;
	}
}

void InitHk(KALMAN *kf)
{
	int i,j;
	for (i = 0; i < NB; i++)
	{
		for (j = 0; j < NA; j++)
		{
			kf->Hk[j + i * NA] = 0.0;
		}
	}
}

/*******************************************************************************/
//1step ´Ö¶Ô×¼
unsigned int StartCoarseAlign(I_NAV_INS * navins,  I_NAV_GNSS_RESULT *gnss)
{
	double NavEuler[3];
	double GnssPosition[3];
	inav_log(INAVMD(LOG_DEBUG),"StartCoarseAlign");
	//¶ÁÈ¡imuÖĞµÄ×ËÌ¬
	NavEuler[0] =  navins->imu.pitch*DEG2RAD;
	NavEuler[1] =  navins->imu.roll*DEG2RAD;
	NavEuler[2] =  navins->imu.heading*DEG2RAD;
	//¶ÁÈ¡¶¨Î»Î»ÖÃ
	GnssPosition[0]=	gnss->latitude*DEG2RAD;
	GnssPosition[1] = 	gnss->longitude * DEG2RAD;
	GnssPosition[2] = 	gnss->altitude ;//¸ß³Ìµ¥Î»m

#if 0
	//Çó´Ö¶Ô×¼Ê±ºòµÄËÄÔªËØ
       CoarseAlign(navins->imu.gyro, navins->imu.accm, GnssPosition, 1, navins->ins.qnb);
	//ËÄÔªËØ×ª×ËÌ¬
	qnb2att(navins->ins.qnb, navins->ins.att);
#endif
	
	navins->ins.att[0] = navins->imu.pitch*DEG2RAD;
	navins->ins.att[1] = navins->imu.roll*DEG2RAD;
	navins->ins.att[2] = navins->imu.heading*DEG2RAD;
	//RTK¹Ì¶¨½â£¬º½ÏòÊ¹ÓÃGNSSº½Ïò
	if(gnss->gnssstatus == NAV_GNSS_STATUS_RTK_FIX )
	{
		navins->ins.att[2] = gnss->heading;
	}	
	att2qnb(NavEuler,navins->ins.qnb);	
	inav_log(INAVMD(LOG_DEBUG),"StartCoarseAlign:att=[%.7f,%.7f,%.7f]",navins->ins.att[0]*RAD2DEG,navins->ins.att[1]*RAD2DEG,navins->ins.att[2]);
	return 0;
}

//2step ins³õÊ¼»¯
void InsInit(INSRESULT *ins, double ts, double* qnb, I_NAV_GNSS_RESULT *pNAV_GNSS_RESULT,double *GnssArmLength,I_NAV_INS * navins) //¹ßµ¼³õÊ¼»¯
{
	int i,j;
	inav_log(INAVMD(LOG_DEBUG),"InsInit");
	ins->ts = ts;
	ins->nts = ins->ts;
	ins->vn[0] 	=	pNAV_GNSS_RESULT->ve;
	ins->vn[1] 	=	pNAV_GNSS_RESULT->vn;
	ins->vn[2] 	=	pNAV_GNSS_RESULT->vu;
	ins->pos[0]	=	pNAV_GNSS_RESULT->latitude*DEG2RAD;
	ins->pos[1]	=	pNAV_GNSS_RESULT->longitude * DEG2RAD;
	ins->pos[2]	=	pNAV_GNSS_RESULT->altitude ;//¸ß³Ìµ¥Î»m
	for (i = 0; i < 4; i++) {
		ins->qnb[i] = qnb[i];
		if (i < 3) {
			
			ins->eb[i] = ins->db[i] = ins->an[i] = ins->fb[i] = ins->fn[i] = ins->web[i] = ins->wnb[i] = 0.0;
		}
	}

	for (i = 0; i < NN; i++)
		for (j = 0; j < NN; j++)
			ins->Mpv[j + i * NN] = 0.0;

	InitEarth(&navins->earth);
	UpdateEarth(&navins->earth, ins->vn, ins->pos);
	ins->Mpv[1] = 1 / navins->earth.clRNh; ins->Mpv[3] = 1 / navins->earth.RMh; ins->Mpv[8] = 1.0;

	// ³õÊ¼»¯¸Ë±Û²¹³¥£¬½«ÌìÏßÏàÎ»ÖĞĞÄµÄËÙ¶ÈºÍÎ»ÖÃ¹éËãÖÁÔØÌå×ø±êÏµ
	inav_log(INAVMD(LOG_DEBUG),"before Lever ,ins->pos=[%.7f, %.7f,%.7f]",ins->pos[0],ins->pos[1],ins->pos[2]);
	Lever(qnb, ins->Mpv,  ins->pos,g_Compensate_Params.gnssArmLength, ins->pos);
	inav_log(INAVMD(LOG_DEBUG),"after Lever ,ins->pos=[%.7f, %.7f,%.7f]",ins->pos[0],ins->pos[1],ins->pos[2]);

#if 0	
	//???³õÊ¼»¯½ÇËÙ¶ÈÆ«²î
	ins->eb[0] = INC_EB0;
	ins->eb[1] = INC_EB1;
	ins->eb[2] = INC_EB2;
#endif	
}

//3step KfInit
void KfInit(KALMAN* kf)  //¿¨¶ûÂüÂË²¨³õÊ¼»¯
{
	int i = 0;
	inav_log(INAVMD(LOG_DEBUG),"KfInit");
	for (i = 0; i < NA; i++)
	{
		// ´Ë´¦³õÊ¼»¯»¹ĞèÒªÓÅ»¯
		if (i >= 0 && i < 2)
			InitPQ(kf, 0.0, DATT_VAR, ANGNRANDOMWALK2, i); //att   ANGNRANDOMWALK2*nts*nts
		else if (i == 2)
			InitPQ(kf, 0.0, DATT_VAR, ANGNRANDOMWALK2, i);
		else if (i >= 3 && i < 5)
			InitPQ(kf, 0.0, DVEL_VAR, VELRANDOMWALK2, i); //vel   VELRANDOMWALK2*nts*nts
		else if (i == 5)
			InitPQ(kf, 0.0, DVEL_VAR, VELRANDOMWALK2, i); //  20 * UGPSHZ * 20 * UGPSHZ*nts*nts
		else if (i >= 6 && i < 8)
			//InitPQ(kf, 0.0, DPOS_VAR, 0.0, i); //pos
			InitPQ(kf, 0.0, DPOS_VAR, INS_POS_VAR, i); 
		else if (i == 8)
			//InitPQ(kf, 0.0, 1 * 1, 0.0, i);//×¢Òâ´Ë´ÎÓÃÁË1 * 1,ÓÉÓÚ¸ß³ÌÊ¹ÓÃÃ×µ¥Î»£¬²»ĞèÒª³ıÒÔRe
			InitPQ(kf, 0.0, 1 * 1, INS_HEAD_VAR, i);
		else if (i >= 9 && i < 12)
			InitPQ(kf, 0.0, GYROBIAS2, 0.0, i); //eb

		else if (i >= 12 && i < 14)
			InitPQ(kf, 0.0, ACCBIAS2, 0.0, i); //db
		else if (i == 14)
			InitPQ(kf, 0.0, ACCBIAS2, 0.0, i);

		else if (i >= 15 && i < 18)
			InitPQ(kf, 0.0, LEVER_VAR, 0.0, i); //lever
	}
	
	PrintOutMatrixMsg(NA,NA,kf->Pxk, "P");
	//PrintOutMatrixMsg(NA,NA,kf->Qk, "Q");

	for (i = 0; i < NB; i++)
	{
		if (i < 3)
			//kf->RK[i] = VEL_VAR;
			kf->RK[i] = VEL_VAR_F;
		else if (i < 5)
			kf->RK[i] = RTK_POS_VAR;

		else if(i==5)
			//kf->RK[i] = 0.1 * 0.1;
			kf->RK[i] = 0.3 * 0.3;//? ¸ß³ÌÎó²î, RTK£¬PPPÀíÂÛ¸ß³ÌÎó²î´óÔ¼ÊÇ3±¶µÄË®Æ½Îó²î
		else
			kf->RK[i]=HEAD_VAR;
	}
	InitHk(kf);

	kf->b = 0.95;
	kf->beta = 1.0;

	for (i = 0; i < NB; i++)
	{
		kf->Rmax[i] = kf->RK[i] * 100;
		kf->Rmin[i] = kf->RK[i] * 0.01;
	}
}
void UpdateKfRHuge(KALMAN* kf)
{
	int i = 0;
	double mvel_var[3],mposvar[3];
	for(i=0;i<3;i++)
	{
		mvel_var[i] 	=	VEL_VAR_F*1.0e8;
		mposvar[i]	=	SPP_POS_VAR*1.0e8;
	}
	mposvar[2]	=	5.0*5.0*1.0e8;//rtk ¸ß³ÌÎó²î0.1m
	
	for (i = 0; i < NB; i++)
	{
		if (i < 3)
			kf->RK[i] = mvel_var[i];

		else if (i <= 5)
			kf->RK[i] = mposvar[i];
	}
}
void UpdateKfR(KALMAN* kf ,I_NAV_GNSS_RESULT *pNavGnss)
{
	int i = 0;
	double mvel_var[3]={0.0},mposvar[3]={0.0};
	inav_log(INAVMD(LOG_DEBUG),"UpdateKfR");
	
	if(1 == pNavGnss->supportposvelstd)
	{
		mposvar[0]	=	(pNavGnss->latstd/Re)*(pNavGnss->latstd/Re);
		mposvar[1]	=	(pNavGnss->logstd/Re)*(pNavGnss->logstd/Re);
		mposvar[2]	=	pNavGnss->hstd*pNavGnss->hstd;
		mvel_var[0] 	=	(pNavGnss->vestd/SAMPLE_Ratio)*(pNavGnss->vestd/SAMPLE_Ratio);
		mvel_var[1] 	=	(pNavGnss->vnstd/SAMPLE_Ratio)*(pNavGnss->vnstd/SAMPLE_Ratio);
		mvel_var[2] 	=	(pNavGnss->vustd/SAMPLE_Ratio)*(pNavGnss->vustd/SAMPLE_Ratio);			
	}
	else if(0 == pNavGnss->supportposvelstd && NAV_GNSS_STATUS_RTK_FIX == pNavGnss->gnssstatus)
	{
		for(i=0;i<3;i++)
		{
			mvel_var[i] 	=	VEL_VAR_F ;
			mposvar[i]	=	RTK_POS_VAR;
		}
		mposvar[2]	=	0.1*0.1;//rtk ¸ß³ÌÎó²î0.1m
	}
	else 
	{
		for(i=0;i<3;i++)
		{
			mvel_var[i] 	=	VEL_VAR_F;
			mposvar[i]	=	SPP_POS_VAR;
		}
		mposvar[2]	=	5.0*5.0;//·Çrtk¸ß³ÌÎó²î5m
	}

	for (i = 0; i < NB; i++)
	{
		if (i < 3)
		{
			kf->RK[i] = mvel_var[i];
		}
	      else if (i >= 3&&i <= 5)
	      	{
			kf->RK[i] = mposvar[i-3];
	      	}
		//else
		//	kf->RK[i]=HEAD_VAR;//ÔİÊ±Î´ÓÃ
	}

	inav_log(INAVMD(LOG_DEBUG),"mposvar[%.7E,%.7E,%.7E]",mposvar[0],mposvar[1],mposvar[2]);
	inav_log(INAVMD(LOG_DEBUG),"mvel_var[%.7E,%.7E,%.7E]",mvel_var[0],mvel_var[1],mvel_var[2]);
	PrintOutMatrixMsg(NB,1,kf->RK, "kf->RK");
	
}
void Cnscl(IMUDATA* imu, double* phim, double* dvbm)
{
	int i;
	double cm[NN], sm[NN];
	double phim_[NN];
	double wmm_vmm[NN],cm_pvm[NN],sm_pwm[NN];
	double temp1[NN], temp2[NN];

	for (i = 0; i < NN; i++) {
		cm[i] = (imu->gyro_pre[i]) / 12.0;
		sm[i] = (imu->accm_pre[i]) / 12.0;
	}
	for (i = 0; i < NN; i++) {
		imu->accm_pre[i] = imu->accm[i];
		imu->gyro_pre[i] = imu->gyro[i];
	}
	cross3(cm, imu->gyro, phim_);
	matrixSum(imu->gyro, phim_, 3, 1, 1, phim);

	cross3(imu->gyro, imu->accm, wmm_vmm);
	for (i = 0; i < NN; i++)
		wmm_vmm[i] = 0.5 * wmm_vmm[i];
	cross3(cm, imu->accm, cm_pvm);
	cross3(sm, imu->gyro, sm_pwm);
	matrixSum(imu->accm, wmm_vmm, 3, 1, 1, temp1);
	matrixSum(cm_pvm, sm_pwm, 3, 1, 1, temp2);
	matrixSum(temp1, temp2, 3, 1, 1, dvbm);
}

void InsUpdate(IMUDATA *imu,INSRESULT *ins,I_NAV_INS * navins) //¹ßµ¼¸üĞÂ
{
	int i;
	double nts=ins->nts;
	double phim_[NN], dvbm_[NN], phim[NN], dvbm[NN];
	double vn01[NN], pos01[NN],dpos[NN],vn1[NN],vn_[NN];
	double wib[NN], fb[NN],web_[NN],wnb_[NN],wnin_[NN],an_[NN];
	double Cnb[NN * NN];
	double dpos_[NN];

	inav_log(INAVMD(LOG_DEBUG),"InsUpdate");
      //PrintOutInsMsg();
//------------------------------------------------------------------------------------
	//Ê¹ÓÃ¶ş×ÓÑùËã·¨£¬Ã¿´ÎÊäÈëÁ½´ÎµÄ¹ßĞÔÆ÷¼ş²ÉÑùÊı¾İºÍÇ°Á½Ê±¿ÌµÄavpÊı¾İ???
	Cnscl(imu, phim_, dvbm_);//Ë«×ÓÑùÌØÊâ´¦Àí
	matrixSum(phim_, ins->eb, 3, 1, -nts, phim);// kg*phim-eb*nts   ÍÓÂİ½ÇËÙ¶È
	matrixSum(dvbm_, ins->db, 3, 1, -nts, dvbm);// ka*dvbm-db*nts ¼ÓËÙ¶È
//------------------------------------------------------------------------------------
	matrixSum(ins->vn, ins->an, 3, 1, nts / 2, vn01);//¼ÆËãntsÕâ¶ÎÊ±¼äÆ½¾ùËÙ¶È
	matmul("NN", 3, 1, 3, 1.0, ins->Mpv, vn01, 0.0, dpos);
	matrixSum(ins->pos, dpos, 3, 1, nts / 2, pos01);//¼ÆËãntsÕâ¶ÎÊ±¼äÎ»ÖÃ

	UpdateEarth(&navins->earth, vn01, pos01);//¸üĞÂ´óµØÊı¾İ
	for (i = 0; i < 3; i++) {wib[i] = phim[i] / nts;fb[i] = dvbm[i] / nts;}
	q2mat(ins->qnb, Cnb);

	matmul("TN", 3, 1, 3, 1, Cnb, navins->earth.wnie,0.0,web_);
	matrixSum(wib, web_, 3, 1, -1, ins->web);

	matmul("TN", 3, 1, 3, 1, Cnb, navins->earth.wnin, 0.0, wnb_);
	matrixSum(wib, wnb_, 3, 1, -1, ins->wnb);
//---------------vn update----------------------------------------------------------------
	qmulv(ins->qnb, fb, ins->fn);
	for (i = 0; i < 3; i++)wnin_[i] = navins->earth.wnin[i] * (-nts/2);
	rotv(wnin_,ins->fn, an_);
	matrixSum(an_, navins->earth.gcc, 3, 1, 1, ins->an);
	for (i = 0; i < 3; i++) { vn_[i] = ins->an[i] * nts; }
	matrixSum(ins->vn, vn_, 3, 1, 1, vn1);
//---------------pos update---------------------------------------------------------------
	ins->Mpv[3] = 1 / navins->earth.RMh; ins->Mpv[1] = 1 / navins->earth.clRNh;
	dpos_[0] = ((ins->vn[1] + vn1[1]) / navins->earth.RMh) * (nts / 2);//Ê¹ÓÃÆ½¾ùËÙ¶È¼ÆËãÎ»ÖÃ±ä»¯
	dpos_[1] = ((ins->vn[0] + vn1[0]) / navins->earth.clRNh) * (nts / 2);
	dpos_[2] = (ins->vn[2] + vn1[2]) *( nts / 2);
	matrixSum(ins->pos, dpos_, 3, 1, 1, ins->pos);
	for (i = 0; i < 3; i++) { ins->vn[i] = vn1[i]; }
//----------------att update--------------------------------------------------------------
	for (i = 0; i < 3; i++)wnin_[i] = navins->earth.wnin[i] * nts;
	UpdateQnb(ins->qnb, phim, wnin_);
	qnb2att(ins->qnb, ins->att);
	inav_log(INAVMD(LOG_DEBUG),"After InsUpdate");
	//PrintOutInsMsg();
}

unsigned char KfFk(INSRESULT * ins, double* Fk,I_NAV_INS * navins)//¿¨¶ûÂüÂË²¨¡¢¹¹½¨FK¾ØÕó,¼ÆËãÒ»´Î×´Ì¬¸üĞÂ¾ØÕó
{
	int i,j;
	double* Ft = zeros(NA, NA);
	double tl = navins->earth.tl, secl = 1 / navins->earth.cl;
	double f_RMh = 1 /navins->earth.RMh, f_RNh = 1 / navins->earth.RNh, f_clRNh = 1 / navins->earth.clRNh;
	double f_RMh2 = f_RMh * f_RMh, f_RNh2 = f_RNh * f_RNh;
	double vn[NN]; for (i = 0; i < NN; i++)vn[i] = ins->vn[i];
	double vE_clRNh = vn[0] * f_clRNh, vE_RNh2 = vn[0] * f_RNh2, vN_RMh2 = vn[1] * f_RMh2;
	double scl;

	double* wnin_ = zeros(3, 1); for (i = 0; i < NN; i++)wnin_[i] = (-navins->earth.wnin[i]);
	double* Maa = zeros(3, 3), * Mav = zeros(3, 3), * Map = zeros(3, 3);
	double* Mva = zeros(3, 3), * Mvv = zeros(3, 3), * Mvp = zeros(3, 3);
	double* Mpv = zeros(3, 3), * Mpp = zeros(3, 3);
	double* Mvv_ = zeros(3, 3), * Mvp_ = zeros(3, 3);
	double* Mp1 = zeros(3, 3), * Mp2 = zeros(3, 3);
	double* Avn = zeros(3, 3), * Awn = zeros(3, 3);
	double* Cnb = zeros(3, 3);
	double* I = eyes(NA);

	inav_log(INAVMD(LOG_DEBUG),"KfFk");

	if(		NULL == Ft || NULL == wnin_
		||	NULL == Maa || NULL == Mav || NULL == Map || NULL == Mva || NULL == Mvv || NULL == Mvp || NULL == Mpv || NULL == Mpp || NULL == Mvv_ || NULL == Mvp_
		||	NULL == Mp1 || NULL == Mp2 || NULL == Avn || NULL == Awn || NULL == Cnb || NULL == I	
	 )
	{
		inav_log(INAVMD(LOG_ERR),"P is NULL");
		free(Ft); free(wnin_); free(Maa); free(Mav); free(Map); free(Mva); free(Mvv); free(Mvp);
		free(Mpv); free(Mpp); free(Mvv_); free(Mvp_); free(Mp1); free(Mp2); free(Avn); free(Awn);
		free(Cnb); free(I);
		return INS_FUN_RETURN_FAIL;
		
	}

	Mp1[1] = (-navins->earth.wnie[2]); Mp1[2] = navins->earth.wnie[1];
	Mp2[2] = vE_clRNh * secl; Mp2[6] = vN_RMh2; Mp2[7] = (-vE_RNh2); Mp2[8] = (-vE_RNh2 * tl);
	askew(vn, Avn);
	askew(navins->earth.wnien, Awn);

	askew(wnin_, Maa);

	Mav[1] = f_RNh; Mav[2] = f_RNh * tl; Mav[3] = (-f_RMh);
	matrixSum(Mp1, Mp2, 3, 3, 1, Map);

	askew(ins->fn, Mva);

	matmul("NN", 3, 3, 3, 1.0, Avn, Mav, 0.0, Mvv_);
	matrixSum(Mvv_, Awn, 3, 3, -1, Mvv);

	matrixSum(Mp1, Map, 3, 3, 1, Mvp_);
	matmul("NN", 3, 3, 3, 1.0, Avn, Mvp_, 0.0, Mvp);
	scl = navins->earth.sl *navins->earth.cl;
	Mvp[2] = Mvp[2] - G0 * (5.27094e-3 * 2 + 2.32718e-5 * 4 * navins->earth.sl * navins->earth.sl) * scl;
	Mvp[8] = Mvp[8] + 3.086e-6;

	Mpp[1] = vE_clRNh * tl; Mpp[6] = (-vN_RMh2); Mpp[7] = (-vE_RNh2) * secl;
	// ¹¹½¨FK¾ØÕó
	q2mat(ins->qnb, Cnb);

	PrintOutMatrixMsg(3,3,Map, "KfFk-Map");
	PrintOutMatrixMsg(3,3,Mvp, "KfFk-Mvp");
	PrintOutMatrixMsg(3,3,Mpp, "KfFk-Mpp");
	PrintOutMatrixMsg(3,3,Cnb, "KfFk-Cnb");
	PrintOutMatrixMsg(3,3,Maa, "KfFk-Maa");
	PrintOutMatrixMsg(3,3,Mva, "KfFk-Mva");
	PrintOutMatrixMsg(3,3,Mav, "KfFk-Mav");
	PrintOutMatrixMsg(3,3,Mvv, "KfFk-Mvv");
	PrintOutMatrixMsg(3,3,Mpv, "KfFk-Mpv");
	PrintOutMatrixMsg(NA,NA,Ft, "KfFk-Ft before");
	for (i = 0; i < NA; i++)
	{
		for (j = 0; j < NA; j++)
		{
			if (i < 3 && j < 3) {
				switch (i)
				{
				case 0:Ft[i + j * NA] = Maa[i + j * NN]; break;
				case 1:Ft[i + j * NA] = Maa[i + j * NN]; break;
				case 2:Ft[i + j * NA] = Maa[i + j * NN]; break;
				}
			}
			else if (i >= 3 && i < 6 && j < 3) {
				switch (i)
				{
				case 3:Ft[i + j * NA] = Mva[i - NN + j * NN]; break;
				case 4:Ft[i + j * NA] = Mva[i - NN + j * NN]; break;
				case 5:Ft[i + j * NA] = Mva[i - NN + j * NN]; break;
				}
			}
			else if (i < 3 && j >= 3 && j < 6) {
				switch (i)
				{
				case 0:Ft[i + j * NA] = Mav[i + (j - NN) * NN]; break;
				case 1:Ft[i + j * NA] = Mav[i + (j - NN) * NN]; break;
				case 2:Ft[i + j * NA] = Mav[i + (j - NN) * NN]; break;
				}
			}
			else if (i >= 3 && i < 6 && j >= 3 && j < 6) {
				switch (i)
				{
				case 3:Ft[i + j * NA] = Mvv[(i - NN) + (j - NN) * NN]; break;
				case 4:Ft[i + j * NA] = Mvv[(i - NN) + (j - NN) * NN]; break;
				case 5:Ft[i + j * NA] = Mvv[(i - NN) + (j - NN) * NN]; break;
				}
			}
			else if (i >= 6 && i < 9 && j >= 3 && j < 6) {
				switch (i)
				{
				case 6:Ft[i + j * NA] = ins->Mpv[(i - NN * 2) + (j - NN) * NN]; break;
				case 7:Ft[i + j * NA] = ins->Mpv[(i - NN * 2) + (j - NN) * NN]; break;
				case 8:Ft[i + j * NA] = ins->Mpv[(i - NN * 2) + (j - NN) * NN]; break;
				}
			}
			else if (i < 3 && j >= 6 && j < 9) {
				Ft[i + j * NA] = Map[i + (j - NN * 2) * NN];
			}
			else if (i >= 3 && i < 6 && j >= 6 && j < 9) {
				Ft[i + j * NA] = Mvp[(i - NN) + (j - NN * 2) * NN];
			}
			else if (i >= 6 && i < 9 && j >= 6 && j < 9) {
				Ft[i + j * NA] = Mpp[(i - NN * 2) + (j - NN * 2) * NN];
			}
			else if (i < 3 && j >= 9 && j < 12) {
				Ft[i + j * NA] = -Cnb[i + (j - NN * 3) * NN];
			}
			else if (i >= 3 && i < 6 && j >= 12 && j < 15) {
				Ft[i + j * NA] = Cnb[(i - NN) + (j - NN * 4) * NN];
			}
		}
	}
	PrintOutMatrixMsg(NA,NA,Ft, "KfFk-Ft");
	inav_log(INAVMD(LOG_DEBUG),"KfFk-Ft[7 + 1 * NA]=%.7f",Ft[7 + 1 * NA]);
	inav_log(INAVMD(LOG_DEBUG),"KfFk-Ft[1 + 7 * NA]=%.7f",Ft[1 + 7 * NA]);
	matrixSum(I, Ft, NA, NA, ins->nts, Fk);//FK=I+Ft

	PrintOutMatrixMsg(NA,NA,Fk, "KfFk-Fk");
	
	free(Ft); free(wnin_); free(Maa); free(Mav); free(Map); free(Mva); free(Mvv); free(Mvp);
	free(Mpv); free(Mpp); free(Mvv_); free(Mvp_); free(Mp1); free(Mp2); free(Avn); free(Awn);
	free(Cnb); free(I);

	return INS_FUN_RETURN_SUCESS;
}

unsigned char  KfTimeUpdate(KALMAN* kf, double* Fk, double nts,int m) //kfÊ±¼ä¸üĞÂ
{
	double* XK = zeros(NA, 1);
	double* FP = zeros(NA, NA);
	double* FPF = zeros(NA, NA);
	double* Pxk_temp = zeros(NA, NA);
	inav_log(INAVMD(LOG_DEBUG),"KfTimeUpdate");
	if(NULL==XK || NULL==FP ||NULL==FPF ||NULL==Pxk_temp )
	{
		inav_log(INAVMD(LOG_ERR),"XK,FP,FPF,Pxk_temp fail");
		free(XK); free(FP); free(FPF); free(Pxk_temp);
		return INS_FUN_RETURN_FAIL;
	}
	PrintOutMatrixMsg(NA,NA,kf->Pxk, "KfTimeUpdate1-P");
	PrintOutMatrixMsg(NA,NA,Fk, "KfTimeUpdate1-Fk");
	matmul("NN", NA, 1, NA, 1.0, Fk, kf->Xk, 0.0, kf->Xk);// X_k=F*X_k-1
	matmul("NN", NA, NA, NA, 1.0, Fk, kf->Pxk, 0.0, FP);// F*P
	PrintOutMatrixMsg(NA,NA,FP, "KfTimeUpdate1-FP");
	matmul("NT", NA, NA, NA, 1.0, FP, Fk, 0.0, FPF);// FPF'
	PrintOutMatrixMsg(NA,NA,kf->Qk, "KfTimeUpdate1-Qk");
	inav_log(INAVMD(LOG_DEBUG),"KfTimeUpdate-m * nts=%.7f",m * nts);
	PrintOutMatrixMsg(NA,NA,FPF, "KfTimeUpdate2-FPF");
	matrixSum(FPF, kf->Qk, NA, NA, m * nts, kf->Pxk);// P_k=F*P_k-1*F+Qk
	PrintOutMatrixMsg(NA,NA,kf->Pxk, "KfTimeUpdate2-P");
	//symmetry(Pxk_temp, NA, kf->Pxk);
	free(XK); free(FP); free(FPF); free(Pxk_temp);
	return INS_FUN_RETURN_SUCESS;
}

void KfHk(double *CW,double *MpvCnb,double *H,int n,int m,double* Hk)//HK
{
	int i,j;
	if (MOTION_STATUS_HEADING== g_STATIC_DETECTION.state)
	{
		for (i = 0; i < m; i++)
		{
			if (i < m - 1)
			{
				for (j = 0; j < n; j++)
				{
					if (i < 6 && j < 3)Hk[i + j * m] = 0.0;
					else if (i < 6 && j >= 3 && j < 9)Hk[i + j * m] = j - 3 == i ? 1.0 : 0.0;
					else if (i < 6 && j >= 9 && j < 15)Hk[i + j * m] = 0.0;
					else if (i < 3 && j >= 15 && j < 18)
					{
						if (0 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-CW[0]); break;
							case 16:Hk[i + j * m] = (-CW[3]); break;
							case 17:Hk[i + j * m] = (-CW[6]); break;
							}
						}
						else if (1 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-CW[1]); break;
							case 16:Hk[i + j * m] = (-CW[4]); break;
							case 17:Hk[i + j * m] = (-CW[7]); break;
							}
						}
						else if (2 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-CW[2]); break;
							case 16:Hk[i + j * m] = (-CW[5]); break;
							case 17:Hk[i + j * m] = (-CW[8]); break;
							}
						}
					}
					else if (i >= 3 && j >= 15 && j < 18)
					{
						if (3 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-MpvCnb[0]); break;
							case 16:Hk[i + j * m] = (-MpvCnb[3]); break;
							case 17:Hk[i + j * m] = (-MpvCnb[6]); break;
							}
						}
						else if (4 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-MpvCnb[1]); break;
							case 16:Hk[i + j * m] = (-MpvCnb[4]); break;
							case 17:Hk[i + j * m] = (-MpvCnb[7]); break;
							}
						}
						else if (5 == i) {
							switch (j)
							{
							case 15:Hk[i + j * m] = (-MpvCnb[2]); break;
							case 16:Hk[i + j * m] = (-MpvCnb[5]); break;
							case 17:Hk[i + j * m] = (-MpvCnb[8]); break;
							}
						}
					}
				}
			}
			else
			{
				for (j = 0; j < n; j++)
				{
					if (j == 9)Hk[i + j * m] = H[0];
					else if (j == 10)Hk[i + j * m] = H[1];
					else if (j == 11)Hk[i + j * m] = H[2];
					else Hk[i + j * m] = 0.0;
				}
			}

		}
	}

	else
	{
		for (i = 0; i < m; i++)
		{
			if(i<m)
			{
				for (j = 0; j < n; j++)
				{
								if (i < 6 && j < 3)Hk[i + j * m] = 0.0;
								else if (i < 6 && j >= 3 && j < 9)Hk[i + j * m] = j - 3 == i ? 1.0 : 0.0;
								else if (i < 6 && j >= 9 && j < 15)Hk[i + j * m] = 0.0;
								else if (i < 3 && j >= 15 && j < 18)
								{
									if (0 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-CW[0]); break;
										case 16:Hk[i + j * m] = (-CW[3]); break;
										case 17:Hk[i + j * m] = (-CW[6]); break;
										}
									}
									else if (1 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-CW[1]); break;
										case 16:Hk[i + j * m] = (-CW[4]); break;
										case 17:Hk[i + j * m] = (-CW[7]); break;
										}
									}
									else if (2 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-CW[2]); break;
										case 16:Hk[i + j * m] = (-CW[5]); break;
										case 17:Hk[i + j * m] = (-CW[8]); break;
										}
									}
								}
								else if (i >= 3 && j >= 15 && j < 18)
								{
									if (3 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-MpvCnb[0]); break;
										case 16:Hk[i + j * m] = (-MpvCnb[3]); break;
										case 17:Hk[i + j * m] = (-MpvCnb[6]); break;
										}
									}
									else if (4 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-MpvCnb[1]); break;
										case 16:Hk[i + j * m] = (-MpvCnb[4]); break;
										case 17:Hk[i + j * m] = (-MpvCnb[7]); break;
										}
									}
									else if (5 == i) {
										switch (j)
										{
										case 15:Hk[i + j * m] = (-MpvCnb[2]); break;
										case 16:Hk[i + j * m] = (-MpvCnb[5]); break;
										case 17:Hk[i + j * m] = (-MpvCnb[8]); break;
										}
									}
								}
							}
			}

			else
			{
				for (j = 0; j < n; j++)
				{
					if(j==2)
					{
						Hk[i + j * m]=1.0;
					}
					else
					{
						Hk[i + j * m]=0.0;
					}
				}
			}
		}
	}
}

void LeverarmTimeCorr(INSRESULT* ins,double *gpsVn,double *gpsPos,double heading,double* lever, double dt,
								int n,int m,double *Zk,double *Hk)
{
	int i, k;

	double CW_[NN * NN],CW[NN*NN];
	double Cnb[NN*NN];
	double MpvCnb[NN* NN],MpvVn[NN];
	double Cwlever[NN],MpvCnblever[NN];
	double vnL_[NN], vnL[NN],posL_[NN],posL[NN];

	double Hk_k_1[NN];
	Hk_k_1[0] = 0.0; Hk_k_1[1] = sin(ins->att[1]) / cos(ins->att[0]); Hk_k_1[2] = cos(ins->att[1]) / cos(ins->att[0]);

	askew(ins->web, CW_);
	q2mat(ins->qnb, Cnb);
	matmul("NN", 3, 3, 3, 1.0, Cnb, CW_, 0.0, CW);
	matmul("NN", 3, 3, 3, 1.0, ins->Mpv, Cnb, 0.0, MpvCnb);
	matmul("NN", 3, 1, 3, 1.0, ins->Mpv, ins->vn, 0.0, MpvVn);

	matmul("NN", 3, 1, 3, 1.0, CW, lever, 0.0, Cwlever);
	matrixSum(ins->vn, Cwlever, 3, 1, 1, vnL_);
	matrixSum(vnL_, ins->an, 3, 1, -dt, vnL);

	matmul("NN", 3, 1, 3, 1.0, MpvCnb, lever, 0.0, MpvCnblever);
	matrixSum(ins->pos, MpvCnblever, 3, 1, 1, posL_);
	matrixSum(posL_, MpvVn, 3, 1, -dt, posL);

#if 1
	if (MOTION_STATUS_HEADING==g_STATIC_DETECTION.state)//µ±Ç°Ã»ÓĞ¸ÄÄ£Ê½
	{
		for (i = 0, k = 0; i < 6; i++)
		{
			if (i < 3)Zk[k++] = vnL[i] - gpsVn[i];
			else Zk[k++] = posL[i - 3] - gpsPos[i - 3];
		}
		//-------º½ÏòÔ¼Êø²Ğ²îZk[6]--H¾ØÕó¹¹½¨--------------------------------------------------------------
		Zk[k]=ins->att[2]-ins->att_pre[2];
		//-----------------------------------------------------------------------------------
		KfHk(CW, MpvCnb,Hk_k_1,n,m,Hk);
	}
#endif
	else
	{
		for (i = 0, k = 0; i < m; i++)
		{
			if (i < 3)Zk[k++] = vnL[i] - gpsVn[i];
			else if(i<6)Zk[k++] = posL[i - 3] - gpsPos[i - 3];
			//else Zk[k++]=ins->att[2]-heading;
			//else Zk[k++]=0.0;
		}
		KfHk(CW, MpvCnb, NULL,n,m,Hk);
	}
}

static unsigned char MeasResp(double* zk,double *Xk, double* Hk, int n, int m, double* v)
{
	double *HX;
	HX = zeros(m, 1);
	if(NULL == HX)
	{
		inav_log(INAVMD(LOG_ERR),"HX fail");
		free(HX);
		return INS_FUN_RETURN_FAIL;
	}
	matmul("NN", m, 1, n, 1.0, Hk, Xk, 0.0, HX);
	matrixSum(zk, HX, m, 1, -1, v);
	free(HX);
	return INS_FUN_RETURN_SUCESS;
}

double  RadAptive(double* P, double* H, double* Zk, double* Rk, double* Rmin, double* Rmax, int n, int m, double beta, double b, double* Rk_)
{
	int i;
	double* Fk = zeros(n, m), * Q = zeros(m, m);
	double ry, beta_;

	inav_log(INAVMD(LOG_DEBUG),"RadAptive");
	if(NULL==Fk || NULL==Q)
	{
		inav_log(INAVMD(LOG_ERR),"Fk,Q fail");
		free(Fk); free(Q);
		return 0;
	}
	//Á¿²â×ÔÊÊÓ¦ÂË²¨
	matmul("NT", n, m, n, 1.0, P, H, 0.0, Fk);//PH'
	matmul("NN", m, m, n, 1.0, H, Fk, 0.0, Q);//HPH'

	for (i = 0; i < m; i++)
	{
		ry = Zk[i] * Zk[i] - Q[i + i * m];
		if (ry < Rmin[i])ry = Rmin[i];
		if (ry > Rmax[i])Rk_[i] = Rmax[i];
		else Rk_[i] = (1 - beta) * Rk[i] + beta * ry;
	}
	beta_ = beta / (beta + b);
	free(Fk); free(Q);
	return beta_;
}
void MatrixRk(double* R,int m, double* Rk)
{
	int i,j;
	inav_log(INAVMD(LOG_DEBUG),"MatrixRk");
	if (g_STATIC_DETECTION.state!=MOTION_STATUS_HEADING)
	{
		for (i = 0; i < m; i++)
		{
			if(i<m)
			{
				for(j=0;j<m;j++)
				{
					Rk[i + j * m] = i == j ? R[i] : 0.0;
				}
			}
			else
			{
				for(j=0;j<m;j++)
				{
					Rk[i + j * m] = i == j ? R[i] : 0.0;
				}
			}
		}
	}
	else
	{
		for (i = 0; i < m; i++)
		{
			if (i < m-1)
			{
				for (j = 0; j < m; j++)
				{
					Rk[i + j * m] = i == j ? R[i] : 0.0;
				}
			}
			else
			{
				for (j = 0; j < m; j++)
				{
					Rk[i + j * m] = i == j ? ANGNRANDOMWALK2:0.0;
				}
			}
		}
	}
	//PrintOutMatrixMsg(m,m,Rk, "R");//´òÓ¡R¾ØÕó
}
int KfMeasUpdate(double* x, double* P, double* H, double* v,double *R, int n, int m, double* X_, double* P_)
{
	double* Fk = zeros(n, m), * Q = zeros(m, m),  *K = zeros(n, m), * I = eyes(n);
	double* Q_ = zeros(m, m), * KQ_ = zeros(n, m), * KQ_K = zeros(n, n);
	double* I_P = zeros(n, n), * I_P_I = zeros(n, n), * RK = zeros(m, n), * KRK = zeros(n, n);
	double* P_temp = zeros(n, n);double* I_KH=zeros(n, n);//I-KH
	int info=0;

	if(		NULL == Fk || NULL == Q || NULL == K || NULL == I
		||	NULL == Q_ || NULL == KQ_ || NULL == KQ_K
		||	NULL == I_P || NULL == I_P_I || NULL == RK || NULL == KRK
		||	NULL == P_temp || NULL == I_KH)
	{
		inav_log(INAVMD(LOG_ERR),"P is NULL");
		free(Fk); free(Q); free(K); free(I); free(Q_); free(KQ_); free(KQ_K);
		free(I_P); free(I_P_I); free(RK); free(KRK); free(P_temp);free(I_KH);
		return INS_FUN_RETURN_FAIL;;
	}

	inav_log(INAVMD(LOG_DEBUG),"KfMeasUpdate");
	matcpy(Q, R, m, m);
	matcpy(X_, x, n, 1);

	matmul("NT", n, m, n, 1.0, P, H, 0.0, Fk);
	matmul("NN", m, m, n, 1.0, H, Fk, 1.0, Q);

	matcpy(Q_, Q, m, m);
	
	if(1 == MeaUpdatePSetting)
	{
		if (!(info = matinv(Q, m))) {
			matmul("NN", n, m, m, 1.0, Fk, Q, 0.0, K);
			matmul("NN", n, 1, m, 1.0, K, v, 1.0, X_);
			matmul("NN", n, n, m, 1.0, K, H, 0.0, I_KH);
			matrixSum(I,I_KH, n, n, -1, I_KH);
			matmul("NN", n, n, n, 1.0, I_KH, P, 0.0, P_);			
		}
	}
	else
	{
		if (!(info = matinv(Q, m))) {
			matmul("NN", n, m, m, 1.0, Fk, Q, 0.0, K);
			matmul("NN", n, 1, m, 1.0, K, v, 1.0, X_);
			matmul("NN", n, m, m, 1.0, K, Q_, 0.0, KQ_);
			matmul("NT", n, n, m, 1.0, KQ_, K, 0.0, KQ_K);
			matrixSum(P, KQ_K, n, n, -1, P_temp);//P-K(HPH'+R)K
		}
		symmetry(P_temp, n, P_);
	}
	free(Fk); free(Q); free(K); free(I); free(Q_); free(KQ_); free(KQ_K);
	free(I_P); free(I_P_I); free(RK); free(KRK); free(P_temp);free(I_KH);
	return INS_FUN_RETURN_SUCESS;
}

unsigned char GnssInsFusion(INSRESULT* ins, I_NAV_GNSS_RESULT* gnss,KALMAN* kf ,double *lever ,double dt) //GnssInsÈÚºÏ¡¢¸Ë±Û¡¢Ê±¼äÍ¬²½²¹³¥¡¢Á¿²â¸üĞÂ
{
	int n, m;
	double *Zk, *Hk, *Rk;
	double GnssPosition[3];
	double GnssVel[3];
	inav_log(INAVMD(LOG_DEBUG),"GnssInsFusion");
	GnssPosition[0]=	gnss->latitude*DEG2RAD;
	GnssPosition[1] = gnss->longitude * DEG2RAD;
	GnssPosition[2] = gnss->altitude ;
	GnssVel[0]			= gnss->ve;
	GnssVel[1]			= gnss->vn;
	GnssVel[2]			= gnss->vu;
	
#if 1
	if (MOTION_STATUS_HEADING==g_STATIC_DETECTION.state)//µ±Ç°Ã»ÓĞ¸ÄÄ£Ê½
	{
		n = NA; m = NB+1;
	}
	else
	{
		n = NA; m = NB;
	}
#endif
	//n = NA; m = NB;
	Hk = zeros(n, m); Rk = zeros(n, m); Zk = zeros(m, 1);
	if(NULL == Hk ||NULL == Rk ||NULL == Zk )
	{
		inav_log(INAVMD(LOG_ERR),"Hk,Rk,Zk fail");
		free(Hk); free(Rk); free(Zk);
		return INS_FUN_RETURN_FAIL;
	}

	// ¸Ë±Û¡¢Ê±¼äÍ¬²½²¹³¥£»·µ»ØZk and Hk
	LeverarmTimeCorr(ins, GnssVel,GnssPosition,gnss->heading,lever, dt, n,m,Zk,Hk);
	inav_log(INAVMD(LOG_DEBUG),"after LeverarmTimeCorr");
	//PrintOutMatrixMsg(m,1,Zk, "Zk");
	//PrintOutMatrixMsg(n,m,Hk, "Hk");

	if(INS_FUN_RETURN_SUCESS != MeasResp(Zk,kf->Xk, Hk, n, m, Zk))
	{
		return INS_FUN_RETURN_FAIL;
	}
	inav_log(INAVMD(LOG_DEBUG),"after MeasResp");
	//PrintOutMatrixMsg(m,1,Zk, "Zk");

#if 0
	//Ò»µ©Ê§Ëø£¬²»ĞèÒª×ÔÊÊÓ¦µ÷Õû£¬R¾ØÕóÎª¼«´óÖµ
	if(NAV_GNSS_STATUS_LOST != g_NAV_GNSS_RESULT.gnssstatus)
	{
		kf->beta = RadAptive(kf->Pxk, Hk, Zk, kf->RK, kf->Rmin, kf->Rmax, n, NB, kf->beta, kf->b, kf->RK);
	}
	
#endif

	// ¹¹½¨R¾ØÕó
	MatrixRk(kf->RK,m, Rk);
	PrintOutMatrixMsg(n,m,kf->RK, "GnssInsFusion-Rk");
	// Á¿²â¸üĞÂ
	if(INS_FUN_RETURN_SUCESS!=KfMeasUpdate(kf->Xk,kf->Pxk,Hk,Zk,Rk,n,m,kf->Xk,kf->Pxk))
	{
		return INS_FUN_RETURN_FAIL;
	}
	inav_log(INAVMD(LOG_DEBUG),"after KfMeasUpdate");
	//PrintOutMatrixMsg(NA,1,kf->Xk, "Xk");
	//PrintOutMatrixMsg(NA,NA,kf->Pxk, "Pxk");

	free(Hk); free(Rk); free(Zk);
	return INS_FUN_RETURN_SUCESS;
}

void KfFeedback(KALMAN* kf, INSRESULT* ins)  //¿¨¶ûÂüÂË²¨·´À¡
{
	double phi[NN];
	int i;
	inav_log(INAVMD(LOG_DEBUG),"KfFeedback");
#if 1
	for (i = 0; i < NN; i++)
	{
		phi[i] = ins->nts*kf->Xk[i];			kf->Xk[i] = (1- ins->nts)*kf->Xk[i];
		ins->vn[i] -= kf->Xk[i + NN];			kf->Xk[i + NN] = 0.0;
		ins->vnstd[i]  = sqrt(kf->Pxk[(i + NN)*NA+i + NN])*SAMPLE_FREQ;	
		ins->pos[i] -= kf->Xk[i + NN * 2];		kf->Xk[i + NN * 2] = 0.0;
		ins->posstd[i]  = sqrt(kf->Pxk[(i + NN * 2)*NA+i + NN * 2]);	
		ins->eb[i]+= (ins->nts*kf->Xk[i + NN * 3]);	kf->Xk[i + NN * 3]= (1-ins->nts)*kf->Xk[i + NN * 3];
		ins->db[i] +=(ins->nts*kf->Xk[i + NN * 4]);		kf->Xk[i + NN * 4] = (1 - ins->nts) * kf->Xk[i + NN * 4];
	}
#endif

#if 0
	for (i = 0; i < NN; i++)
	{
		phi[i] = kf->Xk[i];						kf->Xk[i] = 0.0;
		ins->vn[i] -= kf->Xk[i + NN];			kf->Xk[i + NN] = 0.0;
		ins->pos[i] -= kf->Xk[i + NN * 2];		kf->Xk[i + NN * 2] = 0.0;
		ins->eb[i]+= (kf->Xk[i + NN * 3]);		kf->Xk[i + NN * 3]=0.0;
		ins->db[i] +=(kf->Xk[i + NN * 4]);		kf->Xk[i + NN * 4] =  0.0;
	}
#endif
	qdelphi(ins->qnb, phi, ins->qnb);

	//¸üĞÂ×ËÌ¬£¬ËÄÔªËØ×ª×ËÌ¬
	 qnb2att( ins->qnb, ins->att); 
	for (i = 0; i < NN; i++)
	{
		ins->att_pre[i] = ins->att[i];
	}
	
	
	
}

double averageBias(double cnt,double meas,double average)//Æ½¾ùÆ«ÒÆ
{
	return (1.0/cnt)*meas+((cnt-1.0)/cnt)*average;
}

double average(double* sample,int size)
{
	int i;
	double temp=0;
	for (i = 0; i < size; i++)
	{
		temp += sample[i];
	}
	return temp / size;
}
double compute_var(double* sample,int size)  //¼ÆÊı±äÁ¿
{
	int i;
	double ave;
	double* temp= zeros(size, 1);
	if(NULL == temp)
	{
		inav_log(INAVMD(LOG_ERR),"temp fail");
		return 0;
	}
	double var;
	ave = average(sample,size);
	for (i = 0; i < size; i++)
	{
		temp[i] = sample[i] - ave;
	}
	var=(norm(temp, size) * norm(temp, size)) / size;
	free(temp);
	return var;
}
void static_detection(IMUDATA *imu,double *vn,double *acc, STATIC_DETECTION *pStaticDetect) //¾²Ì¬¼ì²â
{
	int i=0,j=1;
	double var[NN];
	if (pStaticDetect->index  == SAMPLE_SIZE)
	{
		//Êı¾İÒÆÎ»
		for (i = 0; i < SAMPLE_SIZE&&j<SAMPLE_SIZE; i++)
		{
			pStaticDetect->gyro[0][i]	=  pStaticDetect->gyro[0][j];
			pStaticDetect->gyro[1][i]	=  pStaticDetect->gyro[1][j];
			pStaticDetect->gyro[2][i]	=  pStaticDetect->gyro[2][j];
			j++;
		}
		pStaticDetect->gyro[0] [SAMPLE_SIZE-1]	= imu->gyro[0];
		pStaticDetect->gyro[1] [SAMPLE_SIZE-1]	= imu->gyro[1];
		pStaticDetect->gyro[2] [SAMPLE_SIZE-1]	= imu->gyro[2];

		//·½²î¼ÆËã
		var[0] = compute_var(pStaticDetect->gyro[0] ,SAMPLE_SIZE);
		var[1] = compute_var(pStaticDetect->gyro[1] ,SAMPLE_SIZE);
		var[2] = compute_var(pStaticDetect->gyro[2] ,SAMPLE_SIZE);

		//ÔË¶¯×´Ì¬Ì½²â
		if ((norm(var, NN)) < VAR_THRESHSHOLD) //(norm(vn,NN))<VN_3D_THRESHSHOLD
		{
			pStaticDetect->state =  MOTION_STATUS_STATIC;
		}
		else
		{
			pStaticDetect->state =  MOTION_STATUS_MOVING;
		}
	}
	else if (pStaticDetect->index < SAMPLE_SIZE)
	{
		pStaticDetect->gyro[0] [pStaticDetect->index] = imu->gyro[0];
		pStaticDetect->gyro[1] [pStaticDetect->index]  = imu->gyro[1];
		pStaticDetect->gyro[2] [pStaticDetect->index]  = imu->gyro[2];
		pStaticDetect->index++;
		//pStaticDetect->state = MOTION_STATUS_UNKNOW;
	}
}

double difftimeInc2gnss(double inssecond, double gnsssecond)
{
	double dt = 0.0;
	dt = inssecond - gnsssecond;
	if(dt > 604800/2)
	{
		dt= dt -604800;
	}
	else if(dt < -604800/2)
	{
		dt = dt+604800;
	}

	return dt;
}

/*************************************************************************************************************/
int StopNavigation()
{
	int ret = 0;
	
	InitialNavIncParm();
	return ret;

}

void InitialNavIncParm()
{
	memset(&g_NAV_INS,0,sizeof(I_NAV_INS));
	g_NAV_INS.imu.gyro_pre[0] 	=	g_NAV_INS.imu.gyro[0];
	g_NAV_INS.imu.gyro_pre[1] 	=	g_NAV_INS.imu.gyro[1];
	g_NAV_INS.imu.gyro_pre[2] 	=	g_NAV_INS.imu.gyro[2];
	g_NAV_INS.imu.accm_pre[0] 	=	g_NAV_INS.imu.accm[0];
	g_NAV_INS.imu.accm_pre[1] 	=	g_NAV_INS.imu.accm[1];
	g_NAV_INS.imu.accm_pre[2] 	=	g_NAV_INS.imu.accm[2];
	g_NAV_INS.insStatus 			=	NAV_INS_STATUS_WAIT;//ÔİÊ±´ÓNAV_INS_STATUS_WAIT¿ªÊ¼
}

void InitialStaticDetectParm(void)
{
	memset(&g_STATIC_DETECTION,0,sizeof(STATIC_DETECTION));
	g_STATIC_DETECTION.state = MOTION_STATUS_UNKNOW;
}
unsigned int GetNavIncData(IMUDATA *p, void *comb)
{
	CombineDataTypeDef *pComb = (CombineDataTypeDef*)comb;
	
	p->gyro_pre[0] 	=	p->gyro[0];
	p->gyro_pre[1] 	=	p->gyro[1];
	p->gyro_pre[2] 	=	p->gyro[2];
	p->accm_pre[0] 	=	p->accm[0];
	p->accm_pre[1] 	=	p->accm[1];
	p->accm_pre[2] 	=	p->accm[2];

	p->gyro[0] = pComb->imuInfo.gyroGrp[0];
	p->gyro[1] = pComb->imuInfo.gyroGrp[1];
	p->gyro[2] = pComb->imuInfo.gyroGrp[2];
	p->accm[0] = pComb->imuInfo.accelGrp[0];
	p->accm[1] = pComb->imuInfo.accelGrp[1];
	p->accm[2] = pComb->imuInfo.accelGrp[2];
	
	g_NAV_INS.insStatus=	NAV_INS_STATUS_START;
	return 0;
}

IMUDATA * GetNavIncImuPointer()
{
	return &g_NAV_INS.imu;
}

I_NAV_INS * GetNavIncPointer()
{
	return &g_NAV_INS;
}

STATIC_DETECTION * GetStaticDetectPointer()
{
	return &g_STATIC_DETECTION;
}

COMPENSATE_PARAMS * GetCompensateParmsPointer()
{
	return &g_Compensate_Params;
}


//¹ßµ¼ĞÅÏ¢´òÓ¡
void PrintOutInsMsg()
{
	inav_log(INAVMD(LOG_DEBUG),"***********************print out ins msg*******************************");
	inav_log(INAVMD(LOG_DEBUG),"imu.second=%.3f,dt=%.4f",g_NAV_INS.imu.second,g_NAV_INS.dt);
	//ÌîĞ´ÎÀµ¼×ËÌ¬¡¢Î»ÖÃ¡¢ËÙ¶È´òÓ¡
	inav_log(INAVMD(LOG_DEBUG),"ins-att=[%.7f,%.7f,%.7f]",g_NAV_INS.ins.att[0]*RAD2DEG,g_NAV_INS.ins.att[1]*RAD2DEG,g_NAV_INS.ins.att[2]);
	inav_log(INAVMD(LOG_DEBUG),"ins-pos=[%.7f,%.7f,%.7f]",g_NAV_INS.ins.pos[0]*RAD2DEG,g_NAV_INS.ins.pos[1]*RAD2DEG,g_NAV_INS.ins.pos[2]);
	inav_log(INAVMD(LOG_DEBUG),"ins-posstd=[%.7f,%.7f,%.7f]",g_NAV_INS.ins.posstd[0]*RAD2DEG,g_NAV_INS.ins.posstd[1]*RAD2DEG,g_NAV_INS.ins.posstd[2]);
	inav_log(INAVMD(LOG_DEBUG),"ins-vel=[%.7f,%.7f,%.7f]",g_NAV_INS.ins.vn[0],g_NAV_INS.ins.vn[1],g_NAV_INS.ins.vn[2]);
	inav_log(INAVMD(LOG_DEBUG),"ins-velstd=[%.7f,%.7f,%.7f]",g_NAV_INS.ins.vnstd[0],g_NAV_INS.ins.vnstd[1],g_NAV_INS.ins.vnstd[2]);
}
//ÎÀµ¼ĞÅÏ¢´òÓ¡
void PrintOutGNSSMsg()
{
	inav_log(INAVMD(LOG_DEBUG),"***********************print out gnss msg*******************************");
	inav_log(INAVMD(LOG_DEBUG),"gnssstatus=%d",g_NAV_GNSS_RESULT.gnssstatus);
	inav_log(INAVMD(LOG_DEBUG),"gpssecond=%.3f",g_NAV_GNSS_RESULT.gpssecond);
	//ÌîĞ´¹ßµ¼º½Ïò½Ç¡¢Î»ÖÃ¡¢ËÙ¶È¡¢¶¨Î»×´Ì¬´òÓ¡
	inav_log(INAVMD(LOG_DEBUG),"gnss-angle=[%.7f,%.7f,%.7f]",g_NAV_GNSS_RESULT.heading,g_NAV_GNSS_RESULT.pitch,g_NAV_GNSS_RESULT.roll);
	inav_log(INAVMD(LOG_DEBUG),"gnss-pos=[%.7f,%.7f,%.7f]",g_NAV_GNSS_RESULT.latitude,g_NAV_GNSS_RESULT.longitude,g_NAV_GNSS_RESULT.altitude);
	inav_log(INAVMD(LOG_DEBUG),"gnss-vel=[%.7f,%.7f,%.7f]",g_NAV_GNSS_RESULT.ve,g_NAV_GNSS_RESULT.vn,g_NAV_GNSS_RESULT.vu);
	if(1 == g_NAV_GNSS_RESULT.supportposvelstd)
	{
		inav_log(INAVMD(LOG_DEBUG),"gnss-posstd=[%.7f,%.7f,%.7f]",g_NAV_GNSS_RESULT.latstd,g_NAV_GNSS_RESULT.logstd,g_NAV_GNSS_RESULT.hstd);
		inav_log(INAVMD(LOG_DEBUG),"gnss-velstd=[%.7f,%.7f,%.7f]",g_NAV_GNSS_RESULT.vestd,g_NAV_GNSS_RESULT.vnstd,g_NAV_GNSS_RESULT.vustd);
	}
}
//¾ØÕóĞÅÏ¢´òÓ¡
void PrintOutMatrixMsg(int n,int m ,double *A, char * name)
{
#if 1
	char tmp[32]={0};
  	memset(g_MatrixBuff,0,sizeof(g_MatrixBuff));
	if(n<=0 || m<=0)
	{
		inav_log(INAVMD(LOG_ERR),"PrintOutMatrixMsg n=%d,m=%d",n,m);
	}

	inav_log(INAVMD(LOG_DEBUG),"%s:[", name);
	for(int i=0;i<n;i++)
	{	
		for(int j=0;j<m;j++)
		{
			memset(tmp,0,sizeof(tmp));
			sprintf(tmp,"%.7E,",A[i+j*n]);
			strcat(g_MatrixBuff,tmp);
		}
		strcat(g_MatrixBuff,"\r\n");
	}
	inav_log(INAVMD(LOG_DEBUG),"%s",g_MatrixBuff);
	inav_log(INAVMD(LOG_DEBUG),"]");
#endif	
}
void InterfaceKalman()
{
	inav_log(INAVMD(LOG_DEBUG),"g_NAV_INS.insStatus = %d",g_NAV_INS.insStatus);
	switch(g_NAV_INS.insStatus)
	{
		case NAV_INS_STATUS_IDLE:   //0¿ÕÏĞ
		{
		      
		}
		break;

		case NAV_INS_STATUS_START:  //1¿ªÊ¼µ¼º½
		{
			g_NAV_INS.insStatus=	NAV_INS_STATUS_WAIT;
		}
		break;

		case NAV_INS_STATUS_WAIT:  //2µÈ´ıÎÈ¶¨×´Ì¬
		{
			if(1 == g_NAV_GNSS_RESULT.gnssstartflag)//gpsÒÑ¾­Æô¶¯¿ÉÒÔ½øĞĞÈÚºÏ¹ßµ¼
			{
				g_NAV_INS.insStatus=	NAV_INS_STATUS_ROUTH_ALIGN;
			}
		}
		break;

		case NAV_INS_STATUS_ROUTH_ALIGN:  //2µÈ´ıÎÈ¶¨×´Ì¬
		{
			//´Ö¶Ô×¼
			StartCoarseAlign(&g_NAV_INS,&g_NAV_GNSS_RESULT);
			//¸ù¾İgpsĞÅÏ¢¼°µ±Ç°¹ßµ¼ĞÅÏ¢³õÊ¼»¯
			InsInit(&g_NAV_INS.ins, TS, g_NAV_INS.ins.qnb, &g_NAV_GNSS_RESULT,g_Compensate_Params.gnssArmLength,&g_NAV_INS);
			//³õÊ¼»¯EKFÖĞP;Q¾ØÕó
			KfInit(&g_NAV_INS.kf);
			g_NAV_INS.insStatus=	NAV_INS_STATUS_KEEP;
		}
		break;

		case NAV_INS_STATUS_KEEP:  //½øÈëµ¼º½×´Ì¬
		{
			//¹ßµ¼Êı¾İ¸üĞÂ
			InsUpdate(&g_NAV_INS.imu,&g_NAV_INS.ins,&g_NAV_INS);
			//»ñÈ¡Ê±¼ä¸üĞÂµÄ×ª»»¾ØÕó
			if(INS_FUN_RETURN_SUCESS == KfFk(&g_NAV_INS.ins,g_NAV_INS.Fk,&g_NAV_INS))
			{

#if 1//¶¯Ì¬¼ì²é,ÊÇ·ñĞèÒª´ı¿¼ÂÇ
				static_detection(&g_NAV_INS.imu,g_NAV_INS.ins.vn,g_NAV_INS.ins.an, &g_STATIC_DETECTION);
				inav_log(INAVMD(LOG_DEBUG),"g_STATIC_DETECTION.state=%d",g_STATIC_DETECTION.state);
				if(MOTION_STATUS_MOVING != g_STATIC_DETECTION.state)//µ±Ç°·Ç¶¯Ì¬
				{
					//GNSS RTK¹Ì¶¨½â
					if(NAV_GNSS_STATUS_RTK_FIX == g_NAV_GNSS_RESULT.gnssstatus)
					{
						g_NAV_INS.ins.att[2] = g_NAV_GNSS_RESULT.heading*DEG2RAD;
						att2qnb(g_NAV_INS.ins.att,g_NAV_INS.ins.qnb);
						
					}
				}
#endif
				//Ê±¼äÒ»²½¸üĞÂ
				if(INS_FUN_RETURN_SUCESS ==KfTimeUpdate(&g_NAV_INS.kf, g_NAV_INS.Fk,TS, 1))
				{
					//¼ÆËã¹ßµ¼ÓëÎÀµ¼Ê±¼ä²îµ¥Î»s
					g_NAV_INS.dt = difftimeInc2gnss((double)g_NAV_INS.imu.second,(double)g_NAV_GNSS_RESULT.gpssecond);
					inav_log(INAVMD(LOG_DEBUG),"g_NAV_INS.dt=%.7f",g_NAV_INS.dt);
					if(g_NAV_GNSS_RESULT.gnssstatus >= NAV_GNSS_STATUS_SPP)
					{
						//dengwei µ¥µã¶¨Î»¡¢rtk¹Ì¶¨½âµÄÁ¿²âÎó²î²»Í¬£¬ĞèÒª¸üĞÂR ¾ØÕó,
						UpdateKfR(&g_NAV_INS.kf,&g_NAV_GNSS_RESULT);
					}
					else//Ê§ËøÇé¿öÏÂÉèÖÃR¾ØÕóÎó²îÎŞÏŞ´ó
					{
						UpdateKfRHuge(&g_NAV_INS.kf);
					}
					GnssInsFusion(&g_NAV_INS.ins, &g_NAV_GNSS_RESULT, &g_NAV_INS.kf, g_Compensate_Params.gnssArmLength, g_NAV_INS.dt);
									
				}
						
			}

			KfFeedback(&g_NAV_INS.kf,&g_NAV_INS.ins);
			//¹ßµ¼ĞÅÏ¢´òÓ¡
			PrintOutInsMsg();
			//ÎÀµ¼ĞÅÏ¢´òÓ¡
			PrintOutGNSSMsg();
		}
		break;

		case NAV_INS_STATUS_STOP:  //Í£Ö¹¹ßµ¼
		{
			StopNavigation();
		}
		break;

		default:
		{
			inav_log(INAVMD(LOG_ERR),"g_NAV_INS.insStatus = %d",g_NAV_INS.insStatus);
			break;	
		}
	}

}
unsigned int GetNavGnssData(I_NAV_GNSS_RESULT *p, void *comb)
{
	CombineDataTypeDef *pComb = (CombineDataTypeDef*)comb;

	p->gpsweek = pComb->gnssInfo.gpsweek;
	p->gpssecond = pComb->gnssInfo.gpssecond;
	p->heading = pComb->gnssInfo.Heading;
	p->pitch = pComb->gnssInfo.Pitch;
	p->roll = pComb->gnssInfo.Roll;
	p->latitude = pComb->gnssInfo.Lat;
	p->longitude = pComb->gnssInfo.Lon;
	p->altitude = pComb->gnssInfo.Altitude;
	p->ve = pComb->gnssInfo.ve;
	p->vn = pComb->gnssInfo.vn;
	p->vu = pComb->gnssInfo.vu;
	p->baseline = pComb->gnssInfo.baseline;
	p->nsv = pComb->gnssInfo.StarNum;
	p->gnssstatus = pComb->gnssInfo.rtkStatus;
	p->utc = pComb->gnssInfo.timestamp;//?
	p->supportposvelstd = 1;
	p->latstd = pComb->gnssInfo.LatStd;
	p->logstd = pComb->gnssInfo.LonStd;
	p->hstd = pComb->gnssInfo.AltitudeStd;
	p->vestd = pComb->gnssInfo.vestd;
	p->vnstd = pComb->gnssInfo.vnstd;
	p->vustd = pComb->gnssInfo.vustd;
	p->gnssstartflag = 1; //è·å–æ•°æ®æˆåŠŸ
	return 0;
}

#if 0
void TestMain()
{
	//double Fk[NA*NA] = {0};
	//double dt=0.0;
	InitialNavIncParm();
	//åˆå§‹åŒ–é™æ€æ£€æŸ¥å‚æ•°
	InitialStaticDetectParm();
	
	while(1)
	{
		//å¡«å†™ç­‰å¾…gpsä¸­æ–­
		//if( Semaphore_pend(fpgaSem, BIOS_WAIT_FOREVER)==FALSE )/* wait for FPGA Interrupt */
		{/* error report */
//				if( LogFileCreated ) fprintf(fpLogFile,"Error:semTake(FpgaSemID,WAIT_FOREVER)==ERROR\n");  //æ·»åŠ ç›¸å…³æ–‡ä»¶åå†å¯ç”¨
		}

		//GetNavIncData(&g_NAV_INS.imu);
		//GetNavGnssData(&g_NAV_GNSS_RESULT);

		switch(g_NAV_INS.insStatus)
		{
			inav_log(INAVMD(LOG_DEBUG),"g_NAV_INS.insStatus = %d",g_NAV_INS.insStatus);
			case NAV_INS_STATUS_IDLE:   //0ç©ºé—²
			{
			      
			}
			break;

			case NAV_INS_STATUS_START:  //1å¼€å§‹å¯¼èˆª
			{
				g_NAV_INS.insStatus=	NAV_INS_STATUS_WAIT;
			}
			break;

			case NAV_INS_STATUS_WAIT:  //2ç­‰å¾…ç¨³å®šçŠ¶æ€
			{
				if(1 == g_NAV_GNSS_RESULT.gnssstartflag)//gpså·²ç»å¯åŠ¨å¯ä»¥è¿›è¡Œèåˆæƒ¯å¯¼
				{
					g_NAV_INS.insStatus=	NAV_INS_STATUS_ROUTH_ALIGN;
				}
			}
			break;

			case NAV_INS_STATUS_ROUTH_ALIGN:  //2ç­‰å¾…ç¨³å®šçŠ¶æ€
			{
				//ç²—å¯¹å‡†
				StartCoarseAlign(&g_NAV_INS,&g_NAV_GNSS_RESULT);
				//æ ¹æ®gpsä¿¡æ¯åŠå½“å‰æƒ¯å¯¼ä¿¡æ¯åˆå§‹åŒ–
				InsInit(&g_NAV_INS.ins, TS, g_NAV_INS.ins.qnb, &g_NAV_GNSS_RESULT,g_Compensate_Params.gnssArmLength,&g_NAV_INS);
				//åˆå§‹åŒ–EKFä¸­P;QçŸ©é˜µ
				KfInit(&g_NAV_INS.kf);
				g_NAV_INS.insStatus=	NAV_INS_STATUS_KEEP;
			}
			break;

			case NAV_INS_STATUS_KEEP:  //è¿›å…¥å¯¼èˆªçŠ¶æ€
			{
				//æƒ¯å¯¼æ•°æ®æ›´æ–°
				InsUpdate(&g_NAV_INS.imu,&g_NAV_INS.ins,&g_NAV_INS);
				//è·å–æ—¶é—´æ›´æ–°çš„è½¬æ¢çŸ©é˜µ
				KfFk(&g_NAV_INS.ins,g_NAV_INS.Fk,&g_NAV_INS);
#if 1//åŠ¨æ€æ£€æŸ¥,æ˜¯å¦éœ€è¦å¾…è€ƒè™‘
				static_detection(&g_NAV_INS.imu,g_NAV_INS.ins.vn,g_NAV_INS.ins.an, &g_STATIC_DETECTION);
				if(MOTION_STATUS_MOVING != g_STATIC_DETECTION.state)//å½“å‰éåŠ¨æ€
				{
					//GNSS RTKå›ºå®šè§£
					if(NAV_GNSS_STATUS_RTK_FIX == g_NAV_GNSS_RESULT.gnssstatus)
					{
						g_NAV_INS.ins.att[2] = g_NAV_GNSS_RESULT.heading*DEG2RAD;
						att2qnb(g_NAV_INS.ins.att,g_NAV_INS.ins.qnb);
					}
				}			
#endif
				//æ—¶é—´ä¸€æ­¥æ›´æ–°
				KfTimeUpdate(&g_NAV_INS.kf, g_NAV_INS.Fk,TS, 1);
				//è®¡ç®—æƒ¯å¯¼ä¸å«å¯¼æ—¶é—´å·®å•ä½s
				g_NAV_INS.dt = difftimeInc2gnss((double)g_NAV_INS.imu.second,(double)g_NAV_GNSS_RESULT.gpssecond);
				GnssInsFusion(&g_NAV_INS.ins, &g_NAV_GNSS_RESULT, &g_NAV_INS.kf, g_Compensate_Params.gnssArmLength, g_NAV_INS.dt);
				KfFeedback(&g_NAV_INS.kf,&g_NAV_INS.ins);
				//æƒ¯å¯¼ä¿¡æ¯æ‰“å°
				PrintOutInsMsg();
				//å«å¯¼ä¿¡æ¯æ‰“å°
				PrintOutGNSSMsg();
			}
			break;

			case NAV_INS_STATUS_STOP:  //åœæ­¢æƒ¯å¯¼
			{
				StopNavigation();
			}
			break;

			default:
			{
				inav_log(INAVMD(LOG_ERR),"g_NAV_INS.insStatus = %d",g_NAV_INS.insStatus);
				break;	
			}
		}

		if(NAV_INS_STATUS_STOP == g_NAV_INS.insStatus)
		{
			inav_log(INAVMD(LOG_DEBUG),"NAV_INS_STATUS_STOP");
			break;
		}
	}

	
	
}
#endif


#ifdef __cplusplus
}
#endif

