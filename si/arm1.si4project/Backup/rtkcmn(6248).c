/*------------------------------------------------------------------------------
* rtkcmn.c : rtklib common functions
*-----------------------------------------------------------------------------*/
#define _POSIX_C_SOURCE 199309

#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */
#define POLYCRC24Q  0x1864CFBu  /* CRC24Q polynomial */

#pragma DATA_SECTION(gpst0,"sect_EDATA_III");
double gpst0[6]={1980,1, 6,0,0,0}; /* gps time reference */

#pragma DATA_SECTION(gst0,"sect_EDATA_III");
double gst0[6]={1999,8,22,0,0,0}; /* galileo system time reference */

#pragma DATA_SECTION(bdt0,"sect_EDATA_III");
double bdt0[6]={2006,1, 1,0,0,0}; /* beidou time reference */

#pragma DATA_SECTION(leaps,"sect_EDATA_III");
double leaps[17][7]={ /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
	{2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1}
};

#pragma DATA_SECTION(lam_carr,"sect_EDATA_III");
double lam_carr[6]={       /* carrier wave length (m) */
    CLIGHT/FREQ1,CLIGHT/FREQ2,CLIGHT/FREQ5,CLIGHT/FREQ6,CLIGHT/FREQ7,CLIGHT/FREQ8
};

//----------------------------------------------------------------------------------------------------------------------------------------------

#pragma CODE_SECTION(satno,"sect_ECODE_I"); 
int satno(int sys, int prn);

#pragma CODE_SECTION(satsys,"sect_ECODE_I"); 
int satsys(int sat, int *prn);

#pragma CODE_SECTION(satexclude,"sect_ECODE_I");
int satexclude(int sat, int svh, const prcopt_t *opt);

#pragma CODE_SECTION(testsnr,"sect_ECODE_I");
int testsnr(int base, int freq, double el, double snr, const snrmask_t *mask);

#pragma CODE_SECTION(EyeMatrix,"sect_ECODE_I");
void EyeMatrix(double *pMat, int n, double dValue);

#pragma CODE_SECTION(InnerDot,"sect_ECODE_I");
double InnerDot(const double *a, const double *b, int n);

#pragma CODE_SECTION(norm,"sect_ECODE_I");
double norm(const double *a, int n);

#pragma CODE_SECTION(cross3,"sect_ECODE_I");
void cross3(const double *a, const double *b, double *c);

#pragma CODE_SECTION(normv3,"sect_ECODE_I");
int normv3(const double *a, double *b);

#pragma CODE_SECTION(matcpy,"sect_ECODE_I");
void matcpy(double *A, const double *B, int n, int m);

#pragma CODE_SECTION(matmul,"sect_ECODE_I");
void matmul(const char *tr, int n, int k, int m, double alpha, const double *A, const double *B, double beta, double *C);

#pragma CODE_SECTION(ludcmp,"sect_ECODE_I");
int ludcmp(double *A, int n, int *indx, double *d);

#pragma CODE_SECTION(lubksb,"sect_ECODE_I");
void lubksb(const double *A, int n, const int *indx, double *b);

#pragma CODE_SECTION(matinv,"sect_ECODE_I");
int matinv(double *A, int n);

#pragma CODE_SECTION(solve,"sect_ECODE_I");
int solve(const char *tr, const double *A, const double *Y, int n, int m, double *X);

#pragma CODE_SECTION(FilterKernel,"sect_ECODE_I");
int FilterKernel(const double *x, const double *P, const double *H, const double *v, const double *R, int n, int m, double *xp, double *Pp);

#pragma CODE_SECTION(KalmanFilter,"sect_ECODE_I");
int KalmanFilter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m);

#pragma CODE_SECTION(str2num,"sect_ECODE_I");
double str2num(const char *s, int i, int n);

#pragma CODE_SECTION(str2time,"sect_ECODE_I");
int str2time(const char *s, int i, int n, gtime_t *t);

#pragma CODE_SECTION(epoch2time,"sect_ECODE_I");
gtime_t epoch2time(const double *ep);

#pragma CODE_SECTION(time2epoch,"sect_ECODE_I");
void time2epoch(gtime_t t, double *ep);

#pragma CODE_SECTION(gpst2time,"sect_ECODE_I");
gtime_t gpst2time(int week, double sec);

#pragma CODE_SECTION(time2gpst,"sect_ECODE_I");
double time2gpst(gtime_t t, int *week);

#pragma CODE_SECTION(gst2time,"sect_ECODE_I");
gtime_t gst2time(int week, double sec);

#pragma CODE_SECTION(time2gst,"sect_ECODE_I");
double time2gst(gtime_t t, int *week);

#pragma CODE_SECTION(bdt2time,"sect_ECODE_I");
gtime_t bdt2time(int week, double sec);

#pragma CODE_SECTION(time2bdt,"sect_ECODE_I");
double time2bdt(gtime_t t, int *week);

#pragma CODE_SECTION(timeadd,"sect_ECODE_I");
gtime_t timeadd(gtime_t t, double sec);

#pragma CODE_SECTION(timediff,"sect_ECODE_I");
double timediff(gtime_t t1, gtime_t t2);

#pragma CODE_SECTION(gpst2utc,"sect_ECODE_I");
gtime_t gpst2utc(gtime_t t);

#pragma CODE_SECTION(utc2gpst,"sect_ECODE_I");
gtime_t utc2gpst(gtime_t t);

#pragma CODE_SECTION(gpst2bdt,"sect_ECODE_I");
gtime_t gpst2bdt(gtime_t t);

#pragma CODE_SECTION(bdt2gpst,"sect_ECODE_I");
gtime_t bdt2gpst(gtime_t t);

#pragma CODE_SECTION(time2sec,"sect_ECODE_I");
double time2sec(gtime_t time, gtime_t *day);

#pragma CODE_SECTION(utc2gmst,"sect_ECODE_I");
double utc2gmst(gtime_t t, double ut1_utc);

#pragma CODE_SECTION(Bdt2UtcTime,"sect_ECODE_I");
void Bdt2UtcTime(int iBdsWeek, double dBdsTow, double *pUtcTimeArr);

#pragma CODE_SECTION(time2str,"sect_ECODE_I");
void time2str(gtime_t t, char *s, int n);

#pragma CODE_SECTION(time2doy,"sect_ECODE_I");
double time2doy(gtime_t t);

#pragma CODE_SECTION(deg2dms,"sect_ECODE_I");
void deg2dms(double deg, double *dms);

#pragma CODE_SECTION(dms2deg,"sect_ECODE_I");
double dms2deg(const double *dms);

#pragma CODE_SECTION(ecef2pos,"sect_ECODE_I");
void ecef2pos(const double *r, double *pos);

#pragma CODE_SECTION(pos2ecef,"sect_ECODE_I");
void pos2ecef(const double *pos, double *r);

#pragma CODE_SECTION(xyz2enu,"sect_ECODE_I");
void xyz2enu(const double *pos, double *E);

#pragma CODE_SECTION(ecef2enu,"sect_ECODE_I");
void ecef2enu(const double *pos, const double *r, double *e);

#pragma CODE_SECTION(enu2ecef,"sect_ECODE_I");
void enu2ecef(const double *pos, const double *e, double *r);

#pragma CODE_SECTION(covenu,"sect_ECODE_I");
void covenu(const double *pos, const double *P, double *Q);

#pragma CODE_SECTION(covecef,"sect_ECODE_I");
void covecef(const double *pos, const double *Q, double *P);

#pragma CODE_SECTION(satwavelen,"sect_ECODE_I");
double satwavelen(int sat, int frq, const nav_t *nav);

#pragma CODE_SECTION(geodist,"sect_ECODE_I");
double geodist(const double *rs, const double *rr, double *e);

#pragma CODE_SECTION(satazel,"sect_ECODE_I");
double satazel(const double *pos, const double *e, double *azel);

#pragma CODE_SECTION(dops,"sect_ECODE_I");
void dops(int ns, const double *azel, double elmin, double *dop);

#pragma CODE_SECTION(ionmodel,"sect_ECODE_I");
double ionmodel(gtime_t t, const double *ion, const double *pos, const double *azel);

#pragma CODE_SECTION(ionmapf,"sect_ECODE_I");
double ionmapf(const double *pos, const double *azel);

#pragma CODE_SECTION(tropmodel,"sect_ECODE_I");
double tropmodel(gtime_t time, const double *pos, const double *azel, double humi);

#pragma CODE_SECTION(interpc,"sect_ECODE_I");
double interpc(const double coef[], double lat);

#pragma CODE_SECTION(mapf,"sect_ECODE_I");
double mapf(double el, double a, double b, double c);

#pragma CODE_SECTION(nmf,"sect_ECODE_I");
double nmf(gtime_t time, const double pos[], const double azel[], double *mapfw);

#pragma CODE_SECTION(tropmapf,"sect_ECODE_I");
double tropmapf(gtime_t time, const double pos[], const double azel[], double *mapfw);

#pragma CODE_SECTION(interpvar,"sect_ECODE_I");
double interpvar(double ang, const double *var);

#pragma CODE_SECTION(antmodel,"sect_ECODE_I");
void antmodel(const pcv_t *pcv, const double *del, const double *azel, int opt, double *dant);

#pragma CODE_SECTION(csmooth,"sect_ECODE_III");
void csmooth(obs_t *obs, int ns); 


/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
//GPS:1~32, GLO:33~56, BD2:57~91
*-----------------------------------------------------------------------------*/
int satno(int sys, int prn)
{
    if (prn<=0) 
		return 0;

    switch (sys) 
	{
        case SYS_GPS:								
		{
			if (prn<MINPRNGPS||MAXPRNGPS<prn) 
				return 0;

			return prn-MINPRNGPS+1;
		}
        case SYS_GLO:								
		{
			if (prn<MINPRNGLO||MAXPRNGLO<prn) 
				return 0;

			return NSATGPS+prn-MINPRNGLO+1;
		}
        case SYS_GAL:
		{
			if (prn<MINPRNGAL||MAXPRNGAL<prn) 
				return 0;

			return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
		}
        case SYS_QZS:
		{
			if (prn<MINPRNQZS||MAXPRNQZS<prn) 
				return 0;

			return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
		}
        case SYS_CMP:
		{
			if (prn<MINPRNCMP||MAXPRNCMP<prn)		
				return 0;

			return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
		}
        case SYS_LEO:
		{
			if (prn<MINPRNLEO||MAXPRNLEO<prn) 
				return 0;

			return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+prn-MINPRNLEO+1;
		}
        case SYS_SBS:
		{
			if (prn<MINPRNSBS||MAXPRNSBS<prn) 
				return 0;

			return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATLEO+prn-MINPRNSBS+1;
		}
    }

    return 0;
}

/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;

    if (sat<=0||MAXSAT<sat) 
	{
		sat=0;
	}
    else if (sat<=NSATGPS) 
	{
        sys=SYS_GPS; 
		sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) 
	{
        sys=SYS_GLO; 
		sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) 
	{
        sys=SYS_GAL;
		sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) 
	{
        sys=SYS_QZS; 
		sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATCMP) 
	{
        sys=SYS_CMP; 
		sat+=MINPRNCMP-1; 
    }
    else if ((sat-=NSATCMP)<=NSATLEO) 
	{
        sys=SYS_LEO; 
		sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) 
	{
        sys=SYS_SBS; 
		sat+=MINPRNSBS-1; 
    }
    else 
	{
		sat=0;
	}

    if (prn) 
	{
		*prn=sat;
	}

    return sys;
}

/* test excluded satellite -----------------------------------------------------
* test excluded satellite
* args   : int    sat       I   satellite number
*          int    svh       I   sv health flag
*          prcopt_t *opt    I   processing options (NULL: not used)
* return : status (1:excluded,0:not excluded)
*-----------------------------------------------------------------------------*/
int satexclude(int sat, int svh, const prcopt_t *opt)
{
    int sys=satsys(sat,NULL);
		
    if (svh<0)							/* ephemeris unavailable */
	{
		return 1; 
	}

    if (opt)
	{
        if (opt->exsats[sat-1]==1)		/* excluded satellite */
		{
			return 1; 
		}

        if (opt->exsats[sat-1]==2)		/* included satellite */
		{
			return 0; 
		}

        if (!(sys&opt->navsys))			/* unselected sat sys */
		{
			return 1; 
		}
    }

    if (sys==SYS_QZS)					/* mask QZSS LEX health */
	{
		svh&=0xFE; 
	}

    if (svh) 
	{
        //trace(3,"unhealthy satellite: sat=%3d svh=%02X\n",sat,svh);
        return 1;
    }

    return 0;
}

/* test SNR mask ---------------------------------------------------------------
* test SNR mask
* args   : int    base      I   rover or base-station (0:rover,1:base station)
*          int    freq      I   frequency (0:L1,1:L2,2:L3,...)
*          double el        I   elevation angle (rad)
*          double snr       I   C/N0 (dBHz)
*          snrmask_t *mask  I   SNR mask
* return : status (1:masked,0:unmasked)
*-----------------------------------------------------------------------------*/
int testsnr(int base, int freq, double el, double snr, const snrmask_t *mask)
{
    double minsnr=0, a=0;
    int i=0;
    
    if (!mask->ena[base] || freq<0 || freq>=NFREQ) 
	{
		return 0;
	}
    
    a=(el*RAD2DEG+5.0)/10.0;

    i=(int)floor(a); 
	a-=i;

    if(i<1) 
	{
		minsnr = mask->mask[freq][0];
	}
    else if (i>8) 
	{
		minsnr = mask->mask[freq][8];
	}
    else 
	{
		minsnr = (1.0-a)*mask->mask[freq][i-1] + a*mask->mask[freq][i];
	}
    
    return snr<minsnr;
}

void EyeMatrix(double *pMat, int n, double dValue)
{
	int i=0, j=0;

	for (i=0; i<n; i++) 
	{
		for (j=0; j<n; j++)
		{
			if (i==j)
			{
				pMat[i*n+j]=dValue;
			}
			else
			{
				pMat[i*n+j]=0.0;
			}
		}
	}
}

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
double InnerDot(const double *a, const double *b, int n)
{
    double c=0.0;
    
    while (--n>=0) 
	{
		c+=a[n]*b[n];
	}

    return c;
}

/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
double norm(const double *a, int n)
{
    return sqrt(InnerDot(a, a, n));
}

/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void cross3(const double *a, const double *b, double *c)
{
    c[0]=a[1]*b[2]-a[2]*b[1];
    c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}

/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int normv3(const double *a, double *b)
{
    double r;
    if ((r=norm(a,3))<=0.0) return 0;
    b[0]=a[0]/r;
    b[1]=a[1]/r;
    b[2]=a[2]/r;
    return 1;
}

/* copy matrix -----------------------------------------------------------------
* copy matrix
* args   : double *A        O   destination matrix A (n x m)
*          double *B        I   source matrix B (n x m)
*          int    n,m       I   number of rows and columns of matrix
* return : none
*-----------------------------------------------------------------------------*/
void matcpy(double *A, const double *B, int n, int m)
{
    memcpy(A,B,sizeof(double)*n*m);
}
/* matrix routines -----------------------------------------------------------*/

/* multiply matrix (wrapper of blas dgemm) -------------------------------------
* multiply matrix by matrix (C=alpha*A*B+beta*C)
* args   : char   *tr       I  transpose flags ("N":normal,"T":transpose)
*          int    n,k,m     I  size of (transposed) matrix A,B
*          double alpha     I  alpha
*          double *A,*B     I  (transposed) matrix A (n x m), B (m x k)
*          double beta      I  beta
*          double *C        IO matrix C (n x k)
* return : none
*-----------------------------------------------------------------------------*/
void matmul(const char *tr, int n, int k, int m, double alpha, const double *A, const double *B, double beta, double *C)
{
    double d;
    int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);
    
	for (i=0;i<n;i++)
	{ 
		for (j=0;j<k;j++) 
		{
			d=0.0;

			switch (f) 
			{
				case 1: 
					{
						for (x=0;x<m;x++) 
						{
							d+=A[i+x*n]*B[x+j*m]; 
						}

						break;
					}
				case 2:
					{
						for (x=0;x<m;x++) 
						{
							d+=A[i+x*n]*B[j+x*k]; 
						}
						
						break;
					}
				case 3: 
					{
						for (x=0;x<m;x++) 
						{
							d+=A[x+i*m]*B[x+j*m]; 
						}
						
						break;
					}
				case 4: 
					{
						for (x=0;x<m;x++) 
						{
							d+=A[x+i*m]*B[j+x*k]; 
						}
						
						break;
					}
			}

			if (beta==0.0) 
			{
				C[i+j*n]=alpha*d; 
			}
			else 
			{
				C[i+j*n]=alpha*d+beta*C[i+j*n];
			}
		}
	}
}

/* LU decomposition ----------------------------------------------------------*/
#pragma DATA_SECTION(glLudMat_vv,"sect_EDATA_III");
double glLudMat_vv[(MAXOBS*(NFREQ+NEXOBS)*2+2)*1]; 

int ludcmp(double *A, int n, int *indx, double *d)
{
    double big,s,tmp;
    int i,imax=0,j,k;

	memset(glLudMat_vv, 0, sizeof(glLudMat_vv));
    
    *d=1.0;

    for (i=0;i<n;i++) 
	{
        big=0.0; 
		
		for (j=0;j<n;j++) 
		{
			if ((tmp=fabs(A[i+j*n]))>big) 
			{
				big=tmp;
			}
		}

        if (big>0.0) 
		{
			glLudMat_vv[i]=1.0/big; 
		}
		else 
		{
			memset(glLudMat_vv, 0, sizeof(glLudMat_vv));
			return -1;
		}
    }

    for (j=0;j<n;j++) 
	{
        for (i=0;i<j;i++) 
		{
            s=A[i+j*n]; 
			
			for (k=0;k<i;k++) 
			{
				s-=A[i+k*n]*A[k+j*n];
			}
			
			A[i+j*n]=s;
        }

        big=0.0;

        for (i=j;i<n;i++) 
		{
            s=A[i+j*n];
			
			for (k=0;k<j;k++) 
			{
				s-=A[i+k*n]*A[k+j*n];
			}
			
			A[i+j*n]=s;

            if ((tmp=glLudMat_vv[i]*fabs(s))>=big) 
			{
				big=tmp; 
				imax=i;
			}
        }

        if (j!=imax) 
		{
            for (k=0;k<n;k++) 
			{
                tmp=A[imax+k*n]; 
				A[imax+k*n]=A[j+k*n]; 
				A[j+k*n]=tmp;
            }

            *d=-(*d); 
			glLudMat_vv[imax]=glLudMat_vv[j];
        }

        indx[j]=imax;

        if (A[j+j*n]==0.0) 
		{
			memset(glLudMat_vv, 0, sizeof(glLudMat_vv));
			return -1;
		}

        if (j!=n-1)
		{
            tmp=1.0/A[j+j*n];
			
			for (i=j+1;i<n;i++)
			{
				A[i+j*n]*=tmp;
			}
        }
    }

    return 0;
}

/* LU back-substitution ------------------------------------------------------*/
void lubksb(const double *A, int n, const int *indx, double *b)
{
    double s;
    int i,ii=-1,ip,j;
    
    for (i=0;i<n;i++) 
	{
        ip=indx[i]; 
		s=b[ip];
		b[ip]=b[i];

        if (ii>=0) 
		{
			for (j=ii;j<i;j++) 
			{
				s-=A[i+j*n]*b[j]; 
			}
		}
		else if (s) 
		{
			ii=i;
		}

        b[i]=s;
    }

    for (i=n-1;i>=0;i--) 
	{
        s=b[i]; 
		
		for (j=i+1;j<n;j++)
		{
			s-=A[i+j*n]*b[j]; 
		}
		
		b[i]=s/A[i+i*n];
    }
}

/* inverse of matrix ---------------------------------------------------------*/
#pragma DATA_SECTION(m_InvMat_B,"sect_EDATA_III");
double	m_InvMat_B[(MAXOBS*(NFREQ+NEXOBS)*2+2)*(MAXOBS*(NFREQ+NEXOBS)*2+2)]={0};		

#pragma DATA_SECTION(m_InvMat_indx,"sect_EDATA_III");
int	m_InvMat_indx[(MAXOBS*(NFREQ+NEXOBS)*2+2)*1]={0};									 

int matinv(double *A, int n)
{
    double d=0.0;
    int i=0, j=0;
    
	memset(m_InvMat_indx, 0, sizeof(m_InvMat_indx));

	memset(m_InvMat_B, 0, sizeof(m_InvMat_B));
	matcpy(m_InvMat_B,A,n,n);

    if (ludcmp(m_InvMat_B,n,m_InvMat_indx,&d)) 
	{ 
		memset(m_InvMat_indx, 0, sizeof(m_InvMat_indx));
		memset(m_InvMat_B, 0, sizeof(m_InvMat_B));
		return -1;
	}

    for (j=0;j<n;j++)
	{
        for (i=0;i<n;i++) 
		{
			A[i+j*n]=0.0; 
		}

		A[j+j*n]=1.0;

        lubksb(m_InvMat_B,n,m_InvMat_indx,A+j*n);
    }

    return 0;
}

/* solve linear equation -----------------------------------------------------*/
#pragma DATA_SECTION(glSolveMat_B,"sect_EDATA_III");
double glSolveMat_B[MAX_NUM_NB*MAX_NUM_NB]={0};				

int solve(const char *tr, const double *A, const double *Y, int n, int m, double *X)
{
    int info;

	memset(glSolveMat_B, 0, sizeof(glSolveMat_B));
    matcpy(glSolveMat_B, A, n, n);

    if (!(info=matinv(glSolveMat_B,n))) 
	{
		matmul(tr[0]=='N'?"NN":"TN",n,m,n,1.0,glSolveMat_B,Y,0.0,X);
	}

    return info;
}

/* end of matrix routines ----------------------------------------------------*/

/* kalman filter ---------------------------------------------------------------
* kalman filter state update as follows:
*
*   K=P*H*(H'*P*H+R)^-1, xp=x+K*v, Pp=(I-K*H')*P
*
* args   : double *x        I   states vector (n x 1)
*          double *P        I   covariance matrix of states (n x n)
*          double *H        I   transpose of design matrix (n x m)
*          double *v        I   innovation (measurement - model) (m x 1)
*          double *R        I   covariance matrix of measurement error (m x m)
*          int    n,m       I   number of states and measurements
*          double *xp       O   states vector after update (n x 1)
*          double *Pp       O   covariance matrix of states after update (n x n)
* return : status (0:ok,<0:error)
* notes  : matirix stored by column-major order (fortran convention)
*          if state x[i]==0.0, not updates state x[i]/P[i+i*n]
*-----------------------------------------------------------------------------*/
#pragma DATA_SECTION(m_KalmanMat_F,"sect_EDATA_III");
double m_KalmanMat_F[MAX_NUM_NY*MAX_NUM_K]={0};			

#pragma DATA_SECTION(m_KalmanMat_Q,"sect_EDATA_III");
double m_KalmanMat_Q[MAX_NUM_NY*MAX_NUM_NY]={0};		

#pragma DATA_SECTION(m_KalmanMat_K,"sect_EDATA_III");
double m_KalmanMat_K[MAX_NUM_NY*MAX_NUM_K]={0};			

#pragma DATA_SECTION(m_KalmanMat_I,"sect_EDATA_III");
double m_KalmanMat_I[MAX_NUM_K*MAX_NUM_K]={0};			

int FilterKernel(const double *x, const double *P, const double *H,
                 const double *v, const double *R, int n, int m,
                 double *xp, double *Pp)
{
    int info=-1;

	memset(m_KalmanMat_F, 0, sizeof(m_KalmanMat_F));
	memset(m_KalmanMat_Q, 0, sizeof(m_KalmanMat_Q));
	memset(m_KalmanMat_K, 0, sizeof(m_KalmanMat_K));
	memset(m_KalmanMat_I, 0, sizeof(m_KalmanMat_I));

	EyeMatrix(m_KalmanMat_I, n, 1.0);
    
    matcpy(m_KalmanMat_Q, R, m, m);
    matcpy(xp, x, n, 1);
    matmul("NN", n, m, n, 1.0, P, H, 0.0, m_KalmanMat_F);								/* Q=H'*P*H+R */
    matmul("TN", m, m, n, 1.0, H, m_KalmanMat_F, 1.0, m_KalmanMat_Q);

    if (!(info = matinv(m_KalmanMat_Q, m))) 
	{
        matmul("NN", n, m, m, 1.0, m_KalmanMat_F, m_KalmanMat_Q, 0.0, m_KalmanMat_K);   /* K=P*H*Q^-1 */
        matmul("NN", n, 1, m, 1.0, m_KalmanMat_K, v, 1.0, xp);							/* xp=x+K*v */
        matmul("NT", n, n, m, -1.0, m_KalmanMat_K, H, 1.0, m_KalmanMat_I);				/* Pp=(I-K*H')*P */
        matmul("NN", n, n, n, 1.0, m_KalmanMat_I, P, 0.0, Pp);
    }

    return info;
}

#pragma DATA_SECTION(m_KalmanMat_X,"sect_EDATA_III");
double m_KalmanMat_X[MAX_NUM_K*1]={0};								

#pragma DATA_SECTION(m_KalmanMat_Xp,"sect_EDATA_III");
double m_KalmanMat_Xp[MAX_NUM_K*1]={0};								

#pragma DATA_SECTION(m_KalmanMat_P,"sect_EDATA_III");
double m_KalmanMat_P[MAX_NUM_K*MAX_NUM_K]={0};						

#pragma DATA_SECTION(m_KalmanMat_Pp,"sect_EDATA_III");
double m_KalmanMat_Pp[MAX_NUM_K*MAX_NUM_K]={0};						

#pragma DATA_SECTION(m_KalmanMat_H,"sect_EDATA_III");
double m_KalmanMat_H[MAX_NUM_NY*MAX_NUM_K]={0};						

#pragma DATA_SECTION(m_KalmanMat_ix,"sect_EDATA_III");
int	m_KalmanMat_ix[MAX_NUM_NX*1]={0};								 

int KalmanFilter(double *x, double *P, const double *H, const double *v, const double *R, int n, int m)
{
    int i=0, j=0, k=0, info=0;
    
	memset(m_KalmanMat_ix, 0, sizeof(m_KalmanMat_ix));

	for (i=k=0; i<n; i++) 
	{
		if (x[i]!=0.0 && P[i+i*n]>0.0) 
		{
			m_KalmanMat_ix[k++] = i;
		}
	}

	memset(m_KalmanMat_X, 0, sizeof(m_KalmanMat_X));
	memset(m_KalmanMat_Xp, 0, sizeof(m_KalmanMat_Xp));
	memset(m_KalmanMat_P, 0, sizeof(m_KalmanMat_P));
	memset(m_KalmanMat_Pp, 0, sizeof(m_KalmanMat_Pp));
	memset(m_KalmanMat_H, 0, sizeof(m_KalmanMat_H));

    for (i=0; i<k; i++) 
	{
        m_KalmanMat_X[i] = x[m_KalmanMat_ix[i]];								

        for (j=0; j<k; j++) 
		{
			m_KalmanMat_P[i+j*k] = P[m_KalmanMat_ix[i]+m_KalmanMat_ix[j]*n];	
		}

        for (j=0; j<m; j++) 
		{
			m_KalmanMat_H[i+j*k] = H[m_KalmanMat_ix[i]+j*n];					
		}
    }

    info=FilterKernel(m_KalmanMat_X, m_KalmanMat_P, m_KalmanMat_H, v, R, k, m, m_KalmanMat_Xp, m_KalmanMat_Pp);

    for (i=0; i<k; i++) 
	{
        x[m_KalmanMat_ix[i]] = m_KalmanMat_Xp[i];

        for (j=0; j<k; j++) 
		{
			P[m_KalmanMat_ix[i]+m_KalmanMat_ix[j]*n] = m_KalmanMat_Pp[i+j*k];
		}
    }

    return info;
}

/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
double str2num(const char *s, int i, int n)
{
    double value;
    char str[256],*p=str;
    
    if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<n) 
	{
		return 0.0;
	}

    for (s+=i;*s&&--n>=0;s++) 
	{
		*p++=*s=='d'||*s=='D'?'E':*s; 
	}

	*p='\0';

    return sscanf(str,"%lf",&value)==1?value:0.0;
}

/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
int str2time(const char *s, int i, int n, gtime_t *t)
{
    double ep[6];
    char str[256],*p=str;
    
    if (i<0||(int)strlen(s)<i||(int)sizeof(str)-1<i) return -1;
    for (s+=i;*s&&--n>=0;) *p++=*s++; *p='\0';
    if (sscanf(str,"%lf %lf %lf %lf %lf %lf",ep,ep+1,ep+2,ep+3,ep+4,ep+5)<6)
        return -1;
    if (ep[0]<100.0) ep[0]+=ep[0]<80.0?2000.0:1900.0;
    *t=epoch2time(ep);
    return 0;
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep)
{
    const int DayOfYear[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+DayOfYear[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
#pragma DATA_SECTION(mday,"sect_EDATA_III");
int mday[48]={ /* # of days in a month */
    31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
    31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
};

void time2epoch(gtime_t t, double *ep)
{
    int days,sec,mon,day;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);

    for (day=days%1461,mon=0;mon<48;mon++) 
    {
        if (day>=mday[mon]) 
        	day-=mday[mon]; 
        else 
        	break;
    }

    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double time2gpst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

/* galileo system time to time -------------------------------------------------
* convert week and tow in galileo system time (gst) to gtime_t struct
* args   : int    week      I   week number in gst
*          double sec       I   time of week in gst (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t gst2time(int week, double sec)
{
    gtime_t t=epoch2time(gst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* time to galileo system time -------------------------------------------------
* convert gtime_t struct to week and tow in galileo system time (gst)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gst (NULL: no output)
* return : time of week in gst (s)
*-----------------------------------------------------------------------------*/
double time2gst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
gtime_t bdt2time(int week, double sec)
{
    gtime_t t=epoch2time(bdt0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
double time2bdt(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(bdt0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
gtime_t timeadd(gtime_t t, double sec)
{
    double tt;
    
    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}

/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
double timediff(gtime_t t1, gtime_t t2)
{
    //return difftime(t1.time,t2.time)+t1.sec-t2.sec;
	return (t1.time-t2.time)+(t1.sec-t2.sec);
}

/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2utc(gtime_t t)
{
    gtime_t tu;
    int i;
    
    for (i=0;i<(int)sizeof(leaps)/(int)sizeof(*leaps);i++) 
    {
        tu=timeadd(t,leaps[i][6]);
        if (timediff(tu,epoch2time(leaps[i]))>=0.0) return tu;
    }

    return t;
}

/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t utc2gpst(gtime_t t)
{
    int i;
    
    for (i=0;i<(int)sizeof(leaps)/(int)sizeof(*leaps);i++) {
        if (timediff(t,epoch2time(leaps[i]))>=0.0) return timeadd(t,-leaps[i][6]);
    }
    return t;
}

/* gpstime to bdt --------------------------------------------------------------
* convert gpstime to bdt (beidou navigation satellite system time)
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in bdt
* notes  : ref [8] 3.3, 2006/1/1 00:00 BDT = 2006/1/1 00:00 UTC
*          no leap seconds in BDT
*          ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
gtime_t gpst2bdt(gtime_t t)
{
    return timeadd(t,-14.0);
}

/* bdt to gpstime --------------------------------------------------------------
* convert bdt (beidou navigation satellite system time) to gpstime
* args   : gtime_t t        I   time expressed in bdt
* return : time expressed in gpstime
* notes  : see gpst2bdt()
*-----------------------------------------------------------------------------*/
gtime_t bdt2gpst(gtime_t t)
{
    return timeadd(t,14.0);
}

/* time to day and sec -------------------------------------------------------*/
double time2sec(gtime_t time, gtime_t *day)
{
    double ep[6],sec;
    time2epoch(time,ep);
    sec=ep[3]*3600.0+ep[4]*60.0+ep[5];
    ep[3]=ep[4]=ep[5]=0.0;
    *day=epoch2time(ep);
    return sec;
}

/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
double utc2gmst(gtime_t t, double ut1_utc)
{
    const double ep2000[]={2000,1,1,12,0,0};
    gtime_t tut,tut0;
    double ut,t1,t2,t3,gmst0,gmst;
    
    tut=timeadd(t,ut1_utc);
    ut=time2sec(tut,&tut0);
    t1=timediff(tut0,epoch2time(ep2000))/86400.0/36525.0;
    t2=t1*t1; t3=t2*t1;
    gmst0=24110.54841+8640184.812866*t1+0.093104*t2-6.2E-6*t3;
    gmst=gmst0+1.002737909350795*ut;
    
    return fmod(gmst,86400.0)*PI/43200.0; /* 0 <= gmst <= 2*PI */
}

/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
void time2str(gtime_t t, char *s, int n)
{
    double ep[6];
    
    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    time2epoch(t,ep);
    sprintf(s,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
            ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}

/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
double time2doy(gtime_t t)
{
    double ep[6];
    
    time2epoch(t,ep);
    ep[1]=ep[2]=1.0; ep[3]=ep[4]=ep[5]=0.0;
    return timediff(t,epoch2time(ep))/86400.0+1.0;
}

void Bdt2UtcTime(int iBdsWeek, double dBdsTow, double *pUtcTimeArr)
{
	gtime_t GpsTime, UtcTime;

	GpsTime = gpst2time(iBdsWeek+1024+332, dBdsTow);
	GpsTime = bdt2gpst(GpsTime);

	UtcTime=gpst2utc(GpsTime);
	
	time2epoch(UtcTime, pUtcTimeArr);
} 

/* convert degree to deg-min-sec -----------------------------------------------
* convert degree to degree-minute-second
* args   : double deg       I   degree
*          double *dms      O   degree-minute-second {deg,min,sec}
* return : none
*-----------------------------------------------------------------------------*/
void deg2dms(double deg, double *dms)
{
    double sign=deg<0.0?-1.0:1.0,a=fabs(deg);
    dms[0]=floor(a); a=(a-dms[0])*60.0;
    dms[1]=floor(a); a=(a-dms[1])*60.0;
    dms[2]=a; dms[0]*=sign;
}

/* convert deg-min-sec to degree -----------------------------------------------
* convert degree-minute-second to degree
* args   : double *dms      I   degree-minute-second {deg,min,sec}
* return : degree
*-----------------------------------------------------------------------------*/
double dms2deg(const double *dms)
{
    double sign=dms[0]<0.0?-1.0:1.0;
    return sign*(fabs(dms[0])+dms[1]/60.0+dms[2]/3600.0);
}

/* transform ecef to geodetic postion ------------------------------------------
* transform ecef position to geodetic position
* args   : double *r        I   ecef position {x,y,z} (m)
*          double *pos      O   geodetic position {lat,lon,h} (rad,m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void ecef2pos(const double *r, double *pos)
{
    double e2=FE_WGS84*(2.0-FE_WGS84),r2=InnerDot(r,r,2),z,zk,v=RE_WGS84,sinP;
    
    for (z=r[2],zk=0.0; fabs(z-zk)>=1E-4; ) 
	{
        zk=z;
        sinP=z/sqrt(r2+z*z);
        v=RE_WGS84/sqrt(1.0-e2*sinP*sinP);
        z=r[2]+v*e2*sinP;
    }

    pos[0]=r2>1E-12?atan(z/sqrt(r2)):(r[2]>0.0?PI/2.0:-PI/2.0);
    pos[1]=r2>1E-12?atan2(r[1],r[0]):0.0;
    pos[2]=sqrt(r2+z*z)-v;
}

/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
void pos2ecef(const double *pos, double *r)
{
    double sinP=sin(pos[0]),cosP=cos(pos[0]),sinL=sin(pos[1]),cosL=cos(pos[1]);
    double e2=FE_WGS84*(2.0-FE_WGS84),v=RE_WGS84/sqrt(1.0-e2*sinP*sinP);
    
    r[0]=(v+pos[2])*cosP*cosL;
    r[1]=(v+pos[2])*cosP*sinL;
    r[2]=(v*(1.0-e2)+pos[2])*sinP;
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
void xyz2enu(const double *pos, double *E)
{
    double sinP=sin(pos[0]),cosP=cos(pos[0]),sinL=sin(pos[1]),cosL=cos(pos[1]);
    
    E[0]=-sinL;      E[3]=cosL;       E[6]=0.0;
    E[1]=-sinP*cosL; E[4]=-sinP*sinL; E[7]=cosP;
    E[2]=cosP*cosL;  E[5]=cosP*sinL;  E[8]=sinP;
}

/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];
    
    xyz2enu(pos,E);
    matmul("NN",3,1,3,1.0,E,r,0.0,e);
}

/* transform local vector to ecef coordinate -----------------------------------
* transform local tangental coordinate vector to ecef
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *e        I   vector in local tangental coordinate {e,n,u}
*          double *r        O   vector in ecef coordinate {x,y,z}
* return : none
*-----------------------------------------------------------------------------*/
void enu2ecef(const double *pos, const double *e, double *r)
{
    double E[9];
    
    xyz2enu(pos,E);
    matmul("TN",3,1,3,1.0,E,e,0.0,r);
}

/* transform covariance to local tangental coordinate --------------------------
* transform ecef covariance to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *P        I   covariance in ecef coordinate
*          double *Q        O   covariance in local tangental coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covenu(const double *pos, const double *P, double *Q)
{
    double E[9],EP[9];
    
    xyz2enu(pos,E);
    matmul("NN",3,3,3,1.0,E,P,0.0,EP);
    matmul("NT",3,3,3,1.0,EP,E,0.0,Q);
}

/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
void covecef(const double *pos, const double *Q, double *P)
{
    double E[9],EQ[9];
    
    xyz2enu(pos,E);
    matmul("TN",3,3,3,1.0,E,Q,0.0,EQ);
    matmul("NN",3,3,3,1.0,EQ,E,0.0,P);
}

/* coordinate rotation matrix ------------------------------------------------*/
#define Rx(t,X) do { \
    (X)[0]=1.0; (X)[1]=(X)[2]=(X)[3]=(X)[6]=0.0; \
    (X)[4]=(X)[8]=cos(t); (X)[7]=sin(t); (X)[5]=-(X)[7]; \
} while (0)

#define Ry(t,X) do { \
    (X)[4]=1.0; (X)[1]=(X)[3]=(X)[5]=(X)[7]=0.0; \
    (X)[0]=(X)[8]=cos(t); (X)[2]=sin(t); (X)[6]=-(X)[2]; \
} while (0)

#define Rz(t,X) do { \
    (X)[8]=1.0; (X)[2]=(X)[5]=(X)[6]=(X)[7]=0.0; \
    (X)[0]=(X)[4]=cos(t); (X)[3]=sin(t); (X)[1]=-(X)[3]; \
} while (0)

/* satellite carrier wave length -----------------------------------------------
* get satellite carrier wave lengths
* args   : int    sat       I   satellite number
*          int    frq       I   frequency index (0:L1,1:L2,2:L5/3,...)
*          nav_t  *nav      I   navigation messages
* return : carrier wave length (m) (0.0: error)
*-----------------------------------------------------------------------------*/
double satwavelen(int sat, int frq, const nav_t *nav)
{
    const double freq_glo[]={FREQ1_GLO,FREQ2_GLO,FREQ3_GLO};
    const double dfrq_glo[]={DFRQ1_GLO,DFRQ2_GLO,0.0};
    int i,sys=satsys(sat,NULL);
    
    if (sys==SYS_GLO) 
	{
        if (0<=frq&&frq<=2) 
		{
            for (i=0;i<nav->ng;i++) 
			{
                if (nav->geph[i].sat!=sat) 
					continue;

                return CLIGHT/(freq_glo[frq]+dfrq_glo[frq]*nav->geph[i].frq);
            }
        }
    }
    else if (sys==SYS_CMP) 
	{	
		//原来代码
        //if      (frq==0) return CLIGHT/FREQ1_CMP; /* B1 */
        //else if (frq==1) return CLIGHT/FREQ2_CMP; /* B2 */
        //else if (frq==2) return CLIGHT/FREQ3_CMP; /* B3 */

		//wjz
		if      (frq==0) return CLIGHT/FREQ1_CMP; /* B1 */
		else if (frq==1) return CLIGHT/FREQ3_CMP; /* B3 */
		else if (frq==2) return CLIGHT/FREQ2_CMP; /* B2 */
    }
    else 
	{
        if      (frq==0) return CLIGHT/FREQ1; /* L1/E1 */
        else if (frq==1) return CLIGHT/FREQ2; /* L2 */
        else if (frq==2) return CLIGHT/FREQ5; /* L5/E5a */
        else if (frq==3) return CLIGHT/FREQ6; /* L6/LEX */
        else if (frq==4) return CLIGHT/FREQ7; /* E5b */
        else if (frq==5) return CLIGHT/FREQ8; /* E5a+b */
    }

    return 0.0;
}

/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position (ecef at transmission) (m)
*          double *rr       I   receiver position (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
double geodist(const double *rs, const double *rr, double *e)
{
    double r;
    int i;
    
    if (norm(rs, 3)<RE_WGS84) 
	{
		return -1.0;
	}

    for (i=0;i<3;i++) 
	{
		e[i]=rs[i]-rr[i];
	}

    r=norm(e,3);

    for (i=0;i<3;i++) 
	{
		e[i]/=r;
	}

    return r+OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT;
}

/* satellite azimuth/elevation angle -------------------------------------------
* compute satellite azimuth/elevation angle
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *e        I   receiver-to-satellilte unit vevtor (ecef)
*          double *azel     IO  azimuth/elevation {az,el} (rad) (NULL: no output)
*                               (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* return : elevation angle (rad)
*-----------------------------------------------------------------------------*/
double satazel(const double *pos, const double *e, double *azel)
{
    double az=0.0,el=PI/2.0,enu[3];
    
    if (pos[2]>-RE_WGS84)
	{
        ecef2enu(pos,e,enu);
        az=InnerDot(enu,enu,2)<1E-12?0.0:atan2(enu[0],enu[1]);

        if (az<0.0) 
		{
			az+=2*PI;
		}

        el=asin(enu[2]);
    }

    if (azel) 
	{
		azel[0]=az; 
		azel[1]=el;
	}

    return el;
}

/* compute dops ----------------------------------------------------------------
* compute DOP (dilution of precision)
* args   : int    ns        I   number of satellites
*          double *azel     I   satellite azimuth/elevation angle (rad)
*          double elmin     I   elevation cutoff angle (rad)
*          double *dop      O   DOPs {GDOP,PDOP,HDOP,VDOP}
* return : none
* notes  : dop[0]-[3] return 0 in case of dop computation error
*-----------------------------------------------------------------------------*/
#define SQRT(x)     ((x)<0.0?0.0:sqrt(x))

#pragma DATA_SECTION(g_DopMat_H,"sect_EDATA_III");
double g_DopMat_H[4*MAXSAT]={0};

void dops(int ns, const double *azel, double elmin, double *dop)
{
    double Q[16],cosel,sinel;
    int i,n;

	memset(g_DopMat_H, 0, sizeof(g_DopMat_H));
    
    for (i=0;i<4;i++)
    { 
    	dop[i]=0.0;
	}

    for (i=n=0;i<ns&&i<MAXSAT;i++) 
    {
        if (azel[1+i*2]<elmin||azel[1+i*2]<=0.0)
		{
        	continue;
		}

        cosel=cos(azel[1+i*2]);
        sinel=sin(azel[1+i*2]);

        g_DopMat_H[  4*n]=cosel*sin(azel[i*2]);
        g_DopMat_H[1+4*n]=cosel*cos(azel[i*2]);
        g_DopMat_H[2+4*n]=sinel;
        g_DopMat_H[3+4*n++]=1.0;
    }

    if (n<4) 
	{
    	return;
    }

    matmul("NT",4,4,n,1.0,g_DopMat_H,g_DopMat_H,0.0,Q);

    if (!matinv(Q,4)) 
    {
        dop[0]=SQRT(Q[0]+Q[5]+Q[10]+Q[15]); /* GDOP */
        dop[1]=SQRT(Q[0]+Q[5]+Q[10]);       /* PDOP */
        dop[2]=SQRT(Q[0]+Q[5]);             /* HDOP */
        dop[3]=SQRT(Q[10]);                 /* VDOP */
    }
}

/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
const double ion_default[]={ /* 2004/1/1 */
        0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
        0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
};

double ionmodel(gtime_t t, const double *ion, const double *pos, const double *azel)
{
    double tt,f,psi,phi,lam,amp,per,x;
    int week;
    
    if (pos[2]<-1E3||azel[1]<=0) return 0.0;
    if (norm(ion,8)<=0.0) ion=ion_default;
    
    /* earth centered angle (semi-circle) */
    psi=0.0137/(azel[1]/PI+0.11)-0.022;
    
    /* subionospheric latitude/longitude (semi-circle) */
    phi=pos[0]/PI+psi*cos(azel[0]);
    if      (phi> 0.416) phi= 0.416;
    else if (phi<-0.416) phi=-0.416;
    lam=pos[1]/PI+psi*sin(azel[0])/cos(phi*PI);
    
    /* geomagnetic latitude (semi-circle) */
    phi+=0.064*cos((lam-1.617)*PI);
    
    /* local time (s) */
    tt=43200.0*lam+time2gpst(t,&week);
    tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */
    
    /* slant factor */
    f=1.0+16.0*pow(0.53-azel[1]/PI,3.0);
    
    /* ionospheric delay */
    amp=ion[0]+phi*(ion[1]+phi*(ion[2]+phi*ion[3]));
    per=ion[4]+phi*(ion[5]+phi*(ion[6]+phi*ion[7]));
    amp=amp<    0.0?    0.0:amp;
    per=per<72000.0?72000.0:per;
    x=2.0*PI*(tt-50400.0)/per;
    
    return CLIGHT*f*(fabs(x)<1.57?5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)):5E-9);
}

//---------------------------------------------------------------------------------------------
// ionosphere mapping function
// compute ionospheric delay mapping function by single layer model
// args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
//          double *azel     I   azimuth/elevation angle {az,el} (rad)
// return : ionospheric mapping function
//---------------------------------------------------------------------------------------------
double ionmapf(const double *pos, const double *azel)
{
    if (pos[2]>=HION)
		return 1.0;

    return 1.0/cos(asin((RE_WGS84+pos[2])/(RE_WGS84+HION)*sin(PI/2.0-azel[1])));
}

//对流层模型函数
/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double tropmodel(gtime_t time, const double *pos, const double *azel, double humi)
{
    const double temp0=15.0;					/* temparature at sea level */
    double hgt,pres,temp,e,z,trph,trpw;
    
    if (pos[2]<-100.0 || 1E4<pos[2] || azel[1]<=0) 
	{
		return 0.0;
	}
    
    /* standard atmosphere */
    hgt=pos[2]<0.0?0.0:pos[2];
    
    pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
    temp=temp0-6.5E-3*hgt+273.16;
    e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
    
    /* saastamoninen model */
    z=PI/2.0-azel[1];
    trph=0.0022768*pres/(1.0-0.00266*cos(2.0*pos[0])-0.00028*hgt/1E3)/cos(z);
    trpw=0.002277*(1255.0/temp+0.05)*e/cos(z);

    return trph+trpw;
}

double interpc(const double coef[], double lat)
{
    int i=(int)(lat/15.0);
    if (i<1) return coef[0]; else if (i>4) return coef[4];
    return coef[i-1]*(1.0-lat/15.0+i)+coef[i]*(lat/15.0-i);
}

double mapf(double el, double a, double b, double c)
{
    double sinel=sin(el);
    return (1.0+a/(1.0+b/(1.0+c)))/(sinel+(a/(sinel+b/(sinel+c))));
}

/* ref [5] table 3 */
/* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
const double coef[][5]={
    { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
    { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
    { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
    
    { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
    { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
    { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
    
    { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
    { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
    { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
};


const double aht[]={ 2.53E-5, 5.49E-3, 1.14E-3}; /* height correction */
double nmf(gtime_t time, const double pos[], const double azel[], double *mapfw)
{
    double y,cosy,ah[3],aw[3],dm,el=azel[1],lat=pos[0]*RAD2DEG,hgt=pos[2];
    int i;
    
    if (el<=0.0) {
        if (mapfw) *mapfw=0.0;
        return 0.0;
    }
    /* year from doy 28, added half a year for southern latitudes */
    y=(time2doy(time)-28.0)/365.25+(lat<0.0?0.5:0.0);
    
    cosy=cos(2.0*PI*y);
    lat=fabs(lat);
    
    for (i=0;i<3;i++) {
        ah[i]=interpc(coef[i  ],lat)-interpc(coef[i+3],lat)*cosy;
        aw[i]=interpc(coef[i+6],lat);
    }
    /* ellipsoidal height is used instead of height above sea level */
    dm=(1.0/sin(el)-mapf(el,aht[0],aht[1],aht[2]))*hgt/1E3;
    
    if (mapfw) *mapfw=mapf(el,aw[0],aw[1],aw[2]);
    
    return mapf(el,ah[0],ah[1],ah[2])+dm;
}

//对流层映射函数
/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF
* args   : gtime_t t        I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [5] (NMF) and [9] (GMF)
*          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
double tropmapf(gtime_t time, const double pos[], const double azel[], double *mapfw)
{    
    if (pos[2]<-1000.0||pos[2]>20000.0) 
    {
        if (mapfw) 
		{
        	*mapfw=0.0;
		}

        return 0.0;
    }

    return nmf(time,pos,azel,mapfw); /* NMF */
}

/* interpolate antenna phase center variation --------------------------------*/
double interpvar(double ang, const double *var)
{
    double a=ang/5.0; /* ang=0-90 */
    int i=(int)a;
    if (i<0) return var[0]; else if (i>=18) return var[18];
    return var[i]*(1.0-a+i)+var[i+1]*(a-i);
}

/* receiver antenna model ------------------------------------------------------
* compute antenna offset by antenna phase center parameters
* args   : pcv_t *pcv       I   antenna phase center parameters
*          double *azel     I   azimuth/elevation for receiver {az,el} (rad)
*          int     opt      I   option (0:only offset,1:offset+pcv)
*          double *dant     O   range offsets for each frequency (m)
* return : none
* notes  : current version does not support azimuth dependent terms
*-----------------------------------------------------------------------------*/
void antmodel(const pcv_t *pcv, const double *del, const double *azel, int opt, double *dant)
{
    double e[3],off[3],cosel=cos(azel[1]);
    int i,j;
    
    e[0]=sin(azel[0])*cosel;
    e[1]=cos(azel[0])*cosel;
    e[2]=sin(azel[1]);
    
    for (i=0;i<NFREQ;i++) 
	{
        for (j=0;j<3;j++)
		{
			off[j]=pcv->off[i][j]+del[j];
		}
        
        dant[i]=-InnerDot(off,e,3)+(opt?interpvar(90.0-azel[1]*RAD2DEG,pcv->var[i]):0.0);
    }

    //trace(5,"antmodel: dant=%6.3f %6.3f\n",dant[0],dant[1]);
}

/* carrier smoothing -----------------------------------------------------------
* carrier smoothing by Hatch filter
* args   : obs_t  *obs      IO  raw observation data/smoothed observation data
*          int    ns        I   smoothing window size (epochs)
* return : none
*-----------------------------------------------------------------------------*/
#pragma DATA_SECTION(glPs,"sect_EDATA_III");
double glPs[2][MAXSAT][NFREQ]={{{0}}};

#pragma DATA_SECTION(glLp,"sect_EDATA_III");
double glLp[2][MAXSAT][NFREQ]={{{0}}};

#pragma DATA_SECTION(glCsMatN,"sect_EDATA_III");
int glCsMatN[2][MAXSAT][NFREQ]={{{0}}};

void csmooth(obs_t *obs, int ns)
{
    double dcp;
    int i,j,s,r;
    obsd_t *p;
    
    for (i=0;i<obs->n;i++) 
    {
        p=&obs->data[i]; 
        s=p->sat; 
        r=p->rcv;
        
        for (j=0;j<NFREQ;j++) 
        {
            if (s<=0||MAXSAT<s||r<=0||2<r)
            { 
            	continue;
            }

            if (p->P[j]==0.0||p->L[j]==0.0) 
			{
            	continue;
            }

            if (p->LLI[j]) 
			{
            	glCsMatN[r-1][s-1][j]=0;
			}
            
            if (glCsMatN[r-1][s-1][j]==0) 
			{
            	glPs[r-1][s-1][j]=p->P[j];
			}
            else 
            {
                dcp=lam_carr[j]*(p->L[j]-glLp[r-1][s-1][j]);
                glPs[r-1][s-1][j]=p->P[j]/ns+(glPs[r-1][s-1][j]+dcp)*(ns-1)/ns;
            }

            if (++glCsMatN[r-1][s-1][j]<ns) 
			{
            	p->P[j]=0.0; 
			}
            else 
			{
            	p->P[j]=glPs[r-1][s-1][j];
			}

            glLp[r-1][s-1][j]=p->L[j];
        }
    }
}


