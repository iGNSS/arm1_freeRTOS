/***********************************

Matrix operation module

************************************/
#include <stdlib.h>
#include "math.h"
#include "Matrix.h"
#include "string.h"
#include "inavlog.h"

#ifdef __cplusplus
extern "C" {
#endif

double    glLudMat_vv[MAXMatrixSize] = {0}; 
double	m_InvMat_B[MAXMatrixSize*MAXMatrixSize]={0};		
int		m_InvMat_indx[MAXMatrixSize]={0};	
char	g_MatrixBuff[1024*4]={0};		


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
int matinv(double *A, int n)
{
    double d=0.0;
    int i=0, j=0;

	if(n > MAXMatrixSize)
	{
		return -1;
	}
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

/*----------------- -----------------------------------------------------------
* args   : 
*          double *Cnb      I   Cnb(3*3)
*          double *qnb0    IO  qnb0 (4 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void m2qnb(double* Cnb, double* qnb0)// 变换矩阵计算四元数
{
	int i;
	double C11, C12, C13, C21, C22, C23, C31, C32, C33;
	double q0t,q1t,q2t,q3t;
	//姿态转换矩阵
	C11 = Cnb[0]; C12 = Cnb[3]; C13 = Cnb[6];
	C21 = Cnb[1]; C22 = Cnb[4]; C23 = Cnb[7];
	C31 = Cnb[2]; C32 = Cnb[5]; C33 = Cnb[8];

	q0t = 0.5 * sqrt(1 + C11 + C22 + C33);
	q1t = 0.5 * sqrt(1 + C11 - C22 - C33);
	q2t = 0.5 * sqrt(1 - C11 + C22 - C33);
	q3t = 0.5 * sqrt(1 - C11 - C22 + C33);
	if (C11 >= C22 + C33) {
		qnb0[1] = q1t;
		qnb0[0] = (C32 - C23) / (4 * q1t);
		qnb0[2] = (C12 + C21) / (4 * q1t);
		qnb0[3] = (C13 + C31) / (4 * q1t);
	}
	else if (C22 >= C11 + C33) {
		qnb0[2] = q2t;
		qnb0[0] = (C13 - C31) / (4 * q2t);
		qnb0[1] = (C12 + C21) / (4 * q2t);
		qnb0[3] = (C23 + C32) / (4 * q2t);
	}
	else if (C33 >= C11 + C22) {
		qnb0[3] = q3t;
		qnb0[0] = (C21 - C12) / (4 * q3t);
		qnb0[1] = (C13 + C31) / (4 * q3t);
		qnb0[2] = (C23 + C32) / (4 * q3t);
	}
	else {
		qnb0[0] = q0t;
		qnb0[1] = (C32 - C23) / (4 * q0t);
		qnb0[2] = (C13 - C31) / (4 * q0t);
		qnb0[3] = (C21 - C12) / (4 * q0t);
	}
	for (i = 0; i < 4; i++)// 单位化
	{
		qnb0[i] /= norm(qnb0, 4);
	}
}

/*----------------- -----------------------------------------------------------
* args   : 
*          double *qnb0    I matrix qnb0 (4 x 1)
*          double *Cnb     IO   Cnb(3*3)
* return : none
*-----------------------------------------------------------------------------*/
void q2mat(double* qnb, double* Cnb)  //四元数转换矩阵
{
	double q11 = qnb[0] * qnb[0], q12 = qnb[0] * qnb[1], q13 = qnb[0] * qnb[2], q14 = qnb[0] * qnb[3];
	double q22 = qnb[1] * qnb[1], q23 = qnb[1] * qnb[2], q24 = qnb[1] * qnb[3];
	double q33 = qnb[2] * qnb[2], q34 = qnb[2] * qnb[3];
	double q44 = qnb[3] * qnb[3];
	Cnb[0] = q11 + q22 - q33 - q44; Cnb[3] = 2 * (q23 - q14);		Cnb[6] = 2 * (q24 + q13);
	Cnb[1] = 2 * (q23 + q14);		Cnb[4] = q11 - q22 + q33 - q44;	Cnb[7] = 2 * (q34 - q12);
	Cnb[2] = 2 * (q24 - q13);		Cnb[5] = 2 * (q34 + q12);		Cnb[8] = q11 - q22 - q33 + q44;
}


/*----------------- -----------------------------------------------------------
* args   : 
*          double *Cnb      I    att(3*1)
*          double *qnb0    IO  qnb0 (4 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void att2qnb(double* att, double* qnb) //姿态转四元数
{
	int i;
	double att_2[3];
	double sin_a[3], cos_a[3];

	for (i = 0; i < 3; i++)
	{
		att_2[i] = att[i] / 2;
		sin_a[i] = sin(att_2[i]);
		cos_a[i] = cos(att_2[i]);
	}
	qnb[0] = cos_a[0] * cos_a[1] * cos_a[2] - sin_a[0] * sin_a[1] * sin_a[2];
	qnb[1] = sin_a[0] * cos_a[1] * cos_a[2] - cos_a[0] * sin_a[1] * sin_a[2];
	qnb[2] = cos_a[0] * sin_a[1] * cos_a[2] + sin_a[0] * cos_a[1] * sin_a[2];
	qnb[3] = cos_a[0] * cos_a[1] * sin_a[2] + sin_a[0] * sin_a[1] * cos_a[2];
}

/*----------------- -----------------------------------------------------------
* args   : 
*          double *Cnb      I   qnb0(4*1)
*          double *qnb0    IO  att (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
void qnb2att(double* qnb, double* att) //四元数转姿态
{
	double Cnb11 = 0, Cnb12 = 0, Cnb13 = 0;
	double Cnb21, Cnb22, Cnb23;
	double Cnb31, Cnb32, Cnb33;
	double q11 = qnb[0] * qnb[0], q12 = qnb[0] * qnb[1], q13 = qnb[0] * qnb[2], q14 = qnb[0] * qnb[3];
	double q22 = qnb[1] * qnb[1], q23 = qnb[1] * qnb[2], q24 = qnb[1] * qnb[3];
	double q33 = qnb[2] * qnb[2], q34 = qnb[2] * qnb[3];
	double q44 = qnb[3] * qnb[3];

	Cnb11 = q11 + q22 - q33 - q44; Cnb12 = 2 * (q23 - q14); Cnb13 = 2 * (q24 + q13);
	Cnb21 = 2 * (q23 + q14); Cnb22 = q11 - q22 + q33 - q44; Cnb23 = 2 * (q34 - q12);
	Cnb31 = 2 * (q24 - q13); Cnb32 = 2 * (q34 + q12); Cnb33 = q11 - q22 - q33 + q44;

	att[0] = asin(Cnb32);// 俯仰
	att[1] = atan2(-Cnb31, Cnb33);// 横滚
	att[2] = atan2(-Cnb12, Cnb22);// 航向角
}

/* sum matrix --------------------------------------------------------------------
*  sum  matrix by matrix (ab=a+coef*b)
*          int    n,k           I  size of matrix a,b
*          double *a,*b     I  matrix a(n x m), b (n x m)
*          double coef      I  coef
*          double *ab       IO matrix ab (n x m)
* return : none
*-----------------------------------------------------------------------------*/
void matrixSum(double* a, double* b, int n, int m, double coef, double* ab)  //矩阵求和
{
	int i, j;
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < m; j++)
			ab[j + i * m] = a[j + i * m] + coef * b[j + i * m];
	}
}

void qmulv(double* qnb, double* fb,double *fn) //向量通过四元数做3D旋转?
{
	double q01 = -qnb[1] * fb[0] - qnb[2] * fb[1] - qnb[3] * fb[2];
	double q02 = qnb[0] * fb[0] + qnb[2] * fb[2] -qnb[3] * fb[1];
	double q03 = qnb[0] * fb[1] + qnb[3] * fb[0] - qnb[1] * fb[2];
	double q04 = qnb[0] * fb[2] + qnb[1] * fb[1] - qnb[2] * fb[0];
	fn[0] = -q01 * qnb[1] + q02 * qnb[0] - q03 * qnb[3] + q04 * qnb[2];
	fn[1] = -q01 * qnb[2] + q03 * qnb[0] - q04 * qnb[1] + q02 * qnb[3];
	fn[2] = -q01 * qnb[3] + q04 * qnb[0] - q02 * qnb[2] + q03 * qnb[1];
}

void rotv(double* wnin, double* fn, double* an_) //旋转向量
{
	int i;
	double n = norm(wnin, 3);
	double n2 = n * n;
	double c, f;
	double q[4];
	if (n2 < (PI / 180.0 * PI / 180.0))	// 0.017^2
	{
		double n4 = n2 * n2;
		c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
		f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
	}
	else
	{
		double n_2 = sqrt(n2) / 2.0;
		c = cos(n_2);
		f = sin(n_2) / n_2 * 0.5;
	}
	q[0] = c;
	for (i = 0; i < 3; i++)q[i + 1] = f * wnin[i];
	double  q01 = -q[1] * fn[0] - q[2] * fn[1] - q[3] * fn[2];
	double q02 = q[0] * fn[0] + q[2] * fn[2] - q[3] * fn[1];
	double q03 = q[0] * fn[1] + q[3] * fn[0] - q[1] * fn[2];
	double q04 = q[0] * fn[2] + q[1] * fn[1] - q[2] * fn[0];
	an_[0] = -q01 * q[1] + q02 * q[0] - q03 * q[3] + q04 * q[2];
	an_[1] = -q01 * q[2] + q03 * q[0] - q04 * q[1] + q02 * q[3];
	an_[2] = -q01 * q[3] + q04 * q[0] - q02 * q[2] + q03 * q[1];
}

void UpdateQnb(double* qnb, double* rv_ib, double* rv_in)  //四元数更新
{
	int i;
	double qnb_[4];
	double n, n2,n4,n_2;
	double rvib_[4],rvin_[4], rvib_f,rvin_f;
	double qb[4];
	n = norm(rv_ib, 3); n2 = n * n;

	if (n2 < (PI / 180.0 * PI / 180.0))	// 0.017^2
	{
		n4 = n2 * n2;
		rvib_[0]= 1.0 - n2 * (1.0 / F2) +n4 * (1.0 / F4);
		rvib_f = 0.5 - n2 * (1.0 / F3) +n4 * (1.0 / F5);
	}
	else
	{
		n_2 = sqrt(n2) / 2.0;
		rvib_[0]= cos(n_2);
		rvib_f = sin(n_2) / n_2 * 0.5;
	}
	for (i = 0; i < 3; i++) { rvib_[i+1] = rvib_f * rv_ib[i]; }
	// 四元数乘法计算
	qnbmul(qnb, rvib_,qb);
	//-----------------------------------------------------------------------
	n = norm(rv_in, 3); n2 = n * n;
	if (n2 < (PI / 180.0 * PI / 180.0))	// 0.017^2
	{
		n4 = n2 * n2;
		rvin_[0] = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
		rvin_f =  -0.5 +n2 * (1.0 / F3) -n4 * (1.0 / F5);
	}
	else
	{
		n_2 = sqrt(n2) / 2.0;
		rvin_[0] = cos(n_2);
		rvin_f = -sin(n_2) / n_2 * 0.5;
	}
	for (i = 0; i < 3; i++) { rvin_[i + 1] = rvin_f * rv_in[i]; }
	qnbmul(rvin_, qb, qnb_);
	//----单位化---------------------------------------------------------------------
	n = norm(qnb_, 4); n2 = n * n;
	if (n2 > 1.000001 || n2 < 0.999999)
	{
		for (i = 0; i < 4; i++) { qnb[i] = qnb_[i] / n; }
	}
	else
	{
		for (i = 0; i < 4; i++) { qnb[i] = qnb_[i]; }
	}
}

void qnbmul(double* qnb1, double* qnb2, double* qnb) //四元数乘法
{
	qnb[0] = qnb1[0] * qnb2[0] - qnb1[1] * qnb2[1] - qnb1[2] * qnb2[2] - qnb1[3] * qnb2[3];
	qnb[1] = qnb1[0] * qnb2[1] + qnb1[1] * qnb2[0] + qnb1[2] * qnb2[3] - qnb1[3] * qnb2[2];
	qnb[2] = qnb1[0] * qnb2[2] + qnb1[2] * qnb2[0] + qnb1[3] * qnb2[1] - qnb1[1] * qnb2[3];
	qnb[3] = qnb1[0] * qnb2[3] + qnb1[3] * qnb2[0] + qnb1[1] * qnb2[2] - qnb1[2] * qnb2[1];
}


double* zeros(int n, int m)  // 返回一个零n*m矩阵
{
	int i,j;
	double* p;
	if (m <= 0 || n <= 0)return 0;
	if (!(p = (double*)malloc(sizeof(double)* n * m))) 
	{
		inav_log(INAVMD(LOG_ERR),"p fail");
		return 0;
	}
	for (i =0; i < n; i++)
	{
		for(j=0;j<m;j++)
		{
			p[i+j*n]=0;
		}
	}
	return p;
}

double* eyes(int n)
{
	int i;
	double* p;
	if (n <= 0)return 0;
	if (!(p=zeros(n, n)))
	{
		inav_log(INAVMD(LOG_ERR),"p fail");
		return 0;
	}
	for (i =0; i < n; i++)
	{
		p[i + i * n] = 1.0;
	}
	return p;
}
#if 0
double* mat(int n, int m)//创建矩阵
{
	double* p;
	if (!(p = (double*)malloc(sizeof(double) * n * m))) 
	{
		inav_log(INAVMD(LOG_ERR),"p fail");
		return 0;
	}
	return p;
}
#endif

void askew(double* web, double* CW)//反对称阵
{
	CW[0] = 0.0;		CW[3] = (-web[2]);	CW[6] = web[1];
	CW[1] = web[2];		CW[4] = 0.0;		CW[7] = (-web[0]);
	CW[2] = (-web[1]);	CW[5] = web[0];		CW[8] = 0.0;
}


void symmetry(double* P, int n, double* P_)  //矩阵对称化处理
{
	int i, j;
	for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++) {
			P_[i + j * n] = P_[j + i * n] = (P[i + j * n] + P[j + i * n]) / 2;
		}
	}
}

void rv2q(double* phi, double* qnb) //..转四元数
{
	int i;
	double c, f;
	double n=norm(phi, 3);
	double n2 = n * n;

	if (n2 < (PI / 180.0 * PI / 180.0))	// 0.017^2
	{
		double n4 = n2 * n2;
		c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
		f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
	}
	else
	{
		double n_2 = n/ 2.0;
		c = cos(n_2);
		f = sin(n_2) / n;
	}
	qnb[0] = c; for (i = 0; i < 3; i++)qnb[i + 1] = f * phi[i];
}

void qdelphi(double* qnb, double* phi,double *qnb_)
{
	double* temp_qnb = zeros(4, 1);
	if(NULL == temp_qnb)
	{
		inav_log(INAVMD(LOG_ERR),"temp_qnb fail");
		return;
	}
	rv2q(phi, temp_qnb);
	qnbmul(temp_qnb, qnb,qnb_);
	free(temp_qnb);
}

#ifdef __cplusplus
}
#endif

/*	End	***********************************************************************/
