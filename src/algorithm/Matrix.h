/***********************************************************************************
This file DEFINED all header files of Matrix operation.
Application using Matrix  operation should include this file first.
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
#ifndef _MATRIX_H
#define _MATRIX_H

#include "Constant.h"


//����������߶�
#define  MAXMatrixSize 7

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void EyeMatrix(double *pMat, int n, double dValue);//dValue*eye(n)
double InnerDot(const double *a, const double *b, int n);//a'*b
double norm(const double *a, int n);//|| a ||
void cross3(const double *a, const double *b, double *c);//(a x b) (3 x 1)
int normv3(const double *a, double *b);// (3 x 1) || b || = 1
void matcpy(double *A, const double *B, int n, int m);
void matmul(const char *tr, int n, int k, int m, double alpha, const double *A, const double *B, double beta, double *C);
int matinv(double *A, int n);
void m2qnb(double* Cnb, double* qnb0);// �任���������Ԫ��
void q2mat(double* qnb, double* Cnb) ; //��Ԫ��ת������
void att2qnb(double* att, double* qnb); //��̬ת��Ԫ��
void qnb2att(double* qnb, double* att); //��Ԫ��ת��̬
void matrixSum(double* a, double* b, int n, int m, double coef, double* ab) ; //�������
void qmulv(double* qnb, double* fb,double *fn) ;//����ͨ����Ԫ����3D��ת?
void rotv(double* wnin, double* fn, double* an_) ;//��ת����
void UpdateQnb(double* qnb, double* rv_ib, double* rv_in) ; //��Ԫ������
void qnbmul(double* qnb1, double* qnb2, double* qnb); //��Ԫ���˷�
double* zeros(int n, int m);  // ����һ����n*m����
double* eyes(int n);
//double* mat(int n, int m);//��������
void askew(double* web, double* CW);//���Գ���
void symmetry(double* P, int n, double* P_) ; //����Գƻ�����
void rv2q(double* phi, double* qnb) ;//..ת��Ԫ��
void qdelphi(double* qnb, double* phi,double *qnb_);

#ifdef  __cplusplus
}
#endif

#endif	/*_MATRIX_H*/
/*End***********************************************************************************/



