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


//求逆矩阵最大尺度
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
void m2qnb(double* Cnb, double* qnb0);// 变换矩阵计算四元数
void q2mat(double* qnb, double* Cnb) ; //四元数转换矩阵
void att2qnb(double* att, double* qnb); //姿态转四元数
void qnb2att(double* qnb, double* att); //四元数转姿态
void matrixSum(double* a, double* b, int n, int m, double coef, double* ab) ; //矩阵求和
void qmulv(double* qnb, double* fb,double *fn) ;//向量通过四元数做3D旋转?
void rotv(double* wnin, double* fn, double* an_) ;//旋转向量
void UpdateQnb(double* qnb, double* rv_ib, double* rv_in) ; //四元数更新
void qnbmul(double* qnb1, double* qnb2, double* qnb); //四元数乘法
double* zeros(int n, int m);  // 返回一个零n*m矩阵
double* eyes(int n);
//double* mat(int n, int m);//创建矩阵
void askew(double* web, double* CW);//反对称阵
void symmetry(double* P, int n, double* P_) ; //矩阵对称化处理
void rv2q(double* phi, double* qnb) ;//..转四元数
void qdelphi(double* qnb, double* phi,double *qnb_);

#ifdef  __cplusplus
}
#endif

#endif	/*_MATRIX_H*/
/*End***********************************************************************************/



