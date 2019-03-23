/*  * Space Vector lib for BGC
    * Author: KhangND
    * V1:
    
    * Created 30/10/2018
*/

#ifndef BGC_MATH_H
#define BGC_MATH_H
#ifdef __cplusplus
extern "C" {
#endif


#include "BLDC.h"

/**/
typedef struct {
    double elem[3][3];
}mat3;

/**/
void svmtest(void);
/**/
double bgc_sin (int deg);
/**/
double bgc_cos(int deg);
/**/
void bgc_Vab_Cal(BLDC *bldc0);

/**/
void bgc_bldchdl(volatile BLDC *bldc0, double u0);

/**/
void bgc_SVPWM(volatile BLDC *bldc0);
/**/
void multi_mat3(mat3* A, mat3* B, mat3* C);
/**/
void transpose_mat3(mat3* A, mat3* At);
/**/
double bgc_tsin(double x);
/**/
double bgc_fsin(double x);
/**/
double bgc_fcos(double x);
/**/
double bgc_tatan(double x);
/**/
double bgc_fatan2(double y, double x);
/**/
/**/
#ifdef __cplusplus
}
#endif
#endif
