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
    float elem[3][3];
}mat3;

/**/
void svmtest(void);
/**/
float bgc_sin (int deg);
/**/
float bgc_cos(int deg);
/**/
void bgc_Vab_Cal(BLDC *bldc0);

/**/
void bgc_bldchdl(volatile BLDC *bldc0, float u0);

/**/
void bgc_SVPWM(volatile BLDC *bldc0);
/**/
void multi_mat3(mat3* A, mat3* B, mat3* C);
/**/
void transpose_mat3(mat3* A, mat3* At);
/**/
float bgc_tsin(float x);
/**/
float bgc_fsin(float x);
/**/
float bgc_fcos(float x);
/**/
float bgc_tatan(float x);
/**/
float bgc_fatan2(float y, float x);
/**/
/**/
#ifdef __cplusplus
}
#endif
#endif
