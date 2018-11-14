/*  * Space Vector lib for BGC
    * Author: KhangND
    * V1:
    
    * Created 30/10/2018
*/

#ifndef SVM_H
#define SVM_H

#include "BLDC.h"
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
void bgc_SVPWM(volatile BLDC *bldc0, int CTR);
#endif
