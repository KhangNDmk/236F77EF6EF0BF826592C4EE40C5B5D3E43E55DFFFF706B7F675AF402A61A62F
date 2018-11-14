/*  * BLDC lib for BGC
    * Author: KhangND

    
    * Created 30/10/2018
*/

/* system include*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "gpio.h"
/**/
#include "BLDC.h"


void pid_init(pid_obj *pid , float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki= Ki;
    pid->Kd = Kd;
    pid->A0 =  Kp + Ki + Kd ;
    pid->A1 =  -Kp - 2*Kd ;
    pid->A2 = Kd ;
    pid->state[0] =0;
    pid->state[1] =0;
}


float pid_controller(pid_obj* pid0 , float in)
{
    float out;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (pid0->A0 * in) + (pid0->A1 * pid0->state[0]) 
           + (pid0->A2 * pid0->state[1]) + (pid0->state[2]);

    /* Update state */
    pid0->state[1] = pid0->state[0];
    pid0->state[0] = in;
    pid0->state[2] = out;


    return (out);
}
/**/
void bldc_init(volatile BLDC *bldc0, float dphi0 , float phi0 ,float Vdc0 , float Voffset0, float K0)
{
    bldc0->Voffset = Voffset0;
    bldc0->Kf = K0;
    bldc0->Vdc = Vdc0;
    bldc0->dphi = dphi0;
    bldc0->angle = phi0;
}
/**/

/**/
void bldc1_alaign()
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
}
/**/
void test_bldc(void)
{
    asm("nop");
}
