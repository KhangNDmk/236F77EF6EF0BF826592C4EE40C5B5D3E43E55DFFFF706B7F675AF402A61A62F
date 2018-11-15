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


void pid_init(PID_OBJ *pid , float Kp, float Ki, float Kd)
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


float pid_controller(PID_OBJ* pid0 , float in)
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
void bldc2_init()
{
    bldc2.ID = 2;
    bldc2.pole =4;
    bldc2.Vdc = VDC;
    bldc2.Voffset = 3;
    bldc2.CTR = (int)(htim1.Init.Period/2) ;
    bldc2.angle =0;

}
/**/

void bldc_init()
{
    bldc2_init();
}
/**/
int bldc_stop(volatile BLDC *bldc0)
{
    if(bldc0->ID == bldc1.ID)
    {
        /* Stop PWM*/
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

        /* Stop Enable*/
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

        return 0;
    }

    if(bldc0->ID == bldc2.ID)
    {
        /* Stop PWM*/
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

        /* Stop Enable*/
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

        return 0;
    }

    if(bldc0->ID == bldc3.ID)
    {
        /* Stop PWM*/
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);

        /* Stop Enable*/
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);

        return 0;
    }
    return 1;
}


/* Start bldc_start() */
int bldc_start(volatile BLDC *bldc0)
{
    if(bldc0->ID == bldc1.ID)
    {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

        return 0;
    }

    if(bldc0->ID == bldc2.ID)
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);

        return 0;
    }

    if(bldc0->ID == bldc3.ID)
    {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);

        return 0;
    }
    return 1;
}
/* Stop bldc_start() */


/* Start bldc_alaign() */
int bldc_alaign(BLDC *bldc0)
{
    if(bldc0->ID == bldc1.ID)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return 0;
    }

    if(bldc0->ID == bldc2.ID)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        return 0;
    }

    if(bldc0->ID == bldc2.ID)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 100);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        return 0;
    }
    return 1;
}
/* Stop bldc_alaign() */


/**/
void test_bldc(void)
{
    asm("nop");
}
