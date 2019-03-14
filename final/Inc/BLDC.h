/*  * BLDC lib for BGC
    * Author: KhangND
    * V1: Create BLDC and PID struct
    * v2: Create
    * Created 30/10/2018
*/

#ifndef BLDC_H
#define BLDC_H
#ifdef __cplusplus
extern "C" {
#endif


/**/
#define VDC             12      /* Dien ap nguon*/
#define VMAX            8       /* Dien ap max dieu che vector*/
#define PR10            99     /* Counter cua Timer tao PWM */
#define BLDC_CKTM       0.01       /* chu ki interrupt dieu khien BLDC */
#define DELTA_ANGLE     0.01       /* == BLDC_CKTM * BLDC_POLE  */
/**/


/**/
//bldc1: TIM1, Enable: PD0, PD2, PD4
//bldc2: TIM3, Enable: PD1, PD3, PD5
//bldc3: TIM4, Enable: PA8, PA10, PA14

/**/


/* BLDC struct              */
/* State[0] in(k)           */
/* State[1] in(k-1)         */
/* State[0] out(k)          */
typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float A0;
    float A1;
    float A2;
    float state[3];
} PID_OBJ;
/**/

/**/
int cal_error(int ref, int fb);
/**/

/**/
typedef struct
{
    float Kf;               /* hang so ti le V/F */
    float Vdc;              /* dien ap nguon */    
    float Voffset;          /**/
    float Vref;             /* Vref cua dong co*/
    float w;                /* van toc goc , tinh theo */
    float angle;            /* goc cua dong co  */
    float dphi;
    float Va;
    float Vb;
    unsigned int CTR;
    unsigned int ID;
    unsigned int pole;
    unsigned int duty1;
    unsigned int duty2;
    unsigned int duty3;
}BLDC;
/**/

/**/
void pid_init(PID_OBJ *pid , float Kp, float Ki, float Kd);

/**/
float pid_controller(PID_OBJ* S , float in);

/**/
void test_bldc(void);
/**/

/**/

/**/
volatile BLDC bldc1;
volatile BLDC bldc2;
volatile BLDC bldc3;
extern volatile BLDC bldc1;
extern volatile BLDC bldc2;
extern volatile BLDC bldc3;

#ifdef __cplusplus
}
#endif
#endif
