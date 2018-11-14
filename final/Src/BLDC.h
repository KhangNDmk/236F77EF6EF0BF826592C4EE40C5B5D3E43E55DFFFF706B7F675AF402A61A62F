/*  * BLDC lib for BGC
    * Author: KhangND
    * V1: Create BLDC and PID struct
    
    * Created 30/10/2018
*/

#ifndef BLDC_H
#define BLDC_H

/**/
#define VDC             12      /* Dien ap nguon*/
#define VMAX            8       /* Dien ap max dieu che vector*/
#define PR10            99     /* Counter cua Timer tao PWM */
#define BLDC_CKTM       0.01       /* chu ki interrupt dieu khien BLDC */
#define DELTA_ANGLE     0.01       /* == BLDC_CKTM * BLDC_POLE  */

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
} pid_obj;
/**/

/**/

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
    unsigned int duty1;
    unsigned int duty2;
    unsigned int duty3;
}BLDC;
/**/

/**/
void pid_init(pid_obj *pid , float Kp, float Ki, float Kd);

/**/
float pid_controller(pid_obj* S , float in);

/**/
void test_bldc(void);
/**/
void bldc_init(volatile BLDC *bldc0, float dphi0 , float phi0 ,float Vdc0 , float Voffset0, float K0);
/**/
void bldc1_alaign();

#endif
