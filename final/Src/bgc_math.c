/*  * Space Vector lib for BGC
    * Author: KhangND
    * V1:
    
    * Created 30/10/2018
*/

#include <bgc_math.h>
#include "BLDC.h"
#include "math.h"
static const double sine_table[91]={0,0.0175,0.0349,0.0523,0.0698,0.0872,0.1045,0.1219,0.1392,0.1564,0.1736,
                                    0.1908,0.2079,0.2250,0.2419,0.2588,0.2756,0.2924,0.3090,0.3256,0.3420,
                                    0.3584,0.3746,0.3907,0.4067,0.4226,0.4384,0.4540,0.4695,0.4848,0.5000,
                                    0.5150,0.5299,0.5446,0.5592,0.5736,0.5878,0.6018,0.6157,0.6293,0.6428,
                                    0.6561,0.6691,0.6820,0.6947,0.7071,0.7193,0.7314,0.7431,0.7547,0.7660,
                                    0.7771,0.7880,0.7986,0.8090,0.8192,0.8290,0.8387,0.8480,0.8572,0.8660,
                                    0.8746,0.8829,0.8910,0.8988,0.9063,0.9135,0.9205,0.9272,0.9336,0.9397,
                                    0.9455,0.9511,0.9563,0.9613,0.9659,0.9703,0.9744,0.9781,0.9816,0.9848,
                                    0.9877,0.9903,0.9925,0.9945,0.9962,0.9976,0.9986,0.9994,0.9998,1.0000};

/**/


/* Calculate sin's value from sine_table*/
double bgc_sin(int deg)
{
    if(deg<91) return sine_table[deg];
    else if (deg <181) return sine_table[180-deg];
    else if (deg < 271) return (- sine_table[deg-180]);
    else return ( - sine_table[360-deg]) ;
}
/* end bgc_sin*/


/* Calculate cosin's value from sine_table */
/* return cos(deg)*/
double bgc_cos(int deg)
{
    if (deg < 91) return sine_table[90-deg];
    else if (deg < 181) return (- sine_table[deg-90]);
    else if (deg < 271) return (- sine_table[270-deg]);
    else return sine_table[deg-270]; 
}
/* end bgc_cos*/

/**/

/*begin matrix multiply 3x3 */
void multi_mat3(mat3* A, mat3* B, mat3* C)
{
    int i,j,k;
    double sum;
    for(i=0;i<3;i++)                            /* la dong i cua ma tran A*/
    {
        for(j=0;j<3;j++)                        /* la cot j cua ma tran B*/
        {
            sum=0;
            for(k=0;k<3;k++)
            {
                sum+=A->elem[i][k]*B->elem[k][j];
            }
            C->elem[i][j]=sum;
        }
    }
}
/*end matrix multiply 3x3*/

/*begin transpose matrix 3x3*/
void transpose_mat3(mat3* A, mat3* At)
{
    int i,j;
    for(i=0;i<3;i++)
    {
        for (j=0;j<3;j++)
        {
            At->elem[i][j]=A->elem[j][i];
        }
    }
}
/*end transpose matrix 3x3*/

/*begin test fast sine algorithm for input range 0->90 deg */
double bgc_tsin(double x)
{
#define     a0      0.707106781187
#define     a2      -0.872348075361
#define     a4      0.179251759526
#define     a6      -0.0142718282624
#define     b1      -1.11067032264
#define     b3      0.4561589075945
#define     b5      -0.0539104694791
#define     deg     0.01111111111111
    double A, B, m1, m2, m3, m4, m5, m6;
    m1 = deg * x - 0.5;
    m2 = m1 * m1;
    m3 = m2 * m1;
    m4 = m3 * m1;
    m5 = m4 * m1;
    m6 = m5 * m1;

    A = a0 + a2 * m2 + a4 * m4 + a6 * m6;
    B = b1 * m1 + b3 * m3 + b5 * m5;
#undef     a0
#undef     a2
#undef     a4
#undef     a6
#undef     b1
#undef     b3
#undef     b5
#undef     deg

    return A-B;
}
/*end test fast sine algorithm*/

/* begin fast sine for input 0->360 deg*/
double bgc_fsin(double x)
{
    if(x>270) return -bgc_tsin(360-x);
    else if (x>180) return -bgc_tsin(x-180);
    else if (x>90) return bgc_tsin(180-x);
    else return bgc_tsin(x);
}
/* end fast sine for input 0->360 deg*/

/* begin fast cosine for input 0->360 deg*/
double bgc_fcos(double x)
{
    if(x>270) return bgc_tsin(x-270);
    else if (x>180) return -bgc_tsin(270-x);
    else if (x>90) return -bgc_tsin(x-90);
    else return bgc_tsin(90-x);
}
/* end fast cosine for input 0->360 deg*/



/* begin test atan for input 0->+oo  */
double bgc_tatan(double x)
{
#define         a0          0.25
#define         a1          0.63611725
#define         a3          -0.817719
#define         a5          1.48697
#define         a7          -1.57588
#define         pi          3.14159265358
    double x1,x2, x3,x5,x7;
    x1=0.5*(x-1)/(x+1);
    x2=x1*x1;
    x3=x2*x1;
    x5=x3*x2;
    x7=x5*x2;
    double y;
    y = a0 + (a1*x1) + (a3*x3) + (a5*x5) + (a7*x7);

#undef a0
#undef a1
#undef a3
#undef a5
#undef a7

    return pi*y;
}
/* end test atan for input 0-> +oo */



/* begin fast atan2 for 2 inputs  -oo-> +oo */
/* return value in range -pi -> pi */
double bgc_fatan2(double y, double x)
{
    if(x>0&&y>=0) return bgc_tatan(y/x);
    else if (x>0&&y<0) return -bgc_tatan(-y/x);
    else if (x<0&&y<0) return -bgc_tatan(y/x)-pi;
    else return  -bgc_tatan(-y/x)+pi;

}
/* end fast atan2 for 2 inputs  -oo-> +oo */


void bgc_bldchdl(volatile BLDC *bldc0, double w0)
{
    bldc0->w = w0;
    bldc0->Vref = w0*(bldc0->Kf)+ bldc0->Voffset;
    double V= (bldc0->Vref)/VDC;
    V=fabs(V);
    if(V>VMAX) V=VMAX;
    /* update BLDC's angle */
    bldc0->angle += (bldc0->w)*( bldc0->dphi);
    /* angle format */
    if(bldc0->angle  >= 360) bldc0->angle  -= 360; 
    if(bldc0->angle <0) bldc0->angle  +=360;
    int deg0 = (int)(bldc0->angle) ;
    /* cal Va, Vb*/
    bldc0->Va = V* bgc_cos(deg0);
    bldc0->Vb = V* bgc_sin(deg0);
}

/* Generate bldc0 duty cycle */
/* CTR : max counter value of PWM chanel */
/* BLDC : used motor */
void bgc_SVPWM(volatile BLDC *bldc0)
{
    double Va = bldc0->Va;
    double Vb = bldc0->Vb;
    double a = fabs(Va)+0.5774*fabs(Vb);
    double b = fabs(Va)-0.5774*fabs(Vb);
    double c = 1.1547*fabs(Vb);
    int buff=0;
    int CTR=bldc0->CTR;
    if(Vb < 0) buff+=100;
    if(Va < 0) buff+=10;
    if(b < 0) buff+=1;
    
    switch (buff)
    {
        case 0:     /*S1*/
            bldc0->duty1 = CTR*(1+a); 
            bldc0->duty2 = CTR*(1-a+2*c);
            bldc0->duty3 = CTR*(1-a);
        break;
        case 1:     /*S2/Q1*/
            bldc0->duty1 = CTR*(1+2*b+c);
            bldc0->duty2 = CTR*(1+c);
            bldc0->duty3 = CTR*(1-c);
        break;
        case 10:    /*S3*/
            bldc0->duty1 = CTR*(1-a) ;
            bldc0->duty2 = CTR*(1+a);
            bldc0->duty3 = CTR*(1+a-2*c);
        break;
        case 11:    /*S2/Q2*/
            bldc0->duty1 = CTR*(1-2*a+c);
            bldc0->duty2 = CTR*(1+c);
            bldc0->duty3 = CTR*(1-c);
        break;
        case 100:   /*S6*/
            bldc0->duty1 = CTR*(1+a);
            bldc0->duty2 = CTR*(1-a); 
            bldc0->duty3 = CTR*(1-a+2*c);
        break;
        case 101:   /*S5/Q4*/
            bldc0->duty1 = CTR*(1+2*b+c);
            bldc0->duty2 = CTR*(1-c); 
            bldc0->duty3 = CTR*(1+c);
        break;
        case 110:   /*S4*/
            bldc0->duty1 = CTR*(1-a);
            bldc0->duty2 = CTR*(1+a-2*c); 
            bldc0->duty3 = CTR*(1+a);
        break;
        case 111:   /*S5/Q3*/
            bldc0->duty1 = CTR*(1-2*a+c);
            bldc0->duty2 = CTR*(1-c); 
            bldc0->duty3 = CTR*(1+c);
        break;
        default:
            asm("nop");
            asm("nop");

        break;
        
    }
}
/**/
void svmtest(void)
{
}
