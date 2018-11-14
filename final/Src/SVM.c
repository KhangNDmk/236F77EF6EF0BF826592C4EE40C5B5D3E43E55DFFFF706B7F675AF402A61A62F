/*  * Space Vector lib for BGC
    * Author: KhangND
    * V1:
    
    * Created 30/10/2018
*/

#include "SVM.h"
#include "BLDC.h"
#include "math.h"
static const float sine_table[91]={0,0.0175,0.0349,0.0523,0.0698,0.0872,0.1045,0.1219,0.1392,0.1564,0.1736,
                                    0.1908,0.2079,0.2250,0.2419,0.2588,0.2756,0.2924,0.3090,0.3256,0.3420,
                                    0.3584,0.3746,0.3907,0.4067,0.4226,0.4384,0.4540,0.4695,0.4848,0.5000,
                                    0.5150,0.5299,0.5446,0.5592,0.5736,0.5878,0.6018,0.6157,0.6293,0.6428,
                                    0.6561,0.6691,0.6820,0.6947,0.7071,0.7193,0.7314,0.7431,0.7547,0.7660,
                                    0.7771,0.7880,0.7986,0.8090,0.8192,0.8290,0.8387,0.8480,0.8572,0.8660,
                                    0.8746,0.8829,0.8910,0.8988,0.9063,0.9135,0.9205,0.9272,0.9336,0.9397,
                                    0.9455,0.9511,0.9563,0.9613,0.9659,0.9703,0.9744,0.9781,0.9816,0.9848,
                                    0.9877,0.9903,0.9925,0.9945,0.9962,0.9976,0.9986,0.9994,0.9998,1.0000
};

/**/

/* Calculate sin's value from sine_table*/
float bgc_sin(int deg)
{
    if(deg<91) return sine_table[deg];
    else if (deg <181) return sine_table[180-deg];
    else if (deg < 271) return (- sine_table[deg-180]);
    else return ( - sine_table[360-deg]) ;
}

/* Calculate cosin's value from sine_table */
float bgc_cos(int deg)
{
    if (deg < 91) return sine_table[90-deg];
    else if (deg < 181) return (- sine_table[deg-90]);
    else if (deg < 271) return (- sine_table[270-deg]);
    else return sine_table[deg-270];
}

/**/
void bgc_bldchdl(volatile BLDC *bldc0, float w0)
{
    bldc0->w = w0;
    bldc0->Vref = w0*(bldc0->Kf)+ bldc0->Voffset;
    float V= (bldc0->Vref)/VDC;
    V=fabs(V);
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

/*  * Generate bldc0 duty cycle 
    * 
*/
void bgc_SVPWM(volatile BLDC *bldc0, int CTR)
{
    float Va = bldc0->Va;
    float Vb = bldc0->Vb;
    float a = fabs(Va)+0.5774*fabs(Vb);
    float b = fabs(Va)-0.5774*fabs(Vb);
    float c = 1.1547*fabs(Vb);
    int buff=0;
    
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
