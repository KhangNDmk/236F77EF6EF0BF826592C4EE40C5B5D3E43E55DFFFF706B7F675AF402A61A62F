/*
 * BGC.c
 *
 *  Created on: Feb 19, 2019
 *      Author: MK
 */


#include "BGC.h"
#include "i2c.h"
void BGC_init(BGC *bgc0, MPU9250 *mpu0)
{

    MPU9250_Init(&hi2c1, mpu0, AFS_4G, GFS_250DPS, MFS_16BITS);
    bgc0->mode = BGC_MODE_SET;
}



void BGC_Controller(BGC *bgc0, MPU9250 *mpu0 , int ev_user)
{
    switch (bgc0->mode) {
        case BGC_MODE_RUN:
            MPU9250_read(&hi2c1, mpu0);
            BGC_GetQm(bgc0, mpu0);
            bgc0->RM_imu = qua2rot(bgc0->q_m);
            bgc0->RM_gimbal = BGC_CalRM_gimbal(bgc0);
            if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)==GPIO_PIN_SET)
            {
                bgc0->mode = BGC_MODE_SET;
            };
            break;
        case BGC_MODE_SET:
        MPU9250_read(&hi2c1, mpu0);
        BGC_SetQref(bgc0, mpu0);
        bgc0->RM_imuRef = qua2rot(bgc0->q_r);
        if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)==GPIO_PIN_SET)
        {
            bgc0->mode = BGC_MODE_RUN;
        };
            break;
    }
}

/**/



/* begin ham tinh gia tri dat cho goc cua dong co - goc gimbal */
void BGC_CalMA_ref(BGC *bgc0)
{

}


/* begin ham doc gia tri hien tai cua goc dong co - goc gimbal */
void BGC_GetMA(BGC *bgc0)
{

}


/* ham lay huong hien tai cua MPU9250 lam huong dat cho gimbal */
void BGC_SetQref(BGC *bgc0, MPU9250 *mpu0)
{
    for (int i = 0; i < 4; ++i)
    {
        bgc0->q_r.q[i]=mpu0->q[i];
    }

    bgc0->RM_imuRef = qua2rot(bgc0->q_r);
}

/* begin ham tinh gia tri ma tran xoay cua gimbal, theo he truc ZXY */
/* chon khau Z1, X2, Y3 , huong ve phia camera*/
mat3 BGC_CalRM_gimbal(BGC *bgc0)
{
    double p1=bgc0->phi_M1;
    double p2=bgc0->phi_M2;
    double p3=bgc0->phi_M3;

    double c1=bgc_fcos(p1);
    double c2=bgc_fcos(p2);
    double c3=bgc_fcos(p3);

    double s1=bgc_fsin(p1);
    double s2=bgc_fsin(p2);
    double s3=bgc_fsin(p3);

    mat3 Rtemp;

    Rtemp.elem[0][0]=c1*c3-s1*s2*s3;
    Rtemp.elem[0][1]=-c2*s1;
    Rtemp.elem[0][2]=c1*s3+c3*s1*s2;
    Rtemp.elem[1][0]=c3*s1+c1*s2*s3;
    Rtemp.elem[1][1]=c1*c2;
    Rtemp.elem[1][2]=s1*s3-c1*c3*s2;
    Rtemp.elem[2][0]=-c2*s3;
    Rtemp.elem[2][1]=s2;
    Rtemp.elem[2][2]=c2*c3;

    return Rtemp;
}

/* ham doc gia tri hien tai cua MPU9250 lam feedback cho gimbal */
void BGC_GetQm(BGC *bgc0, MPU9250 *mpu0)
{
    for (int i = 0; i < 4; ++i)
    {
        bgc0->q_m.q[i]=mpu0->q[i];
    }
}


/* begin qua2rot */
/*chuyen quaternion sang dang ma tran xoay */
mat3 qua2rot(quaternion qua0 )
{
    double q1, q2, q3, q4;
    double q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;
    mat3 Rtemp;

    q1=qua0.q[0];
    q2=qua0.q[1];
    q3=qua0.q[2];
    q4=qua0.q[3];

    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q1q4 = q1 * q4;

    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q2q4 = q2 * q4;

    q3q3 = q3 * q3;
    q3q4 = q3 * q4;

    q4q4 = q4 * q4;

        Rtemp.elem[0][0] = 2 * (q1q1 + q2q2) - 1;    /*DMC(1,1)=2*(q1^2+q2^2)-1    */
        Rtemp.elem[0][1] = 2 * (q1q4 + q2q3);        /*DMC(1,2)=2*(q1*q4+q2*q3)    */
        Rtemp.elem[0][2] = 2 * (q2q4 - q1q3);        /*DMC(1,3)=2*(q2*q4-q1*q3)    */
        Rtemp.elem[1][0] = 2 * (q2q3 - q1q4);        /*DMC(2,1)=2*(q2*q3-q1*q4)    */
        Rtemp.elem[1][1] = 2 * (q1q1 + q3q3) - 1;    /*DMC(2,2)=2*(q1^2+q3^2)-1    */
        Rtemp.elem[1][2] = 2 * (q1q2 + q3q4);        /*DMC(2,3)=2*(q1*q2+q3*q4)    */
        Rtemp.elem[2][0] = 2 * (q1q3 + q2q4);        /*DMC(3,1)=2*(q1*q3+q2*q4)    */
        Rtemp.elem[2][1] = 2 * (q3q4 - q1q2);        /*DMC(3,2)=2*(q3*q4-q1*q2)    */
        Rtemp.elem[2][2] = 2 * (q1q1 + q4q4) - 1;    /*DMC(3,3)=2*(q1^2+q4^2)-1    */

        return Rtemp;
}
/* end qua2rot */

/**/


