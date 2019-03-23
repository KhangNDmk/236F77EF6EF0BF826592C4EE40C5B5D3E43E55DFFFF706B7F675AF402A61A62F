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

    bgc0->mode = BGC_MODE_RUN;
}



void BGC_Controller(BGC *bgc0, int ev_user)
{
    switch (bgc0->mode) {
        case BGC_MODE_RUN:

            break;
        case BGC_MODE_INIT:

            break;
        case BGC_MODE_CALIB:

            break;

        default:
            break;
    }
}

void BGC_CalMotorAngle(BGC *bgc0)
{

}

void BGC_CalRM_imuRef(BGC *bgc0)
{

}

void BGC_SetMotorAngle(BGC *bgc0)
{

}

/* ham lay gia tri hien tai cua MPU9250 lam gia tri dat cho gimbal */
void BGC_SetQref(BGC *bgc0, MPU9250 *mpu0)
{
    for (int i = 0; i < 4; ++i)
    {
        bgc0->q_r.q[i]=mpu0->q[i];
    }
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
    mat3 R;

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

        R.elem[0][0] = 2 * (q1q1 + q2q2) - 1;    /*DMC(1,1)=2*(q1^2+q2^2)-1    */
        R.elem[0][1] = 2 * (q1q4 + q2q3);        /*DMC(1,2)=2*(q1*q4+q2*q3)    */
        R.elem[0][2] = 2 * (q2q4 - q1q3);        /*DMC(1,3)=2*(q2*q4-q1*q3)    */
        R.elem[1][0] = 2 * (q2q3 - q1q4);        /*DMC(2,1)=2*(q2*q3-q1*q4)    */
        R.elem[1][1] = 2 * (q1q1 + q3q3) - 1;    /*DMC(2,2)=2*(q1^2+q3^2)-1    */
        R.elem[1][2] = 2 * (q1q2 + q3q4);        /*DMC(2,3)=2*(q1*q2+q3*q4)    */
        R.elem[2][0] = 2 * (q1q3 + q2q4);        /*DMC(3,1)=2*(q1*q3+q2*q4)    */
        R.elem[2][1] = 2 * (q3q4 - q1q2);        /*DMC(3,2)=2*(q3*q4-q1*q2)    */
        R.elem[2][2] = 2 * (q1q1 + q4q4) - 1;    /*DMC(3,3)=2*(q1^2+q4^2)-1    */

        return R;
}
/* end qua2rot */

/**/


