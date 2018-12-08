/*
 * IMU_filter.c
 *
 *  Created on: Dec 8, 2018
 *      Author: MK
 */

/*Noi dung Filter.c */
#include "IMU_filter.h"
#include "math.h"
#include "MPU6050.h"
#include "Filter.h"

void MPU6050_filter_Init(Madgwick_6DOFparam *p0)
{
    p0->q0 = 1.0f;
    p0->q1 = 0.0f;
    p0->q2 = 0.0f;
    p0->q3 = 0.0f;
}

void MPU6050_filter(MPU6050 *m0, Madgwick_6DOFparam *p0, RPY_filter* rpy0 , float beta0, float sampleFreq)
{
    // Rate of change of quaternion from gyroscope
    p0->qDot1 = 0.5f * (-p0->q1 * m0->GyroX - p0->q2 * m0->GyroY - p0->q3 * m0->GyroZ);
    p0->qDot2 = 0.5f * (p0->q0 * m0->GyroX + p0->q2 * m0->GyroZ - p0->q3 * m0->GyroY);
    p0->qDot3 = 0.5f * (p0->q0 * m0->GyroY - p0->q1 * m0->GyroZ + p0->q3 * m0->GyroX);
    p0->qDot4 = 0.5f * (p0->q0 * m0->GyroZ + p0->q1 * m0->GyroY - p0->q2 * m0->GyroX);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((m0->AccX == 0.0f) && (m0->AccY == 0.0f) && (m0->AccZ == 0.0f)))
    {
        // Normalise accelerometer measurement
        p0->recipNorm = invSqrt(m0->AccX * m0->AccX + m0->AccY * m0->AccY + m0->AccZ * m0->AccZ);
        m0->AccX *= p0->recipNorm;
        m0->AccY *= p0->recipNorm;
        m0->AccZ *= p0->recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        p0->_2q0 = 2.0f * p0->q0;
        p0->_2q1 = 2.0f * p0->q1;
        p0->_2q2 = 2.0f * p0->q2;
        p0->_2q3 = 2.0f * p0->q3;
        p0->_4q0 = 4.0f * p0->q0;
        p0->_4q1 = 4.0f * p0->q1;
        p0->_4q2 = 4.0f * p0->q2;
        p0->_8q1 = 8.0f * p0->q1;
        p0->_8q2 = 8.0f * p0->q2;
        p0->q0q0 = p0->q0 * p0->q0;
        p0->q1q1 = p0->q1 * p0->q1;
        p0->q2q2 = p0->q2 * p0->q2;
        p0->q3q3 = p0->q3 * p0->q3;

        // Gradient decent algorithm corrective step
        p0->s0 = p0->_4q0 * p0->q2q2 + p0->_2q2 * m0->AccX + p0->_4q0 * p0->q1q1 - p0->_2q1 * m0->AccY;
        p0->s1 = p0->_4q1 * p0->q3q3 - p0->_2q3 * m0->AccX + 4.0f * p0->q0q0 * p0->q1 - p0->_2q0 * m0->AccY - p0->_4q1 + p0->_8q1 * p0->q1q1 + p0->_8q1 * p0->q2q2 + p0->_4q1 * m0->AccZ;
        p0->s2 = 4.0f * p0->q0q0 * p0->q2 + p0->_2q0 * m0->AccX + p0->_4q2 * p0->q3q3 - p0->_2q3 * m0->AccY - p0->_4q2 + p0->_8q2 * p0->q1q1 + p0->_8q2 * p0->q2q2 + p0->_4q2 * m0->AccZ;
        p0->s3 = 4.0f * p0->q1q1 * p0->q3 - p0->_2q1 * m0->AccX + 4.0f * p0->q2q2 * p0->q3 - p0->_2q2 * m0->AccY;
        p0->recipNorm = invSqrt(p0->s0 * p0->s0 + p0->s1 * p0->s1 + p0->s2 * p0->s2 + p0->s3 * p0->s3); // normalise step magnitude
        p0->s0 *= p0->recipNorm;
        p0->s1 *= p0->recipNorm;
        p0->s2 *= p0->recipNorm;
        p0->s3 *= p0->recipNorm;

        // Apply feedback step
        p0->qDot1 -= beta0 * p0->s0;
        p0->qDot2 -= beta0 * p0->s1;
        p0->qDot3 -= beta0 * p0->s2;
        p0->qDot4 -= beta0 * p0->s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    p0->q0 += p0->qDot1 * (1.0f / sampleFreq);
    p0->q1 += p0->qDot2 * (1.0f / sampleFreq);
    p0->q2 += p0->qDot3 * (1.0f / sampleFreq);
    p0->q3 += p0->qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    p0->recipNorm = invSqrt(p0->q0 * p0->q0 + p0->q1 * p0->q1 + p0->q2 * p0->q2 + p0->q3 * p0->q3);

    p0->q0 *= p0->recipNorm;
    p0->q1 *= p0->recipNorm;
    p0->q2 *= p0->recipNorm;
    p0->q3 *= p0->recipNorm;

    // chuyen sang RPY
    rpy0->sinR = 2.0*((double)(p0->q0*p0->q1 + p0->q2*p0->q3 ));
    rpy0->cosR = 1- 2*((double)(p0->q1*p0->q1 + p0->q2*p0->q2));

    rpy0->sinP = 2*((double)(p0->q0*p0->q2 - p0->q1*p0->q3));

    rpy0->sinY = 2*((double)(p0->q0*p0->q3 - p0->q1*p0->q2));
    rpy0->cosY = 1- 2*((double)(p0->q2*p0->q2 + p0->q3*p0->q3));

/* CAN SUA LAI CHO NAY  */
    rpy0->R = atan2(rpy0->sinR,rpy0->cosR)*57.29578;
    rpy0->P = asin(rpy0->sinP)*57.29578;
    rpy0->Y = atan2(rpy0->sinY,rpy0->cosY)*57.29578;
}

/*Ket thuc Filter.c*/
