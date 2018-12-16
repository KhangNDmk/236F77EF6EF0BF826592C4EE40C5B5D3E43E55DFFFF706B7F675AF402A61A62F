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

/* khoi tao tham so bo loc Madgwick*/
void MPU6050_filter_Init(Madgwick_6DOFparam *p0)
{
    p0->q0 = 1.0f;
    p0->q1 = 0.0f;
    p0->q2 = 0.0f;
    p0->q3 = 0.0f;
}
/* het hamf khoi tao madgwick*/

/*Doc thong so MPU*/
void MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050 *mpu)
{
    uint8_t data[14];
    int i=0;
    uint32_t data32[14];
    i2cRead(hi2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, data, 14);

    for(i=0; i<14; i++)
    {
        data32[i]=data[i];
    }
    /* read and format Acc parameter */
    mpu->AccX_raw = (data32[0]<<8)+data32[1];
    mpu->AccY_raw = (data32[2]<<8)+data32[3];
    mpu->AccZ_raw = (data32[4]<<8)+data32[5];

    mpu->AccX = (float)mpu->AccX_raw * mpu->Acc_factor;
    mpu->AccY = (float)mpu->AccY_raw * mpu->Acc_factor;
    mpu->AccZ = (float)mpu->AccZ_raw * mpu->Acc_factor;

    /* Format gyroscope data */
    mpu->GyroX_raw = (data32[8]<<8)+data32[9];
    mpu->GyroY_raw = (data32[10]<<8)+data32[11];
    mpu->GyroZ_raw = (data32[12]<<8)+data32[13];

    /* tru offset*/
    mpu->GyroX = ((float)mpu->GyroX_raw - mpu->GyroX_offset)*mpu->Gyro_factor;
    mpu->GyroY = ((float)mpu->GyroY_raw - mpu->GyroY_offset)*mpu->Gyro_factor;
    mpu->GyroZ = ((float)mpu->GyroZ_raw - mpu->GyroZ_offset)*mpu->Gyro_factor;
}
/*Ket thuc ham doc thong so MPU*/

/**/
void MPU6050_ReadOffset(I2C_HandleTypeDef *hi2c, MPU6050 *mpu)
{
    uint8_t data[14];
    int i,j;
    uint32_t data32[14];
    mpu->GyroX_offset = 0;
    mpu->GyroY_offset = 0;
    mpu->GyroZ_offset = 0;

    for(i=0; i<100; i++)
    {
        i2cRead(hi2c, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, data, 14);
        for(j=0; j<14; j++)
        {
            data32[j]=data[j];
        }
        mpu->GyroX_offset += ((data32[8] << 8) + data32[9]);
        mpu->GyroY_offset += ((data32[10] << 8)+ data32[11]);
        mpu->GyroZ_offset += ((data32[12] << 8) + data32[13]);
        HAL_Delay(5);
    }

    mpu->GyroX_offset *= 0.01;
    mpu->GyroY_offset *= 0.01;
    mpu->GyroZ_offset *= 0.01;
}
/**/

/**/
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

    // chuyen sang RPY,
    //Can kiem tra lai
    rpy0->sinR = 2.0*((double)(p0->q0*p0->q1 + p0->q2*p0->q3 ));
    rpy0->cosR = 1- 2*((double)(p0->q1*p0->q1 + p0->q2*p0->q2));

    rpy0->sinP = 2*((double)(p0->q0*p0->q2 - p0->q1*p0->q3));

    rpy0->sinY = 2*((double)(p0->q0*p0->q3 + p0->q1*p0->q2));
    rpy0->cosY = 1- 2*((double)(p0->q2*p0->q2 + p0->q3*p0->q3));

/* CAN SUA LAI CHO NAY  */
    rpy0->R = atan2(rpy0->sinR,rpy0->cosR)*57.29578;
    rpy0->P = asin(rpy0->sinP)*57.29578;
    rpy0->Y = atan2(rpy0->sinY,rpy0->cosY)*57.29578;
}

/*Ket thuc Filter.c*/
