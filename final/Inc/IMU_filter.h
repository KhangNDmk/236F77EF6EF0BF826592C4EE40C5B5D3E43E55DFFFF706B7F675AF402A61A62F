/*
 * IMU_filter.h
 *
 *  Created on: Dec 8, 2018
 *      Author: MK
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Noi dung header */
#include "MPU6050.h"
/**/
typedef struct
{
    float recipNorm;
    float q0, q1, q2, q3;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
}Madgwick_6DOFparam;

typedef struct
{
    double R, P, Y;
    double sinR, cosR, sinP, cosY, sinY;
}RPY_filter;

void MPU6050_filter_Init(Madgwick_6DOFparam *p0);
void MPU6050_filter(MPU6050 *m0, Madgwick_6DOFparam *p0, RPY_filter* rpy0 , float beta0, float sampleFreq);

/*Ket thuc noi dung header*/
#ifdef __cplusplus
}
#endif
#endif /* IMU_FILTER_H_ */
