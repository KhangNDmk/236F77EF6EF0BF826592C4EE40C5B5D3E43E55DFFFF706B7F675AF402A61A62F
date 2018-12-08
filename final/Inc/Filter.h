#ifndef __Filter_H__
#define __Filter_H__
#ifdef __cplusplus
extern "C" {
#endif

// Mahony Filter parameter
extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile double roll, pitch, yaw;
extern volatile double RPY[3];
// Madgwick Filter parameter
extern volatile float beta;
// declare quaternion variable
extern volatile double  q0,q1,q2,q3;

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
// Inverse Sqrt Fast
float invSqrt(float x);
void qua2Euler(void);

#ifdef __cplusplus
}
#endif
#endif
