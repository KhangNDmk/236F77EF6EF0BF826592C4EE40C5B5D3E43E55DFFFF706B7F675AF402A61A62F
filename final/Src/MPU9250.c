/*
 * MPU9250.c
 *
 *  Created on: Dec 16, 2018
 *      Author: MK
 */

#include "MPU9250.h"
#include "MPU6050.h"
#include "stm32f4xx_hal_i2c.h"
#include <string.h>

/**/
//int flag = 1;

HAL_StatusTypeDef status; // flag to check transaction complete
uint8_t ref[14];          // check data that have been read
/**/

void MPU9250_param(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{

}

void MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250 *mpu, Ascale Ascale0, Gscale Gscale0, Mscale Mscale0)
{
//    iscalibrated = 0;
    uint8_t temp;

    /**/
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, &temp, 1);
    HAL_Delay(100);

    // get stable time source
    temp = 0x01; // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, &temp, 1);
    /**/

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    temp = 0x03;
    i2cWrite(hi2c, MPU9250_ADDRESS, CONFIG, &temp, 1);


    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    temp = 0x04;// Use a 200 Hz rate; the same rate set in CONFIG above
    i2cWrite(hi2c, MPU9250_ADDRESS, SMPLRT_DIV, &temp, 1);


    // config gyroscope
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    i2cRead(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, &temp, 1);
    temp = temp & 0xFD;
    temp = temp & 0xE7;
    temp = temp | ((uint8_t)Gscale0 << 3);
    i2cWrite(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, &temp, 1);


    // config accelerometer
    // Set accelerometer full-scale range configuration
    i2cRead(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, &temp, 1);
    temp = temp & 0xE7;
    temp = temp | ((uint8_t)Ascale0 << 3);
    i2cWrite(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, &temp, 1);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    i2cRead(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2, &temp, 1);
    temp = temp & 0xF0;
    temp = temp |0x03;
    i2cWrite(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2, &temp, 1);

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    temp=0x22;
    i2cWrite(hi2c, MPU9250_ADDRESS, INT_PIN_CFG, &temp, 1);

    temp=0x01;
    i2cWrite(hi2c, MPU9250_ADDRESS, INT_ENABLE, &temp, 1);

    switch (Ascale0)
    {
    case AFS_2G:
          mpu->Acc_factor = 2.0/32768.0;
          break;
    case AFS_4G:
        mpu->Acc_factor = 4.0/32768.0;
          break;
    case AFS_8G:
        mpu->Acc_factor = 8.0/32768.0;
          break;
    case AFS_16G:
        mpu->Acc_factor = 16.0/32768.0;
          break;
    }

    switch (Gscale0)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
        mpu->Gyro_factor = 250.0/32768.0;
          break;
    case GFS_500DPS:
        mpu->Gyro_factor = 500.0/32768.0;
          break;
    case GFS_1000DPS:
        mpu->Gyro_factor = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
        mpu->Gyro_factor = 2000.0/32768.0;
          break;
    }

    switch (Mscale0)
    {
      // Possible magnetometer scales (and their register bit settings) are:
      // 14 bit resolution (0) and 16 bit resolution (1)
      case MFS_14BITS:
          mpu->Mag_factor = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
            break;
      case MFS_16BITS:
          mpu->Mag_factor = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
            break;
    }
}

/**/

