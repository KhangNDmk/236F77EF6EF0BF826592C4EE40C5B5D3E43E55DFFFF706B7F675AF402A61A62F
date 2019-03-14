/*
 * MPU9250.h
 *
 *  Created on: Dec 16, 2018
 *      Author: MK
 */

#ifndef MPU9250_H_
#define MPU9250_H_
#ifdef __cplusplus
extern "C"
{
#endif
/* Noi dung header */
#include "main.h"
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"
//#include "MPU6050.h"
#include <string.h>
#include "Filter.h"

/**/
double e_m,sf;//filter efficient, magnitude error, scale factor, acc norm
#define alphaAcc 0.004f
#define alphaMag 0.0004f
#define alphaDef  0.01f
//volatile float alpha = alphaDef;

/**/
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

/**/
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif
/**/
// Set initial input parameters
typedef enum
{
    AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
} Ascale;

typedef enum
{
    GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
} Gscale;

typedef enum
{
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
} Mscale;

//uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
//uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
//uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
//uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

/**/
typedef struct
{
    uint8_t ascale;
    uint8_t gscale;
    uint8_t mscale;
    float Gyro_factor, Acc_factor, Mag_factor;

    int16_t AccX_raw, AccY_raw, AccZ_raw;
    float AccX_offset, AccY_offset, AccZ_offset;
    float AccX, AccY, AccZ;

    int16_t GyroX_raw, GyroY_raw, GyroZ_raw;
    float GyroX, GyroY, GyroZ;
    float GyroX_offset, GyroY_offset, GyroZ_offset;

    int16_t MagX_raw, MagY_raw, MagZ_raw;
    float MagCalibX, MagCalibY, MagCalibZ;
    float MagX_calibscale, MagY_calibscale, MagZ_calibscale;
    float MagX, MagY, MagZ;
    float MagX_offset, MagY_offset, MagZ_offset;

    float q[4];
    float roll, pitch, yaw;
    float temperature;
    int calib_done;
} MPU9250;

/**/
void i2cRead(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg,uint8_t* data, uint8_t data_length);
void i2cWrite(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg, uint8_t *data, uint8_t data_length);

void MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250 *mpu, Ascale Ascale0,
        Gscale Gscale0, Mscale Mscale0);
void MPU9250_Reset(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void MPU9250_calib(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void MPU9250_Init8963(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void MPU9250_SetParam(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);

void MPU9250_readAcc(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void MPU9250_readMag(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void MPU9250_readGyro(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void Magnet_Calib(I2C_HandleTypeDef *hi2c, MPU9250*mpu);
void MPU9250_Madgwick(MPU9250 *mpu);
void MPU9250_read(I2C_HandleTypeDef *hi2c, MPU9250 *mpu);
void Myfilter9DOF(MPU9250* mpu);
/*Ket thuc noi dung header*/
#ifdef __cplusplus
}
#endif
#endif /* MPU9250_H_ */
