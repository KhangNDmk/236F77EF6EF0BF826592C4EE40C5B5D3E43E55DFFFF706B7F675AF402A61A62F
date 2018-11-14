#ifndef __MPU6050_H__
#define __MPU6050_H__
#ifdef __cplusplus
extern "C" {
#endif

// include hal library
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"

extern volatile float Gyro_offset[3];

// define mpu address
#define MPU6050_ADDRESS                     0xD0/* must shift left a bit due to the effect of the fucntion HAL_I2C_Mem_Read*/
#define MPU6050_WHO_AM_I_ID                 0x68
// define MPU6050 register

#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75
 


// sensitivity of gyro in degree/s
#define MPU6050_GYRO_SENS_250		((float) 131.072)
#define MPU6050_GYRO_SENS_500		((float) 65.536)
#define MPU6050_GYRO_SENS_1000		((float) 32.768)
#define MPU6050_GYRO_SENS_2000		((float) 16.384)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)
// utility function 

typedef struct {
	
	float Gyro_factor;
	float Acc_factor;
	float AccX;
	float AccY;
	float AccZ;
	float GyroX;
	float GyroY;
	float GyroZ;
	float temperature;
}MPU6050;

// Acc sensitivity config (check reg 28)

typedef enum{
	Acc_2G = 0x00,
	Acc_4G = 0x01,
	Acc_8G = 0x02,
	Acc_16G = 0x03
	
}Acc_Set_Sense;

// Gyro sensitivity config  (Check reg 27)
typedef enum{
	Gyro_250s = 0x00,
	Gyro_500s = 0x01,
	Gyro_1000s = 0x02,
	Gyro_2000s = 0x03
}Gyro_Set_Sense;
void i2cRead(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg,uint8_t* data, uint8_t data_length);
void i2cWrite(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg, uint8_t *data, uint8_t data_length);
void MPU6050_Init(I2C_HandleTypeDef *hi2c,MPU6050 *mpu,  Acc_Set_Sense AccSensitivity, Gyro_Set_Sense GyroSensitivity);

void MPU6050_ReadAcc(I2C_HandleTypeDef *hi2c,MPU6050 *mpu);
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,MPU6050 *mpu);
void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c,MPU6050 *mpu);
void calib(I2C_HandleTypeDef *hi2c, MPU6050 *mpu);
#ifdef __cplusplus
}
#endif
#endif
