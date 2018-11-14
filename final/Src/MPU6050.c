#include "MPU6050.h"
#include "stm32f4xx_hal_i2c.h"
#include<string.h>
volatile float Gyro_offset[3]={0,0,0};
int iscalibrated=0; // check whether imu is calibrated or not
int calib_count;
int flag = 1;
uint8_t buffer[2];// buffer to transfer to i2c
HAL_StatusTypeDef status;// flag to check transaction complete
uint8_t ref[14]; // check data that have been read
void i2cWrite(I2C_HandleTypeDef *hi2c,uint16_t address, uint16_t reg, uint8_t*data, uint8_t data_length)
{
	status = HAL_OK;
	flag =1;
	status = HAL_I2C_Mem_Write(hi2c,address, reg,I2C_MEMADD_SIZE_8BIT,data,data_length,1000);
	if(status != HAL_OK)
	{
		flag = 0;
	}
}



void i2cRead(I2C_HandleTypeDef*hi2c, uint16_t address, uint16_t reg,uint8_t* data, uint8_t data_length)
{
  status = HAL_OK;
	flag = 1;
	status = HAL_I2C_Mem_Read(hi2c,address,reg,I2C_MEMADD_SIZE_8BIT,data,data_length,1000);
	if(status!=HAL_OK)
	{
		flag =0;
	}
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c,MPU6050 *mpu, Acc_Set_Sense AccSensitivity, Gyro_Set_Sense GyroSensitivity)
{
	iscalibrated = 0;
	uint8_t temp;
	while(HAL_I2C_IsDeviceReady(hi2c,MPU6050_ADDRESS,1,100));
	i2cRead(hi2c,MPU6050_ADDRESS,MPU6050_WHO_AM_I,&temp,1);
//while(temp==MPU6050_WHO_AM_I);
	buffer[0] = 0x00;
	
	
	i2cWrite(hi2c,MPU6050_ADDRESS,MPU6050_PWR_MGMT_1,buffer,1);
	//config samping rate
	buffer[0]=0x01;  //Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	buffer[1]=0x03;  // Disable FSYNC and set Gyro to 41kHz filtering and 1kHz sampling
	i2cWrite(hi2c,MPU6050_ADDRESS,MPU6050_SMPLRT_DIV,buffer,2);
	
	// config accelerometer
	i2cRead(hi2c,MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG,&temp,1);
	temp = (temp&0xE7)|(uint8_t)AccSensitivity<<3;
	i2cWrite(hi2c,MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG,&temp,1);
	
	// config gyroscope
	i2cRead(hi2c,MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,&temp,1);
	temp = (temp&0xE7)|(uint8_t)GyroSensitivity<<3;
	i2cWrite(hi2c,MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,&temp,1);
	switch (AccSensitivity) {
		case Acc_2G:
			mpu->Acc_factor = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case Acc_4G:
			mpu->Acc_factor = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case Acc_8G:
			mpu->Acc_factor = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case Acc_16G:
			mpu->Acc_factor = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	switch (GyroSensitivity) {
		case Gyro_250s:
			mpu->Gyro_factor = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case Gyro_500s:
			mpu->Gyro_factor = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case Gyro_1000s:
			mpu->Gyro_factor = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case Gyro_2000s:
			mpu->Gyro_factor = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
}


void MPU6050_ReadAcc(I2C_HandleTypeDef *hi2c,MPU6050 *mpu)
{
	uint8_t data[6];
	i2cRead(hi2c,MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H, data, 6);
	mpu->AccX = (int16_t)(data[0] << 8 | data[1]);	
	mpu->AccY = (int16_t)(data[2] << 8 | data[3]);
	mpu->AccZ = (int16_t)(data[4] << 8 | data[5]);
}
void MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,MPU6050 *mpu)
{
	uint8_t data[6];
	i2cRead(hi2c,MPU6050_ADDRESS,MPU6050_GYRO_XOUT_H, data, 6);
	mpu->GyroX = (int16_t)(data[0] << 8 | data[1]);	
	mpu->GyroY = (int16_t)(data[2] << 8 | data[3]);
	mpu->GyroZ = (int16_t)(data[4] << 8 | data[5]);
}

 void MPU6050_ReadAll(I2C_HandleTypeDef *hi2c,MPU6050 *mpu)
{
	uint8_t data[14];
	int16_t temp;
	
	i2cRead(hi2c,MPU6050_ADDRESS,MPU6050_ACCEL_XOUT_H, data, 14);
	mpu->AccX = (float)((int16_t)(data[0] << 8 | data[1])) *mpu->Acc_factor ;	
	mpu->AccY = (float)((int16_t)(data[2] << 8 | data[3])) * mpu->Acc_factor ;
	mpu->AccZ =  (float)((int16_t)(data[4] << 8 | data[5])) * mpu->Acc_factor ;
	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	mpu->temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	/* Format gyroscope data */
	mpu->GyroX = (float)((int16_t)(data[8] << 8 | data[9])) * mpu->Gyro_factor - Gyro_offset[0] * iscalibrated ;
	mpu->GyroY = (float)((int16_t)(data[10] << 8 | data[11])) * mpu->Gyro_factor - Gyro_offset[1] * iscalibrated;
	mpu->GyroZ = (float)((int16_t)(data[12] << 8 | data[13])) * mpu->Gyro_factor - Gyro_offset[2] * iscalibrated;
	memcpy(&ref,&data,14);
}

// calculate offset of imu
void calib(I2C_HandleTypeDef *hi2c, MPU6050 *mpu)
{
	for(calib_count=0;calib_count<200;calib_count++)
	{
		MPU6050_ReadAll(hi2c,mpu);
		Gyro_offset[0] += (float)mpu->GyroX/200;
		Gyro_offset[1] += (float)mpu->GyroY/200;
		Gyro_offset[2] += (float)mpu->GyroZ/200;
	}
	iscalibrated =1;
}

