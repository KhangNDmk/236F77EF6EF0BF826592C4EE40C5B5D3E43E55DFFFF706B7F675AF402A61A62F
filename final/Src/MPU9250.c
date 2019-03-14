/*
 * MPU9250.c
 *
 *  Created on: Dec 16, 2018
 *      Author: MK
 */

#include "MPU9250.h"

float avg_rad;
/**/
//int flag = 1;
//HAL_StatusTypeDef status; // flag to check transaction complete
//uint8_t ref[14];          // check data that have been read
/*begin MPU9250_read*/
void MPU9250_read(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    MPU9250_readGyro(hi2c, mpu);
    MPU9250_readAcc(hi2c, mpu);
    MPU9250_readMag(hi2c, mpu);
    MPU9250_Madgwick(mpu);
//    Myfilter9DOF(mpu);
}
/*end MPU9250_read*/

/*begin MPU9250_Reset*/
void MPU9250_Reset(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    uint8_t temp;
    temp = 0x80;
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, &temp, 1);
    HAL_Delay(10);
}
/*end MPU9250_Reset*/

/*begin MPU9250_readgyro*/
void MPU9250_readGyro(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    uint8_t rawData[6];
    // Read the six raw data registers into data array
    i2cRead(hi2c, MPU9250_ADDRESS, GYRO_XOUT_H, &rawData[0], 6);
    // Turn the MSB and LSB into a signed 16-bit value
    mpu->GyroX_raw = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
    mpu->GyroY_raw = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
    mpu->GyroZ_raw = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

    // Now we'll calculate the accleration value into actual g's
    mpu->GyroX = (float) mpu->GyroX_raw * mpu->Gyro_factor - mpu->GyroX_offset;
    mpu->GyroY = (float) mpu->GyroY_raw * mpu->Gyro_factor - mpu->GyroY_offset;
    mpu->GyroZ = (float) mpu->GyroZ_raw * mpu->Gyro_factor - mpu->GyroZ_offset;
}
/*end MPU9250_readgyro */

/*begin MPU9250_readAcc*/
void MPU9250_readAcc(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    uint8_t rawData[6];
    // Read the six raw data registers into data array
    i2cRead(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H, &rawData[0], 6);
    // Turn the MSB and LSB into a signed 16-bit value
    mpu->AccX_raw = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]);
    mpu->AccY_raw = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
    mpu->AccZ_raw = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

    // get actual g value, this depends on scale being set
    mpu->AccX = (float) mpu->AccX_raw * mpu->Acc_factor - mpu->AccX_offset;
    mpu->AccY = (float) mpu->AccY_raw * mpu->Acc_factor - mpu->AccY_offset;
    mpu->AccZ = (float) mpu->AccZ_raw * mpu->Acc_factor - mpu->AccZ_offset;
}
/*end MPU9250_readAcc*/

/*begin MPU9250_readMag*/
void MPU9250_readMag(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    uint8_t rawData[7];
    uint8_t temp;

    i2cRead(hi2c, AK8963_ADDRESS, AK8963_ST1, &temp, 1);

    if (temp & 0x01)
    {
        i2cRead(hi2c, AK8963_ADDRESS, AK8963_XOUT_L, &rawData[0], 7);
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if (!(c & 0x08)) // Check if magnetic sensor overflow set, if not then report data
        {
            mpu->MagX_raw = (int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
            mpu->MagY_raw = (int16_t) (((int16_t) rawData[3] << 8) | rawData[2]); // Data stored as little Endian
            mpu->MagZ_raw = (int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
        }
    }

        if(mpu->calib_done==1)
        {
        	mpu->MagX = (float) mpu->MagX_raw * mpu->Mag_factor * mpu->MagCalibX * mpu->MagX_calibscale - mpu->MagX_offset;
        	mpu->MagY = (float) mpu->MagY_raw * mpu->Mag_factor * mpu->MagCalibY * mpu->MagY_calibscale - mpu->MagY_offset;
        	mpu->MagZ = (float) mpu->MagZ_raw * mpu->Mag_factor * mpu->MagCalibZ * mpu->MagZ_calibscale - mpu->MagZ_offset;
        }
        else
        {
        	mpu->MagX = (float) mpu->MagX_raw * mpu->Mag_factor * mpu->MagCalibX;
        	mpu->MagY = (float) mpu->MagY_raw * mpu->Mag_factor * mpu->MagCalibY;
        	mpu->MagZ = (float) mpu->MagZ_raw * mpu->Mag_factor * mpu->MagCalibZ;
        }


}
/*end MPU9250_readMag*/

void Magnet_Calib(I2C_HandleTypeDef *hi2c, MPU9250*mpu)
{
	uint16_t i = 0, sample_count = 500;
	int32_t mag_bias[3] = {0,0,0};
	int32_t mag_scale[3] = {0,0,0};
	int32_t mag_max[3] = {-32767, -32767, -32767};
	int32_t mag_min[3] = {32767, 32767, 32767};
	//HAL_Delay(4000);
	for(i = 0; i<sample_count; i++)
	{
		MPU9250_readMag(hi2c, mpu);
		if(mpu->MagX_raw>mag_max[0])
			mag_max[0] = mpu->MagX_raw;
		if(mpu->MagY_raw>mag_max[1])
			mag_max[1] = mpu->MagY_raw;
		if(mpu->MagZ_raw>mag_max[2])
			mag_max[2] = mpu->MagZ_raw;




		if(mpu->MagX_raw<mag_min[0])
			mag_min[0] = mpu->MagX_raw;
		if(mpu->MagY_raw<mag_min[1])
			mag_min[1] = mpu->MagY_raw;
		if(mpu->MagZ_raw<mag_min[2])
			mag_min[2] = mpu->MagZ_raw;
		HAL_Delay(12);
	}
	mag_bias[0] = (mag_max[0] + mag_min[0])/2;
	mag_bias[1] = (mag_max[1] + mag_min[1])/2;
	mag_bias[2] = (mag_max[2] + mag_min[2])/2;
	mpu->MagX_offset = mag_bias[0] * mpu->Mag_factor * mpu->MagCalibX;
	mpu->MagY_offset = mag_bias[1] * mpu->Mag_factor * mpu->MagCalibY;
	mpu->MagZ_offset = mag_bias[2] * mpu->Mag_factor * mpu->MagCalibZ;

	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;
	avg_rad = mag_scale[0]+mag_scale[1]+mag_scale[2];
	avg_rad/=3.0;
	mpu->MagX_calibscale = avg_rad/((float)mag_scale[0]);
	mpu->MagY_calibscale = avg_rad/((float)mag_scale[1]);
	mpu->MagZ_calibscale = avg_rad/((float)mag_scale[2]);
	mpu->calib_done = 1;

}
/*begin MPU9250_calib*/
void MPU9250_calib(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    uint8_t temp;
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] =
    { 0, 0, 0 }, accel_bias[3] =
    { 0, 0, 0 };
    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    MPU9250_Reset(hi2c, mpu);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    temp = 0x01;
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, &temp, 1);
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_2, &temp, 1);

    // Configure device for bias calculation
    // Disable all interrupts
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, INT_ENABLE, &temp, 1);

    // Disable FIFO
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, FIFO_EN, &temp, 1);

    // Turn on internal clock source
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, &temp, 1);

    // Disable I2C master
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, I2C_MST_CTRL, &temp, 1);

    // Disable FIFO and I2C master modes
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, USER_CTRL, &temp, 1);

    // Reset FIFO and DMP
    temp = 0x0C;
    i2cWrite(hi2c, MPU9250_ADDRESS, USER_CTRL, &temp, 1);
    HAL_Delay(10);

    // Configure MPU9250 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    temp = 0x01;
    i2cWrite(hi2c, MPU9250_ADDRESS, CONFIG, &temp, 1);
    // Set sample rate to 1 kHz
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, SMPLRT_DIV, &temp, 1);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, &temp, 1);
    // Set accelerometer full-scale to 2 g, maximum sensitivity
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, &temp, 1);

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    // Enable FIFO
    temp = 0x40;
    i2cWrite(hi2c, MPU9250_ADDRESS, USER_CTRL, &temp, 1);
    // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
    temp = 0x78;
    i2cWrite(hi2c, MPU9250_ADDRESS, FIFO_EN, &temp, 1);
    HAL_Delay(40);

    // At end of sample accumulation, turn off FIFO sensor read
    temp = 0x00;
    i2cWrite(hi2c, MPU9250_ADDRESS, FIFO_EN, &temp, 1);
    // read FIFO sample count
    i2cRead(hi2c, MPU9250_ADDRESS, FIFO_COUNTH, &data[0], 2);
    fifo_count = ((uint16_t) data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] =
        { 0, 0, 0 };
        int16_t gyro_temp[3] =
        { 0, 0, 0 };
        // read data for averaging
        i2cRead(hi2c, MPU9250_ADDRESS, FIFO_R_W, &data[0], 12);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]);
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);
        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];

    }
    // Normalize sums to get average count biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    mpu->GyroX_offset = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    mpu->GyroY_offset = (float) gyro_bias[1] / (float) gyrosensitivity;
    mpu->GyroZ_offset = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] =
    { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
    // Read factory accelerometer trim values
    i2cRead(hi2c, MPU9250_ADDRESS, XA_OFFSET_H, &data[0], 2);
    accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];

    i2cRead(hi2c, MPU9250_ADDRESS, YA_OFFSET_H, &data[0], 2);
    accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];

    i2cRead(hi2c, MPU9250_ADDRESS, ZA_OFFSET_H, &data[0], 2);
    accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] =
    { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if (accel_bias_reg[ii] & mask)
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    mpu->AccX_offset = (float) accel_bias[0] / (float) accelsensitivity;
    mpu->AccY_offset = (float) accel_bias[1] / (float) accelsensitivity;
    mpu->AccZ_offset = (float) accel_bias[2] / (float) accelsensitivity;
}
/**/

/*begin MPU9250_Init8963*/
void MPU9250_Init8963(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t temp;
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    // Power down magnetometer
    temp = 0x00;
    i2cWrite(hi2c, AK8963_ADDRESS, AK8963_CNTL, &temp, 1);
    HAL_Delay(10);
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    temp = 0x0F;
    i2cWrite(hi2c, AK8963_ADDRESS, AK8963_CNTL, &temp, 1);
    HAL_Delay(10);

    // Read the x-, y-, and z-axis calibration values
//    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);
    i2cRead(hi2c, AK8963_ADDRESS, AK8963_ASAX, &rawData[0], 3);
    mpu->MagCalibX = (float) (rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
    mpu->MagCalibY = (float) (rawData[1] - 128) / 256.0f + 1.0f;
    mpu->MagCalibZ = (float) (rawData[2] - 128) / 256.0f + 1.0f;
    // Power down magnetometer
    temp = 0x00;
    i2cWrite(hi2c, AK8963_ADDRESS, AK8963_CNTL, &temp, 1);
    HAL_Delay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    // Set magnetometer data resolution and sample ODR
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
    temp = (mpu->mscale << 4) | 0x06;
    i2cWrite(hi2c, AK8963_ADDRESS, AK8963_CNTL, &temp, 1);
    HAL_Delay(10);
    Magnet_Calib(hi2c,mpu);
    /**/
//    mpu->MagX_offset = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
//    mpu->MagY_offset = +120.;  // User environmental x-axis correction in milliGauss
//    mpu->MagZ_offset = +125.;  // User environmental x-axis correction in milliGauss
}
/*end MPU9250_Init8963*/

/*begin MPU9250_Init*/
void MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250 *mpu, Ascale Ascale0, Gscale Gscale0, Mscale Mscale0)
{
    // Store user setting
	mpu->calib_done = 0;
    mpu->ascale = Ascale0;
    mpu->gscale = Gscale0;
    mpu->mscale = Mscale0;

    /* store default quaternion*/
    mpu->q[0] = 1.0f;
    mpu->q[1] = 0.0f;
    mpu->q[2] = 0.0f;
    mpu->q[3] = 0.0f;
    uint8_t whoami;
    /**/

    /**/
    i2cRead(hi2c, MPU9250_ADDRESS, WHO_AM_I_MPU9250, &whoami, 1);
    if (whoami == 0x71)
    {
        MPU9250_Reset(hi2c, mpu);
        MPU9250_calib(hi2c, mpu);
        HAL_Delay(20);
        MPU9250_SetParam(hi2c, mpu);
        MPU9250_Init8963(hi2c, mpu);
    }
}
/*end MPU9250_Init*/

/**/
void MPU9250_SetParam(I2C_HandleTypeDef *hi2c, MPU9250 *mpu)
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
    temp = 0x01;    // Use a 500 Hz rate; the same rate set in CONFIG above
    i2cWrite(hi2c, MPU9250_ADDRESS, SMPLRT_DIV, &temp, 1);

    // config gyroscope
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    i2cRead(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, &temp, 1);
    temp = temp & 0xFD;
    temp = temp & 0xE7;
    temp = temp | ((uint8_t) mpu->gscale << 3);
    i2cWrite(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, &temp, 1);

    // config accelerometer
    // Set accelerometer full-scale range configuration
    i2cRead(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, &temp, 1);
    temp = temp & 0xE7;
    temp = temp | ((uint8_t) mpu->ascale << 3);
    i2cWrite(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, &temp, 1);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    i2cRead(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2, &temp, 1);
    temp = temp & 0xF0;
    temp = temp | 0x03;
    i2cWrite(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2, &temp, 1);

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    temp = 0x22;
    i2cWrite(hi2c, MPU9250_ADDRESS, INT_PIN_CFG, &temp, 1);

    temp = 0x01;
    i2cWrite(hi2c, MPU9250_ADDRESS, INT_ENABLE, &temp, 1);

    switch (mpu->ascale)
    {
    case AFS_2G:
        mpu->Acc_factor = 2.0 / 32768.0;
        break;
    case AFS_4G:
        mpu->Acc_factor = 4.0 / 32768.0;
        break;
    case AFS_8G:
        mpu->Acc_factor = 8.0 / 32768.0;
        break;
    case AFS_16G:
        mpu->Acc_factor = 16.0 / 32768.0;
        break;
    }

    switch (mpu->gscale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
        mpu->Gyro_factor = 250.0 / 32768.0;
        break;
    case GFS_500DPS:
        mpu->Gyro_factor = 500.0 / 32768.0;
        break;
    case GFS_1000DPS:
        mpu->Gyro_factor = 1000.0 / 32768.0;
        break;
    case GFS_2000DPS:
        mpu->Gyro_factor = 2000.0 / 32768.0;
        break;
    }

    switch (mpu->mscale)
    {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
        mpu->Mag_factor = 10.0 * 4912.0 / 8190.0; // Proper scale to return milliGauss
        break;
    case MFS_16BITS:
        mpu->Mag_factor = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss
        break;
    }
}

void MPU9250_Madgwick(MPU9250 *mpu)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3,
            q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float roll, pitch, yaw;
//    float PI = 3.14159265358979323846f;
    float ax = mpu->AccX;
    float ay = mpu->AccY;
    float az = mpu->AccZ;
    float gx = mpu->GyroX * PI / 180.0f;
    float gy = mpu->GyroY * PI / 180.0f;
    float gz = mpu->GyroZ * PI / 180.0f;
    float mx = mpu->MagY;
    float my = mpu->MagX;
    float mz = mpu->MagZ;
    float q0 = mpu->q[0];
    float q1 = mpu->q[1];
    float q2 = mpu->q[2];
    float q3 = mpu->q[3];
//    float sampleFreq = 500.0f;
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2
                - mx * q3q3;

        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3
                - my * q3q3;

        _2bx = sqrt(hx * hx + hy * hy);

        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2
                + mz * q3q3;

        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
                - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
                - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
                + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
                - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
                + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
                + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
                + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
                + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    mpu->q[0] = q0;
    mpu->q[1] = q1;
    mpu->q[2] = q2;
    mpu->q[3] = q3;

    yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    pitch *= 180.0f / PI;
    yaw *= 180.0f / PI;
//    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll *= 180.0f / PI;
    mpu->roll = roll;
    mpu->pitch = pitch;
    mpu->yaw = yaw;
}

/**/
void i2cWrite(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg, uint8_t *data, uint8_t data_length)
{
    HAL_I2C_Mem_Write(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, data, data_length, 1000);
}

void i2cRead(I2C_HandleTypeDef *hi2c, uint16_t address, uint16_t reg, uint8_t *data, uint8_t data_length)
{
    HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, data, data_length, 1000);

}

//void Myfilter9DOF(float wx, float wy, float wz,
//                  float ax, float ay, float az,
//                                    float magx, float magy, float magz)

void Myfilter9DOF(MPU9250* mpu)
{
    //static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;


    float ax = mpu->AccX;
    float ay = mpu->AccY;
    float az = mpu->AccZ;
    float wx = mpu->GyroX * PI / 180.0f;
    float wy = mpu->GyroY * PI / 180.0f;
    float wz = mpu->GyroZ * PI / 180.0f;
    float magx = mpu->MagX;
    float magy = mpu->MagY;
    float magz = mpu->MagZ;


    static float q0_last = 1;
    static float q1_last = 0;
    static float q2_last = 0;
    static float q3_last = 0;

/*    static float x_ang_last = 0;
    static float y_ang_last = 0;
    static float z_ang_last = 0;*/

    //static bool update_first_time_flag = true;

    float q0_dot = 0, q1_dot = 0, q2_dot = 0, q3_dot = 0, norm = 0;
    float alphaAccSf = 0, accnorm = 0, magnorm = 0;
        //float e_m = 0, sf = 0;//filter efficient, magnitude error, scale factor, acc norm
    float gx = 0, gy = 0, gz = 0; //predicted gravity
    float mx = 0, my = 0; //predicted gravity
    float To = 0;
    float del_q0 = 0, del_q1 = 0, del_q2 = 0, del_q3 = 0, delnorm = 0;//delta quaternions
    float r0 = 0, r1 = 0, r2 = 0, r3 = 0; //Temp multiply
    float xAccelerationValue = 0, yAccelerationValue = 0, zAccelerationValue = 0;
    float xMagValue = 0, yMagValue = 0, zMagValue = 0;
    float xAngleEstValue = 0, yAngleEstValue = 0, zAngleEstValue = 0;
//    float xAngleVelValue = 0, yAngleVelValue = 0, zAngleVelValue = 0;

    /*prediction*/

    q0_dot = wx * q1 + wy * q2 + wz * q3;
    q1_dot = - wx * q0 + wz * q2 - wy * q3;
    q2_dot = - wy * q0 - wz * q1 + wx * q3;
    q3_dot = - wz * q0 + wy * q1 - wx * q2;

    q0 = q0 + q0_dot * 0.5f /sampleFreq;
    q1 = q1 + q1_dot * 0.5f /sampleFreq;
    q2 = q2 + q2_dot * 0.5f /sampleFreq;
    q3 = q3 + q3_dot * 0.5f /sampleFreq;

    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) ;

    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    /*accCorection*/
    //adaptive gain
//    e_m = abs(sqrt(ax * ax + ay * ay + az * az) - 0.95) / 0.95;
//
//    if(e_m<0.1)
//        sf=1;
//    else if(e_m<0.2)
//        sf=1-10*(e_m-0.1);
//    else
//        sf=0;
//
//    alphaAccSf = alphaAcc * sf;

    alphaAccSf = alphaAcc;

    //normalize acc
    accnorm = sqrt(ax * ax + ay * ay + az * az);
    xAccelerationValue = ax / accnorm;
    yAccelerationValue = ay / accnorm;
    zAccelerationValue = az / accnorm;

    //Predicted Gravity
    gx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * xAccelerationValue + 2 * (q1 * q2 + q0 * q3) * yAccelerationValue + 2 * (q1 * q3 - q0 * q2) * zAccelerationValue;
    gy = 2 * (q1 * q2 - q0 * q3) * xAccelerationValue + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * yAccelerationValue + 2 * (q2 * q3 + q1 * q0) * zAccelerationValue;
    gz = 2 * (q1 * q3 + q0 * q2) * xAccelerationValue + 2 * (q2 * q3 - q1 * q0) * yAccelerationValue + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * zAccelerationValue;

    //delta quaternions
    del_q0 = sqrt((gz + 1) / 2);
    del_q1 = - gy / sqrt(2 * (gz + 1));
    del_q2 = gx / sqrt(2 * (gz + 1));
    del_q3 = 0;

    //interpolation
    del_q0 = (1 - alphaAccSf) * 1 + alphaAccSf * del_q0;
    del_q1 = alphaAccSf * del_q1;
    del_q2 = alphaAccSf * del_q2;

    //normalize delta
    delnorm = sqrt(del_q0 * del_q0 + del_q1 * del_q1 + del_q2 * del_q2 + del_q3 * del_q3);
    del_q0 = del_q0 / delnorm;
    del_q1 = del_q1 / delnorm;
    del_q2 = del_q2 / delnorm;
    del_q3 = del_q3 / delnorm;

    //correction (multiply)
    r0 = q0 * del_q0 - q1 * del_q1 - q2 * del_q2 - q3 * del_q3;
    r1 = q0 * del_q1 + q1 * del_q0 + q2 * del_q3 - q3 * del_q2;
    r2 = q0 * del_q2 - q1 * del_q3 + q2 * del_q0 + q3 * del_q1;
    r3 = q0 * del_q3 + q1 * del_q2 - q2 * del_q1 + q3 * del_q0;

    //update
    q0=r0;
    q1=r1;
    q2=r2;
    q3=r3;

    /*Magnetic Correction*/
    //normalize Mag
    magnorm = sqrt(magx * magx + magy * magy + magz * magz);
    xMagValue = magx / magnorm;
    yMagValue = magy / magnorm;
    zMagValue = magz / magnorm;

    //Predicted Gravity
    mx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * xMagValue + 2 *(q1 * q2 + q0 * q3) * yMagValue + 2 * (q1 * q3 - q0 * q2) * zMagValue;
    my = 2 *(q1 * q2 - q0 * q3) * xMagValue + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * yMagValue + 2 * (q2 * q3 + q1 * q0) * zMagValue;
//    mz = 2 *(q1 * q3 + q0 * q2) * xMagValue + 2 * (q2 * q3 - q1 * q0) * yMagValue + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * zMagValue;

    To = sqrt(xMagValue * xMagValue + yMagValue * yMagValue);

    //delta quaternions
    del_q0 = sqrt((To + mx * sqrt(To)) / (2 * To));
    del_q1 = 0;
    del_q2 = 0;
    del_q3 = my / (sqrt(2 * (To + mx * sqrt(To))));

    //interpolation
    del_q0 = (1 - alphaMag) * 1 + alphaMag * del_q0;
    del_q1 = 0;
    del_q2 = 0;
    del_q3 = alphaMag * del_q3;

    //normalize delta
    delnorm = sqrt(del_q0 * del_q0 + del_q1 * del_q1 + del_q2 * del_q2 + del_q3 * del_q3);
    del_q0 = del_q0 / delnorm;
    del_q1 = del_q1 / delnorm;
    del_q2 = del_q2 / delnorm;
    del_q3 = del_q3 / delnorm;

    //correction (multiply)
    r0 = q0 * del_q0 - q1 * del_q1 - q2 * del_q2 - q3 * del_q3;
    r1 = q0 * del_q1 + q1 * del_q0 + q2 * del_q3 - q3 * del_q2;
    r2 = q0 * del_q2 - q1 * del_q3 + q2 * del_q0 + q3 * del_q1;
    r3 = q0 * del_q3 + q1 * del_q2 - q2 * del_q1 + q3 * del_q0;

    //update
    q0=r0;
    q1=r1;
    q2=r2;
    q3=r3;
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 0.95f || norm > 1.05f)
    {
        q0 = q0_last;
        q1 = q1_last;
        q2 = q2_last;
        q3 = q3_last ;

    }

    /*update last values */
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;
    q3_last = q3;
    /*Quatenions to Euler angle*/

    xAngleEstValue = atan2f((2 * ( - q0 * q1 + q2 * q3)),(1 - 2 * (q1 * q1 + q2 * q2))) * 57.29564f;
    yAngleEstValue = asinf(2 * ( - q0 * q2 - q3 * q1)) * 57.29564f;
    zAngleEstValue = atan2f((2 * ( - q0 * q3 + q1 * q2)),(1 - 2 * (q2 * q2 + q3 * q3))) * 57.29564f;
        RPY[0] = xAngleEstValue;
        RPY[1] = yAngleEstValue;
        RPY[2] = zAngleEstValue;

        mpu->roll = xAngleEstValue;
        mpu->pitch = yAngleEstValue;
        mpu->yaw = zAngleEstValue;
}
