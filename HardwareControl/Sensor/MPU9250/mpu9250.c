/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file mpu9250.c
 * @brief MPU9250 firmware
 *
 * Long description.
 * @date 2024-07-22
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "mpu9250.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct mpu9250{
    mpu9250Clksel_t        	clkSel;         /*!< MPU9250 clock source */
    mpu9250DlpfConfig_t     dlpfConfig;     /*!< MPU9250 digital low pass filter (DLPF) */
    mpu9250SleepMode_t    	sleepMode;     	/*!< MPU9250 sleep mode */
    mpu9250FsSel_t        	fsSel;         	/*!< MPU9250 gyroscope full scale range */
    mpu9250AfsSel_t       	afsSel;        	/*!< MPU9250 accelerometer full scale range */
    mpu9250AccelBias_t    	accelBias;     	/*!< Accelerometer bias */
    mpu9250GyroBias_t     	gyroBias;      	/*!< Gyroscope bias */
    mpu9250FuncI2cRead		i2cRead;		/*!< MPU9250 read bytes*/
    mpu9250FuncI2cWrite		i2cWrite;		/*!< MPU9250 write bytes*/
    mpu9250CommMode_t     	commMode;      	/*!< Interface protocol */
    float				accelScalingFactor;	/*!< MPU9250 accelerometer scaling factor */
    float 				gyroScalingFactor;	/*!< MPU9250 gyroscope scaling factor */
} mpu9250_t;
/********** Local Macro definition section ************************************/
#define CONVERT_ACCEL_UNIT		9.80665
/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mpu9250Handle_t mlsMpu9250Init(void)
{
	mpu9250Handle_t handle = calloc(1, sizeof(mpu9250_t));
	if(handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsMpu9250SetConfig(mpu9250Handle_t handle, mpu9250Config_t config)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Update accelerometer scaling factor */
	switch(config.afsSel)
	{
	case MPU9250_AFS_SEL_2G:
		handle->accelScalingFactor = (2.0f /32768.0f);
		break;
	case MPU9250_AFS_SEL_4G:
		handle->accelScalingFactor = (4.0f / 32768.0f);
		break;

	case MPU9250_AFS_SEL_8G:
		handle->accelScalingFactor = (8.0f / 32768.0f);
		break;

	case MPU9250_AFS_SEL_16G:
		handle->accelScalingFactor = (16.0f / 32768.0f);
		break;

	default:
		break;
	}

	/* Update gyroscope scaling factor */
	switch (config.fsSel)
	{
	case MPU9250_FS_SEL_250:
		handle->gyroScalingFactor = (250.0f / 32768.0f);
		break;

	case MPU9250_FS_SEL_500:
		handle->gyroScalingFactor = (500.0f / 32768.0f);
		break;

	case MPU9250_FS_SEL_1000:
		handle->gyroScalingFactor = (1000.0f / 32768.0f);
		break;

	case MPU9250_FS_SEL_2000:
		handle->gyroScalingFactor = (2000.0f / 32768.0f);
		break;

	default:
		break;
	}

	/* Update handle structure */
	handle->clkSel = config.clkSel;
	handle->dlpfConfig = config.dlpfConfig;
	handle->sleepMode = config.sleepMode;
	handle->fsSel = config.fsSel;
	handle->afsSel = config.afsSel;
	handle->accelBias.xAxis = config.accelBias.xAxis;
	handle->accelBias.yAxis = config.accelBias.yAxis;
	handle->accelBias.zAxis = config.accelBias.zAxis;
	handle->gyroBias.xAxis = config.gyroBias.xAxis;
	handle->gyroBias.yAxis = config.gyroBias.yAxis;
	handle->gyroBias.zAxis = config.gyroBias.zAxis;
	handle->i2cWrite = config.i2cWrite;
	handle->i2cRead = config.i2cRead;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250Config(mpu9250Handle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	/* Reset mpu9250 */
	uint8_t buffer = 0;
	buffer = 0x80;											// Write a one to bit 7 reset bit; toggle reset device
	handle->i2cWrite(MPU9250_PWR_MGMT_1, &buffer, 1);
	mlsHardwareInfoDelay(100);

	/* Wake up mpu9250 */
	buffer = (handle->sleepMode << 6) & 0x40;				// Clear sleep mode bit (6), enable all sensors
	handle->i2cWrite(MPU9250_PWR_MGMT_1, &buffer, 1);
	mlsHardwareInfoDelay(100);								// Wait for all registers to reset

	/* Configure clock source */
	buffer = 0;
	buffer = handle->clkSel & 0x07;
	handle->i2cWrite(MPU9250_PWR_MGMT_1, &buffer, 1);
	mlsHardwareInfoDelay(200);

	/* Configure digital low pass filter */
	buffer = 0;
	buffer = handle->dlpfConfig & 0x07;
	handle->i2cWrite(MPU9250_CONFIG, &buffer, 1);

	/* Configure sample rate divider */
	buffer = 0;
	buffer = 0x04;											// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	handle->i2cWrite(MPU9250_SMPRT_DIV, &buffer, 1);		// Use a 200 Hz rate; a rate consistent with the filter update rate

	/* Configure gyroscope range */
	buffer = 0;
	handle->i2cRead(MPU9250_GYRO_CONFIG, &buffer, 1);		// get current GYRO_CONFIG register value
	buffer = buffer & ~0xE0;                                // Clear self-test bits [7:5]
	buffer = buffer & ~0x03;                                // Clear Fchoice bits [1:0]
	buffer = buffer & ~0x18;                                // Clear GYRO_FS_SEL bits [4:3]
	buffer = buffer | ((uint8_t)(handle->fsSel) << 3);       	// Set full scale range for the gyro
	buffer = buffer | ((uint8_t)(~(0x03)) & 0x03);   			// Set Fchoice for the gyro
//	buffer = (handle->fsSel << 3) & 0x18;
	handle->i2cWrite(MPU9250_GYRO_CONFIG, &buffer, 1);

	/* Configure accelerometer range */
	buffer = 0;
	handle->i2cRead(MPU9250_ACCEL_CONFIG, &buffer, 1);		// get current ACCEL_CONFIG register value
	buffer = buffer & ~0xE0;                                // Clear self-test bits [7:5]
    buffer = buffer & ~0x18;                                // Clear ACCEL_FS_SEL bits [4:3]
    buffer = buffer | ((uint8_t)(handle->afsSel) << 3);  		// Set full scale range for the accelerometer
//	buffer = (handle->afsSel << 3) & 0x18;
	handle->i2cWrite(MPU9250_ACCEL_CONFIG, &buffer, 1);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	buffer = 0;
	handle->i2cRead(MPU9250_ACCEL_CONFIG2, &buffer, 1);			// get current ACCEL_CONFIG2 register value
	buffer = buffer & ~0x0F;                                    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	buffer = buffer | (~(0x01 << 3) & 0x08);    				// Set accel_fchoice_b to 1
	buffer = buffer | ((uint8_t)(handle->dlpfConfig) & 0x07);  	// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	handle->i2cWrite(MPU9250_ACCEL_CONFIG2, &buffer, 1);

	/* Configure interrupt and enable bypass.
	 * Set Interrupt pin active high, push-pull, Clear and read of INT_STATUS,
	 * enable I2C_BYPASS_EN in INT_PIN_CFG register so additional chips can
	 * join the I2C bus and can be controlled by master.
	 */
	buffer = 0x22;
	handle->i2cWrite(MPU9250_INT_PIN_CFG, &buffer, 1);
	buffer = 0x01;
	handle->i2cWrite(MPU9250_INT_ENABLE, &buffer, 1);
	mlsHardwareInfoDelay(100);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetAccelRaw(mpu9250Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (rawX == NULL) || (rawY == NULL) || (rawZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t accelRawData[6];
	handle->i2cRead(MPU9250_ACCEL_XOUT_H, accelRawData, 6);

	*rawX = (int16_t)((accelRawData[0] << 8) + accelRawData[1]);
	*rawY = (int16_t)((accelRawData[2] << 8) + accelRawData[3]);
	*rawZ = (int16_t)((accelRawData[4] << 8) + accelRawData[5]);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetAccelCalib(mpu9250Handle_t handle, int16_t *calibX, int16_t *calibY, int16_t *calibZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (calibX == NULL) || (calibY == NULL) || (calibZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t accelRawData[6];
	handle->i2cRead(MPU9250_ACCEL_XOUT_H, accelRawData, 6);

	*calibX = (int16_t)((accelRawData[0] << 8) + accelRawData[1]) - handle->accelBias.xAxis;
	*calibY = (int16_t)((accelRawData[2] << 8) + accelRawData[3]) - handle->accelBias.yAxis;
	*calibZ = (int16_t)((accelRawData[4] << 8) + accelRawData[5]) - handle->accelBias.zAxis;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetAccelScale(mpu9250Handle_t handle, float *scaleX, float *scaleY, float *scaleZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (scaleX == NULL) || (scaleY == NULL) || (scaleZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t accelRawData[6];
	handle->i2cRead(MPU9250_ACCEL_XOUT_H, accelRawData, 6);

	*scaleX = (float)((int16_t)((accelRawData[0] << 8) + accelRawData[1]) - handle->accelBias.xAxis) * handle->accelScalingFactor * CONVERT_ACCEL_UNIT;
	*scaleY = (float)((int16_t)((accelRawData[2] << 8) + accelRawData[3]) - handle->accelBias.yAxis) * handle->accelScalingFactor * CONVERT_ACCEL_UNIT;
	*scaleZ = (float)((int16_t)((accelRawData[4] << 8) + accelRawData[5]) - handle->accelBias.zAxis) * handle->accelScalingFactor * CONVERT_ACCEL_UNIT;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetGyroRaw(mpu9250Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (rawX == NULL) || (rawY == NULL) || (rawZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t gyroRawData[6];
	handle->i2cRead(MPU9250_GYRO_XOUT_H, gyroRawData, 6);

	*rawX = (int16_t)((gyroRawData[0] << 8) + gyroRawData[1]);
	*rawY = (int16_t)((gyroRawData[2] << 8) + gyroRawData[3]);
	*rawZ = (int16_t)((gyroRawData[4] << 8) + gyroRawData[5]);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetGyroCalib(mpu9250Handle_t handle, int16_t *calibX, int16_t *calibY, int16_t *calibZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (calibX == NULL) || (calibY == NULL) || (calibZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t gyroRawData[6];
	handle->i2cRead(MPU9250_GYRO_XOUT_H, gyroRawData, 6);

	*calibX = (int16_t)((gyroRawData[0] << 8) + gyroRawData[1]) - handle->gyroBias.xAxis;
	*calibY = (int16_t)((gyroRawData[2] << 8) + gyroRawData[3]) - handle->gyroBias.yAxis;
	*calibZ = (int16_t)((gyroRawData[4] << 8) + gyroRawData[5]) - handle->gyroBias.zAxis;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetGyroScale(mpu9250Handle_t handle, float *scaleX, float *scaleY, float *scaleZ)
{
	/* Check if handle structure or pointer data is NULL */
	if ((handle == NULL) || (scaleX == NULL) || (scaleY == NULL) || (scaleZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}
	uint8_t gyroRawData[6];
	handle->i2cRead(MPU9250_GYRO_XOUT_H, gyroRawData, 6);

	*scaleX = (float)((int16_t)((gyroRawData[0] << 8) + gyroRawData[1]) - handle->gyroBias.xAxis) * handle->gyroScalingFactor;
	*scaleY = (float)((int16_t)((gyroRawData[2] << 8) + gyroRawData[3]) - handle->gyroBias.yAxis) * handle->gyroScalingFactor;
	*scaleZ = (float)((int16_t)((gyroRawData[4] << 8) + gyroRawData[5]) - handle->gyroBias.zAxis) * handle->gyroScalingFactor;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250SetAccelBias(mpu9250Handle_t handle, int16_t biasX, int16_t biasY, int16_t biasZ)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->accelBias.xAxis = biasX;
	handle->accelBias.yAxis = biasY;
	handle->accelBias.zAxis = biasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250SetGyroBias(mpu9250Handle_t handle, int16_t biasX, int16_t biasY, int16_t biasZ)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->gyroBias.xAxis = biasX;
	handle->gyroBias.yAxis = biasY;
	handle->gyroBias.zAxis = biasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetAccelBias(mpu9250Handle_t handle, int16_t *biasX, int16_t *biasY, int16_t *biasZ)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*biasX = handle->accelBias.xAxis;
	*biasY = handle->accelBias.yAxis;
	*biasZ = handle->accelBias.zAxis;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250GetGyroBias(mpu9250Handle_t handle, int16_t *biasX, int16_t *biasY, int16_t *biasZ)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*biasX = handle->gyroBias.xAxis;
	*biasY = handle->gyroBias.yAxis;
	*biasZ = handle->gyroBias.zAxis;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMpu9250Calib6Axis(mpu9250Handle_t handle)
{
	int bufferSize = BUFFER_CALIB_DEFAULT;
	int meanAx, meanAy, meanAz, meanGx, meanGy, meanGz;
	long i = 0, buffAx = 0, buffAy = 0, buffAz = 0, buffGx = 0, buffGy =0, buffGz = 0;

	/* Dismiss 100 first value*/
	while (i < (bufferSize + 101))
	{
		int16_t accelRawX, accelRawY, accelRawZ;
		int16_t gyroRawX, gyroRawY, gyroRawZ;

		mlsMpu9250GetAccelRaw(handle, &accelRawX, &accelRawY, &accelRawZ);
		mlsMpu9250GetGyroRaw(handle, &gyroRawX, &gyroRawY, &gyroRawZ);

		if (i > 100 && i <= (bufferSize + 100))
		{
			buffAx += accelRawX;
			buffAy += accelRawY;
			buffAz += accelRawZ;
			buffGx += gyroRawX;
			buffGy += gyroRawY;
			buffGz += gyroRawZ;
		}

		if (i == (bufferSize + 100))
		{
			meanAx = buffAx /bufferSize;
			meanAy = buffAy /bufferSize;
			meanAz = buffAz /bufferSize;
			meanGx = buffGx /bufferSize;
			meanGy = buffGy /bufferSize;
			meanGz = buffGz /bufferSize;
		}

		i++;
	}

	handle->accelBias.xAxis = meanAx;
	handle->accelBias.yAxis = meanAy;
	handle->accelBias.zAxis = meanAz;
	handle->gyroBias.xAxis = meanGx;
	handle->gyroBias.yAxis = meanGy;
	handle->gyroBias.zAxis = meanGz;

	if(handle->accelBias.zAxis > 0L)
	{
		handle->accelBias.zAxis -= (1.0f / handle->accelScalingFactor);
	}
	else
	{
		handle->accelBias.zAxis += (1.0f / handle->accelScalingFactor);
	}

	return MLS_SUCCESS;
}

void mlsMpu9250Destroy(mpu9250Handle_t handle)
{
	free(handle);
}
/**@}*/
