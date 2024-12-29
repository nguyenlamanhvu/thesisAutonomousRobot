/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file ImuPeripheral.c
 * @brief Library about peripheral for IMU
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Peripheral.h"
#include "HardwareInfo.h"
#include "mpu9250.h"
#include "ak8963.h"
#include "Madgwick.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct {
	float 	mpuAccelX;
	float	mpuAccelY;
	float	mpuAccelZ;
	float 	mpuGyroX;
	float	mpuGyroY;
	float	mpuGyroZ;
	float 	mpuMagX;
	float	mpuMagY;
	float	mpuMagZ;
}imuData_t;
/********** Local Macro definition section ************************************/
#ifdef USE_ACC_GYRO_MPU9250

#define DEFAULT_ACCEL_BIAS_X				0
#define DEFAULT_ACCEL_BIAS_Y				0
#define DEFAULT_ACCEL_BIAS_Z				0

#define DEFAULT_GYRO_BIAS_X					0
#define DEFAULT_GYRO_BIAS_Y					0
#define DEFAULT_GYRO_BIAS_Z					0

#ifdef USE_MAGNETOMETER_MPU9250
#define DEFAULT_MAG_HARD_IRON_X				0
#define DEFAULT_MAG_HARD_IRON_Y				0
#define DEFAULT_MAG_HARD_IRON_Z				0

#define DEFAULT_MAG_SOFT_IRON_X				0
#define DEFAULT_MAG_SOFT_IRON_Y				0
#define DEFAULT_MAG_SOFT_IRON_Z				0
#endif

#endif

#ifdef USE_MADGWICK_FILTER
#define DEFAULT_MADGWICK_BETA  				0.12f
#define DEFAULT_MADGWICK_SAMPLE_FREQ  		IMU_FILTER_FREQUENCY
#endif
/********** Local (static) variable definition ********************************/
imuData_t	imuDataMpu;

#ifdef USE_ACC_GYRO_MPU9250
mpu9250Handle_t mpu9250Handle = NULL;

#ifdef USE_MAGNETOMETER_MPU9250
ak8963Handle_t ak8963Handle = NULL;
#endif

#endif

#ifdef USE_MADGWICK_FILTER
imuMadgwickHandle_t imuMadgwickHandle = NULL;
#endif
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t mlsPeriphImuInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#ifdef USE_ACC_GYRO_MPU9250
	/* Initialize MPU9250 pointer*/
	mpu9250Handle = mlsMpu9250Init();
	if(mpu9250Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mpu9250Config_t mpu9250Config = {
		.clkSel = MPU9250_CLKSEL_AUTO,
		.dlpfConfig = MPU9250_41ACEL_42GYRO_BW_HZ,
		.sleepMode = MPU9250_DISABLE_SLEEP_MODE,
		.fsSel = MPU9250_FS_SEL_2000,
		.afsSel = MPU9250_AFS_SEL_16G,
		.accelBias = {
			.xAxis = DEFAULT_ACCEL_BIAS_X,
			.yAxis = DEFAULT_ACCEL_BIAS_Y,
			.zAxis = DEFAULT_ACCEL_BIAS_Z
		},
		.gyroBias = {
			.xAxis = DEFAULT_GYRO_BIAS_X,
			.yAxis = DEFAULT_GYRO_BIAS_Y,
			.zAxis = DEFAULT_GYRO_BIAS_Z
		},
		.i2cWrite = mlsHardwareInfoMpu9250WriteBytes,
		.i2cRead = mlsHardwareInfoMpu9250ReadBytes
	};


	/* Set configuration for MPU9250*/
	errorCode = mlsMpu9250SetConfig(mpu9250Handle, mpu9250Config);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure MPU9250*/
	errorCode = mlsMpu9250Config(mpu9250Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Calibrate MPU9250 before get value*/
	errorCode = mlsMpu9250Calib6Axis(mpu9250Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

#ifdef USE_MAGNETOMETER_MPU9250
	/* Initialize AK8963 pointer*/
	ak8963Handle = mlsAk8963Init();
	if(ak8963Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	ak8963Config_t ak8963Config = {
		.oprerationMode = AK8963_MODE_CONT_MEASUREMENT_2,
		.mfsSel = AK8963_MFS_16BIT,
		.magHardIronBiasX = DEFAULT_MAG_HARD_IRON_X,
		.magHardIronBiasY = DEFAULT_MAG_HARD_IRON_Y,
		.magHardIronBiasZ = DEFAULT_MAG_HARD_IRON_Z,
		.magSoftIronBiasX = DEFAULT_MAG_SOFT_IRON_X,
		.magSoftIronBiasY = DEFAULT_MAG_HARD_IRON_Y,
		.magSoftIronBiasZ = DEFAULT_MAG_SOFT_IRON_Z,
		.i2cWrite = mlsHardwareInfoAk8963WriteBytes,
		.i2cRead = mlsHardwareInfoAk8963ReadBytes
	};

	/* Set configuration for AK8963*/
	errorCode = mlsAk8963SetConfig(ak8963Handle, ak8963Config);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure Ak8963*/
	errorCode = mlsAk8963Config(ak8963Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Calibrate AK8963 before get value*/
	errorCode = mlsAk8963Calib3Axis(ak8963Handle);

	return errorCode;
#endif

#elif USE_IMU_ADIS16488

#endif
	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphImuFilterInit(void)
{
#ifdef USE_MADGWICK_FILTER
	/* Config madgwick filter */
	mlsErrorCode_t errorCode = MLS_ERROR;

	imuMadgwickHandle = mlsImuMadgwickInit();
	if(imuMadgwickHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	imuMadgwickCfg_t imuMadgwickCfg = {
			.beta = DEFAULT_MADGWICK_BETA,
			.sampleFreq = DEFAULT_MADGWICK_SAMPLE_FREQ
	};

	errorCode = mlsImuMadgwickSetConfig(imuMadgwickHandle, imuMadgwickCfg);

	return errorCode;
#endif
}

mlsErrorCode_t mlsPeriphImuGetAccel(float *accelX, float *accelY, float *accelZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

#ifdef USE_ACC_GYRO_MPU9250
	if (mpu9250Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = mlsMpu9250GetAccelScale(mpu9250Handle, accelX, accelY, accelZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

mlsErrorCode_t mlsPeriphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#ifdef USE_ACC_GYRO_MPU9250
	if (mpu9250Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = mlsMpu9250GetGyroScale(mpu9250Handle, gyroX, gyroY, gyroZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

mlsErrorCode_t mlsPeriphImuGetMag(float *magX, float *magY, float *magZ)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if defined(USE_ACC_GYRO_MPU9250) && defined(USE_MAGNETOMETER_MPU9250)
	if (ak8963Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = mlsAk8963GetMagScale(ak8963Handle, magX, magY, magZ);
#elif USE_IMU_ADIS16488

#endif
	return errorCode;
}

mlsErrorCode_t mlsPeriphImuGet9Axis(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	// Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
	// Magneto direction is Right-Hand, Y-Forward, Z-Down
	float accelX, accelY, accelZ;
	errorCode = mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	imuDataMpu.mpuAccelX = +accelX;
	imuDataMpu.mpuAccelY = +accelY;
	imuDataMpu.mpuAccelZ = -accelZ;

	float gyroX, gyroY, gyroZ;
	errorCode = mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	imuDataMpu.mpuGyroX = -gyroX;
	imuDataMpu.mpuGyroY = -gyroY;
	imuDataMpu.mpuGyroZ = +gyroZ;

	float magX, magY, magZ;
	errorCode = mlsPeriphImuGetMag(&magX, &magY, &magZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	imuDataMpu.mpuMagX = -magY;
	imuDataMpu.mpuMagY = -magX;
	imuDataMpu.mpuMagZ = -magZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphImuUpdateQuat(float deltaT)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	if (imuMadgwickHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
//	errorCode = mlsImuMadgwickUpdate6Dof(imuMadgwickHandle, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, deltaT);
	errorCode = mlsImuMadgwickUpdate9Dof(imuMadgwickHandle,
			imuDataMpu.mpuGyroX, imuDataMpu.mpuGyroY, imuDataMpu.mpuGyroZ,
			imuDataMpu.mpuAccelX, imuDataMpu.mpuAccelY, imuDataMpu.mpuAccelZ,
			imuDataMpu.mpuMagX, imuDataMpu.mpuMagY, imuDataMpu.mpuMagZ, deltaT);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphImuGetQuat(float *q0, float *q1, float *q2, float *q3)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	if (imuMadgwickHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	errorCode = mlsImuMadgwickGetQuaternion(imuMadgwickHandle, q0, q1, q2, q3);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return errorCode;
}
/**@}*/
