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

#elif defined(USE_IMU_BNO055)
bno055Handle_t bno055Handle = NULL;

uint8_t systemSensor, gyro, accel, mag;
uint8_t system_status, self_test_results, system_error;
#endif

#ifdef USE_MADGWICK_FILTER
imuMadgwickHandle_t imuMadgwickHandle = NULL;
#endif
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/
static uint32_t BaseControlGetElaspedTime(uint32_t *time)
{
	uint32_t timeNow = mlsHardwareInfoGetTickMs();
	uint32_t elaspedTime = timeNow - *time;
	*time = timeNow;

	return elaspedTime;
}
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

#elif defined(USE_IMU_BNO055)
	/* Initialize BNO055 pointer*/
	bno055Handle = mlsBno055Init();
	if(bno055Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	bno055Config_t bno055Config = {
		.mode = BNO_MODE_NDOF,
		.i2cWrite = mlsHardwareInfoBno055WriteBytes,
		.i2cRead = mlsHardwareInfoBno055ReadBytes
	};

	/* Set configuration for BNO055*/
	errorCode = mlsBno055SetConfig(bno055Handle, bno055Config);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Configure BN0O55*/
	errorCode = mlsBno055Config(bno055Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

//	errorCode = bno055_setExtCrystalUse(bno055Handle, true);
//	if (errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}

//	do
//	{
//		errorCode = bno055_isFullyCalibrate(bno055Handle);
//		if (errorCode != MLS_SUCCESS)
//		{
//			return errorCode;
//		}
//
//		errorCode = bno055_getSensorOffsets(bno055Handle);
//		if (errorCode != MLS_SUCCESS)
//		{
//			return errorCode;
//		}
//
//		errorCode = bno055_getSystemStatus(bno055Handle, &system_status, &self_test_results, &system_error);
//		if (errorCode != MLS_SUCCESS)
//		{
//			return errorCode;
//		}
//
//		HAL_Delay(100);
//	}while(!bno055Handle->isFullyCalibrated);

	HAL_Delay(2000);

	errorCode = bno055_set_pwr_mode(bno055Handle, BNO_PWR_NORMAL);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_set_opmode(bno055Handle, BNO_MODE_CONFIG);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_acc_conf(bno055Handle, bno055Handle->_acc_range, bno055Handle->_acc_bandwidth, BNO_ACC_MODE_NORMAL);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_gyr_conf(bno055Handle, bno055Handle->_gyr_range, bno055Handle->_gyr_bandwith, BNO_GYR_MODE_NORMAL);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_mag_conf(bno055Handle, bno055Handle->_mag_out_rate, BNO_MAG_PWRMODE_FORCE, bno055Handle->_mag_mode);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_acc_conf(bno055Handle, BNO_ACC_RANGE_16G, BNO_ACC_BAND_62_5, BNO_ACC_MODE_NORMAL);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_gyr_conf(bno055Handle, BNO_GYR_RANGE_2000_DPS, BNO_GYR_BAND_32, BNO_GYR_MODE_NORMAL);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_mag_conf(bno055Handle, BNO_MAG_RATE_20HZ, BNO_MAG_PWRMODE_FORCE, BNO_MAG_MODE_REGULAR);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	bno055Handle->_offsets.accel_offset_x = -26;
	bno055Handle->_offsets.accel_offset_y = -10;
	bno055Handle->_offsets.accel_offset_z = -40;
	bno055Handle->_offsets.mag_offset_x = 124;
	bno055Handle->_offsets.mag_offset_y = 99;
	bno055Handle->_offsets.mag_offset_z = -1;
	bno055Handle->_offsets.gyro_offset_x = -1;
	bno055Handle->_offsets.gyro_offset_y = -1;
	bno055Handle->_offsets.gyro_offset_z = -1;

	bno055Handle->_offsets.accel_radius = -1;
	bno055Handle->_offsets.mag_radius = -1;

	errorCode = bno055_setSensorOffsets(bno055Handle);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = bno055_set_opmode(bno055Handle, bno055Handle->mode);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	HAL_Delay(600);

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

#elif defined(USE_IMU_BNO055)
	if (bno055Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = bno055_acc(bno055Handle, accelX, accelY, accelZ);

//	*accelX = imuDataMpu.mpuAccelX;
//	*accelY = imuDataMpu.mpuAccelY;
//	*accelZ = imuDataMpu.mpuAccelZ;

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

#elif defined(USE_IMU_BNO055)
	if (bno055Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = bno055_gyro(bno055Handle, gyroX, gyroY, gyroZ);
//	*gyroX = imuDataMpu.mpuGyroX;
//	*gyroY = imuDataMpu.mpuGyroY;
//	*gyroZ = imuDataMpu.mpuGyroZ;
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

#elif defined(USE_IMU_BNO055)
	if (bno055Handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	errorCode = bno055_mag(bno055Handle, magX, magY, magZ);

#endif
	return errorCode;
}

//uint32_t elapsedTimeIMU;

mlsErrorCode_t mlsPeriphImuGet9Axis(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
#if defined(USE_ACC_GYRO_MPU9250) && defined(USE_MAGNETOMETER_MPU9250)
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
#elif USE_IMU_ADIS16488

#elif defined(USE_IMU_BNO055)
//	uint32_t tempTest = mlsHardwareInfoGetTickMs();
//	float accelX, accelY, accelZ;
	errorCode = mlsPeriphImuGetAccel(&imuDataMpu.mpuAccelX, &imuDataMpu.mpuAccelY, &imuDataMpu.mpuAccelZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
//	imuDataMpu.mpuAccelX = accelX;
//	imuDataMpu.mpuAccelY = accelY;
//	imuDataMpu.mpuAccelZ = accelZ;

//	float gyroX, gyroY, gyroZ;
	errorCode = mlsPeriphImuGetGyro(&imuDataMpu.mpuGyroX, &imuDataMpu.mpuGyroY, &imuDataMpu.mpuGyroZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
//	imuDataMpu.mpuGyroX = gyroX;
//	imuDataMpu.mpuGyroY = gyroY;
//	imuDataMpu.mpuGyroZ = gyroZ;

//	float magX, magY, magZ;
//	errorCode = mlsPeriphImuGetMag(&magX, &magY, &magZ);
//	if(errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}
//	imuDataMpu.mpuMagX = magX;
//	imuDataMpu.mpuMagY = magY;
//	imuDataMpu.mpuMagZ = magZ;

//	errorCode = bno055_euler(bno055Handle, &(imuDataMpu.mpuEuler));
//	if(errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}

	errorCode = bno055_quaternion(bno055Handle, &(imuDataMpu.mpuQuat));
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
//	elapsedTimeIMU = BaseControlGetElaspedTime(&tempTest);
#endif

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
//	mlsErrorCode_t errorCode = MLS_ERROR;
//	if (imuMadgwickHandle == NULL)
//	{
//		return MLS_ERROR_NULL_PTR;
//	}

//	errorCode = mlsImuMadgwickGetQuaternion(imuMadgwickHandle, q0, q1, q2, q3);
//	if(errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}

//	errorCode = bno055_quaternion(bno055Handle, &(imuDataMpu.mpuQuat));
//	if(errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}
	*q0 = imuDataMpu.mpuQuat.w;
	*q1 = imuDataMpu.mpuQuat.x;
	*q2 = imuDataMpu.mpuQuat.y;
	*q3 = imuDataMpu.mpuQuat.z;

	return MLS_SUCCESS;
}
/**@}*/
