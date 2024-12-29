/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file ak8963.c
 * @brief AK8963 Fimrware
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "ak8963.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct ak8963 {
	ak8963Mode_t  				oprerationMode; 			/*!< Operation mode */
	ak8963MfsSel_t  			mfsSel; 			  		/*!< Magnetometer full scale */
	float                       magHardIronBiasX;       	/*!< Magnetometer hard iron bias of x axis */
	float                       magHardIronBiasY;       	/*!< Magnetometer hard iron bias of y axis */
	float                       magHardIronBiasZ;       	/*!< Magnetometer hard iron bias of z axis */
	float                       magSoftIronBiasX;       	/*!< Magnetometer soft iron bias of x axis */
	float                       magSoftIronBiasY;       	/*!< Magnetometer soft iron bias of y axis */
	float                       magSoftIronBiasZ;       	/*!< Magnetometer soft iron bias of z axis */
	ak8963FuncI2cWrite        	i2cWrite;         			/*!< AK8963 write bytes */
	ak8963FuncI2cRead        	i2cRead;          			/*!< AK8963 read bytes */
	float 						magScalingFactor;			/*!< Magnetometer scaling factor */
	float  						magSensAdjX; 				/*!< Magnetometer sensitive adjust of x axis */
	float  						magSensAdjY;				/*!< Magnetometer sensitive adjust of y axis */
	float  						magSensAdjZ;				/*!< Magnetometer sensitive adjust of z axis */
} ak8963_t;
/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
ak8963Handle_t mlsAk8963Init(void)
{
	ak8963Handle_t handle = calloc(1, sizeof(ak8963_t));
	if (handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsAk8963SetConfig(ak8963Handle_t handle, ak8963Config_t config)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	float magScalingFactor = (10.0f * 4912.0f / 8190.0f);

	switch(config.mfsSel)
	{
	case AK8963_MFS_14BIT:
		magScalingFactor = 10.0f * 4912.0f / 8190.0f;
		break;
	case AK8963_MFS_16BIT:
		magScalingFactor = 10.0f * 4912.0f / 32760.0f;
		break;
	default:
		break;
	}

	handle->oprerationMode = config.oprerationMode;
	handle->mfsSel = config.mfsSel;
	handle->magHardIronBiasX = config.magHardIronBiasX;
	handle->magHardIronBiasY = config.magHardIronBiasX;
	handle->magHardIronBiasZ = config.magHardIronBiasZ;
	handle->magSoftIronBiasX = config.magSoftIronBiasX;
	handle->magSoftIronBiasY = config.magSoftIronBiasY;
	handle->magSoftIronBiasZ = config.magSoftIronBiasZ;
	handle->i2cWrite = config.i2cWrite;
	handle->i2cRead = config.i2cRead;
	handle->magScalingFactor = magScalingFactor;
	handle->magSensAdjX = 0;
	handle->magSensAdjY = 0;
	handle->magSensAdjZ = 0;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963Config(ak8963Handle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Power down AK8963 magnetic sensor */
	uint8_t buffer = 0;
	buffer = 0x00;
	handle->i2cWrite(AK8963_CNTL1, &buffer, 1);
	mlsHardwareInfoDelay(10);

	/* Set fuse ROM access mode */
	buffer = 0x0F;
	handle->i2cWrite(AK8963_CNTL1, &buffer, 1);
	mlsHardwareInfoDelay(10);

//	/* Power down AK8963 magnetic sensor */
//	buffer = 0x00;
//	handle->i2cWrite(AK8963_CNTL1, &buffer, 1);
//	mlsHardwareInfoDelay(10);

	/* Read magnetic sensitivity adjustment */
	uint8_t magRawData[3];
	handle->i2cRead(AK8963_ASAX, magRawData, 3);

	/* Update magnetometer sensitive adjust */
//	handle->magSensAdjX = ((float)(magRawData[0] - 128) / 256.0f + 1.0f);
//	handle->magSensAdjY = ((float)(magRawData[1] - 128) / 256.0f + 1.0f);
//	handle->magSensAdjZ = ((float)(magRawData[2] - 128) / 256.0f + 1.0f);
	handle->magSensAdjX = ((float)(magRawData[0] - 128) / 256.0f + 1.0f) * 0.15f;
	handle->magSensAdjY = ((float)(magRawData[1] - 128) / 256.0f + 1.0f) * 0.15f;
	handle->magSensAdjZ = ((float)(magRawData[2] - 128) / 256.0f + 1.0f) * 0.15f;

	/* Power down AK8963 magnetic sensor */
	buffer = 0x00;
	handle->i2cWrite(AK8963_CNTL1, &buffer, 1);
	mlsHardwareInfoDelay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	/* Configure magnetic operation mode and range */
	buffer = 0x00;
	buffer = handle->oprerationMode & 0x0F;
	buffer |= (handle->mfsSel << 4) & 0x10;
	handle->i2cWrite(AK8963_CNTL1, &buffer, 1);
	mlsHardwareInfoDelay(10);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963GetMagRaw(ak8963Handle_t handle, int16_t *rawX, int16_t *rawY, int16_t *rawZ)
{
	/* Check if handle structure or pointer data is NULL */
	if((handle == NULL) || (rawX == NULL) || (rawY == NULL) || (rawZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}

	uint8_t magRawData[7];
	handle->i2cRead(AK8963_XOUT_L, magRawData, 7);

	if((magRawData[6] & 0x08))
	{
		return MLS_ERROR;
	}

	*rawX = (int16_t)((int16_t)(magRawData[1] << 8) | magRawData[0]);
	*rawY = (int16_t)((int16_t)(magRawData[3] << 8) | magRawData[2]);
	*rawZ = (int16_t)((int16_t)(magRawData[5] << 8) | magRawData[4]);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963GetMagCalib(ak8963Handle_t handle, float *calibX, float *calibY, float *calibZ)
{
	/* Check if handle structure or pointer data is NULL */
	if((handle == NULL) || (calibX == NULL) || (calibY == NULL) || (calibZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}

	int16_t rawX = 0, rawY = 0, rawZ = 0;
	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsAk8963GetMagRaw(handle, &rawX, &rawY, &rawZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	*calibX = ((float)rawX * handle->magSensAdjX - handle->magHardIronBiasX / handle->magScalingFactor) * handle->magSoftIronBiasX;
	*calibY = ((float)rawY * handle->magSensAdjY - handle->magHardIronBiasY / handle->magScalingFactor) * handle->magSoftIronBiasY;
	*calibZ = ((float)rawZ * handle->magSensAdjZ - handle->magHardIronBiasZ / handle->magScalingFactor) * handle->magSoftIronBiasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963GetMagScale(ak8963Handle_t handle, float *scaleX, float *scaleY, float *scaleZ)
{
	/* Check if handle structure or pointer data is NULL */
	if((handle == NULL) || (scaleX == NULL) || (scaleY == NULL) || (scaleZ == NULL))
	{
		return MLS_ERROR_NULL_PTR;
	}

	int16_t rawX = 0, rawY = 0, rawZ = 0;
	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsAk8963GetMagRaw(handle, &rawX, &rawY, &rawZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	*scaleX = ((float)rawX * handle->magSensAdjX * handle->magScalingFactor - handle->magHardIronBiasX) * handle->magSoftIronBiasX;
	*scaleY = ((float)rawY * handle->magSensAdjY * handle->magScalingFactor - handle->magHardIronBiasY) * handle->magSoftIronBiasY;
	*scaleZ = ((float)rawZ * handle->magSensAdjZ * handle->magScalingFactor - handle->magHardIronBiasZ) * handle->magSoftIronBiasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963SetHardIronBias(ak8963Handle_t handle, float biasX, float biasY, float biasZ)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->magHardIronBiasX = biasX;
	handle->magHardIronBiasY = biasY;
	handle->magHardIronBiasZ = biasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963SGetHardIronBias(ak8963Handle_t handle, float *biasX, float *biasY, float *biasZ)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*biasX = handle->magHardIronBiasX;
	*biasY = handle->magHardIronBiasY;
	*biasZ = handle->magHardIronBiasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963SetSoftIronBias(ak8963Handle_t handle, float biasX, float biasY, float biasZ)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->magSoftIronBiasX = biasX;
	handle->magSoftIronBiasY = biasY;
	handle->magSoftIronBiasZ = biasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963GetSoftIronBias(ak8963Handle_t handle, float *biasX, float *biasY, float *biasZ)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*biasX = handle->magSoftIronBiasX;
	*biasY = handle->magSoftIronBiasY;
	*biasZ = handle->magSoftIronBiasZ;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsAk8963Calib3Axis(ak8963Handle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	int16_t magMax[3] = { -32767, -32767, -32767}, magMin[3] = {32767, 32767, 32767};
	uint32_t i;
	uint16_t sampleCount;

	/* 15 seconds for calib magnetometer*/
	if(handle->oprerationMode == AK8963_MODE_CONT_MEASUREMENT_1)
		sampleCount = 128;
	else if(handle->oprerationMode == AK8963_MODE_CONT_MEASUREMENT_2)
		sampleCount = 1500;

	for(i = 0; i < sampleCount + 100; i++)			/*!< Dismiss 100 first value */
	{
		if(i > 100 && i <= (sampleCount + 100))
		{
			int16_t rawX = 0, rawY = 0, rawZ = 0;
			mlsAk8963GetMagRaw(handle, &rawX, &rawY, &rawZ);

			if(rawX > magMax[0])
				magMax[0] = rawX;
			if(rawX < magMin[0])
				magMin[0] = rawX;

			if(rawY > magMax[1])
				magMax[1] = rawY;
			if(rawY < magMin[1])
				magMin[1] = rawY;

			if(rawZ > magMax[2])
				magMax[2] = rawZ;
			if(rawZ < magMin[2])
				magMin[2] = rawZ;
		}
		if(handle->oprerationMode == AK8963_MODE_CONT_MEASUREMENT_1)
			mlsHardwareInfoDelay(135);		// at 8 Hz ODR, new mag data is available every 125 ms
		if(handle->oprerationMode == AK8963_MODE_CONT_MEASUREMENT_2)
			mlsHardwareInfoDelay(12);		// at 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	handle->magHardIronBiasX = (float)((magMax[0] + magMin[0]) / 2) * handle->magScalingFactor * handle->magSensAdjX;
	handle->magHardIronBiasY = (float)((magMax[1] + magMin[1]) / 2) * handle->magScalingFactor * handle->magSensAdjY;
	handle->magHardIronBiasZ = (float)((magMax[2] + magMin[2]) / 2) * handle->magScalingFactor * handle->magSensAdjZ;

	// Get soft iron correction estimate
	float scaleTemp[3];

	scaleTemp[0] = (magMax[0] - magMin[0]) * handle->magSensAdjX / 2;
	scaleTemp[1] = (magMax[1] - magMin[1]) * handle->magSensAdjY / 2;
	scaleTemp[2] = (magMax[2] - magMin[2]) * handle->magSensAdjZ / 2;

	float magScaleAvg = (scaleTemp[0] + scaleTemp[1] + scaleTemp[2]) / 3.0f;

	handle->magSoftIronBiasX = magScaleAvg / ((float)scaleTemp[0]);
	handle->magSoftIronBiasY = magScaleAvg / ((float)scaleTemp[1]);
	handle->magSoftIronBiasZ = magScaleAvg / ((float)scaleTemp[2]);

	return MLS_SUCCESS;
}

void mlsAk8963Destroy(ak8963Handle_t handle)
{
	free(handle);
}
/**@}*/
