/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file PID.c
 * @brief Library about PID algorithm
 *
 * Long description.
 * @date 2024-10-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "stdlib.h"
#include "stddef.h"
#include "PID.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct motorPID {
    float Kp;
    float Ki;
    float Kd;
    float setPoint;
    float realValue;
} motorPID_t;
/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
motorPIDHandle_t mlsMotorPIDInit(void)
{
	/* Allocate memory for handle structure */
	motorPIDHandle_t handle = calloc(1, sizeof(motorPID_t));
	if(handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsMotorPIDSetConfig(motorPIDHandle_t handle, motorPIDCfg_t config)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->Kd = config.Kp;
	handle->Ki = config.Ki;
	handle->Kd = config.Kd;
	handle->setPoint = config.setPoint;
	handle->realValue = 0;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDSetKp(motorPIDHandle_t handle, float Kp)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->Kp = Kp;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDSetKi(motorPIDHandle_t handle, float Ki)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->Ki = Ki;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDSetKd(motorPIDHandle_t handle, float Kd)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->Kd = Kd;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDSetSetPoint(motorPIDHandle_t handle, float setPoint)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->setPoint = setPoint;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDGetRealVaule(motorPIDHandle_t handle, float *realValue)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*realValue = handle->realValue;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDGetKp(motorPIDHandle_t handle, float *Kp)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*Kp = handle->Kp;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDGetKi(motorPIDHandle_t handle, float *Ki)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*Ki = handle->Ki;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDGetKd(motorPIDHandle_t handle, float *Kd)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*Kd = handle->Kd;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDGetSetPoint(motorPIDHandle_t handle, float *setPoint)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*setPoint = handle->setPoint;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorPIDCalculate(motorPIDHandle_t handle)
{
	/* Check input conditions */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	return MLS_SUCCESS;
}
/**@}*/
