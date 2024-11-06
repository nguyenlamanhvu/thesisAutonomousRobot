/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Motor.c
 * @brief Library motor
 *
 * Long description.
 * @date 2024-10-05
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Motor.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
typedef struct motor {
	uint8_t 					dir;		/*!< Direction */
	uint32_t					freqHz;		/*!< PWM frequency in Hz */
	float						duty;		/*!< PWM duty cycle */
	uint8_t						isRun;		/*!< Running status*/
	motorFuncSetPwmDuty			setPwmDuty;	/*!< Function set PWM duty */
	motorFuncSetPwmFrequency	setPwmFreq;	/*!< Function set PWM frequency */
	motorFuncStartPwm			startPwm;	/*!< Function start PWM */
	motorFuncStopPwm			stopPwm;	/*!< Function stop PWM */
	motorFuncSetDir				setDir;		/*!< Function set direction */
} motor_t;
/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
motorHandle_t mlsMotorInit(void)
{
	motorHandle_t handle =  calloc(1, sizeof(motor_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

mlsErrorCode_t mlsMotorSetConfig(motorHandle_t handle, motorConfig_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->dir = config.dir;
	handle->duty = config.duty;
	handle->freqHz = config.freqHz;
	handle->isRun = 0;
	handle->setPwmDuty = config.setPwmDuty;
	handle->setPwmFreq = config.setPwmFreq;
	handle->startPwm = config.startPwm;
	handle->stopPwm = config.stopPwm;
	handle->setDir = config.setDir;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorStart(motorHandle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->startPwm();
	handle->isRun = 1;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorStop(motorHandle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->stopPwm();
	handle->isRun = 0;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorSetPwmDuty(motorHandle_t handle, float duty)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->setPwmDuty(duty);
	handle->duty = duty;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorSetPwmFreq(motorHandle_t handle, uint32_t freqHz)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->setPwmFreq(freqHz);
	handle->freqHz = freqHz;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorSetDir(motorHandle_t handle, uint8_t dir)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->setDir(dir);
	handle->dir = dir;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsMotorDestroy(motorHandle_t handle)
{
	if (handle != NULL)
	{
		free(handle);
	}

	return MLS_SUCCESS;
}
/**@}*/
