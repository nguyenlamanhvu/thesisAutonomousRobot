/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Motor.h
 * @brief Library motor
 *
 * Long description.
 * @date 2024-10-05
 * @author	Anh Vu
 */


#ifndef ACTUATOR_MOTOR_H_
#define ACTUATOR_MOTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "stdlib.h"
#include "string.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef mlsErrorCode_t (*motorFuncSetPwmDuty)(float duty);
typedef mlsErrorCode_t (*motorFuncSetPwmFrequency)(uint32_t freq);
typedef mlsErrorCode_t (*motorFuncStartPwm)(void);
typedef mlsErrorCode_t (*motorFuncStopPwm)(void);
typedef mlsErrorCode_t (*motorFuncSetDir)(uint8_t dir);

typedef struct motor* motorHandle_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	uint8_t 					dir;		/*!< Direction */
	uint32_t					freqHz;		/*!< PWM frequency in Hz */
	float						duty;		/*!< PWM duty cycle */
	motorFuncSetPwmDuty			setPwmDuty;	/*!< Function set PWM duty */
	motorFuncSetPwmFrequency	setPwmFreq;	/*!< Function set PWM frequency */
	motorFuncStartPwm			startPwm;	/*!< Function start PWM */
	motorFuncStopPwm			stopPwm;	/*!< Function stop PWM */
	motorFuncSetDir				setDir;		/*!< Function set direction */
} motorConfig_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize motor with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
motorHandle_t mlsMotorInit(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorSetConfig(motorHandle_t handle, motorConfig_t config);

/*
 * @brief   Start motor.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorStart(motorHandle_t handle);

/*
 * @brief   Stop motor.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorStop(motorHandle_t handle);

/*
 * @brief   Set PWM duty cycle.
 *
 * @param 	handle Handle structure.
 * @param 	duty PWM duty cycle.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorSetPwmDuty(motorHandle_t handle, float duty);

/*
 * @brief   Set PWM frequency.
 *
 * @param 	handle Handle structure.
 * @param 	freqHz PWM frequency.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorSetPwmFreq(motorHandle_t handle, uint32_t freqHz);

/*
 * @brief   Set motor direction.
 *
 * @param 	handle Handle structure.
 * @param 	dir Direction.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorSetDir(motorHandle_t handle, uint8_t dir);

/*
 * @brief   Destroy step motor.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorDestroy(motorHandle_t handle);
#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_MOTOR_H_ */
/**@}*/
