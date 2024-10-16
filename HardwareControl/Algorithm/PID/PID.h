/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file PID.h
 * @brief Library about PID algorithm
 *
 * Long description.
 * @date 2024-10-13
 * @author	Anh Vu
 */


#ifndef ALGORITHM_PID_PID_H_
#define ALGORITHM_PID_PID_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "stdint.h"
#include "math.h"
#include "errorCode.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/
typedef struct motorPID *motorPIDHandle_t;
/**
 * @brief   Configuration structure.
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setPoint;
} motorPIDCfg_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize PID with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
motorPIDHandle_t mlsMotorPIDInit(void);

/*
 * @brief   Set configuration parameters.
 * @param 	handle: Handle structure.
 * @param   config: Configuration structure.
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsMotorPIDSetConfig(motorPIDHandle_t handle, motorPIDCfg_t config);

/*
 * @brief   Set Kp value.
 *
 * @param   handle Handle structure.
 * @param   Kp Kp.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDSetKp(motorPIDHandle_t handle, float Kp);

/*
 * @brief   Set Ki value.
 *
 * @param   handle Handle structure.
 * @param   Ki Ki.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDSetKi(motorPIDHandle_t handle, float Ki);

/*
 * @brief   Set Kd value.
 *
 * @param   handle Handle structure.
 * @param   Kd Kd.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDSetKd(motorPIDHandle_t handle, float Kd);

/*
 * @brief   Set Set Point value.
 *
 * @param   handle Handle structure.
 * @param   setPoint Set Point.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDSetSetPoint(motorPIDHandle_t handle, float setPoint);

/*
 * @brief   Get Kp.
 *
 * @param   handle Handle structure.
 * @param   *Kp Pointer of Kp.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDGetKp(motorPIDHandle_t handle, float *Kp);

/*
 * @brief   Get Ki.
 *
 * @param   handle Handle structure.
 * @param   *Ki Pointer of Ki.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDGetKi(motorPIDHandle_t handle, float *Ki);

/*
 * @brief   Get Kd.
 *
 * @param   handle Handle structure.
 * @param   *Kd Pointer of Kd.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDGetKd(motorPIDHandle_t handle, float *Kd);

/*
 * @brief   Get Set Point.
 *
 * @param   handle Handle structure.
 * @param   *setPoint Pointer of set point.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDGetSetPoint(motorPIDHandle_t handle, float *setPoint);

/*
 * @brief   Get real value.
 *
 * @param   handle Handle structure.
 * @param   *realValue Pointer of real value.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsMotorPIDGetRealVaule(motorPIDHandle_t handle, float *realValue);
#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_PID_PID_H_ */
/**@}*/
