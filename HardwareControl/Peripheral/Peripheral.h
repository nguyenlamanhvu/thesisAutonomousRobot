/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Peripheral.h
 * @brief Library of peripheral
 *
 * Long description.
 * @date 2024-07-28
 * @author	Anh Vu
 */

#ifndef PERIPHERAL_PERIPHERAL_H_
#define PERIPHERAL_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize IMU with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuInit(void);

/*
 * @brief   Get accelerometer data.
 *
 * @param   handle: Handle structure.
 * @param   accelX: Accelerometer data x axis.
 * @param   accelY: Accelerometer data y axis.
 * @param   accelZ: Accelerometer data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetAccel(float *accelX, float *accelY, float *accelZ);

/*
 * @brief   Get gyroscope data.
 *
 * @param   handle: Handle structure.
 * @param   gyroX: gyroscope data x axis.
 * @param   gyroY: gyroscope data y axis.
 * @param   gyroZ: gyroscope data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetGyro(float *gyroX, float *gyroY, float *gyroZ);

/*
 * @brief   Get magnetometer data.
 *
 * @param   handle: Handle structure.
 * @param   magX: magnetometer data x axis.
 * @param   magY: magnetometer data y axis.
 * @param   magZ: magnetometer data z axis.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetMag(float *magX, float *magY, float *magZ);

/*
 * @brief   Initialize UART with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartInit(void);

/*
 * @brief   Send data by using uart.
 *
 * @param[1]   	data: Data string.
 * @param[2]	len: Length of data string.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartSend(uint8_t *data, uint16_t len);

/*
 * @brief   Read data by using uart.
 *
 * @param   data: Data string.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphUartRead(uint8_t *data);

/*
 * @brief   Initialize IMU Filter with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuFilterInit(void);

/*
 * @brief   Update quaternion data.
 *
 * @param   None
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuUpdateQuat(void);

/*
 * @brief   Get quaternion data.
 *
 * @param   handle: Handle structure.
 * @param   q0: quaternion 0.
 * @param   q1: quaternion 1.
 * @param   q2: quaternion 2.
 * @param   q3: quaternion 3.
 *
 * @return
 *      - MLS_SUCCESS: 		Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphImuGetQuat(float *q0, float *q1, float *q2, float *q3);

/*
 * @brief   Initialize Motor with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorInit(void);

/*
 * @brief   Start Left Motor.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftStart(void);

/*
 * @brief   Stop Left Motor.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftStop(void);

/*
 * @brief   Set Speed of Left Motor.
 *
 * @param   speed: Speed of Motor.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftSetSpeed(float speed);

/*
 * @brief   Set Direction of Left Motor.
 *
 * @param   Dir: Direction of Motor.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftSetDir(uint8_t dir);

/*
 * @brief   Start Right Motor.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightStart(void);

/*
 * @brief   Stop Right Motor.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightStop(void);

/*
 * @brief   Set Speed of Right Motor.
 *
 * @param   speed: Speed of Motor.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightSetSpeed(float speed);

/*
 * @brief   Set Direction of Right Motor.
 *
 * @param   Dir: Direction of Motor.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightSetDir(uint8_t dir);

/*
 * @brief   Initialize Encoder with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphEncoderInit(void);

/*
 * @brief   Get tick value from left encoder.
 *
 * @param   *tick: Pointer of tick value.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphEncoderLeftGetTick(uint32_t *tick);

/*
 * @brief   Get tick value from right encoder.
 *
 * @param   *tick: Pointer of tick value.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphEncoderRightGetTick(uint32_t *tick);
/*
 * @brief   Initialize Motor PID with default parameters.
 * @note    This function must be called first.
 * @param   None.
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorPIDInit(void);

/*
 * @brief   Set Kp to left motor.
 *
 * @param   Kp: Kp.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDSetKp(float Kp);

/*
 * @brief   Set Ki to left motor.
 *
 * @param   Ki: Ki.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDSetKi(float Ki);

/*
 * @brief   Set Kd to left motor.
 *
 * @param   Kd: Kd.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDSetKd(float Kd);

/*
 * @brief   Set set point to left motor.
 *
 * @param   setPoint: set point.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDSetSetPoint(float setPoint);

/*
 * @brief   Get real value from left motor.
 *
 * @param   *realValue: pointer of realValue.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDGetRealValue(float *realValue);

/*
 * @brief   Set Kp to right motor.
 *
 * @param   Kp: Kp.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDSetKp(float Kp);

/*
 * @brief   Set Ki to right motor.
 *
 * @param   Ki: Ki.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDSetKi(float Ki);

/*
 * @brief   Set Kd to right motor.
 *
 * @param   Kd: Kd.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDSetKd(float Kd);

/*
 * @brief   Set set point to right motor.
 *
 * @param   setPoint: set point.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDSetSetPoint(float setPoint);

/*
 * @brief   Get real value from right motor.
 *
 * @param   *realValue: pointer of realValue.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDGetRealValue(float *realValue);

/*
 * @brief   Get Kp of left motor.
 *
 * @param   *Kp: pointer of Kp.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDGetKp(float *Kp);

/*
 * @brief   Get Ki of left motor.
 *
 * @param   *Ki: pointer of Ki.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDGetKi(float *Ki);

/*
 * @brief   Get Kd of left motor.
 *
 * @param   *Kd: pointer of Kd.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDGetKd(float *Kd);

/*
 * @brief   Get set point of left motor.
 *
 * @param   *setPoint: pointer of set point.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorLeftPIDGetSetPoint(float *setPoint);

/*
 * @brief   Get Ki of right motor.
 *
 * @param   *Ki: pointer of Ki.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDGetKp(float *Kp);

/*
 * @brief   Get Ki of right motor.
 *
 * @param   *Ki: pointer of Ki.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDGetKi(float *Ki);

/*
 * @brief   Get Kd of right motor.
 *
 * @param   *Kd: pointer of Kd.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDGetKd(float *Kd);

/*
 * @brief   Get set point of right motor.
 *
 * @param   *setPoint: pointer of set point.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsPeriphMotorRightPIDGetSetPoint(float *setPoint);

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_PERIPHERAL_H_ */
/**@}*/
