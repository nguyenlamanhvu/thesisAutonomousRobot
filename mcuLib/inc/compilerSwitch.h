/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file errorCode.h
 * @brief Library about compiler switch
 *
 * Long description.
 * @date 2024-09-26
 * @author	Anh Vu
 */

#ifndef INC_COMPILERSWITCH_H_
#define INC_COMPILERSWITCH_H_

#ifdef __cplusplus
extern "C" {
#endif

/********** Include section ***************************************************/

/********** Constant  and compile switch definition section *******************/
#define USE_UART_ROS		0
#define USE_UART_MATLAB		0
#define USE_UART_GUI		1
#define USE_MAGNETOMETER_MPU9250
#define USE_ACC_GYRO_MPU9250
//#define USE_IMU_ADIS16488
#define USE_MADGWICK_FILTER	1
#define USE_ROS_LOG_DEBUG
//#define USE_ROS_LOG_REPEAT_CONNECTED_DEBUG
#define PUBLISH_MAG_DEBUG
/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/

#ifdef __cplusplus
}
#endif

#endif /* INC_COMPILERSWITCH_H_ */
/**@}*/
