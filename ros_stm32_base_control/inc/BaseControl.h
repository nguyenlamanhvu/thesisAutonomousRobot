/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControl.h
 * @brief Firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */

#ifndef ROS_STM32_BASE_CONTROL_INC_BASECONTROL_H_
#define ROS_STM32_BASE_CONTROL_INC_BASECONTROL_H_

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
/* Time update index */
#define IMU_PUBLISH_TIME_INDEX			0	/* Time index publish IMU information*/
#define IMU_UPDATE_TIME_INDEX			1	/* Time index update IMU information*/
#define CONTROL_MOTOR_TIME_INDEX		2	/* Time index control motor DC*/
#define VEL_PUBLISH_TIME_INDEX			3	/* Time index publish velocity*/
#define DRIVE_INFORMATION_TIME_INDEX	4	/* Time index publish drive information*/

/* Frequency of publish/subscribe */
#define IMU_PUBLISH_FREQUENCY			15		/* Frequency in Hz to publish IMU information*/
#define IMU_UPDATE_FREQUENCY			1000	/* Frequency in Hz to update IMU information*/
#define CONTROL_MOTOR_FREQUENCY			60		/* Frequency in Hz to control DC motor*/
#define VEL_PUBLISH_FREQUENCY			10		/* Frequency in Hz to publish velocity*/
#define DRIVE_INFORMATION_FREQUENCY		10		/* Frequency in Hz to publish drive information*/
/********** Function declaration section **************************************/

/*
 * @brief Base Control Init
 * @return Error Code
 */
mlsErrorCode_t mlsBaseControlInit(void);

/*
 * @brief Base Control Main
 * @return Error Code
 */
mlsErrorCode_t mlsBaseControlMain(void);

#ifdef __cplusplus
}
#endif

#endif /* ROS_STM32_BASE_CONTROL_INC_BASECONTROL_H_ */
/**@}*/
