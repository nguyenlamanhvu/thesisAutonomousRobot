/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file DifferentialDrive.h
 * @brief Differential Drive algorithm
 *
 * Long description.
 * @date <The date when the file was initial crated>
 * @author	Anh Vu
 */

#ifndef ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_
#define ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_

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
typedef struct diffDrive *diffDriveHandle_t;

typedef struct
{
	float wheelRadius;		/*!< Wheel radius. Unit: mm */
	float wheelSeperation; 	/*!< Wheel seperation. Unit: mm */
	float minLinVel;		/*!< Min linear velocity. Unit: m/s */
	float maxLinVel;		/*!< Max linear velocity. Unit: m/s */
	float minAngVel;		/*!< Min angular velocity. Unit: rad/s */
	float maxAngVel;		/*!< Max angular velocity. Unit: rad/s */
	float tickToRad;		/*!< Convert encoder tick to wheel position */
} diffDriveCfg_t;
/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Initialize differential drive with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
diffDriveHandle_t mlsDiffDriveInit(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveSetConfig(diffDriveHandle_t handle, diffDriveCfg_t config);

/*
 * @brief   Calculate velocity for each wheel based on expected linear and angular velocity.
 *
 * @param 	handle Handle structure.
 * @param 	lin_vel Expected linear velocity.
 * @param 	ang_vel Expected angular velocity.
 * @param 	left_wheel_vel Output velocity of left wheel.
 * @param 	right_wheel_vel Output velocity of right wheel.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveCalculateWheelVel(diffDriveHandle_t handle, float linVel, float angVel, float *leftWheelVel, float *rightWheelVel);

/*
 * @brief   Get encoder tick.
 *
 * @note 	This function gets the previous ticks set by "diff_drive_set_tick" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_tick Left tick.
 * @param 	right_tick Right tick.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveGetTick(diffDriveHandle_t handle, int64_t *leftTick, int64_t *rightTick);

/*
 * @brief   Get position of wheels in radian.
 *
 * @note 	This function gets the previous wheel positions set by "diff_drive_set_rad" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_rad Left radian.
 * @param 	right_rad Right radian.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveGetRad(diffDriveHandle_t handle, float *leftRad, float *rightRad);

/*
 * @brief   Get velocity of wheels in m/s.
 *
 * @note 	This function gets the previous wheel velocity set by "diff_drive_set_vel" called.
 *
 * @param 	handle Handle structure.
 * @param 	left_vel Left velocity.
 * @param 	right_vel Right velocity.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveGetVel(diffDriveHandle_t handle, float *leftVel, float *rightVel);

/*
 * @brief   Calculate odometry.
 *
 * @param 	handle Handle structure.
 * @param 	step_time Step time in ms.
 * @param 	left_tick Left tick.
 * @param 	right_tick Right tick.
 * @param 	theta Current yaw angel.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveCalculateOdom(diffDriveHandle_t handle, float stepTime, int32_t leftTick, int32_t rightTick, float theta);

/*
 * @brief   Get odometry.
 *
 * @param 	handle Handle structure.
 * @param 	odom_pose_x Position x.
 * @param 	odom_pose_y Position y.
 * @param 	odom_pose_theta Yaw angle.
 * @param 	odom_vel_lin Linear velocity.
 * @param 	odom_vel_ang Angular velocity.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
mlsErrorCode_t mlsDiffDriveGetOdom(diffDriveHandle_t handle, float *odomPoseX, float *odomPoseY, float *odomPoseTheta,
									float *odomVelLin, float *odomVelAng);

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_ */
/**@}*/
