/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Differential.c
 * @brief Differential Drive algorithm
 *
 * Long description.
 * @date 08-12-2024
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "DifferentialDrive.h"
#include "Peripheral.h"
#include <stdlib.h>
/********** Local Constant and compile switch definition section **************/
#define NUM_WHEEL 2
#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1
/********** Local Type definition section *************************************/
typedef struct diffDrive
{
	float wheelRadius;			  /*!< Wheel radius. Unit: mm */
	float wheelSeperation;		  /*!< Wheel seperation. Unit: mm */
	float minLinVel;			  /*!< Min linear velocity. Unit: m/s */
	float maxLinVel;			  /*!< Max linear velocity. Unit: m/s */
	float minAngVel;			  /*!< Min angular velocity. Unit: rad/s */
	float maxAngVel;			  /*!< Max angular velocity. Unit: rad/s */
	float tickToRad;			  /*!< Convert encoder tick to wheel position */
	int64_t prevTick[NUM_WHEEL]; /*!< Previous encoder tick */
	float prevRad[NUM_WHEEL];	  /*!< Previous position. Unit: radian */
	float prevVel[NUM_WHEEL];	  /*!< Previous velocity. Unit: m/s */
	float prevTheta;			  /*!< Previous theta angle. Unit: rad */
	float odomPoseX;			  /*!< Odom pose x */
	float odomPoseY;			  /*!< Odom pose y */
	float odomPoseZ;			  /*!< Odom pose z */
	float odomPoseTheta;		  /*!< Odom pose theta */
	float odomVelLin;			  /*!< Odom linear velocity */
	float odomVelAng;			  /*!< Odom angular velocity */
} diffDrive_t;
/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/
static float constrain(float x, float low_val, float high_val)
{
	float value;
	if (x > high_val)
	{
		value = high_val;
	}
	else if (x < low_val)
	{
		value = low_val;
	}
	else
	{
		value = x;
	}
	return value;
}
/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
diffDriveHandle_t mlsDiffDriveInit(void)
{
	diffDriveHandle_t handle = calloc(1, sizeof(diffDrive_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

mlsErrorCode_t mlsDiffDriveSetConfig(diffDriveHandle_t handle, diffDriveCfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	handle->wheelRadius = config.wheelRadius;
	handle->wheelSeperation = config.wheelSeperation;
	handle->minLinVel = config.minLinVel;
	handle->maxLinVel = config.maxLinVel;
	handle->minAngVel = config.minAngVel;
	handle->maxAngVel = config.maxAngVel;
	handle->tickToRad = config.tickToRad;
	handle->prevTick[LEFT_WHEEL] = 0;
	handle->prevTick[RIGHT_WHEEL] = 0;
	handle->prevRad[LEFT_WHEEL] = 0;
	handle->prevRad[RIGHT_WHEEL] = 0;
	handle->prevVel[LEFT_WHEEL] = 0;
	handle->prevVel[RIGHT_WHEEL] = 0;
	handle->prevTheta = 0;
	handle->odomPoseX = 0;
	handle->odomPoseY = 0;
	handle->odomPoseZ = 0;
	handle->odomPoseTheta = 0;
	handle->odomVelAng = 0;
	handle->odomVelLin = 0;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveCalculateWheelVel(diffDriveHandle_t handle, float linVel, float angVel, float *leftWheelVel, float *rightWheelVel)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	float wheelVelocityCmd[2];

	wheelVelocityCmd[LEFT_WHEEL] = linVel - (angVel * handle->wheelSeperation / 2);
	wheelVelocityCmd[RIGHT_WHEEL] = linVel + (angVel * handle->wheelSeperation /2);

	wheelVelocityCmd[LEFT_WHEEL] = constrain(wheelVelocityCmd[LEFT_WHEEL], handle->minLinVel, handle->maxLinVel);
	wheelVelocityCmd[RIGHT_WHEEL] = constrain(wheelVelocityCmd[RIGHT_WHEEL], handle->minLinVel, handle->maxLinVel);

	*leftWheelVel = wheelVelocityCmd[LEFT_WHEEL];
	*rightWheelVel = wheelVelocityCmd[RIGHT_WHEEL];

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveGetTick(diffDriveHandle_t handle, int64_t *leftTick, int64_t *rightTick)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*leftTick = handle->prevTick[LEFT_WHEEL];
	*rightTick = handle->prevTick[RIGHT_WHEEL];

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveGetRad(diffDriveHandle_t handle, float *leftRad, float *rightRad)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*leftRad = handle->prevRad[LEFT_WHEEL];
	*rightRad = handle->prevRad[RIGHT_WHEEL];

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveGetVel(diffDriveHandle_t handle, float *leftVel, float *rightVel)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*leftVel = handle->prevVel[LEFT_WHEEL];
	*rightVel = handle->prevVel[RIGHT_WHEEL];

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveCalculateOdom(diffDriveHandle_t handle, float stepTime, int32_t leftTick, int32_t rightTick, float theta)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	float wheel_l = 0.0f, wheel_r = 0.0f; // rotation value of wheel [rad]
	float delta_s = 0.0f, delta_theta = 0.0f;
	float lin_vel = 0.0f, ang_vel = 0.0f; // v = translational velocity [m/s], w = rotational velocity [rad/s]

	if(stepTime == 0)	return MLS_ERROR;

	wheel_l = handle->tickToRad * (float)leftTick;
	wheel_r = handle->tickToRad * (float)rightTick;

	if(isnan(wheel_l))	wheel_l = 0.0f;
	if (isnan(wheel_r))	wheel_r = 0.0f;

	delta_s = handle->wheelRadius * (wheel_r + wheel_l) / 2.0f;
	delta_theta = theta - handle->prevTheta;

	/* Compute odometric pose */
	handle->odomPoseX += delta_s * cos(handle->odomPoseTheta + (delta_theta / 2.0));
	handle->odomPoseY += delta_s * sin(handle->odomPoseTheta + (delta_theta / 2.0));
	handle->odomPoseTheta += delta_theta;

	/* Compute odometric instantaneouse velocity */
	lin_vel = delta_s / stepTime;
	ang_vel = delta_theta / stepTime;

	handle->odomVelLin = lin_vel;
	handle->odomVelAng = ang_vel;

	handle->prevTick[LEFT_WHEEL] = leftTick;
	handle->prevTick[RIGHT_WHEEL] = rightTick;
	handle->prevRad[LEFT_WHEEL] += handle->tickToRad * (float)leftTick;
	handle->prevRad[RIGHT_WHEEL] += handle->tickToRad * (float)rightTick;
	handle->prevVel[LEFT_WHEEL] = wheel_l / stepTime;
	handle->prevVel[RIGHT_WHEEL] = wheel_r / stepTime;
	handle->prevTheta = theta;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsDiffDriveGetOdom(diffDriveHandle_t handle, float *odomPoseX, float *odomPoseY, float *odomPoseTheta,
									float *odomVelLin, float *odomVelAng)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	*odomPoseX = handle->odomPoseX;
	*odomPoseY = handle->odomPoseY;
	*odomPoseTheta = handle->odomPoseTheta;
	*odomVelLin = handle->odomVelLin;
	*odomVelAng = handle->odomVelAng;

	return MLS_SUCCESS;
}
/**@}*/
