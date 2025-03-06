/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControl.c
 * @brief Firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "BaseControl.h"
#include "BaseControlPrivate.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
uint8_t gBaseControlTimeUpdateFlag[10];
extern TIM_HandleTypeDef htim6;

#if (USE_UART_GUI == 1)
extern uint8_t gGuiUpdateParameter;
#endif
/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t mlsBaseControlInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	/* Initialize peripherals */
	errorCode = mlsPeriphImuInit();
//#if USE_MADGWICK_FILTER
//	errorCode = mlsPeriphImuFilterInit();
//#endif
	errorCode = mlsPeriphMotorInit();
	errorCode = mlsPeriphEncoderInit();
	errorCode = mlsPeriphMotorPIDInit();

	/* Start Timer Interrupt*/
	errorCode = mlsBaseControlStartTimerInterrupt(&htim6);
#if (USE_UART_ROS == 1)
	/* Initialize ROS */
	mlsBaseControlROSSetup();
	/* Initialize Base Control */
	mlsBaseControlSetup();
#elif (USE_UART_MATLAB == 1 || USE_UART_GUI == 1)
	errorCode = mlsPeriphUartInit();
#endif
	return errorCode;
}

mlsErrorCode_t mlsBaseControlMain(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

#if (USE_UART_ROS == 1)
	/* Update variable */
//	mlsBaseControlUpdateVariable(mlsBaseControlConnectStatus());

	/* Update TF */
	mlsBaseControlUpdateTfPrefix(mlsBaseControlConnectStatus());

	/* Control motor*/
	if(gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] == 1)
	{
		mlsBaseControlUpdateGoalVel();
//		if(mlsBaseControlGetControlVelocityTime() > CONTROL_MOTOR_TIMEOUT)
//		{
//			mlsBaseControlSetVelocityZero();
//		}
//		else
//		{
//			mlsBaseControlSetVelocityGoal();
//		}
		mlsBaseControlSetVelocityGoal();
		mlsBaseControlCalculatePID();
		mlsBaseControlSetControlValue();
		gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 0;
	}

	/* Publish motor velocity data to topic "robot_vel", wheel velocity to topic "robot_wheel_vel"*/
	if(gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlPublishMortorVelocityMsg();
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 0;
	}

	/* Publish driver information */
//	if(gBaseControlTimeUpdateFlag[DRIVE_INFORMATION_TIME_INDEX] == 1)
//	{
//		/* Publish Odom, TF and JointState, */
//		mlsBaseControlPublishDriveInformationMsg();
//		gBaseControlTimeUpdateFlag[DRIVE_INFORMATION_TIME_INDEX] = 0;
//	}

	/* Update IMU */
	if(gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] == 1)
	{
		mlsBaseControlGet9Axis();
		gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] = 0;
	}
//
//	/* Calculate filter IMU */
//	if(gBaseControlTimeUpdateFlag[IMU_USING_FILTER_INDEX] == 1)
//	{
//		mlsBaseControlUpdateImu();
//		gBaseControlTimeUpdateFlag[IMU_USING_FILTER_INDEX] = 0;
//	}

	/* Publish IMU data to topic "imu"*/
	if(gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlPublishImuMsg();
		gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 0;
	}

	/* Send log message*/
	mlsBaseControlSendLogMsg();

	/* Spin NodeHandle to keep synchorus */
	mlsBaseControlSpinOnce();

	/* Keep rosserial connection */
	mlsBaseControlWaitSerialLink(mlsBaseControlConnectStatus());

	errorCode = MLS_SUCCESS;

#elif (USE_UART_MATLAB == 1)
	/* Update IMU */
	if(gBaseControlTimeUpdateFlag[IMU_UPDATE_FREQUENCY] == 1)
	{
		mlsBaseControlUpdateImu();
		gBaseControlTimeUpdateFlag[IMU_UPDATE_FREQUENCY] = 0;
	}

	/* Publish IMU data to MATLAB*/
	if(gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlUartPublishIMU();
		gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 0;
	}

#elif (USE_UART_GUI == 1)
	/* Receive parameter from GUI */
	if(gGuiUpdateParameter == 1)
	{
		mlsBaseControlGuiReceiveData();
		//Turn off flag
		gGuiUpdateParameter = 0;
	}

	/* Control motor*/
	if(gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] == 1)
	{
		mlsBaseControlCalculatePID();
		mlsBaseControlSetControlValue();
		gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 0;
	}

	/* Publish motor velocity data to gui */
	if(gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] == 1)
	{
		mlsBaseControlGuiPublishData();
//		mlsBaseControlCalculatePIDParameter();
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 0;
	}
	errorCode = MLS_SUCCESS;
#endif

	return errorCode;
}

/**@}*/
