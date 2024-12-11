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
extern uint8_t leftFuzzyMode;
extern uint8_t rightFuzzyMode;
extern uint8_t rightMotorRun;
extern uint8_t leftMotorRun;
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
//	errorCode = mlsPeriphImuInit();
//	errorCode = mlsPeriphImuFilterInit();
	errorCode = mlsPeriphMotorInit();
	errorCode = mlsPeriphEncoderInit();
	errorCode = mlsPeriphMotorPIDInit();

//	mlsPeriphMotorRightSetSpeed(100);
//	mlsPeriphMotorLeftSetSpeed(50);

	/* Start Timer Interrupt*/
	errorCode = mlsBaseControlStartTimerInterrupt(&htim6);
#if (USE_UART_ROS == 1)
	/* Initialize ROS*/
	mlsBaseControlROSSetup();
#elif (USE_UART_MATLAB == 1 || USE_UART_GUI == 1)
	errorCode = mlsPeriphUartInit();
#endif
	return errorCode;
}
int32_t rightTick;
extern float Vel;
mlsErrorCode_t mlsBaseControlMain(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
//	HAL_Delay(100);
//	mlsPeriphEncoderRightGetTick(&rightTick);
//	mlsPeriphMotorRightCalculateVelocity(rightTick, 100, &Vel);
#if (USE_UART_ROS == 1)
//	/* Control motor*/
//	if(gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] == 1)
//	{
//		mlsBaseControlUpdateGoalVel();
//		if(mlsBaseControlGetControlVelocityTime() > CONTROL_MOTOR_TIMEOUT)
//		{
//			mlsBaseControlSetVelocityZero();
//		}
//		else
//		{
//			mlsBaseControlSetVelocityGoal();
//		}
//		mlsBaseControlCalculatePID();
//		mlsBaseControlSetControlValue();
//		gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 0;
//	}
//
//	/* Publish motor velocity data to topic "robot_vel"*/
//	if(gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] == 1)
//	{
//		mlsBaseControlPublishMortorVelocityMsg();
//		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 0;
//	}
//
//	/* Update IMU */
//	if(gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] == 1)
//	{
//		mlsBaseControlUpdateImu();
//		gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] = 0;
//	}
//
//	/* Publish IMU data to topic "imu"*/
//	if(gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] == 1)
//	{
//		mlsBaseControlPublishImuMsg();
//		gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 0;
//	}
//
//	/* Send log message*/
//	mlsBaseControlSendLogMsg();
//
//	/* Spin NodeHandle to keep synchorus */
//	mlsBaseControlSpinOnce();
//
//	/* Keep rosserial connection */
//	mlsBaseControlWaitSerialLink(mlsBaseControlConnectStatus());

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

	/* Compute Fuzzy*/
	if(gBaseControlTimeUpdateFlag[COMPUTE_FUZZY_CONTROLLER_INDEX] == 1 /*&& leftMotorRun == 1 */&& (rightFuzzyMode == 1 || leftFuzzyMode == 1))
	{
		mlsBaseControlCalculateFuzzy();
		gBaseControlTimeUpdateFlag[COMPUTE_FUZZY_CONTROLLER_INDEX] = 0;
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
