/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file BaseControlPrivate.cpp
 * @brief Private firmware communicates with ROS over rosserial protocol.
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "stdio.h"
#include "stdbool.h"

#include "ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include "HardwareInfo.h"
#include "BaseControlPrivate.h"
#include "math.h"
/********** Local Constant and compile switch definition section **************/
#define ROS_TOPIC_IMU                       "imu"
#define ROS_TOPIC_MAG						"mag"
#define ROS_TOPIC_VEL						"robot_vel"
#define ROS_TOPIC_CALLBACK_VEL				"callback_robot_vel"
#define pi 3.14159265
/********** Local Type definition section *************************************/
#if (USE_UART_MATLAB == 1 || USE_UART_GUI == 1)
typedef union {
    float floatValue;
    uint8_t byteArray[4];
}FloatByteArray;
#endif

typedef enum {
	UPDATE_TIME_PID = 0x00,
	UPDATE_TIME_CONTROL_ROBOT =0x01,
	UPDATE_TIME_FUZZY = 0x02,
}updateTime_t;
/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
#if (USE_UART_MATLAB == 1)

#elif (USE_UART_GUI == 1)
extern dataFrame_t gGuiRxDataFrame;
extern dataFrame_t gGuiTxDataFrame;
#endif

uint8_t rightMotorRun = 0;
uint8_t leftMotorRun = 0;
/********** Local (static) function declaration section ***********************/
static sensor_msgs::Imu BaseControlGetIMU(void);
static sensor_msgs::MagneticField BaseControlGetMag(void);
static ros::Time BaseControlGetROSTime(void);
static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg);

/********** Local (static) variable definition section ************************/
ros::NodeHandle rosNodeHandle;    	/*!< ROS node handle */
char rosLogBuffer[100];          	/*!< ROS log message buffer */
unsigned long rosPrevUpdateTime[3];	/*!< ROS previous update time */

char imuFrameId[20];

sensor_msgs::Imu imuMsg;           	/*!< ROS IMU message */
sensor_msgs::MagneticField magMsg;	/*!< ROS Magnetic message*/
geometry_msgs::Twist velocityMsg;	/*!< ROS velocity message*/
nav_msgs::Odometry odometryMsg;		/*!< ROS odometry message*/

ros::Subscriber<geometry_msgs::Twist> callbackVelSub(ROS_TOPIC_CALLBACK_VEL, BaseControlCallbackCommandVelocity);

ros::Publisher imuPub(ROS_TOPIC_IMU, &imuMsg);
ros::Publisher magPub(ROS_TOPIC_MAG, &magMsg);
ros::Publisher velPub(ROS_TOPIC_VEL, &velocityMsg);

float goalVelocity[2] = {0.0, 0.0};            	/*!< Velocity to control motor */
float goalReceiveVelocity[2] = {0.0, 0.0};   	/*!< Velocity receive from "callback_robot_vel" topic */
float goalMotorVelocity[2] = {0.0, 0.0};		/*!< Velocity of 2 motors */

#if (USE_UART_MATLAB == 1)
FloatByteArray uartData;
uint8_t DataToSend[36];
uint8_t i;
#elif (USE_UART_GUI == 1)
FloatByteArray uartData;
static uint8_t getLeftParameter = 0;
static uint8_t getRightParameter = 0;
uint8_t leftFuzzyMode = 0;
uint8_t rightFuzzyMode = 0;
#endif

uint32_t stepControlRobotTime = 0;
/********** Local function definition section *********************************/
static ros::Time BaseControlGetROSTime(void)
{
	return rosNodeHandle.now();
}

static uint32_t BaseControlGetElaspedTime(uint32_t *time)
{
	uint32_t timeNow = mlsHardwareInfoGetTickMs();
	uint32_t elaspedTime = timeNow - *time;
	*time = timeNow;

	return elaspedTime;
}

static sensor_msgs::Imu BaseControlGetIMU(void)
{
	float accelX, accelY, accelZ;
	float gyroX, gyroY, gyroZ;
	float q0, q1, q2, q3;

	mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
	mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);
	mlsPeriphImuGetQuat(&q0, &q1, &q2, &q3);

	sensor_msgs::Imu imuMsg_;

	imuMsg_.angular_velocity.x = gyroX;
	imuMsg_.angular_velocity.y = gyroY;
	imuMsg_.angular_velocity.z = gyroZ;

	imuMsg_.linear_acceleration.x = accelX;
	imuMsg_.linear_acceleration.y = accelY;
	imuMsg_.linear_acceleration.z = accelZ;

	imuMsg_.orientation.x = q0;
	imuMsg_.orientation.y = q1;
	imuMsg_.orientation.z = q2;
	imuMsg_.orientation.w = q3;

	imuMsg_.angular_velocity_covariance[0] = 0;
	imuMsg_.angular_velocity_covariance[1] = 0;
	imuMsg_.angular_velocity_covariance[2] = 0;
	imuMsg_.angular_velocity_covariance[3] = 0;
	imuMsg_.angular_velocity_covariance[4] = 0;
	imuMsg_.angular_velocity_covariance[5] = 0;
	imuMsg_.angular_velocity_covariance[6] = 0;
	imuMsg_.angular_velocity_covariance[7] = 0;
	imuMsg_.angular_velocity_covariance[8] = 0;

	imuMsg_.linear_acceleration_covariance[0] = 0;
	imuMsg_.linear_acceleration_covariance[1] = 0;
	imuMsg_.linear_acceleration_covariance[2] = 0;
	imuMsg_.linear_acceleration_covariance[3] = 0;
	imuMsg_.linear_acceleration_covariance[4] = 0;
	imuMsg_.linear_acceleration_covariance[5] = 0;
	imuMsg_.linear_acceleration_covariance[6] = 0;
	imuMsg_.linear_acceleration_covariance[7] = 0;
	imuMsg_.linear_acceleration_covariance[8] = 0;

	imuMsg_.orientation_covariance[0] = 0;
	imuMsg_.orientation_covariance[1] = 0;
	imuMsg_.orientation_covariance[2] = 0;
	imuMsg_.orientation_covariance[3] = 0;
	imuMsg_.orientation_covariance[4] = 0;
	imuMsg_.orientation_covariance[5] = 0;
	imuMsg_.orientation_covariance[6] = 0;
	imuMsg_.orientation_covariance[7] = 0;
	imuMsg_.orientation_covariance[8] = 0;

	return imuMsg_;
}

static sensor_msgs::MagneticField BaseControlGetMag(void)
{
	float magX, magY, magZ;

	mlsPeriphImuGetMag(&magX, &magY, &magZ);

	sensor_msgs::MagneticField magMsg_;

	magMsg_.magnetic_field.x = magX;
	magMsg_.magnetic_field.y = magY;
	magMsg_.magnetic_field.z = magZ;

	magMsg_.magnetic_field_covariance[0] = 0;
	magMsg_.magnetic_field_covariance[1] = 0;
	magMsg_.magnetic_field_covariance[2] = 0;
	magMsg_.magnetic_field_covariance[3] = 0;
	magMsg_.magnetic_field_covariance[4] = 0;
	magMsg_.magnetic_field_covariance[5] = 0;
	magMsg_.magnetic_field_covariance[6] = 0;
	magMsg_.magnetic_field_covariance[7] = 0;
	magMsg_.magnetic_field_covariance[8] = 0;

	return magMsg_;
}

static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg)
{
	/* Get goal velocity */
	goalReceiveVelocity[LINEAR] = callbackVelMsg.linear.x;
	goalReceiveVelocity[ANGULAR] = callbackVelMsg.angular.z;

	/* Constrain velocity */

	/* Update control robot time */
	stepControlRobotTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_CONTROL_ROBOT]);
}

/********** Global function definition section ********************************/
void mlsBaseControlROSSetup(void)
{
    rosNodeHandle.initNode(); /*!< Init ROS node handle */

    rosNodeHandle.subscribe(callbackVelSub);	/*!< Subscribe to "callback_robot_vel" topic to get control robot velocity*/

    rosNodeHandle.advertise(imuPub);	/*!< Register the publisher to "imu" topic */
    rosNodeHandle.advertise(magPub);	/*!< Register the publisher to "mag" topic */
    rosNodeHandle.advertise(velPub);	/*!< Register the publisher to "robot_vel" topic */

    rosPrevUpdateTime[UPDATE_TIME_PID] = mlsHardwareInfoGetTickMs();
    rosPrevUpdateTime[UPDATE_TIME_CONTROL_ROBOT] = mlsHardwareInfoGetTickMs();
    rosPrevUpdateTime[UPDATE_TIME_FUZZY] = mlsHardwareInfoGetTickMs();
}

void mlsBaseControlSpinOnce(void)
{
    rosNodeHandle.spinOnce();
}

void mlsBaseControlWaitSerialLink(bool isConnected)
{
	static bool waitFlag = false;

	if (isConnected)
	{
		if (waitFlag == false)
		{
			mlsHardwareInfoDelay(10);

			waitFlag = true;
		}
	}
	else
	{
		waitFlag = false;
	}
}

bool mlsBaseControlConnectStatus(void)
{
	return rosNodeHandle.connected();
}

void mlsBaseControlSendLogMsg(void)
{
	static bool logFlag = false;

	if (rosNodeHandle.connected())
	{
		if (logFlag == false)
		{
			sprintf(rosLogBuffer, "--------------------------");
			rosNodeHandle.loginfo(rosLogBuffer);

			sprintf(rosLogBuffer, "Connected to openSTM32-Board");
			rosNodeHandle.loginfo(rosLogBuffer);

			sprintf(rosLogBuffer, "--------------------------");
			rosNodeHandle.loginfo(rosLogBuffer);

#ifdef USE_ROS_LOG_REPEAT_CONNECTED_DEBUG
			logFlag = false;
#else
			logFlag = true;
#endif
		}
	}
	else
	{
		logFlag = false;
	}
}

void mlsBaseControlPublishImuMsg(void)
{
	/* Get IMU data*/
	imuMsg = BaseControlGetIMU();

	imuMsg.header.stamp = BaseControlGetROSTime();
//	imuMsg.header.frame_id = imuFrameId;

	/* Publish IMU message*/
	imuPub.publish(&imuMsg);

#ifdef PUBLISH_MAG_DEBUG
	/* Get Mag data*/
	magMsg = BaseControlGetMag();
	magMsg.header.stamp = BaseControlGetROSTime();

	/* Publish Mag message*/
	magPub.publish(&magMsg);
#endif

#ifdef USE_ROS_LOG_DEBUG
	float q0, q1, q2, q3;
	float roll, pitch, yaw;

	mlsPeriphImuGetQuat(&q0, &q1, &q2, &q3);

    roll = 180.0 / 3.14 * atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    pitch = 180.0 / 3.14 * asin(2 * (q0 * q2 - q3 * q1));
    yaw = 180.0 / 3.14 * atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);

    sprintf(rosLogBuffer, "roll: %7.4f\tpitch: %7.4f\tyaw: %7.4f\t", roll, pitch, yaw);
    rosNodeHandle.loginfo(rosLogBuffer);
#endif
}

void mlsBaseControlPublishMortorVelocityMsg(void)
{
	/* Get motor velocity */
	velocityMsg.linear.x = goalVelocity[LINEAR];
	velocityMsg.angular.z = goalVelocity[ANGULAR];

	/* Publish motor velocity message to "robot_vel" topic*/
	velPub.publish(&velocityMsg);
}

void mlsBaseControlPublishDriveInformationMsg(void)
{
	odometryMsg.header.stamp = BaseControlGetROSTime();
//	odometryMsg.header.frame_id = odomFrameId;
}

void mlsBaseControlUpdateGoalVel(void)
{
	goalVelocity[LINEAR] = goalReceiveVelocity[LINEAR];
	goalVelocity[ANGULAR] = goalReceiveVelocity[ANGULAR];
}

mlsErrorCode_t mlsBaseControlStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle)
{
	return mlsHardwareInfoStartTimerInterrupt(timBaseHandle);
}

#if (USE_UART_MATLAB == 1)
mlsErrorCode_t mlsBaseControlUartPublishIMU(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	float accelX, accelY, accelZ;
	float gyroX, gyroY, gyroZ;
	float magX, magY, magZ;

	errorCode = mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = accelX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+0] = uartData.byteArray[i];
	}
	uartData.floatValue = accelY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+4] = uartData.byteArray[i];
	}
	uartData.floatValue = accelZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+8] = uartData.byteArray[i];
	}

	mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = gyroX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+12] = uartData.byteArray[i];
	}
	uartData.floatValue = gyroY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+16] = uartData.byteArray[i];
	}
	uartData.floatValue = gyroZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+20] = uartData.byteArray[i];
	}

	mlsPeriphImuGetMag(&magX, &magY, &magZ);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	uartData.floatValue = magX;
	for(i = 0; i < 4; i++) {
		DataToSend[i+24] = uartData.byteArray[i];
	}
	uartData.floatValue = magY;
	for(i = 0; i < 4; i++) {
		DataToSend[i+28] = uartData.byteArray[i];
	}
	uartData.floatValue = magZ;
	for(i = 0; i < 4; i++) {
		DataToSend[i+32] = uartData.byteArray[i];
	}

	errorCode = mlsPeriphUartSend(DataToSend);

	return errorCode;
}
#elif (USE_UART_GUI == 1)
mlsErrorCode_t mlsBaseControlGuiPublishParameter(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	//Will finish when I have time.
	/*Clear data buffer */
	memset(gGuiTxDataFrame.dataBuff, 0, sizeof(gGuiTxDataFrame.dataBuff));

	return errorCode;
}

mlsErrorCode_t mlsBaseControlGuiPublishData(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	/* Check data in Rx buffer */
	if(gGuiRxDataFrame.header == 0x00 && gGuiRxDataFrame.mode == 0x00 && gGuiRxDataFrame.footer == 0x00)
	{
		return MLS_SUCCESS;
	}

	/* Clear data buffer */
	memset(gGuiTxDataFrame.dataBuff, 0, sizeof(gGuiTxDataFrame.dataBuff));
	/* Clear data length */
	gGuiTxDataFrame.length = 0;

	gGuiTxDataFrame.header = 0x0A;

	if(gGuiRxDataFrame.mode == GUI_SET_LEFT_RUN_MODE || gGuiRxDataFrame.mode == GUI_SET_LEFT_STOP_MODE || gGuiRxDataFrame.mode == GUI_STOP_LEFT_FUZZY_MODE)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_LEFT_SPEED_MODE;
		errorCode = mlsPeriphMotorLeftPIDGetRealValue(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length = sizeof(FloatByteArray);
	}
	else if(gGuiRxDataFrame.mode == GUI_SET_RIGHT_RUN_MODE || gGuiRxDataFrame.mode == GUI_SET_RIGHT_STOP_MODE || gGuiRxDataFrame.mode == GUI_STOP_RIGHT_FUZZY_MODE)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_RIGHT_SPEED_MODE;
		errorCode = mlsPeriphMotorRightPIDGetRealValue(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length = sizeof(FloatByteArray);
	}
	else if(getLeftParameter == 1)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_PARAMETER_LEFT;
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetSetPoint(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorLeftPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);

		/*!< Clear Rx buffer */
		memset(&gGuiRxDataFrame, 0, sizeof(dataFrame_t));

		//Turn off flag
		getLeftParameter = 0;
	}
	else if(getRightParameter == 1)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_PARAMETER_RIGHT;
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetSetPoint(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorRightPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);

		/*!< Clear Rx buffer */
		memset(&gGuiRxDataFrame, 0, sizeof(dataFrame_t));

		//Turn off flag
		getRightParameter = 0;
	}
	else if(leftFuzzyMode == 1)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_LEFT_FUZZY_MODE;
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetRealValue(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorLeftPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);

	}
	else if(rightFuzzyMode == 1)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_RIGHT_FUZZY_MODE;
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetRealValue(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorRightPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get set point */
		mlsPeriphMotorRightPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);

	}
	else
	{
		/*!< Clear Tx buffer */
		memset(&gGuiTxDataFrame, 0, sizeof(dataFrame_t));
		return MLS_SUCCESS;
	}

	gGuiTxDataFrame.footer = 0x06;

	errorCode = mlsPeriphUartSend((uint8_t*)&gGuiTxDataFrame, sizeof(gGuiTxDataFrame));
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsBaseControlGuiReceiveData(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	switch(gGuiRxDataFrame.mode)
	{
	case GUI_SET_LEFT_STOP_MODE:
		mlsPeriphMotorLeftStop();
		leftMotorRun = 0;
		break;
	case GUI_SET_LEFT_FUZZY_MODE:
		leftFuzzyMode = 1;
		rightFuzzyMode = 0;
	case GUI_SET_LEFT_RUN_MODE:
		leftMotorRun = 1;
		mlsPeriphMotorLeftStart();
		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff, 4);
		errorCode = mlsPeriphMotorLeftPIDSetSetPoint(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 4, 4);
		errorCode = mlsPeriphMotorLeftPIDSetKp(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 8, 4);
		errorCode = mlsPeriphMotorLeftPIDSetKi(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 12, 4);
		errorCode = mlsPeriphMotorLeftPIDSetKd(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		break;
	case GUI_SET_RIGHT_STOP_MODE:
		mlsPeriphMotorRightStop();
		rightMotorRun = 0;
		break;
	case GUI_SET_RIGHT_FUZZY_MODE:
		rightFuzzyMode = 1;
		leftFuzzyMode = 0;
	case GUI_SET_RIGHT_RUN_MODE:
		rightMotorRun = 1;
		mlsPeriphMotorRightStart();
		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff, 4);
		errorCode = mlsPeriphMotorRightPIDSetSetPoint(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 4, 4);
		errorCode = mlsPeriphMotorRightPIDSetKp(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 8, 4);
		errorCode = mlsPeriphMotorRightPIDSetKi(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}

		memcpy(uartData.byteArray, gGuiRxDataFrame.dataBuff + 12, 4);
		errorCode = mlsPeriphMotorRightPIDSetKd(uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		break;
	case GUI_GET_PARAMETER_LEFT:
		//Turn on flag
		getLeftParameter = 1;
		break;
	case GUI_GET_PARAMETER_RIGHT:
		//Turn on flag
		getRightParameter = 1;
		break;
	case GUI_STOP_LEFT_FUZZY_MODE:
		leftFuzzyMode = 0;
	case GUI_STOP_RIGHT_FUZZY_MODE:
		rightFuzzyMode = 0;
	default:
		break;
	}

	return MLS_SUCCESS;
}

//void mlsBaseControlCalculatePIDTunningGUI(void)
//{
//	int32_t leftTick, rightTick;
//	/* Update time */
//	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_PID]);
//	/* Get tick from encoder */
//	mlsPeriphEncoderLeftGetTick(&leftTick);
//	mlsPeriphEncoderRightGetTick(&rightTick);
//	/* Calculate linear velocity of 2 motors*/
//	mlsPeriphMotorLeftCalculateVelocity(leftTick, stepTime, &goalMotorVelocity[LEFT]);
//	mlsPeriphMotorRightCalculateVelocity(rightTick, stepTime, &goalMotorVelocity[RIGHT]);
//	/* Update real value to PID controller */
//	mlsPeriphMotorLeftPIDUpdateRealValue(goalMotorVelocity[LEFT]);
//	mlsPeriphMotorRightPIDUpdateRealValue(goalMotorVelocity[RIGHT]);
//	/* Calculate PID */
//	mlsPeriphMotorLeftPIDCalculate();
//	mlsPeriphMotorRightPIDCalculate();
//}
#endif

mlsErrorCode_t mlsBaseControlUpdateImu(void)
{
	return mlsPeriphImuUpdateQuat();
}

void mlsBaseControlCalculatePID(void)
{
	int32_t leftTick, rightTick;
	/* Update time */
	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_PID]);
	/* Get tick from encoder */
	mlsPeriphEncoderLeftGetTick(&leftTick);
//	mlsPeriphEncoderRightGetTick(&rightTick);
	/* Calculate linear velocity of 2 motors*/
	mlsPeriphMotorLeftCalculateVelocity(leftTick, stepTime, &goalMotorVelocity[LEFT]);
//	mlsPeriphMotorRightCalculateVelocity(rightTick, stepTime, &goalMotorVelocity[RIGHT]);
	/* Update real value to PID controller */
	mlsPeriphMotorLeftPIDUpdateRealValue(goalMotorVelocity[LEFT]);
//	mlsPeriphMotorRightPIDUpdateRealValue(goalMotorVelocity[RIGHT]);
	/* Calculate PID */
	mlsPeriphMotorLeftPIDCalculate(stepTime);
//	mlsPeriphMotorRightPIDCalculate(stepTime);
}

uint32_t mlsBaseControlGetControlVelocityTime(void)
{
	return stepControlRobotTime;
}

void mlsBaseControlSetVelocityZero(void)
{
	mlsPeriphMotorLeftPIDSetSetPoint(0);
	mlsPeriphMotorRightPIDSetSetPoint(0);
}

void mlsBaseControlSetVelocityGoal(void)
{
	float rightWheelVelocity = 0.0, leftWheelVelocity = 0.0;

	mlsPeriphMotorLeftPIDSetSetPoint(leftWheelVelocity);
	mlsPeriphMotorRightPIDSetSetPoint(rightWheelVelocity);
}

void mlsBaseControlSetControlValue(void)
{
	mlsPeriphMotorLeftPIDSetControl();
	mlsPeriphMotorRightPIDSetControl();
}

void mlsBaseControlPublishTest(int32_t tick)
{
	sprintf(rosLogBuffer, "Right tick: %ld", tick);
	rosNodeHandle.loginfo(rosLogBuffer);
}
float rightMotorVel = 0;
float leftMotorVel;
void mlsBaseControlCalculateFuzzy(void)
{
	/* Update time */
	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_FUZZY]);
	int32_t rightTick, leftTick;

//	mlsPeriphMotorRightFuzzyCalculate(stepTime);

	mlsPeriphMotorLeftFuzzyCalculate(stepTime);
}
volatile  double sweap=0;
float leftVel;
volatile double_t k=0;
uint8_t DataToSend[8];
int8_t i=0;
void mlsBaseControlCalculatePIDParameter(void)
{
	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_FUZZY]);
	sweap = 99.0*sin(2 *pi * 0.1 * 40 * (pow(50, (k) / (200.0 * 40)) - 1) / log(50.0));
	uartData.floatValue = sweap;
	k=k+1;
  	if(k == 4001)
  	{
  		mlsPeriphMotorLeftSetSpeed(0);
  	}
  	else
  	{
  	  	mlsPeriphMotorLeftSetSpeed(sweap);
  	//  	errorCode = mlsPeriphUartSend(uartData.byteArray);
  	  	for(i = 0; i < 4; i++) {
  			DataToSend[i] = uartData.byteArray[i];
  		}

  	  	int32_t leftTick;

  	  	mlsPeriphEncoderLeftGetTick(&leftTick);
  	  	mlsPeriphMotorLeftCalculateVelocity(leftTick, stepTime, &leftVel);
  	  	uartData.floatValue = leftVel;
  	  	for(i = 0; i < 4; i++) {
  			DataToSend[i+4] = uartData.byteArray[i];
  		}
  	  	HAL_UART_Transmit(&huart2, DataToSend, 8,100);
  	}
}
/********** Class function implementation section *****************************/

/**@}*/
