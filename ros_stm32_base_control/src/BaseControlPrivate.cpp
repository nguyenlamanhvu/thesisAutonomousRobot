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
#include "std_msgs/Float32MultiArray.h"
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
#include "DifferentialDrive.h"
/********** Local Constant and compile switch definition section **************/
#define ROS_TOPIC_IMU                       "imu"
#define ROS_TOPIC_MAG						"mag"
#define ROS_TOPIC_VEL						"robot_vel"
#define ROS_TOPIC_WHEEL_VEL					"robot_wheel_vel"
#define ROS_TOPIC_CALLBACK_VEL				"cmd_vel"

const float magnetic_declination = -0.85;  // HCM city
#define ROS_TOPIC_JOINT_STATES              "joint_states"
#define ROS_TOPIC_ODOM                      "odom"
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
	UPDATE_TIME_CONTROL_ROBOT = 0x01,
	UPDATE_TIME_PUBLISH_DRIVE = 0x02,
	UPDATE_TIME_UPDATE_MADGWICK =0x03,
}updateTime_t;
/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
#if (USE_UART_MATLAB == 1)

#elif (USE_UART_GUI == 1)
extern dataFrame_t gGuiRxDataFrame;
extern dataFrame_t gGuiTxDataFrame;
#endif

extern imuData_t	imuDataMpu;
/********** Local (static) function declaration section ***********************/
static sensor_msgs::Imu BaseControlGetIMU(void);
static sensor_msgs::MagneticField BaseControlGetMag(void);
static ros::Time BaseControlGetROSTime(void);
static void BaseControlCallbackCommandVelocity(const geometry_msgs::Twist &callbackVelMsg);

/********** Local (static) variable definition section ************************/
ros::NodeHandle rosNodeHandle;    	/*!< ROS node handle */
char rosLogBuffer[100];          	/*!< ROS log message buffer */
unsigned long rosPrevUpdateTime[4];	/*!< ROS previous update time */

char imuFrameId[20];
char magFrameId[20];
char odomHeaderFrameId[30];
char odomChildFrameId[30];
char jointStateHeaderFrameId[30];

sensor_msgs::Imu imuMsg;           	/*!< ROS IMU message */
sensor_msgs::MagneticField magMsg;	/*!< ROS Magnetic message*/
geometry_msgs::Twist velocityMsg;	/*!< ROS velocity message*/
nav_msgs::Odometry odometryMsg;		/*!< ROS odometry message*/
sensor_msgs::JointState joint_states;    /*!< ROS joint states message */
geometry_msgs::TransformStamped odom_tf; /*!< ROS transform stamped message */
tf::TransformBroadcaster tf_broadcaster; /*!< ROS tf broadcaster message */
std_msgs::Float32MultiArray wheelVelocityMsg; /*!< ROS left + right wheel velocity (RPM) message */

ros::Subscriber<geometry_msgs::Twist> callbackVelSub(ROS_TOPIC_CALLBACK_VEL, BaseControlCallbackCommandVelocity);

ros::Publisher imuPub(ROS_TOPIC_IMU, &imuMsg);
ros::Publisher magPub(ROS_TOPIC_MAG, &magMsg);
ros::Publisher velPub(ROS_TOPIC_VEL, &velocityMsg);
ros::Publisher odomPub(ROS_TOPIC_ODOM, &odometryMsg);
ros::Publisher jointStatesPub(ROS_TOPIC_JOINT_STATES, &joint_states);
ros::Publisher wheelVelPub(ROS_TOPIC_WHEEL_VEL, &wheelVelocityMsg);

char get_prefix[10];
char *get_tf_prefix = get_prefix;

float goalVelocity[2] = {0.0, 0.0};            	/*!< Velocity to control motor */
float goalReceiveVelocity[2] = {0.0, 0.0};   	/*!< Velocity receive from "callback_robot_vel" topic */
float goalMotorVelocity[2] = {0.0, 0.0};		/*!< Velocity of 2 motors */

float odom_pose_x;
float odom_pose_y;
float odom_pose_theta;
float odom_vel_lin;
float odom_vel_ang;

#if (USE_UART_MATLAB == 1)
FloatByteArray uartData;
uint8_t DataToSend[36];
uint8_t i;
#elif (USE_UART_GUI == 1)
FloatByteArray uartData;
static uint8_t getLeftParameter = 0;
static uint8_t getRightParameter = 0;
#endif

diffDriveHandle_t robot_kinematic_handle = NULL;
/********** Local function definition section *********************************/
static ros::Time BaseControlGetROSTime(void)
{
	return rosNodeHandle.now();
}

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

static uint32_t BaseControlGetElaspedTime(uint32_t *time)
{
	uint32_t timeNow = mlsHardwareInfoGetTickMs();
	uint32_t elaspedTime = timeNow - *time;
	*time = timeNow;

	return elaspedTime;
}

static sensor_msgs::Imu BaseControlGetIMU(void)
{
//	float accelX, accelY, accelZ;
//	float gyroX, gyroY, gyroZ;
//	float q0, q1, q2, q3;

//	mlsPeriphImuGetAccel(&accelX, &accelY, &accelZ);
//	mlsPeriphImuGetGyro(&gyroX, &gyroY, &gyroZ);
//	mlsPeriphImuGetQuat(&q0, &q1, &q2, &q3);

	sensor_msgs::Imu imuMsg_;

	imuMsg_.angular_velocity.x = imuDataMpu.mpuGyroX;
	imuMsg_.angular_velocity.y = imuDataMpu.mpuGyroY;
	imuMsg_.angular_velocity.z = imuDataMpu.mpuGyroZ;

	imuMsg_.linear_acceleration.x = imuDataMpu.mpuAccelX;
	imuMsg_.linear_acceleration.y = imuDataMpu.mpuAccelY;
	imuMsg_.linear_acceleration.z = imuDataMpu.mpuAccelZ;

	imuMsg_.orientation.x = imuDataMpu.mpuQuat.x;
	imuMsg_.orientation.y = imuDataMpu.mpuQuat.y;
	imuMsg_.orientation.z = imuDataMpu.mpuQuat.z;
	imuMsg_.orientation.w = imuDataMpu.mpuQuat.w;

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

	magMsg_.magnetic_field.x = -magY;
	magMsg_.magnetic_field.y = -magX;
	magMsg_.magnetic_field.z = -magZ;

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
	goalReceiveVelocity[LINEAR] = constrain(goalReceiveVelocity[LINEAR], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
	goalReceiveVelocity[ANGULAR] = constrain(goalReceiveVelocity[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

	/* Update control robot time */
	rosPrevUpdateTime[UPDATE_TIME_CONTROL_ROBOT] = mlsHardwareInfoGetTickMs();
}

static void BaseControlUpdateOdom(void)
{
    mlsDiffDriveGetOdom(robot_kinematic_handle,
                        &odom_pose_x,
                        &odom_pose_y,
                        &odom_pose_theta,
                        &odom_vel_lin,
                        &odom_vel_ang);

    odometryMsg.header.frame_id = odomHeaderFrameId;
    odometryMsg.child_frame_id = odomChildFrameId;

    odometryMsg.pose.pose.position.x = odom_pose_x;
    odometryMsg.pose.pose.position.y = odom_pose_y;
    odometryMsg.pose.pose.position.z = 0;
    odometryMsg.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose_theta);

    odometryMsg.twist.twist.linear.x = odom_vel_lin;
    odometryMsg.twist.twist.angular.z = odom_vel_ang;
}

static void BaseControlUpdateJointState(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    mlsDiffDriveGetRad(robot_kinematic_handle, &joint_states_pos[LEFT], &joint_states_pos[RIGHT]);
    mlsDiffDriveGetVel(robot_kinematic_handle, &joint_states_vel[LEFT], &joint_states_vel[RIGHT]);

    joint_states.position = (double *)joint_states_pos;
    joint_states.velocity = (double *)joint_states_vel;
}

static void BaseControlUpdateTf(geometry_msgs::TransformStamped &odom_tf)
{
    odom_tf.header = odometryMsg.header;
    odom_tf.child_frame_id = odometryMsg.child_frame_id;
    odom_tf.transform.translation.x = odometryMsg.pose.pose.position.x;
    odom_tf.transform.translation.y = odometryMsg.pose.pose.position.y;
    odom_tf.transform.translation.z = odometryMsg.pose.pose.position.z;
    odom_tf.transform.rotation = odometryMsg.pose.pose.orientation;
}

static void BaseControlInitOdom(void)
{
    odom_pose_x = 0;
    odom_pose_y = 0;
    odom_pose_theta = 0;
    odom_vel_lin = 0;
    odom_vel_ang = 0;

    odometryMsg.pose.pose.position.x = 0.0;
    odometryMsg.pose.pose.position.y = 0.0;
    odometryMsg.pose.pose.position.z = 0.0;

    odometryMsg.pose.pose.orientation.x = 0.0;
    odometryMsg.pose.pose.orientation.y = 0.0;
    odometryMsg.pose.pose.orientation.z = 0.0;
    odometryMsg.pose.pose.orientation.w = 0.0;

    odometryMsg.twist.twist.linear.x = 0.0;
    odometryMsg.twist.twist.angular.z = 0.0;
}

static void BaseControlInitJointState(void)
{
    static char *joint_states_name[] = {(char *)"wheel_left_joint", (char *)"wheel_right_joint"};

    joint_states.header.frame_id = jointStateHeaderFrameId;
    joint_states.name = joint_states_name;

    joint_states.name_length = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length = WHEEL_NUM;
}

static bool BaseControlCalcOdom(float stepTime)
{
    float theta;
    float q0, q1, q2, q3;

    mlsPeriphImuGetQuat(&q0, &q1, &q2, &q3);
    theta = atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);

    int32_t leftTick, rightTick;

    mlsPeriphEncoderLeftGetTick(&leftTick);
    mlsPeriphEncoderRightGetTick(&rightTick);

    mlsDiffDriveCalculateOdom(robot_kinematic_handle, stepTime, leftTick, rightTick, theta);
    return true;
}
/********** Global function definition section ********************************/
void mlsBaseControlROSSetup(void)
{
    rosNodeHandle.initNode(); /*!< Init ROS node handle */

    rosNodeHandle.subscribe(callbackVelSub);	/*!< Subscribe to "callback_robot_vel" topic to get control robot velocity*/

    rosNodeHandle.advertise(imuPub);			/*!< Register the publisher to "imu" topic */
    rosNodeHandle.advertise(magPub);			/*!< Register the publisher to "mag" topic */
//    rosNodeHandle.advertise(velPub);			/*!< Register the publisher to "robot_vel" topic */
//    rosNodeHandle.advertise(odomPub);          	/*!< Register the publisher to "odom" topic */
//    rosNodeHandle.advertise(jointStatesPub);  	/*!< Register the publisher to "joint_states" topic */
    rosNodeHandle.advertise(wheelVelPub);		/*!< Register the publisher to "robot_wheel_vel" topic */

    tf_broadcaster.init(rosNodeHandle); /*!< Init TransformBroadcaster */
    BaseControlInitOdom();
    BaseControlInitJointState();

    rosPrevUpdateTime[UPDATE_TIME_PID] = mlsHardwareInfoGetTickMs();
    rosPrevUpdateTime[UPDATE_TIME_CONTROL_ROBOT] = mlsHardwareInfoGetTickMs();
    rosPrevUpdateTime[UPDATE_TIME_PUBLISH_DRIVE] = mlsHardwareInfoGetTickMs();
	rosPrevUpdateTime[UPDATE_TIME_UPDATE_MADGWICK] = mlsHardwareInfoGetTickMs();
}

void mlsBaseControlSetup(void)
{
    robot_kinematic_handle = mlsDiffDriveInit();
    diffDriveCfg_t diff_drive_cfg = {
        .wheelRadius = WHEEL_RADIUS,
        .wheelSeperation = WHEEL_SEPARATION,
        .minLinVel = MIN_LINEAR_VELOCITY,
        .maxLinVel = MAX_LINEAR_VELOCITY,
        .minAngVel = MIN_ANGULAR_VELOCITY,
        .maxAngVel = MAX_ANGULAR_VELOCITY,
        .tickToRad = TICK2RAD
    };
    mlsDiffDriveSetConfig(robot_kinematic_handle, diff_drive_cfg);
    rosPrevUpdateTime[UPDATE_TIME_UPDATE_MADGWICK] = mlsHardwareInfoGetTickMs();
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

void mlsBaseControlUpdateVariable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
            BaseControlInitOdom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}

void mlsBaseControlPublishImuMsg(void)
{
	/* Get IMU data*/
	imuMsg = BaseControlGetIMU();

	imuMsg.header.stamp = BaseControlGetROSTime();
	imuMsg.header.frame_id = imuFrameId;

	/* Publish IMU message*/
	imuPub.publish(&imuMsg);

#ifdef PUBLISH_MAG_DEBUG
	/* Get Mag data*/
	magMsg = BaseControlGetMag();
	magMsg.header.stamp = BaseControlGetROSTime();
	magMsg.header.frame_id = magFrameId;

	/* Publish Mag message*/
	magPub.publish(&magMsg);
#endif

#ifdef USE_ROS_LOG_DEBUG
	float q0, q1, q2, q3;
	float roll, pitch, yaw;

	mlsPeriphImuGetQuat(&q0, &q1, &q2, &q3);
//	float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
//    a12 = 2.0f * (q0 * q1 + q2 * q3);
//    a22 = q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2;
//    a31 = 2.0f * (q3 * q0 + q1 * q2);
//    a32 = 2.0f * (q1 * q2 - q3 * q1);
//    a33 = q3 * q3 - q0 * q0 - q1 * q1 + q2 * q2;
//
//    roll = atan2f(a31, a33);
//    pitch = -asinf(a32);
//    yaw = atan2f(a12, a22);
//
//    roll *= 180.0f / PI;
//    pitch *= 180.0f / PI;
//    yaw *= 180.0f / PI;
    roll = 180.0 / 3.14 * atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
    pitch = 180.0 / 3.14 * asin(2 * (q0 * q2 - q3 * q1));
    yaw = 180.0 / 3.14 * atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
    yaw += magnetic_declination;

//    if (yaw >= +180.f)
//    	yaw -= 360.f;
//    else if (yaw < -180.f)
//    	yaw += 360.f;

    sprintf(rosLogBuffer, "roll: %7.4f\tpitch: %7.4f\tyaw: %7.4f\t", roll, pitch, yaw);
    rosNodeHandle.loginfo(rosLogBuffer);
#endif
}

void mlsBaseControlPublishMortorVelocityMsg(void)
{
//	/* Get motor velocity */
//	velocityMsg.linear.x = goalVelocity[LINEAR];
//	velocityMsg.angular.z = goalVelocity[ANGULAR];
//
//	/* Publish motor velocity message to "robot_vel" topic*/
//	velPub.publish(&velocityMsg);

#if(0)	//Test wheelVelocityMsg
	goalMotorVelocity[LEFT] = 100;
	goalMotorVelocity[RIGHT] = -100;
#endif
	/* Get wheel velocity (RPM) */
//	wheelVelocityMsg.data[LEFT] = goalMotorVelocity[LEFT];
//	wheelVelocityMsg.data[RIGHT] = goalMotorVelocity[RIGHT];
	float tempVelocity[2];
	tempVelocity[0] = goalMotorVelocity[0];
	tempVelocity[1] = goalMotorVelocity[1];
	wheelVelocityMsg.data_length = 2;
	wheelVelocityMsg.data = tempVelocity;

	/* Publish wheel velocity message to "robot_wheel_vel" topic*/
	wheelVelPub.publish(&wheelVelocityMsg);
}

void mlsBaseControlPublishDriveInformationMsg(void)
{
//	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_PUBLISH_DRIVE]);
//	ros::Time stamp_now = BaseControlGetROSTime();
//
//	/* Calculate odometry */
//	BaseControlCalcOdom((float)(stepTime * 0.001f));
//
//	/* Publish odometry message */
//	BaseControlUpdateOdom();
//	odometryMsg.header.stamp = stamp_now;
//	odomPub.publish(&odometryMsg);
//
//	/* Publish TF message */
//	BaseControlUpdateTf(odom_tf);
//	odom_tf.header.stamp = stamp_now;
//	tf_broadcaster.sendTransform(odom_tf);
//
//	/* Publish jointStates message */
//	BaseControlUpdateJointState();
//	joint_states.header.stamp = stamp_now;
//	jointStatesPub.publish(&joint_states);
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

	if(gGuiRxDataFrame.mode == GUI_SET_LEFT_RUN_MODE || gGuiRxDataFrame.mode == GUI_SET_LEFT_STOP_MODE)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_LEFT_SPEED_MODE;
		errorCode = mlsPeriphMotorLeftPIDGetRealValue(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		if(gGuiRxDataFrame.mode == GUI_SET_LEFT_STOP_MODE)
		{
			mlsPeriphMotorLeftPIDSetControlValue(0);
		}
		errorCode = mlsPeriphMotorLeftPIDGetControl(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
	}
	else if(gGuiRxDataFrame.mode == GUI_SET_RIGHT_RUN_MODE || gGuiRxDataFrame.mode == GUI_SET_RIGHT_STOP_MODE)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_RIGHT_SPEED_MODE;
		errorCode = mlsPeriphMotorRightPIDGetRealValue(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		if(gGuiRxDataFrame.mode == GUI_SET_RIGHT_STOP_MODE)
		{
			mlsPeriphMotorRightPIDSetControlValue(0);
		}
		errorCode = mlsPeriphMotorLeftPIDGetControl(&uartData.floatValue);
		if(errorCode != MLS_SUCCESS)
		{
			return errorCode;
		}
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
	}
	else if(getLeftParameter == 1)
	{
		gGuiTxDataFrame.mode = GUI_RECEIVE_PARAMETER_LEFT;
		/*!< Get set point */
		mlsPeriphMotorLeftPIDGetSetPoint(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get control value */
		mlsPeriphMotorLeftPIDGetControl(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorLeftPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Ki */
		mlsPeriphMotorLeftPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kd */
		mlsPeriphMotorLeftPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 16, uartData.byteArray, sizeof(FloatByteArray));
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
		/*!< Get control value */
		mlsPeriphMotorRightPIDGetControl(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 4, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kp */
		mlsPeriphMotorRightPIDGetKp(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 8, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Ki */
		mlsPeriphMotorRightPIDGetKi(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 12, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);
		/*!< Get Kd */
		mlsPeriphMotorRightPIDGetKd(&uartData.floatValue);
		memcpy(gGuiTxDataFrame.dataBuff + 16, uartData.byteArray, sizeof(FloatByteArray));
		gGuiTxDataFrame.length += sizeof(FloatByteArray);

		/*!< Clear Rx buffer */
		memset(&gGuiRxDataFrame, 0, sizeof(dataFrame_t));

		//Turn off flag
		getRightParameter = 0;
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
		mlsPeriphMotorLeftPIDClearParameter();
		break;
	case GUI_SET_LEFT_RUN_MODE:
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
		mlsPeriphMotorRightPIDClearParameter();
		break;
	case GUI_SET_RIGHT_RUN_MODE:
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
	default:

		break;
	}

	return MLS_SUCCESS;
}
#endif

mlsErrorCode_t mlsBaseControlUpdateImu(void)
{
	/* Update time */
	uint32_t stepTime = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_UPDATE_MADGWICK]);
	float deltaT = stepTime * 0.001f;
	return mlsPeriphImuUpdateQuat(deltaT);
}

mlsErrorCode_t mlsBaseControlGet9Axis(void)
{
	return mlsPeriphImuGet9Axis();
}

int32_t leftTick, rightTick;
uint32_t stepTimeTest;
void mlsBaseControlCalculatePID(void)
{

	/* Update time */
	stepTimeTest = BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_PID]);
	if(stepTimeTest <= 2)	return;
	/* Get tick from encoder */
	mlsPeriphEncoderLeftGetTick(&leftTick);
	mlsPeriphEncoderRightGetTick(&rightTick);
	/* Calculate linear velocity of 2 motors*/
	mlsPeriphMotorLeftCalculateVelocity(leftTick, stepTimeTest, &goalMotorVelocity[LEFT]);
	mlsPeriphMotorRightCalculateVelocity(rightTick, stepTimeTest, &goalMotorVelocity[RIGHT]);
	/* Update real value to PID controller */
	mlsPeriphMotorLeftPIDUpdateRealValue(goalMotorVelocity[LEFT]);
	mlsPeriphMotorRightPIDUpdateRealValue(goalMotorVelocity[RIGHT]);
	/* Calculate PID */
	mlsPeriphMotorLeftPIDCalculate(stepTimeTest);
	mlsPeriphMotorRightPIDCalculate(stepTimeTest);
}

uint32_t mlsBaseControlGetControlVelocityTime(void)
{
	return BaseControlGetElaspedTime(&rosPrevUpdateTime[UPDATE_TIME_CONTROL_ROBOT]);
}

void mlsBaseControlSetVelocityZero(void)
{
	mlsPeriphMotorLeftPIDSetSetPoint(0);
	mlsPeriphMotorRightPIDSetSetPoint(0);
}

void mlsBaseControlSetVelocityGoal(void)
{
	float rightWheelVelocity = 0.0, leftWheelVelocity = 0.0;

	mlsDiffDriveCalculateWheelVel(robot_kinematic_handle,
	                              goalVelocity[LINEAR],
	                              goalVelocity[ANGULAR],
	                              &leftWheelVelocity,
	                              &rightWheelVelocity);

	mlsPeriphMotorLeftPIDSetSetPoint(leftWheelVelocity * CONVERT_VELOCITY);
	mlsPeriphMotorRightPIDSetSetPoint(rightWheelVelocity * CONVERT_VELOCITY);
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

void mlsBaseControlUpdateTfPrefix(bool isConnected)
{
	static bool isChecked = false;
	char log_msg[60];

	if (isConnected)
	{
		if (isChecked == false)
		{
			rosNodeHandle.getParam("~tf_prefix", &get_tf_prefix);

			if (!strcmp(get_tf_prefix, ""))
			{
				sprintf(odomHeaderFrameId, "odom");
				sprintf(odomChildFrameId, "base_footprint");

				sprintf(imuFrameId, "imu_link");
				sprintf(magFrameId, "mag_link");
				sprintf(jointStateHeaderFrameId, "base_link");
			}
			else
			{
				strcpy(odomHeaderFrameId, get_tf_prefix);
				strcpy(odomChildFrameId, get_tf_prefix);

				strcpy(imuFrameId, get_tf_prefix);
				strcpy(magFrameId, get_tf_prefix);
				strcpy(jointStateHeaderFrameId, get_tf_prefix);

				strcat(odomHeaderFrameId, "/odom");
				strcat(odomChildFrameId, "/base_footprint");

				strcat(imuFrameId, "/imu_link");
				strcat(magFrameId, "/mag_link");
				strcat(jointStateHeaderFrameId, "/base_link");
			}

			sprintf(log_msg, "Setup TF on Odometry [%s]", odomHeaderFrameId);
			rosNodeHandle.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on IMU [%s]", imuFrameId);
			rosNodeHandle.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on MAG [%s]", magFrameId);
			rosNodeHandle.loginfo(log_msg);

			sprintf(log_msg, "Setup TF on JointState [%s]", jointStateHeaderFrameId);
			rosNodeHandle.loginfo(log_msg);

			isChecked = true;
		}
	}
	else
	{
		isChecked = false;
	}
}
/********** Class function implementation section *****************************/

/**@}*/
