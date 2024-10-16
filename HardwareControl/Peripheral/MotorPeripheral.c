/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file MotorPeripheral.c
 * @brief Library about peripheral for Motor
 *
 * Long description.
 * @date 2024-10-05
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "Peripheral.h"
#include "Motor.h"
#include "HardwareInfo.h"
#include "Encoder.h"
#include "PID.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
/* Robot parameters */
#define WHEEL_RADIUS                0.033                                   /*!< Wheel radius in meter */
#define WHEEL_SEPARATION            0.165                                   /*!< Wheel separate distance in meter */
#define TURNING_RADIUS              0.08                                    /*!< Turning radius in meter */
#define MAX_LINEAR_VELOCITY         (WHEEL_RADIUS * 2 * PI * 60 / 60)       /*!< Max linear velocity */
#define MAX_ANGULAR_VELOCITY        (MAX_LINEAR_VELOCITY / TURNING_RADIUS)  /*!< Max angular velocity */
#define MIN_LINEAR_VELOCITY         -MAX_LINEAR_VELOCITY                    /*!< Min linear velocity */
#define MIN_ANGULAR_VELOCITY        -MAX_ANGULAR_VELOCITY                   /*!< Min angular velocity */

/* Step motor direction index */
#define MOTORLEFT_DIR_FORWARD       0
#define MOTORLEFT_DIR_BACKWARD      1
#define MOTORRIGHT_DIR_FORWARD      1
#define MOTORRIGHT_DIR_BACKWARD     0

/* Encoder counter mode index */
#define ENCODER_COUNTER_MODE_UP  		0
#define ENCODER_COUNTER_MODE_DOWN  		1

/* Step driver parameters */
#define MICROSTEP_DIV               19.7        /*!< Step driver microstep divider */
#define NUM_PULSE_PER_ROUND         500         /*!< The number of pulse per round of motor */

#define PI                  3.14159265359

/*
 *  Convert from velocity (m/s) to frequency (Hz) for motor driver.
 *
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ        ((NUM_PULSE_PER_ROUND*MICROSTEP_DIV)/(2*PI*WHEEL_RADIUS))

#define DEFAULT_MOTOR_DUTY	50

/* PID default value */
#define MOTOR_LEFT_KP		0
#define MOTOR_LEFT_KI		0
#define MOTOR_LEFT_KD		0

#define MOTOR_RIGHT_KP		0
#define MOTOR_RIGHT_KI		0
#define MOTOR_RIGHT_KD		0
/********** Local (static) variable definition ********************************/
motorHandle_t motorLeftHandle = NULL;
motorHandle_t motorRightHandle = NULL;
encoderHandle_t encoderLeftHandle = NULL;
encoderHandle_t encoderRightHandle = NULL;
motorPIDHandle_t motorLeftPIDHandle = NULL;
motorPIDHandle_t motorRightPIDHandle = NULL;
/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t mlsPeriphMotorInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	/* Initialize left motor */
	motorLeftHandle = mlsMotorInit();
	if (motorLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	motorConfig_t motorLeftConfig = {
			.dir = 0,
			.duty = 0,
			.freqHz = 0,
			.setPwmDuty = mlsHardwareInfoLeftMotorSetDuty,
			.setPwmFreq = mlsHardwareInfoLeftMotorSetFrequency,
			.startPwm = mlsHardwareInfoLeftMotorStart,
			.stopPwm = mlsHardwareInfoLeftMotorStop,
			.setDir = mlsHardwareInfoLeftMotorSetDir,
	};

	errorCode = mlsMotorSetConfig(motorLeftHandle, motorLeftConfig);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Initialize right motor */
	motorRightHandle = mlsMotorInit();
	if (motorRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	motorConfig_t motorRightConfig = {
			.dir = 0,
			.duty = 0,
			.freqHz = 0,
			.setPwmDuty = mlsHardwareInfoRightMotorSetDuty,
			.setPwmFreq = mlsHardwareInfoRightMotorSetFrequency,
			.startPwm = mlsHardwareInfoRightMotorStart,
			.stopPwm = mlsHardwareInfoRightMotorStop,
			.setDir = mlsHardwareInfoRightMotorSetDir,
	};

	errorCode = mlsMotorSetConfig(motorRightHandle, motorRightConfig);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	mlsMotorSetPwmFreq(motorLeftHandle, 0);
	mlsMotorSetPwmDuty(motorLeftHandle, DEFAULT_MOTOR_DUTY);
	mlsMotorSetDir(motorLeftHandle, MOTORLEFT_DIR_FORWARD);
	mlsMotorStart(motorLeftHandle);

	mlsMotorSetPwmFreq(motorRightHandle, 0);
	mlsMotorSetPwmDuty(motorRightHandle, DEFAULT_MOTOR_DUTY);
	mlsMotorSetDir(motorRightHandle, MOTORRIGHT_DIR_FORWARD);
	mlsMotorStart(motorRightHandle);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftStart(void)
{
	if(motorLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorStart(motorLeftHandle);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftStop(void)
{
	if(motorLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorStop(motorLeftHandle);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftSetSpeed(float speed)
{
	if(motorLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	if (speed < 0)
	{
		mlsMotorSetDir(motorLeftHandle, MOTORLEFT_DIR_BACKWARD);
		mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_DOWN);
		mlsMotorSetPwmFreq(motorLeftHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorLeftHandle, DEFAULT_MOTOR_DUTY);
	}
	else
	{
		mlsMotorSetDir(motorLeftHandle, MOTORLEFT_DIR_FORWARD);
		mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_UP);
		mlsMotorSetPwmFreq(motorLeftHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorLeftHandle, DEFAULT_MOTOR_DUTY);
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftSetDir(uint8_t dir)
{
	if(motorLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorSetDir(motorLeftHandle, dir);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightStart(void)
{
	if(motorRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorStart(motorRightHandle);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightStop(void)
{
	if(motorRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorStop(motorRightHandle);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightSetSpeed(float speed)
{
	if(motorRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	if (speed < 0)
	{
		mlsMotorSetDir(motorRightHandle, MOTORRIGHT_DIR_BACKWARD);
		mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_DOWN);
		mlsMotorSetPwmFreq(motorRightHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorRightHandle, DEFAULT_MOTOR_DUTY);
	}
	else
	{
		mlsMotorSetDir(motorRightHandle, MOTORRIGHT_DIR_FORWARD);
		mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_UP);
		mlsMotorSetPwmFreq(motorRightHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorRightHandle, DEFAULT_MOTOR_DUTY);
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightSetDir(uint8_t dir)
{
	if(motorRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorSetDir(motorRightHandle, dir);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphEncoderInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	/* Initialize left encoder */
	encoderLeftHandle = mlsEncoderInit();
	if(encoderLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	encoderConfig_t encoderLeftConfig = {
			.maxReload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
			.startEnc = mlsHardwareInfoLeftEncoderStart,
			.stopEnc = mlsHardwareInfoLeftEncoderStop,
			.setCounter = mlsHardwareInfoLeftEncoderSetCounter,
			.getCounter = mlsHardwareInfoLeftEncoderGetCounter,
			.setMode = mlsHardwareInfoLeftEncoderSetMode,
	};

	errorCode = mlsEncoderSetConfig(encoderLeftHandle, encoderLeftConfig);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Initialize right encoder */
	encoderRightHandle = mlsEncoderInit();
	if(encoderRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	encoderConfig_t encoderRightConfig = {
			.maxReload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
			.startEnc = mlsHardwareInfoRightEncoderStart,
			.stopEnc = mlsHardwareInfoRightEncoderStop,
			.setCounter = mlsHardwareInfoRightEncoderSetCounter,
			.getCounter = mlsHardwareInfoRightEncoderGetCounter,
			.setMode = mlsHardwareInfoRightEncoderSetMode,
	};

	errorCode = mlsEncoderSetConfig(encoderRightHandle, encoderRightConfig);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_UP);
	mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_UP);

	mlsEncoderStart(encoderLeftHandle);
	mlsEncoderStart(encoderRightHandle);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphEncoderLeftGetTick(uint32_t *tick)
{
	if(encoderLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	uint32_t temp;

	mlsEncoderGetCounter(encoderLeftHandle, &temp);
	mlsEncoderSetCounter(encoderLeftHandle, MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	*tick = temp - (MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphEncoderRightGetTick(uint32_t *tick)
{
	if(encoderRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	uint32_t temp;

	mlsEncoderGetCounter(encoderRightHandle, &temp);
	mlsEncoderSetCounter(encoderRightHandle, MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	*tick = temp - (MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorPIDInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;

	/* Initialize left motor PID */
	motorLeftPIDHandle = mlsMotorPIDInit();
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	motorPIDCfg_t motorLeftPIDConfig = {
			.Kp = MOTOR_LEFT_KP,
			.Ki = MOTOR_LEFT_KI,
			.Kd = MOTOR_LEFT_KD,
			.setPoint = 0,
	};

	errorCode = mlsMotorPIDSetConfig(motorLeftPIDHandle, motorLeftPIDConfig);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	/* Initialize right encoder */
	motorRightPIDHandle = mlsMotorPIDInit();
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	motorPIDCfg_t motorRightPIDConfig = {
			.Kp = MOTOR_RIGHT_KP,
			.Ki = MOTOR_RIGHT_KI,
			.Kd = MOTOR_RIGHT_KD,
			.setPoint = 0,
	};

	errorCode = mlsMotorPIDSetConfig(motorRightPIDHandle, motorRightPIDConfig);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetKp(float Kp)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKp(motorLeftPIDHandle, Kp);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetKi(float Ki)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKi(motorLeftPIDHandle, Ki);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetKd(float Kd)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKd(motorLeftPIDHandle, Kd);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetSetPoint(float setPoint)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetSetPoint(motorLeftPIDHandle, setPoint);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetRealValue(float *realValue)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetRealVaule(motorLeftPIDHandle, realValue);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetKp(float Kp)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKp(motorRightPIDHandle, Kp);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetKi(float Ki)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKi(motorRightPIDHandle, Ki);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetKd(float Kd)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetKd(motorRightPIDHandle, Kd);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetSetPoint(float setPoint)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetSetPoint(motorRightPIDHandle, setPoint);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetRealValue(float *realValue)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetRealVaule(motorRightPIDHandle, realValue);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetKp(float *Kp)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKp(motorLeftPIDHandle, Kp);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetKi(float *Ki)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKi(motorLeftPIDHandle, Ki);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetKd(float *Kd)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKd(motorLeftPIDHandle, Kd);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetSetPoint(float *setPoint)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetSetPoint(motorLeftPIDHandle, setPoint);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetKp(float *Kp)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKp(motorRightPIDHandle, Kp);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetKi(float *Ki)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKi(motorRightPIDHandle, Ki);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetKd(float *Kd)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetKd(motorRightPIDHandle, Kd);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetSetPoint(float *setPoint)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetSetPoint(motorRightPIDHandle, setPoint);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}
/**@}*/
