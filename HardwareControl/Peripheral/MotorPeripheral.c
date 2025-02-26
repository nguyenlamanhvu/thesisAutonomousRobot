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
#include "BaseControl.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/
/*
 *  Convert from velocity (m/s) to frequency (Hz) for motor driver.
 *
 *                      2*pi*WHELL_RADIUS
 *  velocity (m/s) =  ------------------------------
 *                    NUM_PULSE_PER_ROUND * STEP_DIV
 *
 */
#define VEL2FREQ        ((NUM_PULSE_PER_ROUND*MICROSTEP_DIV)/(2*PI*WHEEL_RADIUS))

/* Convert motor tick to angular in radian */
#define TICK2RAD        360.0f/(NUM_PULSE_PER_ROUND*MICROSTEP_DIV)*PI/180.0f

/* Convert motor tick to the percentage of wheel */
#define TICK2WHEEL		1.0f/(NUM_PULSE_PER_ROUND * MICROSTEP_DIV)

#define DEFAULT_MOTOR_DUTY			0
#define DEFAULT_MOTOR_FREQUENCY		12500

/* PID default value */
#define MOTOR_LEFT_KP		1.68
#define MOTOR_LEFT_KI		4.5
#define MOTOR_LEFT_KD		0.00175

#define MOTOR_RIGHT_KP		1.68
#define MOTOR_RIGHT_KI		4.5
#define MOTOR_RIGHT_KD		0.00175
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
float mlsPeriphMotorConstrain(float x, float lowVal, float highVal)
{
	float value;
	if (x > highVal)
	{
		value = highVal;
	}
	else if (x < lowVal)
	{
		value = lowVal;
	}
	else
	{
		value = x;
	}
	return value;
}

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

	mlsMotorSetPwmFreq(motorLeftHandle, DEFAULT_MOTOR_FREQUENCY);
	mlsMotorSetPwmDuty(motorLeftHandle, DEFAULT_MOTOR_DUTY);
	mlsMotorSetDir(motorLeftHandle, MOTORLEFT_DIR_FORWARD);
	mlsMotorStart(motorLeftHandle);

	mlsMotorSetPwmFreq(motorRightHandle, DEFAULT_MOTOR_FREQUENCY);
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
//		mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_DOWN);
//		mlsMotorSetPwmFreq(motorLeftHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorLeftHandle, -speed);
	}
	else
	{
		mlsMotorSetDir(motorLeftHandle, MOTORLEFT_DIR_FORWARD);
//		mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_UP);
//		mlsMotorSetPwmFreq(motorLeftHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorLeftHandle, speed);
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
//		mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_DOWN);
//		mlsMotorSetPwmFreq(motorRightHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorRightHandle, -speed);
	}
	else
	{
		mlsMotorSetDir(motorRightHandle, MOTORRIGHT_DIR_FORWARD);
//		mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_UP);
//		mlsMotorSetPwmFreq(motorRightHandle, (uint32_t)(-speed*VEL2FREQ));
		mlsMotorSetPwmDuty(motorRightHandle, speed);
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

	mlsEncoderSetCounter(encoderLeftHandle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);
	mlsEncoderSetCounter(encoderRightHandle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	mlsEncoderSetMode(encoderLeftHandle, ENCODER_COUNTER_MODE_UP);
	mlsEncoderSetMode(encoderRightHandle, ENCODER_COUNTER_MODE_UP);

	mlsEncoderStart(encoderLeftHandle);
	mlsEncoderStart(encoderRightHandle);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphEncoderLeftGetTick(int32_t *tick)
{
	if(encoderLeftHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	uint32_t temp;

	mlsEncoderGetCounter(encoderLeftHandle, &temp);
	mlsEncoderSetCounter(encoderLeftHandle, MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	//Divide because encoder is used in x4 mode
	*tick = (temp  - (MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2)) / 4;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftCalculateVelocity(int32_t tick, uint32_t stepTime, float *velocity)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	// Convert tick/s -> RPM
//	*velocity = (tick * 1000.0 * TICK2RAD * WHEEL_RADIUS) / stepTime;
	*velocity = (tick * 1000.0 * TICK2WHEEL *60) / stepTime;

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphEncoderRightGetTick(int32_t *tick)
{
	if(encoderRightHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	uint32_t temp;

	mlsEncoderGetCounter(encoderRightHandle, &temp);
	mlsEncoderSetCounter(encoderRightHandle, MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2);

	//Divide temp because encoder is used in x4 mode
	*tick = (temp  - (MICROSTEP_DIV * NUM_PULSE_PER_ROUND / 2)) / 4;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightCalculateVelocity(int32_t tick, uint32_t stepTime, float *velocity)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	// Convert tick/s -> RPM
//	*velocity = (tick * 1000.0 * TICK2RAD * WHEEL_RADIUS) / stepTime;
	*velocity = (tick * 1000.0 * TICK2WHEEL *60) / stepTime;

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

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
			.stepTime = 0.012,
			.controlValue = 0,
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
			.stepTime = 0.012,
			.controlValue = 0,
	};

	errorCode = mlsMotorPIDSetConfig(motorRightPIDHandle, motorRightPIDConfig);
	if (errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	HAL_Delay(10000);
	mlsMotorPIDSetSetPoint(motorLeftPIDHandle, 50);
	mlsMotorPIDSetSetPoint(motorRightPIDHandle, 50);

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

mlsErrorCode_t mlsPeriphMotorLeftPIDCalculate(uint32_t stepTime)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = mlsMotorPIDCalculate(motorLeftPIDHandle,(float)stepTime / 1000.0);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDCalculate(uint32_t stepTime)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = mlsMotorPIDCalculate(motorRightPIDHandle,(float)stepTime / 1000.0);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDUpdateRealValue(float realValue)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDUpdateRealValue(motorLeftPIDHandle, realValue);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDUpdateRealValue(float realValue)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDUpdateRealValue(motorRightPIDHandle, realValue);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetControl(void)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;
	float controlValue;

	errorCode = mlsMotorPIDGetControlValue(motorLeftPIDHandle, &controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = mlsPeriphMotorLeftSetSpeed(controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDGetControl(float* controlValue)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetControlValue(motorLeftPIDHandle, controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDSetControlValue(float controlValue)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetControlValue(motorLeftPIDHandle, controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetControl(void)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;
	float controlValue;

	errorCode = mlsMotorPIDGetControlValue(motorRightPIDHandle, &controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	errorCode = mlsPeriphMotorRightSetSpeed(controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDGetControl(float* controlValue)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDGetControlValue(motorRightPIDHandle, controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDSetControlValue(float controlValue)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;

	errorCode = mlsMotorPIDSetControlValue(motorRightPIDHandle, controlValue);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorLeftPIDClearParameter(void)
{
	if(motorLeftPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	mlsErrorCode_t errorCode = MLS_ERROR;
	motorPIDCfg_t motorLeftPIDConfig = {
			.Kp = MOTOR_LEFT_KP,
			.Ki = MOTOR_LEFT_KI,
			.Kd = MOTOR_LEFT_KD,
	};

	errorCode = mlsMotorPIDClearParameter(motorLeftPIDHandle, motorLeftPIDConfig);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsPeriphMotorRightPIDClearParameter(void)
{
	if(motorRightPIDHandle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	mlsErrorCode_t errorCode = MLS_ERROR;
	motorPIDCfg_t motorRightPIDConfig = {
			.Kp = MOTOR_LEFT_KP,
			.Ki = MOTOR_LEFT_KI,
			.Kd = MOTOR_LEFT_KD,
	};

	errorCode = mlsMotorPIDClearParameter(motorRightPIDHandle, motorRightPIDConfig);

	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}

	return MLS_SUCCESS;
}
/**@}*/
