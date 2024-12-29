/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file <fileName>.c
 * @brief <Brief description>
 *
 * Long description.
 * @date 2024-07-13
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "HardwareInfo.h"
#include "mpu9250.h"
#include "ak8963.h"
/********** Local Constant and compile switch definition section **************/

/********** Local Type definition section *************************************/
#define TIMER_LEFT_MAX_RELOAD 			0xFFFFFFFF
#define HW_LEFTMOTOR_TIM_HANDLE 		htim1
#define HW_LEFTMOTOR_TIM 				TIM1
#define HW_LEFTMOTOR_TIM_CCR 			CCR1
#define HW_LEFTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_1
#define HW_LEFTMOTOR_TIM_CLK_FREQ 		168000000
#define HW_LEFTMOTOR_GPIO 				GPIOE
#define HW_LEFTMOTOR_GPIO_PIN 			GPIO_PIN_10

#define TIMER_RIGHT_MAX_RELOAD 			0xFFFF
#define HW_RIGHTMOTOR_TIM_HANDLE 		htim4
#define HW_RIGHTMOTOR_TIM 				TIM1
#define HW_RIGHTMOTOR_TIM_CCR 			CCR1
#define HW_RIGHTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_1
#define HW_RIGHTMOTOR_TIM_CLK_FREQ 		84000000
#define HW_RIGHTMOTOR_GPIO 				GPIOD
#define HW_RIGHTMOTOR_GPIO_PIN 			GPIO_PIN_11

#define HW_LEFT_ENCODER_TIM_HANDLE 		htim2
#define HW_LEFT_ENCODER_TIM 			TIM2

#define HW_RIGHT_ENCODER_TIM_HANDLE 	htim3
#define HW_RIGHT_ENCODER_TIM 			TIM3

#define ENCODER_COUNTER_MODE_UP  		0
#define ENCODER_COUNTER_MODE_DOWN  		1
/********** Local Macro definition section ************************************/

/********** Global variable definition section ********************************/
uint32_t gBaseControlTimeUpdate[10];
uint8_t gBufferRxData[UART_MAX_LENGTH];
dataFrame_t gGuiRxDataFrame;
dataFrame_t gGuiTxDataFrame;
uint8_t gGuiUpdateParameter = 0;

extern uint8_t gBaseControlTimeUpdateFlag[10];
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6)
  {
    if(gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] >= 10000/IMU_PUBLISH_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[IMU_PUBLISH_TIME_INDEX] = 1;
    	gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX] >= 10000/IMU_UPDATE_FREQUENCY)
	{
    	gBaseControlTimeUpdateFlag[IMU_UPDATE_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[IMU_USING_FILTER_INDEX] >= 10000/IMU_FILTER_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[IMU_USING_FILTER_INDEX] = 1;
		gBaseControlTimeUpdate[IMU_USING_FILTER_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] >= 10000/CONTROL_MOTOR_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[CONTROL_MOTOR_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX] = 0;
    }

    if(gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] >= 10000/VEL_PUBLISH_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[VEL_PUBLISH_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] >= 10000/DRIVE_INFORMATION_FREQUENCY)
	{
		gBaseControlTimeUpdateFlag[DRIVE_INFORMATION_TIME_INDEX] = 1;
		gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX] = 0;
	}

    if(gBaseControlTimeUpdate[COMPUTE_PID_CONTROLLER_INDEX] >= 10000/COMPUTE_PID_CONTROLLER_FREQUENCY)
    {
    	gBaseControlTimeUpdateFlag[COMPUTE_PID_CONTROLLER_INDEX] = 1;
    	gBaseControlTimeUpdate[COMPUTE_PID_CONTROLLER_INDEX] = 0;
    }

    gBaseControlTimeUpdate[IMU_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[IMU_UPDATE_TIME_INDEX]++;
	gBaseControlTimeUpdate[IMU_USING_FILTER_INDEX]++;
	gBaseControlTimeUpdate[CONTROL_MOTOR_TIME_INDEX]++;
	gBaseControlTimeUpdate[VEL_PUBLISH_TIME_INDEX]++;
	gBaseControlTimeUpdate[DRIVE_INFORMATION_TIME_INDEX]++;
	gBaseControlTimeUpdate[COMPUTE_PID_CONTROLLER_INDEX]++;
  }
}

// handles data of gGuiRxDataFrame when complete receive MAX_LEN byte from pc
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == huart2.Instance){
		//Get data from buffer
		memcpy(&gGuiRxDataFrame, gBufferRxData, UART_MAX_LENGTH);
		if((gGuiRxDataFrame.header == 0x0A) && (gGuiRxDataFrame.length == (uint8_t)(Size - 4)) && (gGuiRxDataFrame.footer == 0x05))
		{
			memcpy(&gGuiTxDataFrame, &gGuiRxDataFrame, sizeof(gGuiRxDataFrame));
			gGuiTxDataFrame.footer = 0x06;
			//Send back to PC
			HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&gGuiTxDataFrame, sizeof(gGuiTxDataFrame));
			//Turn on flag
			gGuiUpdateParameter = 1;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gBufferRxData, UART_MAX_LENGTH); // start uart rx DMA again
	}
}

/********** Global function definition section ********************************/
void mlsHardwareInfoDelay(uint32_t timeMs)
{
	HAL_Delay(timeMs);
}

uint32_t mlsHardwareInfoGetTickMs(void)
{
	return HAL_GetTick();
}

mlsErrorCode_t mlsHardwareInfoStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = (mlsErrorCode_t)HAL_TIM_Base_Start_IT(timBaseHandle);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	uint8_t tmpBuffer[1];
	mlsErrorCode_t errorCode = MLS_ERROR;

	tmpBuffer[0] = regAddr;
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, tmpBuffer, 1, 100);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	errorCode = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, buffer, len, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoMpu9250WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	uint8_t bufferSend[len+1];

	bufferSend[0] = regAddr;
	for(uint8_t i = 0; i < len; i++)
	{
		bufferSend[i + 1] = buffer[i];
	}
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, bufferSend, len + 1, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoAk8963ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	uint8_t tmpBuffer[1];
	mlsErrorCode_t errorCode = MLS_ERROR;

	tmpBuffer[0] = regAddr;
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, tmpBuffer, 1, 100);
	if(errorCode != MLS_SUCCESS)
	{
		return errorCode;
	}
	errorCode = HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDRESS, buffer, len, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoAk8963WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	uint8_t bufferSend[len+1];

	bufferSend[0] = regAddr;
	for(uint8_t i = 0; i < len; i++)
	{
		bufferSend[i + 1] = buffer[i];
	}
	errorCode = HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDRESS, bufferSend, len + 1, 100);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoUartReadBytes(uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = HAL_UART_Receive_DMA(&huart2, buffer, len);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoUartWriteBytes(uint8_t *buffer, uint16_t len)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	errorCode = HAL_UART_Transmit_DMA(&huart2, buffer, len);
	return errorCode;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetDuty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t computeValue;
	computeValue = duty * (HW_LEFTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, computeValue);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetFrequency(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, 0);

		return MLS_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apbFreq = HW_LEFTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t)(apbFreq / freq);
	uint16_t timerPrescaler = conduct / TIMER_LEFT_MAX_RELOAD + 1;
	uint16_t timerPeriod = (uint16_t)(conduct / (timerPrescaler + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, timerPeriod);
	__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, timerPrescaler);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorStart(void)
{
	HAL_TIM_PWM_Start(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorStop(void)
{
	HAL_TIM_PWM_Stop(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftMotorSetDir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_LEFTMOTOR_GPIO, HW_LEFTMOTOR_GPIO_PIN, dir);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetDuty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t computeValue;
	computeValue = duty * (HW_RIGHTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, computeValue);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetFrequency(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, 0);

		return MLS_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apbFreq = HW_RIGHTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t)(apbFreq / freq);
	uint16_t timerPrescaler = conduct / TIMER_RIGHT_MAX_RELOAD + 1;
	uint16_t timerPeriod = (uint16_t)(conduct / (timerPrescaler + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, timerPeriod);
	__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, timerPrescaler);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorStart(void)
{
	HAL_TIM_PWM_Start(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorStop(void)
{
	HAL_TIM_PWM_Stop(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightMotorSetDir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_RIGHTMOTOR_GPIO, HW_RIGHTMOTOR_GPIO_PIN, dir);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftEncoderStart(void)
{
//	HAL_TIM_Base_Start(&HW_LEFT_ENCODER_TIM_HANDLE);
	HAL_TIM_Encoder_Start(&HW_LEFT_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftEncoderStop(void)
{
//	HAL_TIM_Base_Stop(&HW_LEFT_ENCODER_TIM_HANDLE);
	HAL_TIM_Encoder_Stop(&HW_LEFT_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftEncoderSetCounter(uint32_t value)
{
	__HAL_TIM_SET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE, value);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftEncoderGetCounter(uint32_t *value)
{
	*value = __HAL_TIM_GET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoLeftEncoderSetMode(uint8_t mode)
{
	/* Reconfigure timer init parameters */
	HW_LEFT_ENCODER_TIM_HANDLE.Instance                 = HW_LEFT_ENCODER_TIM;
	HW_LEFT_ENCODER_TIM_HANDLE.Init.Prescaler           = 0;
	if (mode == ENCODER_COUNTER_MODE_UP) {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	} else {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_DOWN;
	}
	HW_LEFT_ENCODER_TIM_HANDLE.Init.Period              = __HAL_TIM_GET_AUTORELOAD(&HW_LEFT_ENCODER_TIM_HANDLE);
	HW_LEFT_ENCODER_TIM_HANDLE.Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
	HW_LEFT_ENCODER_TIM_HANDLE.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;

	/* Keep last counter value */
	uint32_t last_counter_val = __HAL_TIM_GET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE);

	/* Set timer counter mode */
	HAL_TIM_Base_Init(&HW_LEFT_ENCODER_TIM_HANDLE);

	/* Set timer last counter value */
	__HAL_TIM_SET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE, last_counter_val);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightEncoderStart(void)
{
//	HAL_TIM_Base_Start(&HW_RIGHT_ENCODER_TIM_HANDLE);
	HAL_TIM_Encoder_Start(&HW_RIGHT_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightEncoderStop(void)
{
//	HAL_TIM_Base_Stop(&HW_RIGHT_ENCODER_TIM_HANDLE);
	HAL_TIM_Encoder_Stop(&HW_LEFT_ENCODER_TIM_HANDLE, TIM_CHANNEL_ALL);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightEncoderSetCounter(uint32_t value)
{
	__HAL_TIM_SET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE, value);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightEncoderGetCounter(uint32_t *value)
{
	*value = __HAL_TIM_GET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE);

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsHardwareInfoRightEncoderSetMode(uint8_t mode)
{
	/* Reconfigure timer init parameters */
	HW_RIGHT_ENCODER_TIM_HANDLE.Instance                 = HW_RIGHT_ENCODER_TIM;
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.Prescaler           = 0;
	if (mode == ENCODER_COUNTER_MODE_UP) {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	} else {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_DOWN;
	}
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.Period              = __HAL_TIM_GET_AUTORELOAD(&HW_RIGHT_ENCODER_TIM_HANDLE);
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;

	/* Keep last counter value */
	uint32_t last_counter_val = __HAL_TIM_GET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE);

	/* Set timer counter mode */
	HAL_TIM_Base_Init(&HW_RIGHT_ENCODER_TIM_HANDLE);

	/* Set timer last counter value */
	__HAL_TIM_SET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE, last_counter_val);

	return MLS_SUCCESS;
}
/**@}*/
