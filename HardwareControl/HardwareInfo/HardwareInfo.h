/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file HardwareInfo.h
 * @brief Information of STM32F407
 *
 * Long description.
 * @date 2024-07-11
 * @author	Anh Vu
 */


#ifndef HARDWARECONTROL_INC_HARDWAREINFO_H_
#define HARDWARECONTROL_INC_HARDWAREINFO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/********** Include section ***************************************************/
#include "errorCode.h"
#include "BaseControl.h"
#include "main.h"
#include "compilerSwitch.h"
/********** Constant  and compile switch definition section *******************/

/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief Base Control Init
 * @param timeMs
 * @return Error Code
 */
void mlsHardwareInfoDelay(uint32_t timeMs);

/*
 * @brief Start timer interrupt
 * @param timBaseHandle
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoStartTimerInterrupt(TIM_HandleTypeDef* timBaseHandle);

/*
 * @brief Read data from MPU9250
 * @param[1] address of register in MPU9250
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoMpu9250ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Write data to MPU9250
 * @param[1] address of register in MPU9250
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoMpu9250WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Read data from AK8963
 * @param[1] address of register in AK8963
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoAk8963ReadBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Write data from AK8963
 * @param[1] address of register in AK8963
 * @param[2] buffer which store data
 * @param[3] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoAk8963WriteBytes(uint8_t regAddr, uint8_t *buffer, uint16_t len);

/*
 * @brief Read data from UART
 * @param[1] buffer which store data
 * @param[2] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoUartReadBytes(uint8_t *buffer, uint16_t len);

/*
 * @brief Write data from UART
 * @param[1] buffer which store data
 * @param[2] length of data
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoUartWriteBytes(uint8_t *buffer, uint16_t len);

/*
 * @brief Set duty cycle of Left Motor
 * @param[1] duty cycle
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftMotorSetDuty(float duty);

/*
 * @brief Set frequency of Left Motor
 * @param[1] frequency
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftMotorSetFrequency(uint32_t freq);

/*
 * @brief Start Left Motor
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftMotorStart(void);

/*
 * @brief Stop Left Motor
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftMotorStop(void);

/*
 * @brief Set direction of Left Motor
 * @param[1] direction
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftMotorSetDir(uint8_t dir);

/*
 * @brief Set duty cycle of Right Motor
 * @param[1] duty cycle
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightMotorSetDuty(float duty);

/*
 * @brief Set frequency of Right Motor
 * @param[1] frequency
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightMotorSetFrequency(uint32_t freq);

/*
 * @brief Start Right Motor
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightMotorStart(void);

/*
 * @brief Stop Right Motor
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightMotorStop(void);

/*
 * @brief Set direction of Right Motor
 * @param[1] direction
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightMotorSetDir(uint8_t dir);

/*
 * @brief Start Left Encoder
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftEncoderStart(void);

/*
 * @brief Stop Left Encoder
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftEncoderStop(void);

/*
 * @brief Set counter of Left Encoder
 * @param[1] Counter value
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftEncoderSetCounter(uint32_t value);

/*
 * @brief Get counter of Left Encoder
 * @param[1] Pointer counter value
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftEncoderGetCounter(uint32_t *value);

/*
 * @brief Set mode of Left Encoder
 * @param[1] Encoder mode
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoLeftEncoderSetMode(uint8_t mode);

/*
 * @brief Start Right Encoder
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightEncoderStart(void);

/*
 * @brief Stop Right Encoder
 * @param	none
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightEncoderStop(void);

/*
 * @brief Set counter of Right Encoder
 * @param[1] Counter value
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightEncoderSetCounter(uint32_t value);

/*
 * @brief Get counter of Right Encoder
 * @param[1] Pointer counter value
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightEncoderGetCounter(uint32_t *value);

/*
 * @brief Set mode of Right Encoder
 * @param[1] Encoder mode
 * @return Error Code
 */
mlsErrorCode_t mlsHardwareInfoRightEncoderSetMode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* HARDWARECONTROL_INC_HARDWAREINFO_H_ */
/**@}*/
