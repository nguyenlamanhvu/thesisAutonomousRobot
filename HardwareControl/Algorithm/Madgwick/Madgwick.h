/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file Madgwick.h
 * @brief Madgwick filter fuses the IMU and optionally the MARG.
 *
 * Long description.
 * @date 2024-09-30
 * @author	Anh Vu
 */


#ifndef ALGORITHM_MADGWICK_MADGWICK_H_
#define ALGORITHM_MADGWICK_MADGWICK_H_

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

/********** Type definition section *******************************************/
typedef struct imuMadgwick *imuMadgwickHandle_t;
/**
 * @brief   Configuration structure.
 */
typedef struct {
    float beta;
    float sampleFreq;
} imuMadgwickCfg_t;
/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/
/*
 * @brief   Configure Madgwick AHRS parameters.
 *
 * @param   config Struct pointer.
 *
 * @return
 *      - Madgwick handle structure: Success.
 *      - 0:                         Fail.
 */
imuMadgwickHandle_t mlsImuMadgwickInit(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsImuMadgwickSetConfig(imuMadgwickHandle_t handle, imuMadgwickCfg_t config);

/*
 * @brief   Set beta value.
 *
 * @param   handle Handle structure.
 * @param   beta Beta.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsImuMadgwickSetBeta(imuMadgwickHandle_t handle, float beta);

/*
 * @brief   Set beta value.
 *
 * @param   handle Handle structure.
 * @param   sampleFreq sampleFreq.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsImuMadgwickSetFrequency(imuMadgwickHandle_t handle, float sampleFreq);

/*
 * @brief   Get quaternion.
 *
 * @param   handle Handle structure.
 * @param   q0, q1, q2 q3 Quaternion data.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail.
 */
mlsErrorCode_t mlsImuMadgwickGetQuaternion(imuMadgwickHandle_t handle, float *q0, float *q1, float *q2, float *q3);

/*
 * @brief   Update Madgwick AHRS quaternion with 6 motions.
 *
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail
 */
mlsErrorCode_t mlsImuMadgwickUpdate6Dof(imuMadgwickHandle_t handle, float gx, float gy, float gz, float ax, float ay, float az, float deltaT);

/*
 * @brief   Update Madgwick AHRS quaternion with 9 motions.
 *
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 * @param   mx Magnetometer along x axis.
 * @param   my Magnetometer along y axis.
 * @param   mz Magnetometer along z axis.
 *
 * @return
 *      - MLS_SUCCESS: Success.
 *      - Others:      Fail
 */
mlsErrorCode_t mlsImuMadgwickUpdate9Dof(imuMadgwickHandle_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltaT);
#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_MADGWICK_MADGWICK_H_ */
/**@}*/
