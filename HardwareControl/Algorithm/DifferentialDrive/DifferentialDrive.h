/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file DifferentialDrive.h
 * @brief Differential Drive algorithm
 *
 * Long description.
 * @date <The date when the file was initial crated>
 * @author	Anh Vu
 */

#ifndef ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_
#define ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_

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
typedef struct diffDrive *diffDriveHandle_t;

typedef struct
{
	float wheelRadius;		/*!< Wheel radius. Unit: mm */
	float wheelSeperation; 	/*!< Wheel seperation. Unit: mm */
	float minLinVel;		/*!< Min linear velocity. Unit: m/s */
	float maxLinVel;		/*!< Max linear velocity. Unit: m/s */
	float minAngVel;		/*!< Min angular velocity. Unit: rad/s */
	float maxAngVel;		/*!< Max angular velocity. Unit: rad/s */
	float tickToRad;		/*!< Convert encoder tick to wheel position */
} diffDriveVfg_t;
/********** Type definition section *******************************************/

/********** Macro definition section*******************************************/

/********** Function declaration section **************************************/

#ifdef __cplusplus
}
#endif

#endif /* ALGORITHM_DIFFERENTIALDRIVE_DIFFERENTIALDRIVE_H_ */
/**@}*/
