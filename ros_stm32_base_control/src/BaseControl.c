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

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/

/********** Local function definition section *********************************/

/********** Global function definition section ********************************/
mlsErrorCode_t mlsBaseControlInit(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	/* Initialize ROS*/
	mlsBaseControlROSSetup();

	errorCode = MLS_SUCCESS;
	return errorCode;
}

mlsErrorCode_t mlsBaseControlMain(void)
{
	mlsErrorCode_t errorCode = MLS_ERROR;
	/* Send log message*/
	mlsBaseControlSendLogMsg();

	/* Spin NodeHandle to keep synchorus */
	mlsBaseControlSpinOnce();

	/* Keep rosserial connection */
	mlsBaseControlWaitSerialLink(mlsBaseControlConnectStatus());

	errorCode = MLS_SUCCESS;
	return errorCode;
}

/**@}*/
