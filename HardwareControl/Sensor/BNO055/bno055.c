/**
 * @defgroup <groupIdentifier>	<Group Name> //Optional
 * @ingroup	superGroupIdentifier             //Optional
 * @brief <Brief description>
 *
 * <Long description>
 * @{
 */

/**
 * @file bno055.c
 * @brief Library for IMU BNO055
 *
 * Long description.
 * @date 26-02-2025
 * @author	Anh Vu
 */

/********** Include section ***************************************************/
#include "bno055.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "gpio.h"
#include "HardwareInfo.h"
/********** Local Constant and compile switch definition section **************/
extern uint8_t systemSensor, gyro, accel, mag;
/********** Local Type definition section *************************************/

/********** Local Macro definition section ************************************/

/********** Local (static) variable definition ********************************/

/********** Local (static) function declaration section ***********************/
static mlsErrorCode_t bno055_reset(bno055Handle_t handle);
static mlsErrorCode_t bno055_set_page(bno055Handle_t handle, const bno055_page_t page);
static mlsErrorCode_t bno055_on(bno055Handle_t handle);
static mlsErrorCode_t bno055_set_unit(bno055Handle_t handle, const bno055_temp_unitsel_t t_unit,
                          const bno055_gyr_unitsel_t g_unit,
                          const bno055_acc_unitsel_t a_unit,
                          const bno055_eul_unitsel_t e_unit);
/********** Local function definition section *********************************/
static mlsErrorCode_t bno055_HW_reset(bno055Handle_t handle) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	HAL_GPIO_WritePin(RESET_IMU_PIN_GPIO_Port, RESET_IMU_PIN_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESET_IMU_PIN_GPIO_Port, RESET_IMU_PIN_Pin, 1);
    return MLS_SUCCESS;
}

static mlsErrorCode_t bno055_reset(bno055Handle_t handle) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

    u8 data = 0x20U;
    if (handle->i2cWrite(BNO_SYS_TRIGGER, &data, 1) != MLS_SUCCESS) {
        return BNO_ERR_I2C;
    }
    return MLS_SUCCESS;
}

static mlsErrorCode_t bno055_set_page(bno055Handle_t handle, const bno055_page_t page) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	if (handle->_page != page) {
        return MLS_SUCCESS;
    }
    if (page > 0x01) {
        return BNO_ERR_PAGE_TOO_HIGH;
    }
    mlsErrorCode_t err;
    err = handle->i2cWrite(BNO_PAGE_ID, (u8*)&page, 1);
    if (err != MLS_SUCCESS) {
        return err;
    }
    handle->_page = page;
    HAL_Delay(2);
    return MLS_SUCCESS;
}

static mlsErrorCode_t bno055_on(bno055Handle_t handle) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

    u8 data = 0x00U;
    if (handle->i2cWrite(BNO_SYS_TRIGGER, &data, 1) != MLS_SUCCESS) {
        return BNO_ERR_I2C;
    }
    return MLS_SUCCESS;
}

static mlsErrorCode_t bno055_set_unit(bno055Handle_t handle, const bno055_temp_unitsel_t t_unit,
                          const bno055_gyr_unitsel_t g_unit,
                          const bno055_acc_unitsel_t a_unit,
                          const bno055_eul_unitsel_t e_unit) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

    mlsErrorCode_t err;
    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
    uint8_t data = t_unit | g_unit | a_unit | e_unit;
    if ((err = handle->i2cWrite(BNO_UNIT_SEL, &data, 1)) != MLS_SUCCESS) {
        return err;
    }
    handle->_gyr_unit = g_unit;
    handle->_acc_unit = a_unit;
    handle->_eul_unit = e_unit;
    handle->_temp_unit = t_unit;

//    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
//        return err;
//    }
    return MLS_SUCCESS;
}

/********** Global function definition section ********************************/
bno055Handle_t mlsBno055Init(void)
{
	bno055Handle_t handle = calloc(1, sizeof(bno055_t));
	if(handle == NULL)
	{
		return NULL;
	}
	return handle;
}

mlsErrorCode_t mlsBno055SetConfig(bno055Handle_t handle, bno055Config_t config)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	/* Update handle structure */
	handle->mode = config.mode;
	handle->isFullyCalibrated = false;
	handle->_acc_bandwidth = config._acc_bandwidth;
	handle->_acc_mode = config._acc_mode;
	handle->_acc_range = config._acc_range;
	handle->_acc_unit = config._acc_unit;
	handle->_eul_unit = config._eul_unit;
	handle->_gyr_bandwith = config._eul_unit;
	handle->_gyr_mode = config._gyr_mode;
	handle->_gyr_range = config._gyr_range;
	handle->_gyr_unit = config._gyr_unit;
	handle->_mag_mode = config._mag_mode;
	handle->_mag_out_rate = config._mag_out_rate;
	handle->_mag_pwr_mode = config._mag_pwr_mode;
	handle->_page = config._page;
	handle->_pwr_mode = config._pwr_mode;
	handle->_temp_unit = config._temp_unit;
	handle->_offsets = config._offsets;
	handle->i2cWrite = config.i2cWrite;
	handle->i2cRead = config.i2cRead;

	return MLS_SUCCESS;
}

mlsErrorCode_t mlsBno055Config(bno055Handle_t handle)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t errorCode = MLS_ERROR;
    u8 id = 0;
//    error_bno err;

    if ((errorCode = bno055_HW_reset(handle)) != MLS_SUCCESS) {
		return errorCode;
	}

    errorCode = handle->i2cRead(BNO_CHIP_ID, &id, 1);
    if (errorCode != MLS_SUCCESS || id != BNO_DEF_CHIP_ID) {
        return errorCode;
    }

    if ((errorCode = bno055_reset(handle)) != MLS_SUCCESS) {
		return errorCode;
	}
    HAL_Delay(5000);

    if ((errorCode = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
		return errorCode;
	}
    HAL_Delay(2);

//    u8 remap_buffer = REMAP_CONFIG_P1;
//    errorCode = handle->i2cWrite(BNO_AXIS_MAP_CONFIG, &remap_buffer, 1);
//    if (errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}
    HAL_Delay(50);
//    remap_buffer = REMAP_SIGN_P1;
//    errorCode = handle->i2cRead(BNO_AXIS_MAP_SIGN, &remap_buffer, 1);
//    if (errorCode != MLS_SUCCESS)
//	{
//		return errorCode;
//	}
//    HAL_Delay(50);

    errorCode = bno055_set_unit(handle, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_RPS,
            BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
    if (errorCode != MLS_SUCCESS)
    {
    	return errorCode;
    }
    HAL_Delay(1000);

    if ((errorCode = bno055_set_pwr_mode(handle, BNO_PWR_NORMAL)) != MLS_SUCCESS) {
        return errorCode;
    }
    HAL_Delay(10);

//    if ((errorCode = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
//        return errorCode;
//    }
//    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
//
//
//    u8 data = 0x01U;
//	if (handle->i2cWrite(BNO_SYS_TRIGGER, &data, 1) != MLS_SUCCESS) {
//		return MLS_ERROR;
//	}
//    HAL_Delay(50);
//
//    errorCode = bno055_on(handle);
//    if (errorCode != MLS_SUCCESS)
//    {
//    	return errorCode;
//    }
//    HAL_Delay(10);
    if ((errorCode = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
        return errorCode;
    }
//    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    HAL_Delay(50);


	return MLS_SUCCESS;
}

mlsErrorCode_t bno055_acc_conf(bno055Handle_t handle, const bno055_acc_range_t range,
                          const bno055_acc_band_t bandwidth,
                          const bno055_acc_mode_t mode) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
    if ((err = bno055_set_page(handle, BNO_PAGE_1)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config = range | bandwidth | mode;
    if ((err = handle->i2cWrite(BNO_ACC_CONFIG, &config, 1)) != MLS_SUCCESS) {
        return err;
    }
    handle->_acc_range = range;
    handle->_acc_bandwidth = bandwidth;
    handle->_acc_mode = mode;
    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_gyr_conf(bno055Handle_t handle, const bno055_gyr_range_t range,
                          const bno055_gyr_band_t bandwidth,
                          const bno055_gyr_mode_t mode) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
    if ((err = bno055_set_page(handle, BNO_PAGE_1)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config[2] = {range | bandwidth, mode};
    if ((err = handle->i2cWrite(BNO_GYR_CONFIG_0, config, 2)) !=
        BNO_OK) {
        return err;
    }
    handle->_gyr_range = range;
    handle->_gyr_bandwith = bandwidth;
    handle->_gyr_mode = mode;
    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_mag_conf(bno055Handle_t handle, const bno055_mag_rate_t out_rate,
                          const bno055_mag_pwr_t pwr_mode,
                          const bno055_mag_mode_t mode) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
    if ((err = bno055_set_page(handle, BNO_PAGE_1)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_CONFIG_TIME_DELAY + 5);
    u8 config = out_rate | pwr_mode | mode;
    if ((err = handle->i2cWrite(BNO_MAG_CONFIG, &config, 1)) != MLS_SUCCESS) {
        return err;
    }
    handle->_mag_mode = mode;
    handle->_mag_out_rate = out_rate;
    handle->_mag_pwr_mode = pwr_mode;
    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_acc_y(bno055Handle_t handle, f32* buf) {
	mlsErrorCode_t err;

	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_ACC_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((handle->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_acc_z(bno055Handle_t handle, f32* buf) {
	mlsErrorCode_t err;

	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_ACC_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((handle->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                     : BNO_ACC_SCALE_MG);
    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_acc(bno055Handle_t handle, float *accelX, float *accelY, float *accelZ) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET

    u8 data[6];
    float scale = (handle->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
                                                               : BNO_ACC_SCALE_MG;

    if ((err = handle->i2cRead(BNO_ACC_DATA_X_LSB, data, 6)) != MLS_SUCCESS) {
        return err;
    }
    *accelX = (s16)((data[1] << 8) | data[0]) / scale;
    *accelY = (s16)((data[3] << 8) | data[2]) / scale;
    *accelZ = (s16)((data[5] << 8) | data[4]) / scale;
//    u8 data[2];
//
//    float scale = (handle->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? BNO_ACC_SCALE_M_2
//                                                               : BNO_ACC_SCALE_MG;
//
//    if ((err = handle->i2cRead(BNO_ACC_DATA_X_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    *accelX = (s16)((data[1] << 8) | data[0]) / scale;
//
//    if ((err = handle->i2cRead(BNO_ACC_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
//		return err;
//	}
//    *accelY = (s16)((data[1] << 8) | data[0]) / scale;
//
//    if ((err = handle->i2cRead(BNO_ACC_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
//		return err;
//	}
//    *accelZ = (s16)((data[1] << 8) | data[0]) / scale;

//    bno055_acc_y(handle, (f32*) accelY);
//    bno055_acc_z(handle, (f32*) accelZ);

    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_gyro_y(bno055Handle_t handle, f32* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_GYR_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((handle->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                 : BNO_GYR_SCALE_RPS);
    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_gyro_z(bno055Handle_t handle, f32* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_GYR_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }

    *buf = (s16)((data[1] << 8) | data[0]) /
           ((handle->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                 : BNO_GYR_SCALE_RPS);
    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_gyro(bno055Handle_t handle, float *gyroX, float *gyroY, float *gyroZ) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    f32 scale = (handle->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
                                                         : BNO_GYR_SCALE_RPS;
    if ((err = handle->i2cRead(BNO_GYR_DATA_X_LSB, data, 6)) != MLS_SUCCESS) {
        return err;
    }
    *gyroX = (s16)((data[1] << 8) | data[0]) / scale;
    *gyroY = (s16)((data[3] << 8) | data[2]) / scale;
    *gyroZ = (s16)((data[5] << 8) | data[4]) / scale;
//    u8 data[2];
//    f32 scale = (handle->_gyr_unit == BNO_GYR_UNIT_DPS) ? BNO_GYR_SCALE_DPS
//                                                         : BNO_GYR_SCALE_RPS;
//    if ((err = handle->i2cRead(BNO_GYR_DATA_X_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    *gyroX = (s16)((data[1] << 8) | data[0]) / scale;
//
//    if ((err = handle->i2cRead(BNO_GYR_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    *gyroY = (s16)((data[1] << 8) | data[0]) / scale;
//
//    if ((err = handle->i2cRead(BNO_GYR_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    *gyroZ = (s16)((data[1] << 8) | data[0]) / scale;

//    bno055_gyro_y(handle, (f32*) gyroY);
//    bno055_gyro_z(handle, (f32*) gyroZ);

    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_mag(bno055Handle_t handle, float *magX, float *magY, float *magZ) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = handle->i2cRead(BNO_MAG_DATA_X_LSB, data, 6)) != MLS_SUCCESS) {
        return err;
    }

    *magX = (s16)((data[1] << 8) | data[0]) / BNO_MAG_SCALE;
    *magY = (s16)((data[3] << 8) | data[2]) / BNO_MAG_SCALE;
    *magZ = (s16)((data[5] << 8) | data[4]) / BNO_MAG_SCALE;

    return MLS_SUCCESS;
};

mlsErrorCode_t bno055_quaternion_x(bno055Handle_t handle, f32* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_QUA_DATA_X_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_quaternion_y(bno055Handle_t handle, f32* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_QUA_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_quaternion_z(bno055Handle_t handle, f32* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[2];
    if ((err = handle->i2cRead(BNO_QUA_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
        return err;
    }
    *buf = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_quaternion(bno055Handle_t handle, bno055_vec4_t* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[8];
    if ((err = handle->i2cRead(BNO_QUA_DATA_W_LSB, data, 8)) != MLS_SUCCESS) {
        return err;
    }
    buf->w = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
    buf->x = (s16)((data[3] << 8) | data[2]) / (f32)BNO_QUA_SCALE;
    buf->y = (s16)((data[5] << 8) | data[4]) / (f32)BNO_QUA_SCALE;
    buf->z = (s16)((data[7] << 8) | data[6]) / (f32)BNO_QUA_SCALE;
//    u8 data[2];
//    if ((err = handle->i2cRead(BNO_QUA_DATA_W_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    buf->w = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
//
//    if ((err = handle->i2cRead(BNO_QUA_DATA_X_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    buf->x = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
//
//    if ((err = handle->i2cRead(BNO_QUA_DATA_Y_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    buf->y = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;
//
//    if ((err = handle->i2cRead(BNO_QUA_DATA_Z_LSB, data, 2)) != MLS_SUCCESS) {
//        return err;
//    }
//    buf->z = (s16)((data[1] << 8) | data[0]) / (f32)BNO_QUA_SCALE;

//    bno055_quaternion_x(handle, (f32*)&buf->x);
//    bno055_quaternion_y(handle, (f32*)&buf->y);
//    bno055_quaternion_z(handle, (f32*)&buf->z);
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_euler(bno055Handle_t handle, bno055_euler_t* buf) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 data[6];
    if ((err = handle->i2cRead(BNO_EUL_HEADING_LSB, data, 6)) !=
		MLS_SUCCESS) {
        return err;
    }
    f32 scale = (handle->_eul_unit == BNO_EUL_UNIT_DEG) ? BNO_EUL_SCALE_DEG
                                                     : BNO_EUL_SCALE_RAD;
    buf->yaw = (s16)((data[1] << 8) | data[0]) / scale;
    buf->roll = (s16)((data[3] << 8) | data[2]) / scale;
    buf->pitch = (s16)((data[5] << 8) | data[4]) / scale;
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_getCalibration(bno055Handle_t handle, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
	/* Check if handle structure is NULL */
	if(handle == NULL || sys == NULL || gyro == NULL || accel == NULL || mag == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    u8 calData;
    if ((err = handle->i2cRead(BNO_CALIB_STAT, &calData, 1)) != MLS_SUCCESS) {
        return err;
    }

    if (sys != NULL) {
      *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
      *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
      *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
      *mag = calData & 0x03;
    }

	return MLS_SUCCESS;
}

mlsErrorCode_t bno055_isFullyCalibrate(bno055Handle_t handle) {
	mlsErrorCode_t err = MLS_ERROR;
	if ((err = bno055_getCalibration(handle, &systemSensor, &gyro, &accel, &mag)) != MLS_SUCCESS) {
		return err;
	}

	switch (handle->mode) {
	case BNO_MODE_AO:
		handle->isFullyCalibrated = (accel == 3);
		break;
	case BNO_MODE_MO:
		handle->isFullyCalibrated = (mag == 3);
		break;
	case BNO_MODE_GO:
	case BNO_MODE_M4G: /* No magnetometer calibration required. */
		handle->isFullyCalibrated = (gyro == 3);
		break;
	case BNO_MODE_AM:
	case BNO_MODE_COMPASS:
		handle->isFullyCalibrated = (accel == 3 && mag == 3);
		break;
	case BNO_MODE_AG:
	case BNO_MODE_IMU:
		handle->isFullyCalibrated = (accel == 3 && gyro == 3);
		break;
	case BNO_MODE_MG:
		handle->isFullyCalibrated = (mag == 3 && gyro == 3);
		break;
	default:
		handle->isFullyCalibrated = (systemSensor == 3 && gyro == 3 && accel == 3 && mag == 3);
		break;
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t bno055_getSystemStatus(bno055Handle_t handle, uint8_t *system_status, uint8_t *self_test_status, uint8_t *system_error)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    if (system_status != NULL)
    {
        if ((err = handle->i2cRead(BNO_SYS_STATUS, system_status, 1)) != MLS_SUCCESS) {
            return err;
        }
    }

    if (self_test_status != NULL)
    {
        if ((err = handle->i2cRead(BNO_ST_RESULT, self_test_status, 1)) != MLS_SUCCESS) {
            return err;
        }
    }

    if (system_error != NULL)
    {
        if ((err = handle->i2cRead(BNO_SYS_ERR, system_error, 1)) != MLS_SUCCESS) {
            return err;
        }
    }

    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_getSensorOffsets(bno055Handle_t handle) {
	mlsErrorCode_t err = bno055_isFullyCalibrate(handle);
	if (err != MLS_SUCCESS)	return err;

	if (handle->isFullyCalibrated)
	{
	    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
			return err;
		}
	    HAL_Delay(2);

#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET

	    err = handle->i2cRead(BNO_ACC_OFFSET_X_LSB, (uint8_t*)&handle->_offsets, 22);
	    if (err != MLS_SUCCESS)
	    {
	    	return err;
	    }

	    err = bno055_set_opmode(handle, handle->mode);
	    if (err != MLS_SUCCESS)
		{
			return err;
		}
	}

	return MLS_SUCCESS;
}

mlsErrorCode_t bno055_setSensorOffsets(bno055Handle_t handle) {
	mlsErrorCode_t err;
    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
		return err;
	}
    HAL_Delay(2);

#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET

    uint8_t data = (handle->_offsets.accel_offset_x) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_X_LSB, &data, 1);
    data = (handle->_offsets.accel_offset_x >> 8) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_X_MSB, &data, 1);
    data = (handle->_offsets.accel_offset_y) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_Y_LSB, &data, 1);
    data = (handle->_offsets.accel_offset_y >> 8) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_Y_MSB, &data, 1);
    data = (handle->_offsets.accel_offset_z) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_Z_LSB, &data, 1);
    data = (handle->_offsets.accel_offset_z >> 8) & 0xFF;
    handle->i2cWrite(BNO_ACC_OFFSET_Z_MSB, &data, 1);

    data = (handle->_offsets.mag_offset_x) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_X_LSB, &data, 1);
    data = (handle->_offsets.mag_offset_x >> 8) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_X_MSB, &data, 1);
    data = (handle->_offsets.mag_offset_y) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_Y_LSB, &data, 1);
    data = (handle->_offsets.mag_offset_y >> 8) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_Y_MSB, &data, 1);
    data = (handle->_offsets.mag_offset_z) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_Z_LSB, &data, 1);
    data = (handle->_offsets.mag_offset_z >> 8) & 0xFF;
    handle->i2cWrite(BNO_MAG_OFFSET_Z_MSB, &data, 1);

    data = (handle->_offsets.gyro_offset_x) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_X_LSB, &data, 1);
    data = (handle->_offsets.gyro_offset_x >> 8) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_X_MSB, &data, 1);
    data = (handle->_offsets.gyro_offset_y) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_Y_LSB, &data, 1);
    data = (handle->_offsets.gyro_offset_y >> 8) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_Y_MSB, &data, 1);
    data = (handle->_offsets.gyro_offset_z) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_Z_LSB, &data, 1);
    data = (handle->_offsets.gyro_offset_z >> 8) & 0xFF;
    handle->i2cWrite(BNO_GYR_OFFSET_Z_MSB, &data, 1);

    data = (handle->_offsets.accel_radius) & 0xFF;
    handle->i2cWrite(BNO_ACC_RADIUS_LSB, &data, 1);
    data = (handle->_offsets.accel_radius >> 8) & 0xFF;
    handle->i2cWrite(BNO_ACC_RADIUS_MSB, &data, 1);

    data = (handle->_offsets.mag_radius) & 0xFF;
    handle->i2cWrite(BNO_MAG_RADIUS_LSB, &data, 1);
    data = (handle->_offsets.mag_radius >> 8) & 0xFF;
    handle->i2cWrite(BNO_MAG_RADIUS_MSB, &data, 1);

    err = bno055_set_opmode(handle, handle->mode);
    if (err != MLS_SUCCESS)
	{
		return err;
	}
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_setExtCrystalUse(bno055Handle_t handle, bool usextal)
{
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

	mlsErrorCode_t err;

    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET

    u8 config = 0x80;
    if (usextal) {
    	config = 0x80;
    } else {
    	config = 0x00;
    }

    if ((err = handle->i2cWrite(BNO_SYS_TRIGGER, &config, 1)) != MLS_SUCCESS) {
    	return err;
    }
    HAL_Delay(10);

    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
        return err;
    }
    HAL_Delay(BNO_ANY_TIME_DELAY + 5);

    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_set_pwr_mode(bno055Handle_t handle, bno055_pwr_t pwr_mode) {
	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}
    mlsErrorCode_t err;

    if ((err = bno055_set_opmode(handle, BNO_MODE_CONFIG)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
    if ((err = handle->i2cWrite(BNO_PWR_MODE, (u8*)&pwr_mode, 1)) !=
    		MLS_SUCCESS) {
        return err;
    }
    handle->_pwr_mode = pwr_mode;
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
//    if ((err = bno055_set_opmode(handle, handle->mode)) != MLS_SUCCESS) {
//        return err;
//    }
    HAL_Delay(2);
    return MLS_SUCCESS;
}

mlsErrorCode_t bno055_set_opmode(bno055Handle_t handle, const bno055_opmode_t opmode) {
	mlsErrorCode_t err;

	/* Check if handle structure is NULL */
	if(handle == NULL)
	{
		return MLS_ERROR_NULL_PTR;
	}

#ifdef BNO_AUTO_PAGE_SET
    if ((err = bno055_set_page(handle, BNO_PAGE_0)) != MLS_SUCCESS) {
        return err;
    }
#endif  // BNO_AUTO_PAGE_SET
    if ((err = handle->i2cWrite(BNO_OPR_MODE, (u8*)&opmode, 1)) !=
    		MLS_SUCCESS) {
        return err;
    }
//    HAL_Delay(BNO_ANY_TIME_DELAY + 5);
    HAL_Delay(60);
    return MLS_SUCCESS;
}

/**@}*/
