/*
 * IMU_Hardware.c
 *
 *  Created on: May 15, 2026
 *      Author: Evan McGinnis
 *      evanpmcg@umich.edu
 *      implementation of low-level functions for interfacing with BNO055 IMU sensor
 */

 //TODO: Make functions static

#include "IMU_Hardware.h"
#include "IMU_Config.h"
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include <stdbool.h>
//TODO: update resolution settings - disregard non-included bits

/*************************************************
*           IMU I2C Interface                    *
**************************************************/

// static keyword keeps this confined to hardware layer
// drone will only have one IMU on it for the time being - if this changes, need to change this
// done in order to align with MCH structure; keeps hardware object (imu) separated, so conductor doesn't have to deal
// with it
static IMU_t imu_setup;

void IMU_begin_i2c(I2C_HandleTypeDef *hi2c) {
	imu_setup.hi2c = hi2c; 
}

//writes a single byte to a single register address on IMU
HAL_StatusTypeDef read_IMU_register(uint8_t register_addr, uint8_t* rx_buffer, 
	                                uint8_t num_bytes_to_read) {

	return HAL_I2C_Mem_Read(imu_setup.hi2c,IMU_I2C_ADDR_SHIFTED,register_addr, IMU_REGISTER_ADDR_SIZE,
		                    rx_buffer,num_bytes_to_read,IMU_I2C_TIMEOUT);
}

HAL_StatusTypeDef write_IMU_register(uint8_t register_addr, uint8_t* tx_buffer) {
	return HAL_I2C_Mem_Write(imu_setup.hi2c, IMU_I2C_ADDR_SHIFTED, register_addr, IMU_REGISTER_ADDR_SIZE, tx_buffer,
		                     SINGLE_WRITE_DATA_SIZE, IMU_I2C_TIMEOUT);
}

//does not check if register is already set to selected page
HAL_StatusTypeDef set_IMU_page(IMU_Page_Sel_t page_num) {
	//cast page_num to be a uint8_t explicitly
	return write_IMU_register(IMU_REG_PAGE_ID, (uint8_t*)&page_num);
}

/*************************************************
*           IMU Overall Configuration            *
**************************************************/
//consider changing to be preprocessor based
HAL_StatusTypeDef set_IMU_default_config(void) {
	HAL_StatusTypeDef status;
	IMU_Config_t default_config = {
		.imu_opr_mode = IMU_OPR_MODE,
		.accel = {
			.range = IMU_ACCEL_RANGE,
			.bandwidth = IMU_ACCEL_BANDWIDTH,
			.opr_mode = IMU_ACCEL_OPR_MODE
		},
		.gyro = {
			.range = IMU_GYRO_RANGE,
			.bandwidth = IMU_GYRO_BANDWIDTH,
			.opr_mode = IMU_GYRO_OPR_MODE
		},
		.mag = {
			.data_rate = IMU_MAG_RATE,
			.opr_mode = IMU_MAG_OPR_MODE,
			.pwr_mode = IMU_MAG_PWR_MODE
		}
	};
	imu_setup.config = default_config;
	status = send_IMU_config_to_sensor();
	return status;
}

HAL_StatusTypeDef update_IMU_config(IMU_Config_t* new_config) {
	imu_setup.config = *new_config;
	return send_IMU_config_to_sensor();
}

//send_IMU_config_to_sensor handles switching to page 1, then switches back to page 0
HAL_StatusTypeDef send_IMU_config_to_sensor(void) {
	//get the config object from IMU_t object
	// TODO: check if setting config mode induces a delay in sensor data; is 
	// config mode necessary

	//|= logic ensures that function will keep writing to registers even if one of the writes fails
	HAL_StatusTypeDef status;
	status = set_IMU_page(IMU_PAGE_0);
	status  |= set_IMU_operation_mode(IMU_OPR_MODE_CONFIGMODE);
	//datasheet requires 19ms switching time when switching to config mode
	HAL_Delay(20);
	// skip setting power mode because it will always be normal mode

	// only write to individual sensor config registers if not setting fusion mode
	if (!(imu_setup.config.imu_opr_mode == IMU_OPR_MODE_IMU          || 
		  imu_setup.config.imu_opr_mode == IMU_OPR_MODE_COMPASS      || 
		  imu_setup.config.imu_opr_mode == IMU_OPR_MODE_M4G          || 
		  imu_setup.config.imu_opr_mode == IMU_OPR_MODE_NDOF_FMC_OFF ||
		  imu_setup.config.imu_opr_mode == IMU_OPR_MODE_NDOF)) {

			//TODO: fix weird dereferencing
			status |= set_IMU_page(IMU_PAGE_1);
			status |= set_IMU_accel_config(&(imu_setup.config.accel));
			status |= set_IMU_gyro_config(&(imu_setup.config.gyro));
			status |= set_IMU_mag_config(&(imu_setup.config.mag));
			status |= set_IMU_page(IMU_PAGE_0);
		}
		status |= set_IMU_operation_mode(imu_setup.config.imu_opr_mode);
		return status;
}


HAL_StatusTypeDef set_IMU_operation_mode(IMU_OprMode_t operation_mode) {
	HAL_StatusTypeDef status;
	uint8_t opr_reg;

	status = read_IMU_register(IMU_REG_OPR_MODE, &opr_reg,1);
	if (status != HAL_OK) {
		return status;
	}
	opr_reg &= ~IMU_OPR_MODE_MASK;
	opr_reg |= operation_mode;
	status = write_IMU_register(IMU_REG_OPR_MODE, &opr_reg);
	return status;
}


HAL_StatusTypeDef set_IMU_units(void) {
	uint8_t unit_sel_reg;

	HAL_StatusTypeDef status = read_IMU_register(IMU_REG_UNIT_SEL, &unit_sel_reg, 1);

	if (status != HAL_OK) {
		return status;
	}

	unit_sel_reg &= ~IMU_UNIT_SEL_MASK;

	unit_sel_reg |= (IMU_ACCEL_UNITS | IMU_ANGULAR_RATE_UNITS | IMU_EULER_ANGLE_UNITS | IMU_TEMP_UNITS);

	status = write_IMU_register(IMU_REG_UNIT_SEL, &unit_sel_reg);
	return status;

}
/*************************************************
*           Individual Sensor Config             *
**************************************************/
//accelerometer
HAL_StatusTypeDef set_IMU_accel_config(IMU_AccelConfig_t* accel_config){
	HAL_StatusTypeDef status;
	uint8_t accel_range_flags = accel_config->range;
	uint8_t accel_bandwidth_flags = ((accel_config->bandwidth) << 2);
	uint8_t accel_opr_mode_flags = ((accel_config->opr_mode) << 5);

	uint8_t tx_buf = accel_range_flags | accel_bandwidth_flags | accel_opr_mode_flags;


	status = write_IMU_register(IMU_REG_ACC_CONFIG, &tx_buf);
	return status;
}

//gyro range and bandwidth go in gyro_config register 0 and gyro_power mode goes in gyro_config register 1
// higest 2 bits in gyro_config register 0 reserved
// highest 5 bits in gyro_config register 1 reserved
HAL_StatusTypeDef set_IMU_gyro_config(IMU_GyroConfig_t* gyro_config) {
	HAL_StatusTypeDef status;
	uint8_t gyro_config_reg[2];

	status = read_IMU_register(IMU_REG_GYR_CONFIG_0, gyro_config_reg,2);
	if (status != HAL_OK) {
		return status;
	}

	uint8_t gyr_range_flags = gyro_config->range;
	uint8_t gyr_bandwidth_flags = ((gyro_config->bandwidth) << 3);

	uint8_t gyro_opr_mode_flags = ((gyro_config->opr_mode));

	uint8_t reg_0_combined_flags = gyr_range_flags | gyr_bandwidth_flags;

	//mask first 6 bits of config reg 0
	gyro_config_reg[0] &= ~(IMU_GYR_CONFIG_0_MASK);
	gyro_config_reg[0] |= reg_0_combined_flags;
	//gyro_config_reg is a pointer to first item in the array, which is reg 0
	status = write_IMU_register(IMU_REG_GYR_CONFIG_0, gyro_config_reg);
	if (status != HAL_OK) {
		return status;
	}

	gyro_config_reg[1] &= ~(IMU_GYR_CONFIG_1_MASK);
	gyro_config_reg[1] |= gyro_opr_mode_flags;
	status = write_IMU_register(IMU_REG_GYR_CONFIG_1, &gyro_config_reg[1]);
	return status;
}

HAL_StatusTypeDef set_IMU_mag_config(IMU_MagConfig_t* mag_config) {
	HAL_StatusTypeDef status;
	uint8_t mag_config_reg;

	uint8_t mag_config_flags = (mag_config->data_rate | ((mag_config->opr_mode) << 3) | ((mag_config->pwr_mode) << 5));

	status = read_IMU_register(IMU_REG_MAG_CONFIG, &mag_config_reg, 1);
	if (status != HAL_OK) {
		return status;
	}

	mag_config_reg &= ~(IMU_MAG_CONFIG_MASK);
	mag_config_reg |= (mag_config_flags);

	status = write_IMU_register(IMU_REG_MAG_CONFIG, &mag_config_reg);
	return status;
}

//TODO: Implement
HAL_StatusTypeDef IMU_remap_axes(void) {
	return HAL_OK;
}
/*************************************************
*              Sensor Calibration                *
**************************************************/

/*Status for all sensors & entire system is stored in one register. 
* sys_status is highest two bits, gyro_status is next two, accel_status is next two, mag_status is lowest two
*/
HAL_StatusTypeDef check_IMU_calibration_status(void) {
	uint8_t calib_reg;
	HAL_StatusTypeDef status = read_IMU_register(IMU_REG_CALIB_STAT, &calib_reg, 1);
	if (status != HAL_OK) {
		return status;
	}
	
	imu_setup.cal_status.sys_good = ((calib_reg & 0xC0) >> 6);
	imu_setup.cal_status.gyro_good = ((calib_reg & 0x30) >> 4);
	imu_setup.cal_status.accel_good = ((calib_reg & 0x0C) >> 2);
	imu_setup.cal_status.mag_good = ((calib_reg & 0x03));
	return status;
}

// to be implemented later
HAL_StatusTypeDef IMU_calibrate_sensors(void) {
	//filler
	HAL_StatusTypeDef status = HAL_OK;
	status = IMU_set_calibration_profile();
	return status;
}
HAL_StatusTypeDef IMU_set_calibration_profile(void) {
	//temp filler
	HAL_StatusTypeDef status = HAL_OK;
	return status;
}
/*************************************************
*           Sensor Data Functions                *
**************************************************/
//read all of the euler angle registers into a buffer, then extract them from the buffer to convert into true values
HAL_StatusTypeDef IMU_get_euler_data(float* EUL_heading, float* EUL_roll, float* EUL_pitch) {
	HAL_StatusTypeDef status;
	uint8_t euler_data[6];
	// read all registers at once for speed

	status = read_IMU_register(IMU_REG_EUL_DATA_X_LSB, euler_data, 6);
	if (status != HAL_OK) {
		return status;
	}

	int16_t EUL_heading_raw = (int16_t)(euler_data[0] | (euler_data[1] << 8));
	int16_t EUL_roll_raw =    (int16_t)(euler_data[2] | (euler_data[3] << 8));
	int16_t EUL_pitch_raw =   (int16_t)(euler_data[4] | (euler_data[5] << 8));

	*EUL_heading = (float) EUL_heading_raw / IMU_EULER_ANGLE_SCALAR;
	*EUL_roll    = (float) EUL_roll_raw    / IMU_EULER_ANGLE_SCALAR;
	*EUL_pitch   = (float) EUL_pitch_raw   / IMU_EULER_ANGLE_SCALAR;

	return status;
 }

HAL_StatusTypeDef IMU_get_quat_data(float* QUAT_w, float* QUAT_x, float* QUAT_y, float* QUAT_z) {
	HAL_StatusTypeDef status;
	uint8_t quat_data[8];

	status = read_IMU_register(IMU_REG_QUA_DATA_W_LSB, quat_data, 8);
	if (status != HAL_OK) {
		return status;
	}

	// combine LSB and MSB, and add sign
	int16_t QUAT_w_raw = (int16_t)(quat_data[0] | (quat_data[1] << 8));
	int16_t QUAT_x_raw = (int16_t)(quat_data[2] | (quat_data[3] << 8));
	int16_t QUAT_y_raw = (int16_t)(quat_data[4] | (quat_data[5] << 8));
	int16_t QUAT_z_raw = (int16_t)(quat_data[6] | (quat_data[7] << 8));

	*QUAT_w = (float) QUAT_w_raw / IMU_QUAT_SCALAR;
	*QUAT_x = (float) QUAT_x_raw / IMU_QUAT_SCALAR;
	*QUAT_y = (float) QUAT_y_raw / IMU_QUAT_SCALAR;
	*QUAT_z = (float) QUAT_z_raw / IMU_QUAT_SCALAR;

	return status;
}

HAL_StatusTypeDef IMU_get_gyro_rawdata(float *pitch, float *roll, float *yaw) {
	HAL_StatusTypeDef status;
	uint8_t gyr_data[6];
	status = read_IMU_register(IMU_REG_GYR_DATA_X_LSB, gyr_data, 6);
	if (status != HAL_OK) {
		return status;
	}

	int16_t GYR_x_raw = (int16_t)(gyr_data[0] | (gyr_data[1] << 8));
	int16_t GYR_y_raw = (int16_t)(gyr_data[2] | (gyr_data[3] << 8));
	int16_t GYR_z_raw = (int16_t)(gyr_data[4] | (gyr_data[5] << 8));

	*pitch = (float) GYR_x_raw / IMU_EULER_ANGLE_SCALAR;
	*roll = (float) GYR_y_raw / IMU_EULER_ANGLE_SCALAR;
	*yaw = (float) GYR_z_raw / IMU_EULER_ANGLE_SCALAR;

	return status;
}

HAL_StatusTypeDef IMU_get_accel_rawdata(float* accel_x, float* accel_y, float* accel_z) {
	HAL_StatusTypeDef status;
	uint8_t accel_rawdata[6];

	status = read_IMU_register(IMU_ACC_DATA_X_LSB, accel_rawdata, 6);
	if (status != HAL_OK) {
		return status;
	}

	// combine LSB and MSB, and add sign
	int16_t accel_x_raw = (int16_t)(accel_rawdata[0] | (accel_rawdata[1] << 8));
	int16_t accel_y_raw = (int16_t)(accel_rawdata[2] | (accel_rawdata[3] << 8));
	int16_t accel_z_raw = (int16_t)(accel_rawdata[4] | (accel_rawdata[5] << 8));

	*accel_x = (float) accel_x_raw / IMU_ACCEL_SCALAR;
	*accel_y = (float) accel_y_raw / IMU_ACCEL_SCALAR;
	*accel_z = (float) accel_z_raw / IMU_ACCEL_SCALAR;

	return status;
}

HAL_StatusTypeDef IMU_get_grav_data(float* grav_x, float* grav_y, float* grav_z) {
	HAL_StatusTypeDef status;
	uint8_t grav_rawdata[6];

	status = read_IMU_register(IMU_REG_GRV_DATA_X_LSB, grav_rawdata, 6);
	if (status != HAL_OK) {
		return status;
	}

	int16_t grav_x_raw = (int16_t)(grav_rawdata[0] | (grav_rawdata[1] << 8));
	int16_t grav_y_raw = (int16_t)(grav_rawdata[2] | (grav_rawdata[3] << 8));
	int16_t grav_z_raw = (int16_t)(grav_rawdata[4] | (grav_rawdata[5] << 8));

	*grav_x = (float) grav_x_raw / IMU_ACCEL_SCALAR;
	*grav_y = (float) grav_y_raw / IMU_ACCEL_SCALAR;
	*grav_z = (float) grav_z_raw / IMU_ACCEL_SCALAR;

	return status;
}

HAL_StatusTypeDef IMU_get_lin_accel_data(float* linaccel_x, float* linaccel_y, float* linaccel_z) {
	HAL_StatusTypeDef status;
	uint8_t linaccel_rawdata[6];

	status = read_IMU_register(IMU_REG_LIA_DATA_X_LSB, linaccel_rawdata,6);
	if (status != HAL_OK) {
		return status;
	}

	int16_t linaccel_x_raw = (int16_t)(linaccel_rawdata[0] | (linaccel_rawdata[1] << 8));
	int16_t linaccel_y_raw = (int16_t)(linaccel_rawdata[2] | (linaccel_rawdata[3] << 8));
	int16_t linaccel_z_raw = (int16_t)(linaccel_rawdata[4] | (linaccel_rawdata[5] << 8));

	*linaccel_x = (float) linaccel_x_raw / IMU_ACCEL_SCALAR;
	*linaccel_y = (float) linaccel_y_raw / IMU_ACCEL_SCALAR;
	*linaccel_z = (float) linaccel_z_raw / IMU_ACCEL_SCALAR;

	return status;
}

HAL_StatusTypeDef IMU_get_chipID(uint8_t* chipID) {
	HAL_StatusTypeDef status;
	status = read_IMU_register(IMU_REG_CHIP_ID, chipID,1);
	if (status != HAL_OK) {
		return status;
	}

	return status;
}
