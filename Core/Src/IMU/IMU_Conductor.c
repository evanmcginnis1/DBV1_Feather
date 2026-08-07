/*
 * IMU_Conductor.c
 *
 *  Created on: May 16, 2026
 *      Author: Evan McGinnis
 *      evanpmcg@umich.edu
 *      connector between hardware and model for BNO055 IMU sensor
 */

 #include <main.h>
 #include <IMU_Hardware.h>
 #include <IMU_Model.h>

//TODO: Implement calibration function
//TODO: Redefine axes
void IMU_init(I2C_HandleTypeDef* hi2c) {
    // BNO055 requires 400ms startup time after power on
    HAL_Delay(400);
    IMU_configure_i2c(hi2c);
    //620ms total delay time
    IMU_enable_external_oscillator();
    //20ms delay time
    IMU_remap_axes(IMU_AXIS_Y, IMU_AXIS_X, IMU_AXIS_Z);
    IMU_change_axis_signs(IMU_AXIS_SIGN_NEGATIVE, IMU_AXIS_SIGN_POSITIVE, IMU_AXIS_SIGN_POSITIVE);
    IMU_default_config();
    //redefine IMU axes
}

void IMU_update_model(IMU_Model_t* imu_model) {
    IMU_get_euler_data(&(imu_model->yaw_rate), &(imu_model->roll_rate), &(imu_model->pitch_rate));
    IMU_get_gyro_rawdata(&(imu_model->pitch_rate), &(imu_model->roll_rate), &(imu_model->yaw_rate));
}

 