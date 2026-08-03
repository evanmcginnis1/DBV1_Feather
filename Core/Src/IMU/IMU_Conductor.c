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


void IMU_init(I2C_HandleTypeDef* hi2c) {
    IMU_begin_i2c(hi2c);
    //IMU_default_config();
    send_IMU_config_to_sensor();
    //does nothing for now
    //IMU_calibrate_sensors();
    //redefine IMU axes
}

void IMU_update_model(IMU_Model_t* imu_model) {
    IMU_get_euler_data(&(imu_model->yaw_rate), &(imu_model->roll_rate), &(imu_model->pitch_rate));
    IMU_get_gyro_rawdata(&(imu_model->pitch_rate), &(imu_model->roll_rate), &(imu_model->yaw_rate));
}

 