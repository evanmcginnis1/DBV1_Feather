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
    send_IMU_config_to_sensor();
    //does nothing for now
    IMU_calibrate_sensors();
}

void IMU_update_model(IMU_Model_t* imu_model) {
    get_IMU_euler_data(&(imu_model->yaw_rate), &(imu_model->roll_rate), &(imu_model->pitch_rate));
    get_IMU_gyro_rawdata(&(imu_model->pitch_rate), &(imu_model->roll_rate), &(imu_model->yaw_rate));
}

 