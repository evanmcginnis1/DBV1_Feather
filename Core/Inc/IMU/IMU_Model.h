/*
 * IMU_Model.h
 *
 *  Created on: May 16, 2026
 *      Author: Evan McGinnis
 *      High-level model for BNO055 IMU
 */

 #ifndef IMU_MODEL_H_
 #define IMU_MODEL_H_

//heading refers to the actual yaw value, not rate
typedef struct {
    float pitch_abs;
    float yaw_abs;
    float roll_abs;
    float pitch_rate;
    float yaw_rate;
    float roll_rate;
} IMU_Model_t;

 #endif /* IMU_MODEL_H_ */