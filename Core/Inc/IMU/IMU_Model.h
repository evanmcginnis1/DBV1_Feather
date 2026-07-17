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
    float pitch;
    float heading;
    float roll;
} IMU_Model_t;

 #endif /* IMU_MODEL_H_ */