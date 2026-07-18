/*
 * pid.h
 *
 * Created on: July 18, 2026
 * Author: Evan McGinnis
 * 
 * Functions for calculating one step of a pid loop
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "IMU_Model.h"

#define PID_PITCH_KP 0
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0

#define PID_ROLL_KP 0
#define PID_ROLL_KI 0
#define PID_ROLL_KD 0

#define PID_YAW_KP 0
#define PID_YAW_KI 0
#define PID_YAW_KD 0

//dshot throttle ranges from 48 to 2048; 2000 steps
#define PID_OUTPUT_MAX 2048
#define PID_FLIGHT_OUTPUT_MIN 48
#define PID_OUTPUT_IDLE 1000

#define PID_LOOP_RATE_HZ 100
//pid constants are stored in a struct so that they can be updated on the fly
typedef struct {
    //gain values
    float kp;
    float ki;
    float kd;
    float accumulated_error;
    float prev_error;
} PID_t;

typedef struct {
    float pitch_pct;
    float roll_pct;
    float yaw_pct;
} Pid_Output_t;

// struct implemented for better code clarity

//initializes 
void pid_init(void);

/*
Requires: pilot commands are in TAER channel sequence. pilot commands for angle correspond to an actual angle, throttle 
          is a number between 48 and 2048. MUST VERIFY THAT QUAD IS ARMED BEFORE CALLING THIS FUNCTION. 
*/

void pid_update(const IMU_Model_t* imu_data, const uint16_t* pilot_command, float* esc_commands_pct);


#endif