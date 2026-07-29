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
    uint16_t pitch_pct;
    uint16_t roll_pct;
    uint16_t yaw_pct;
} Pid_Output_t;

// struct implemented for better code clarity

//initializes 
void pid_init(void);

/*
Requires: pilot commands are in TAER channel sequence. pilot command values are all percent-style integers from 0-1000 
MUST VERIFY THAT QUAD IS ARMED BEFORE CALLING THIS FUNCTION. 
          IMU data contains fused gyroscope data
Modifies: esc_commands_pct array
Effects:  Outputs four values (ranged 0-1000) into esc_commands_pct array
*/

void pid_update(const IMU_Model_t* imu_data, const uint16_t* pilot_command, uint16_t* esc_commands_pct);


#endif