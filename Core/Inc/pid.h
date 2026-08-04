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

#define PID_PITCH_KP 8
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0

#define PID_ROLL_KP 8
#define PID_ROLL_KI 0
#define PID_ROLL_KD 0

#define PID_YAW_KP 8
#define PID_YAW_KI 0
#define PID_YAW_KD 0

//pit output ranges from 0 to 1000
#define PID_OUTPUT_MAX 800
#define PID_FLIGHT_OUTPUT_MIN 0
#define PID_OUTPUT_IDLE 50

#define PID_LOOP_RATE_HZ 100

#define NUM_MOTORS 4
//pid constants are stored in a struct so that they can be updated on the fly
typedef struct {
    //gain values
    float kp;
    float ki;
    float kd;
    float new_accum_error;
    float accumulated_error;
    float prev_error;
} PID_t;

typedef struct {
    uint16_t pitch_pct;
    uint16_t roll_pct;
    uint16_t yaw_pct;
} Pid_Output_t;

// struct implemented for better code clarity

/* 
Requires: Nothing
Modifies: pid_x_info structs, where x is pitch, roll, yaw
Effects: Initializes pid info structs for each axis using defined gain constants. Sets accumulated error and previous 
         error terms to zero. 
*/

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