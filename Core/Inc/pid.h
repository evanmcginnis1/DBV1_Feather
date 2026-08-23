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
#include <stdbool.h>

#define PID_PITCH_KP 0.2
#define PID_PITCH_KI 0
#define PID_PITCH_KD 0

#define PID_ROLL_KP 0.2
#define PID_ROLL_KI 0
#define PID_ROLL_KD 0

#define PID_YAW_KP 0.1
#define PID_YAW_KI 0
#define PID_YAW_KD 0

//dshot throttle ranges from 48 to 2048; 2000 steps
#define PID_OUTPUT_MAX 500
#define PID_OUTPUT_IDLE 150


#define PID_MAX_YAW_RATE_INPUT 200
//euler angle lockup at 45 degrees; not high performance, so don't need huge angles
#define PID_MAX_PITCH_ANGLE_INPUT 30
#define PID_MAX_ROLL_ANGLE_INPUT 30

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
    int16_t pitch_pct;
    int16_t roll_pct;
    int16_t yaw_pct;
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
 * Requires: uart_data has been populated with message from serial port containing new pid gain value
 * Modifies: PID loop gain variables
 * Effects: Updates PID. Returns true if update is confirmed, returns false if invalid input or user cancels
 */
bool pid_update_gains(void);

/*
 * Requires: USB port is availale
 * Modifies: Virtual COM TX buffer
 * Effects: prints gain update message format over serial
 */
void pid_update_gains_prompt(void);
/*
Requires: pilot commands are in TAER channel sequence. pilot command values are all percent-style integers from 0-1000 
MUST VERIFY THAT QUAD IS ARMED BEFORE CALLING THIS FUNCTION. 
          IMU data contains fused gyroscope data
Modifies: esc_commands_pct array
Effects:  Outputs four values (ranged 0-1000) into esc_commands_pct array
*/

void pid_update(const IMU_Model_t* imu_data, const uint16_t* pilot_command, uint16_t* esc_commands_pct);

#endif