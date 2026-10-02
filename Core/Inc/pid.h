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

//allowed range for any gain set over serial or loaded from flash
#define PID_GAIN_MIN 0
#define PID_GAIN_MAX 20

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
Requires: flash_init has been called
Effects: Initializes pid info structs for each axis. Gains are loaded from the metadata of the most recent flight log.
         If there are no logs (or the stored gains are invalid), uses defined gain constants instead. Sets accumulated
         error and previous error terms to zero.
*/
void pid_init(void);

void get_pid_info_pointers(const PID_t** pitch_info, const PID_t** roll_info, const PID_t** yaw_info);

void get_pid_output_pointer(const Pid_Output_t** pid_output_data);

/*
 * Requires: uart_data has been populated with message from serial port containing new pid gain value
 * Modifies: PID loop gain variables
 * Effects: Updates PID. Returns true if update is confirmed, returns false if invalid input or user cancels
 */
bool pid_update_gains(void);

/*
 * Requires: USB port is available
 * Modifies: PID loop gain variables, Virtual COM TX buffer, uart_data_ready
 * Effects: Prompts user for all nine gains at once and blocks until a valid set is confirmed, or user types 'keep'
 *          to leave gains unchanged. Only updates gains in RAM; they are stored when the next log is created
 */
void pid_user_update_all_gains(void);

/*
 * Requires: USB port is availale
 * Modifies: Virtual COM TX buffer
 * Effects: prints gain update message format over serial
 */
void pid_print_update_single_gain_prompt(void);
/*
Requires: pilot commands are in TAER channel sequence. pilot command values are all percent-style integers from 0-1000 
MUST VERIFY THAT QUAD IS ARMED BEFORE CALLING THIS FUNCTION. 
          IMU data contains fused gyroscope data
Modifies: esc_commands_pct array
Effects:  Outputs four values (ranged 0-1000) into esc_commands_pct array
*/

void pid_update(const IMU_Model_t* imu_data, const uint16_t* pilot_commands, uint16_t* esc_commands_pct, float* setpoint_output);

#endif