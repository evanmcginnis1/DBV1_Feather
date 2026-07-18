/*
 * pid.c
 *
 * Created on: July 17, 2026
 * Author: Evan McGinnis
 * 
 * This code is NOT modular; it is based on gathering data from ibus 
 */

#include "pid.h"
#include "IMU_Model.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

static PID_t pid_roll_info;
static PID_t pid_pitch_info;
static PID_t pid_yaw_info;
static Pid_Output_t pid_axis_out_pct;

// Static Functions
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, float* output);
static void pid_step_all(const IMU_Model_t* imu, const float* setpoint);
static void synthesize_pid_commands(const float* throttle_command, float* esc_commands_pct);
static void pilot_command_to_setpoint(const uint16_t* pilot_command, float* setpoint);

void pid_init(void) {
    pid_roll_info.accumulated_error = 0;
    pid_roll_info.prev_error = 0;
    pid_roll_info.kp = PID_ROLL_KP;
    pid_roll_info.ki = PID_ROLL_KI;
    pid_roll_info.kd = PID_ROLL_KD;

    pid_pitch_info.accumulated_error = 0;
    pid_pitch_info.prev_error = 0;
    pid_pitch_info.kp = PID_PITCH_KP;
    pid_pitch_info.ki = PID_PITCH_KI; 
    pid_pitch_info.kd = PID_PITCH_KD;

    pid_yaw_info.accumulated_error = 0;
    pid_yaw_info.prev_error = 0;
    pid_yaw_info.kp = PID_YAW_KP;
    pid_yaw_info.ki = PID_YAW_KI;
    pid_yaw_info.kd = PID_YAW_KD;
}

/* article on quadcopter flight dynamics
https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-1-quadcopter-flight-dynamics-400af73d21db
*/

void pid_update(const IMU_Model_t *imu, const uint16_t *pilot_command, float *esc_commands_pct) {
    float setpoint[4];

    pilot_command_to_setpoint(pilot_command, setpoint);
    pid_step_all(imu, setpoint);
    synthesize_pid_commands(&setpoint[2], esc_commands_pct);
}

//pilot_command[0] = roll
//pilot_command[1] = pitch
//pilot_command[2] = throttle
//pilot_command[3] = yaw
//pilot_command[4] = ARM/DISARM
static void pid_step_all(const IMU_Model_t* imu, const float* setpoint) {
    //PID roll step
    pid_step(&pid_roll_info, &imu->roll, &setpoint[0], &pid_axis_out_pct.roll_pct);
    //PID pitch step
    pid_step(&pid_pitch_info, &imu->pitch, &setpoint[1], &pid_axis_out_pct.pitch_pct);
    //PID yaw step
    pid_step(&pid_yaw_info, &imu->heading, &setpoint[3], &pid_axis_out_pct.yaw_pct);

    return;
}

//KP, KI, KD should be sized so that the output is a float between 0 and 1.0
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, float* output) {
    float error = *setpoint - *actual_val;

    float p = error * pid_info->kp;
    float i = (pid_info->accumulated_error + (error * (1.0  / PID_LOOP_RATE_HZ))) * pid_info->ki;
    float d = ((error - pid_info->prev_error) * PID_LOOP_RATE_HZ) * pid_info->kd;

    *output = p + i + d;

    // stop increading integral term if axis signal is already maxed out
    if (((p + i + d) < 1) && ((p + i + d) > 0)) {
        pid_info->accumulated_error += (error * (1.0 / PID_LOOP_RATE_HZ));
    }
    pid_info->prev_error = error;
}

//map pilot axis input to a number between 0 and 180.
// Pilot command is a number between 1000 and 2000. 
//setpoint[0] = roll
//setpoint[1] = pitch
//setpoint[2] = throttle
//setpoint[3] = yaw
static void pilot_command_to_setpoint(const uint16_t* pilot_command, float* setpoint) {
    //axis commands

    setpoint[0] = (pilot_command[0] - 1500) / (10.0f);
    setpoint[1] = (pilot_command[1] - 1500) / (10.0f);
    setpoint[3] = (pilot_command[3] - 1500) / (10.0f);

    //throttle command normalized to a value between 0 and 1
    setpoint[2] = (pilot_command[2] - 1000) / 1000.0;
}


//TODO: figure out how to handle idle speed, prevent outputting a negative number, prevent esc signal on zero throttle
static void synthesize_pid_commands(const float* throttle_command_pct, float* esc_command_pct) {
    //synthesize commands
    //esc_command index corresponds to motor index on quadcopter
    esc_command_pct[0] = *throttle_command_pct + pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;
    esc_command_pct[1] = *throttle_command_pct + pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    esc_command_pct[2] = *throttle_command_pct - pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    esc_command_pct[3] = *throttle_command_pct - pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;

    if (esc_command_pct[0] > 1) {
        esc_command_pct[0] = 1;
    }
    if (esc_command_pct[1] > 1) {
        esc_command_pct[1] = 1;
    }
    if (esc_command_pct[2] > 1) {
        esc_command_pct[2] = 1;
    }
    if (esc_command_pct[3] > 1) {
        esc_command_pct[3] = 1;
    }

    if (esc_command_pct[0] < 0) {
        esc_command_pct[0] = 0;
    }
    if (esc_command_pct[1] < 0) {
        esc_command_pct[1] = 0;
    }
    if (esc_command_pct[2] < 0) {
        esc_command_pct[2] = 0;
    }
    if (esc_command_pct[3] < 0) {
        esc_command_pct[3] = 0;
    }
}