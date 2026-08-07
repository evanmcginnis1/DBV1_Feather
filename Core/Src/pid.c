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
/*
Requires: pid_info is the specific PID_t object for the chosen axis. actual_val is the quadcopters current state, 
          based on the imu. setpoint is a pointer to the member in the setpoints array representing the chosen axis. 
          output is the pid output for that axis
Modifies: updates prev_error term in given pid_info object with new error. updates output variable
Effects: calculates error, integrated error, and the rate of change of the error. Then, multiplies these with their 
         respective gain values and sums them. 
*/
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, uint16_t* output);
/*
Requires: imu has been updated with data from the imu. setpoint is an array of values representing degrees for pitch and
          roll and degrees per second for yaw.
Modifies: setpoint array
Effects: Calls pid_step for each axis. 
*/
static void pid_step_all(const IMU_Model_t* imu, const float* setpoint);
/*
Requires: throttle_command is an integer value between 0 and 1000 representing the pilot's throttle input. 
          esc_commands_pct is an array holding values to be passed to the esc
Modifies: updates esc_commands_pct array
Effects:  converts axis & throttle commands into individual motor commands. Calls check_integrator function
*/
static void synthesize_pid_commands(const float* throttle_command, uint16_t* esc_commands_pct);
/*
Requires: pilot_command is an array of four values between 0 and 1000
Modifies: setpoint array
Effects: converts pilot pitch, roll command into degrees, yaw into dps, throttle stays the same
*/
static void pilot_command_to_setpoint(const uint16_t* pilot_command, float* setpoint);

/*
Requires: esc_commands_pct is a signed value generated from synthesizing the pid commands into motor commands
Modifies: resets pid_info.accumulated_error variable if it is pushing the motor command value further past its limit. 
Effects:  Checks if esc command is greater than limit or less than minimum. If so, resets accumulated error terms that 
          are pushing further past limit in order to prevent integrator windup. 
*/
static void check_integrator(int16_t* esc_commands_pct);

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

/**
 * @brief Updates runs pid loop one time based on pilot commands and imu data
 * 
 * @param imu IMU data
 * @param pilot_command Pilot control input.
 * @param esc_commands_pct Array to store output
 *
*/
void pid_update(const IMU_Model_t* imu, const uint16_t* pilot_command, uint16_t* esc_commands_pct) {
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
    pid_step(&pid_roll_info, &imu->roll_abs, &setpoint[0], &pid_axis_out_pct.roll_pct);
    //PID pitch step
    pid_step(&pid_pitch_info, &imu->pitch_abs, &setpoint[1], &pid_axis_out_pct.pitch_pct);
    //PID yaw step
    pid_step(&pid_yaw_info, &imu->yaw_rate, &setpoint[3], &pid_axis_out_pct.yaw_pct);

    return;
}

//KP, KI, KD should be sized so that the output is an integer between 0 and 1000
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, uint16_t* output) {
    float error = *setpoint - *actual_val;
    //new_accum_error is part of static struct so that integral clamp can be applied in separate function
    pid_info->new_accum_error = error * (1.0  / PID_LOOP_RATE_HZ);

    float p = error * pid_info->kp;
    float i = (pid_info->accumulated_error + pid_info->new_accum_error) * pid_info->ki;
    float d = ((error - pid_info->prev_error) * PID_LOOP_RATE_HZ) * pid_info->kd;

    *output = lrintf(p + i + d);

    pid_info->prev_error = error;
    pid_info->accumulated_error += pid_info->new_accum_error;
}

//map pilot axis input to a number between 0 and 180.
// Pilot command is a number between 1000 and 2000. 
//setpoint[0] = roll
//setpoint[1] = pitch
//setpoint[2] = throttle
//setpoint[3] = yaw

/*
Requires: pilot_command is an array of input values in percent form, each between 0 and 1000
Modifies: setpoint array
Effects: Converts pilot_command percentages into degrees/dps and leaves throttle value between 0-1000
*/
static void pilot_command_to_setpoint(const uint16_t* pilot_command, float* setpoint) {
    //axis commands

    //roll - should map to value between -90 and +90
    setpoint[0] = ((pilot_command[0] - 500) / 500.0f) * PID_MAX_ROLL_ANGLE_INPUT;
    //pitch - should map to value between -90 and +90
    setpoint[1] = ((pilot_command[1] - 500) / 500.0f) * PID_MAX_PITCH_ANGLE_INPUT;
    //yaw - should map to a value between -500 and 500
    setpoint[3] = ((pilot_command[3] - 500) / 500.0f) * PID_MAX_YAW_RATE_INPUT;

    //throttle command stays as a value between 0 and 1000
    setpoint[2] = pilot_command[2];
}


//TODO: figure out how to handle idle speed, prevent outputting a negative number, prevent esc signal on zero throttle
static void synthesize_pid_commands(const float* throttle_command_pct, uint16_t* esc_command_pct) {
    //synthesize commands
    //esc_command index corresponds to motor index on quadcopter
    int16_t signed_commands[4];

    signed_commands[0] = *throttle_command_pct + pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;
    signed_commands[1] = *throttle_command_pct + pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    signed_commands[2] = *throttle_command_pct - pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    signed_commands[3] = *throttle_command_pct - pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;

    check_integrator(signed_commands);

    //ensure valid output
    for (int i = 0; i < 4; i++) {
        if (signed_commands[i] > 1000) {
            signed_commands[i] = 1000;
        }
        if (signed_commands[i] < 0) {
            signed_commands[i] = 0;
        }
    }

    esc_command_pct[0] = (uint16_t) signed_commands[0];
    esc_command_pct[1] = (uint16_t) signed_commands[1];
    esc_command_pct[2] = (uint16_t) signed_commands[2];
    esc_command_pct[3] = (uint16_t) signed_commands[3];
}

//lots of conditionals, but majority of them are nested and will not be called regularly. 
static void check_integrator(int16_t* esc_commands_pct) {
    if (esc_commands_pct[0] > 1000) {
        if (pid_roll_info.accumulated_error > 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
        if (pid_pitch_info.accumulated_error > 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }

        if (pid_yaw_info.accumulated_error < 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }

    } else if (esc_commands_pct[0] < 0) {
        if (pid_yaw_info.accumulated_error > 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
        if (pid_roll_info.accumulated_error < 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
        if (pid_pitch_info.accumulated_error < 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
    }

    if (esc_commands_pct[1] > 1000) {
        if (pid_roll_info.accumulated_error > 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error > 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
        if (pid_pitch_info.accumulated_error < 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
    } else if (esc_commands_pct[1] < 0) {
        if (pid_pitch_info.accumulated_error > 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
        if (pid_roll_info.accumulated_error < 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error < 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
    }

    if (esc_commands_pct[2] > 1000) {
        if (pid_pitch_info.accumulated_error > 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error > 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
        if (pid_roll_info.accumulated_error < 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
    } else if (esc_commands_pct[2] < 0) {
        if (pid_roll_info.accumulated_error > 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
        if (pid_pitch_info.accumulated_error < 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error < 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
    }

    if (esc_commands_pct[3] > 1000) {
        if (pid_pitch_info.accumulated_error < 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error < 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
        if (pid_roll_info.accumulated_error > 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
    } 
    else if (esc_commands_pct[3] < 0) {
        if (pid_pitch_info.accumulated_error > 0) {
            pid_pitch_info.accumulated_error -= pid_pitch_info.new_accum_error;
        }
        if (pid_yaw_info.accumulated_error > 0) {
            pid_yaw_info.accumulated_error -= pid_yaw_info.new_accum_error;
        }
        if (pid_roll_info.accumulated_error > 0) {
            pid_roll_info.accumulated_error -= pid_roll_info.new_accum_error;
        }
    }
}