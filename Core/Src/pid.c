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
#include <gpio.h>
#include <string.h>
#include <usbd_cdc_if.h>
#include <stddef.h>
#include "USB_Handler.h"


static PID_t pid_roll_info;
static PID_t pid_pitch_info;
static PID_t pid_yaw_info;
static Pid_Output_t pid_axis_out_pct;

static int8_t axis_synth_coeffs[NUM_MOTORS][3] = {{+1, +1, -1},
                                                  {+1, -1, +1}, 
                                                  {-1, +1, +1}, 
                                                  {-1, -1, -1}};

// Static Functions
/*
Requires: pid_info is the specific PID_t object for the chosen axis. actual_val is the quadcopters current state, 
          based on the imu. setpoint is a pointer to the member in the setpoints array representing the chosen axis. 
          output is the pid output for that axis
Modifies: updates prev_error term in given pid_info object with new error. updates output variable
Effects: calculates error, integrated error, and the rate of change of the error. Then, multiplies these with their 
         respective gain values and sums them. 
*/
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, int16_t* output);
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

static void check_idle(int16_t* signed_commands);
static void check_max(int16_t* signed_commands);

//USB update gain static functions
static bool check_new_USB_gain_value(const float* new_gain);
static const char* get_axis_name(const char* target_specifier);
static const char* get_gain_name(const char* target_specifier);
static size_t get_pid_gain_offset(const char* target_specifier);
static PID_t* get_pid_target(const char* target_specifier);
static void print_invalid_pid_command_msg(void);
static void print_invalid_pid_axis_specifier_msg(void);
static void print_invalid_gain_specifier_msg();

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
    float setpoint[NUM_MOTORS];

    pilot_command_to_setpoint(pilot_command, setpoint);
    pid_step_all(imu, setpoint);
    synthesize_pid_commands(&setpoint[2], esc_commands_pct);
}


/* Serial Command Options: (replace x with desired gain value )
p_p x (pitch proportional)
p_i x (pitch integrator gain)
p_d x (pitch derivative gain)

r_p x (roll)
r_i x
r_d x

y_p x (yaw)
y_i x
y_d x
*/
void pid_update_gains(void) {
    uint8_t read_success = 0;
    float new_gain = 0.0f;
    // string should only ever be 3 characters long, plus one null terminator character
    char target_specifier[4];

    pid_update_gains_prompt();
    
    //wait for user input
    wait_for_user_input(10);
    uart_data_ready = false;

    uint8_t uart_buffer[APP_RX_DATA_SIZE];
    uint32_t len = uart_receive_len;
    memcpy(uart_buffer, UserRxBufferFS, sizeof(UserRxBufferFS));
    //need null terminator to prevent sscanf from reading forever without stopping
    add_null_terminator(uart_buffer, &len);

    //limit to 15 characters in float to prevent buffer overflow, but don't ever expect a float that long
    read_success = sscanf((char*)uart_buffer, "%3s %15f", target_specifier, &new_gain);



    if (read_success != 2) {
        print_invalid_pid_command_msg();
        return;
    }
    
    //protect against accidental dangerous gain values. Ignore if user tries to input invalid gain
    check_new_USB_gain_value(&new_gain);

    //write new gain value to pid_info object
    PID_t* pid_axis_info_ptr = get_pid_target(target_specifier);
    size_t pid_gain_offset = get_pid_gain_offset(target_specifier);

    if (pid_axis_info_ptr == NULL) {
        print_invalid_pid_axis_specifier_msg();
        return;
    }

    if (pid_gain_offset == (size_t)-1) {
        print_invalid_gain_specifier_msg();
    }

    float* gain_ptr =  (float*)((uint8_t*)pid_axis_info_ptr + pid_gain_offset);
    *gain_ptr = new_gain;

    const char* axis_target_name = get_axis_name(target_specifier);
    const char* pid_target_name = get_gain_name(target_specifier);

    printf("Updated %s %s to: %.3f", axis_target_name, pid_target_name, new_gain);

    return;
}

void pid_update_gains_prompt(void) {
    printf("Serial PID gain update options: (replace x with desired gain value)\n"
            "p_p x (pitch proportional)\n"
            "p_i x (pitch integrator gain)\n"
            "p_d x (pitch derivative gain)\n"
            "\n"
            "r_p x (roll)\n"
            "r_i x\n"
            "r_d x\n"
            "\n"
            "y_p x (yaw)\n"
            "y_i x\n"
            "y_d x\n");
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

//KP, KI, KD should be sized so that the output is an integer between -1000 and 1000
static void pid_step(PID_t* pid_info, const float* actual_val, const float* setpoint, int16_t* output) {
    float error = *setpoint - *actual_val;
    //new_accum_error is part of static struct so that integral clamp can be applied in separate function
    pid_info->new_accum_error = error * (1.0f / PID_LOOP_RATE_HZ);

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
    setpoint[0] = (((float)pilot_command[0] - 500.0f) / 500.0f) * (float)PID_MAX_ROLL_ANGLE_INPUT;
    //pitch - should map to value between -90 and +90
    setpoint[1] = (((float)pilot_command[1] - 500.0f) / 500.0f) * (float)PID_MAX_PITCH_ANGLE_INPUT;
    //yaw - should map to a value between -500 and 500
    setpoint[3] = (((float)pilot_command[3] - 500.0f) / 500.0f) * (float)PID_MAX_YAW_RATE_INPUT;

    //throttle command stays as a value between 0 and 1000
    setpoint[2] = pilot_command[2] / 2.0f;
}


//TODO: figure out how to handle idle speed, prevent outputting a negative number, prevent esc signal on zero throttle
static void synthesize_pid_commands(const float* throttle_command_pct, uint16_t* esc_command_pct) {
    //synthesize commands
    //esc_command index corresponds to motor index on quadcopter
    int16_t signed_commands[NUM_MOTORS];
    //roll, pitch, yaw

    signed_commands[0] = *throttle_command_pct + pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;
    signed_commands[1] = *throttle_command_pct + pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    signed_commands[2] = *throttle_command_pct - pid_axis_out_pct.roll_pct + pid_axis_out_pct.pitch_pct + pid_axis_out_pct.yaw_pct;
    signed_commands[3] = *throttle_command_pct - pid_axis_out_pct.roll_pct - pid_axis_out_pct.pitch_pct - pid_axis_out_pct.yaw_pct;

    check_integrator(signed_commands);

    //try to shift all motors up or down by same amount. But, if this shift pushes a motor above or below max allowed throttle,
    //then cut off excess
    check_max(signed_commands);
    check_idle(signed_commands);

    for (int motor = 0; motor < NUM_MOTORS; motor++) {
        if (signed_commands[motor] < PID_OUTPUT_IDLE) {
            signed_commands[motor] = PID_OUTPUT_IDLE;
        } else if (signed_commands[motor] > PID_OUTPUT_MAX) {
            signed_commands[motor] = PID_OUTPUT_MAX;
        }
    }

    esc_command_pct[0] = (uint16_t) signed_commands[0];
    esc_command_pct[1] = (uint16_t) signed_commands[1];
    esc_command_pct[2] = (uint16_t) signed_commands[2];
    esc_command_pct[3] = (uint16_t) signed_commands[3];
}


//works by checking direction of overshoot, converting it to a signed value. Then, if accumulated error effective sign is the same as the 
//overshoot direction, removes newly accumulated error
static void check_integrator(int16_t* signed_commands) {
    PID_t* axis_info[3] = {&pid_roll_info, &pid_pitch_info, &pid_yaw_info};
    // array members represent roll, pitch, yaw. If true, then accumulated error is contributing to further overshoot
    bool stop_integrator[3] = {false};
    int8_t direction = 0;
    for (int motor = 0; motor < NUM_MOTORS; motor++) {
        direction = 0; 
        if (signed_commands[motor] > PID_OUTPUT_MAX) {
            direction = 1;
        } else if (signed_commands[motor] < PID_OUTPUT_IDLE) {
            direction = -1;
        }
    
        for (int axis = 0; axis < 3; axis++) {
            //means that accumulated error is contributing to further error
            if ((axis_info[axis]->accumulated_error * direction * axis_synth_coeffs[motor][axis]) > 0) {
                stop_integrator[axis] = true;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
            }
        }
    }
    //prevent subtracting new_accum_error from accumulated error twice if two motors are overshooting in the same way
    for (int axis = 0; axis < 3; axis++) {
        if (stop_integrator[axis]) {
            axis_info[axis]->accumulated_error -= axis_info[axis]->new_accum_error;
        }
    }
}

static void check_idle(int16_t* signed_commands) {
    int16_t shift_distance = 0;
    bool shift_needed = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (signed_commands[i] < PID_OUTPUT_IDLE) {
            shift_needed = true;
            int16_t temp_shift_distance = PID_OUTPUT_IDLE - signed_commands[i];
            if (temp_shift_distance > shift_distance) {
                shift_distance = temp_shift_distance;
            }
        }
    }
    if (shift_needed) {
        for (int i = 0; i < NUM_MOTORS; i++) {
                signed_commands[i] += shift_distance;
        }
    }
}

static void check_max(int16_t* signed_commands) {
    int16_t shift_distance = 0;
    bool shift_needed = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (signed_commands[i] > PID_OUTPUT_MAX) {
            shift_needed = true;
            int16_t temp_shift_distance = signed_commands[i] - PID_OUTPUT_MAX;
            if (temp_shift_distance > shift_distance) {
                shift_distance = temp_shift_distance;
            }
        }
    }
    if (shift_needed) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            signed_commands[i] -= shift_distance;
        }
    }
}


/**********************************************************************************************
                        USB Serial Gain update helper functions
***********************************************************************************************/
static bool check_new_USB_gain_value(const float* new_gain) {
        if (*new_gain < 0) {
        CDC_Transmit_FS((uint8_t*)"Gain must be positive. Please try again.\n", 40);
        return false;
    } else if (*new_gain > 20) {
        CDC_Transmit_FS((uint8_t*)"Gain must be less than 20. Please try again.\n", 44);
        return false;
    }
    return true;
}


static const char* get_axis_name(const char* target_specifier) {
    if (target_specifier[0] == 'p') {
        return "pitch";
    } else if (target_specifier[0] == 'r') {
        return "roll";
    } else if (target_specifier[0] == 'y') {
        return "yaw";
    } else {
        return "INVALID";
    }
}

static const char* get_gain_name(const char* target_specifier) {
    if (target_specifier[2] == 'p') {
        return "proportional gain";
    } else if (target_specifier[2] == 'i') {
        return "integrator gain";
    } else if (target_specifier[2] == 'd') {
        return "derivative gain";
    } else {
        return "INVALID";
    }
}

static size_t get_pid_gain_offset(const char* target_specifier) {
    switch(target_specifier[2]) {
        case 'p': return offsetof(PID_t, kp);
        case 'i': return offsetof(PID_t, ki);
        case 'd': return offsetof(PID_t, kd);
        default: return (size_t)-1 ;
    }
}

static PID_t* get_pid_target(const char* target_specifier) {
    switch (target_specifier[0]) {
        case 'p': return &pid_pitch_info;
        case 'r': return &pid_roll_info;
        case 'y': return &pid_yaw_info;
        default: return NULL;
    }
}

static void print_invalid_pid_command_msg(void) {
    printf("Failed to parse pid command input. Double check that your syntax is correct and try again. \n");
}

static void print_invalid_pid_axis_specifier_msg(void) {
    printf("Invalid axis specifier. Axis specifier must be p_, r_, y_");
}

static void print_invalid_gain_specifier_msg() {
    printf("Invalid gain target specifier. Gain specifier must be p, i, or d");
}