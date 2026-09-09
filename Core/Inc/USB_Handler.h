/*
 * USB_Handler.h
 *
 * Created on: August 19, 2026
 * Author: Evan McGinnis
 * 
 */

#ifndef USB_HANDLER_H
#define USB_HANDLER_H


#include "IMU_Model.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdbool.h>


#define UPPERCASE_TO_LOWERCASE_DIFFERENCE 32
typedef enum {
    DOWNLOAD_LOGS,
    UPDATE_PID_GAINS,
    UNLOCK,
    INVALID_COMMAND,
} User_USB_Commands_t;

typedef enum {
    CONFIRM,
    CANCEL,
    INVALID_INPUT,
} User_Confirmation_Result_t;
/*
 * Requires: uart_buffer is a uint8_t type array
 * Modifies: nothing
 * Effects: 
 */

void run_usb_tests(void);

User_USB_Commands_t get_usb_command();

void unlock_state(bool* disarm_locked);

bool confirm_user_action(const char* user_action_string);



void wait_for_user_input(uint32_t delay_ms_between_checks);
/*
 * Requires: uart_buffer has been filled, Len is the size of the string that has been written to the uart buffer
 * Modifies: uart_buffer
 * Effects: Adds a null terminator to the uart rx string. Returns true unless the string is exactly the same size as the
 * buffer. If string is same size, returns false and does not append null terminator 
 */
bool add_null_terminator(uint8_t* uart_buffer, const uint32_t* Len);


#endif