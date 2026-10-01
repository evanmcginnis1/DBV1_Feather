/*
 * USB_Handler.c
 *
 * Created on: August 19, 2026
 * Author: Evan McGinnis
 * 
 */

#include "USB_Handler.h"
#include "GD25Q16.h"
#include "State.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "SPIFlash_Conductor.h"
#include "Helper_Functions.h"
#include <inttypes.h>

//static functions

/*
 * Requires: user_command variable has been set to a valid value
 * Modifies: UART TX buffer, UserRXBufferFS, uart_data_ready, 
 * Effects: Sends user a message with their input, asks for confirmation. If confirmation given, return true. Else false
 */
static bool confirm_usb_mode_input(const User_USB_Commands_t* user_command);


/* 
 * Requires: user has sent a new message over serial after being prompted to confirm
 * Modifies: nothing
 * Effects: Returns true if user confirms their choice, false if user cancels 
 */
static User_Confirmation_Result_t check_user_confirmation_input(uint8_t uart_buffer[APP_RX_DATA_SIZE], const uint32_t Len);


/*
 * Requires: Nothing
 * Modifies: input
 * Effects: If there are capital letters in input, makes them lowercase. Leaves all other characters alone
 */
static void normalize_input(uint8_t* input, const uint32_t* Len);

static void prompt_for_usb_commands(void);

static char* usb_mode_to_string(const User_USB_Commands_t* user_command);
static const char* flight_state_to_string(const Quadcopter_State_t* state);

static void print_metadata(const FlightLogger_Metadata_t* metadata);
static void print_flightlog_data_header(void);
static void print_flightlog_datapoint(const FlightLog_Packet_t* datapoint);
/*
 * Requires: nothing
 * Modifies: USB TX buffer
 * Effects: Sends confirmation message over USB virtual COM to user
 */
static void send_confirmation_msg(const char* user_command_string);

User_Confirmation_Result_t check_user_confirmation_input(uint8_t uart_buffer[APP_RX_DATA_SIZE], const uint32_t Len);

// "Log Counter, packet_version, Pitch_P, Pitch_I, Pitch_D, Roll_P, Roll_I, Roll_D, Yaw_P, Yaw_I, Yaw_D \n"
static void print_flightlog_metadata_header(void);

/*
 * Requires: chunk_map_list is an array of size FLASH_NUM_CHUNKS
 * Modifies: UART input buffer, 
 * Effects: Prompts user to select the log that they want and returns chosen log number. If user chooses invalid log,
            return INVALID_LOG_CHOICE
 */
static uint32_t select_log_to_download(const uint32_t* chunk_map_list);

/**************************************************** */

//Tests
static void test_prompt_for_usb_commands(void);
static void test_check_user_confirmation_input(void);
static void test_normalize_usb_input(void);
static void test_confirm_usb_input(void);

void run_usb_tests(void) {
    while(1) {

        test_prompt_for_usb_commands();
        test_check_user_confirmation_input();
        test_normalize_usb_input();
        test_confirm_usb_input();
        HAL_Delay(5000);
    }

}

static void test_prompt_for_usb_commands(void) {
    printf("testing prompt_for_usb_commands: \n");
    prompt_for_usb_commands();
}

static void test_check_user_confirmation_input(void) {
    printf("Testing check_user_confirmation_input: \n");

    char* test_inputs[6] = {"y", "n", "z", "Y", "yessir", ""};
    for (int i = 0; i < 6; i++) {

        uint32_t len = strlen(test_inputs[i]);
        char mutable_string[len + 1];
        strcpy(mutable_string, test_inputs[i]);
        User_Confirmation_Result_t result = check_user_confirmation_input((uint8_t*)mutable_string, strlen(test_inputs[i]));
        switch (result) {
            case CONFIRM:
                printf("Input: %s, Result: CONFIRM\n", test_inputs[i]);
                break;
            case CANCEL:
                printf("Input: %s, Result: CANCEL\n", test_inputs[i]);
                break;
            case INVALID_INPUT:
                printf("Input: %s, Result: INVALID_INPUT\n", test_inputs[i]);
                break;
        }
    }
}



static void test_normalize_usb_input(void) {
    char* test_inputs[5] = {"abc", "ABC", "AbC", "aBc", "abc123"};
    char* expected_outputs[5] = {"abc", "abc", "abc", "abc", "abc123"};
    for (int i = 0; i < 5; i++) {
        
        uint32_t len = strlen(test_inputs[i]);
        char mutable_string[len + 1];
        strcpy(mutable_string, test_inputs[i]);

        normalize_input((uint8_t*)mutable_string, &len);
        printf("Expected output: %s, Actual output: %s\n", expected_outputs[i], mutable_string);
    }
}

static void test_confirm_usb_input(void) {
    User_USB_Commands_t unlock = UNLOCK;
    User_USB_Commands_t download_logs = DOWNLOAD_LOGS;
    User_USB_Commands_t update_pid = UPDATE_PID_GAINS;
    User_USB_Commands_t invalid = INVALID_COMMAND;
    printf("Testing confirm_usb_input: \n"); 

    printf("Testing 'unlock' message response: \n");
    confirm_usb_mode_input(&unlock);

    printf("Testing 'DOWNLOAD_LOGS' message response: \n");
    confirm_usb_mode_input(&download_logs);

    printf("Testing 'PID_UPDATE_GAINS' message response: \n");
    confirm_usb_mode_input(&update_pid);

    printf("Testing with invalid input: \n");
    confirm_usb_mode_input(&invalid);
}
/*************************************************** */


// Command options: 
/*
 * update_pid
 * download_logs
 * unlock
 */
//makes a copy of the uart buffer to prevent it being overwritten while function is working
User_USB_Commands_t get_usb_command(void) {

    uint8_t uart_buffer[APP_RX_DATA_SIZE];
    //wait for initial user input; can be any character

    prompt_for_usb_commands();

    wait_for_user_input(10);
    uart_data_ready = false;

    // copy uart buffer to prevent volatility issues of it updating in background while this function is running
    memcpy(uart_buffer, UserRxBufferFS, sizeof(UserRxBufferFS)); 
    uint32_t len = uart_receive_len;

    //if uart_buffer data is too long, command is invalid
    User_USB_Commands_t user_command;
    //make entry all lowercase, leave non alphabetical characters alone
    normalize_input(uart_buffer, &uart_receive_len);
    //if uart_buffer data is too long, command is invalid
    if (!add_null_terminator(uart_buffer, &len)) {
        return INVALID_COMMAND;
    }


    if (strcmp((char*)uart_buffer, "update_pid") == 0) {
        user_command = UPDATE_PID_GAINS;
    } else if (strcmp((char*)uart_buffer, "download_logs") == 0) {
        user_command = DOWNLOAD_LOGS;
    } else if (strcmp((char*)uart_buffer, "unlock") == 0) {
        user_command = UNLOCK;
    } else {
        return INVALID_COMMAND;
    }
    
    if (confirm_usb_mode_input(&user_command)) {
        printf("confirmed command\n");
        return user_command;
    } else {
        return INVALID_COMMAND;
    }
}


static bool confirm_usb_mode_input(const User_USB_Commands_t* user_command) {
    if (*user_command == INVALID_COMMAND) {
        return false;
    }
    const char* user_command_string = usb_mode_to_string(user_command);
    return confirm_user_action(user_command_string);
}

static char* usb_mode_to_string(const User_USB_Commands_t* user_command) {
    if (*user_command == UPDATE_PID_GAINS) {
        return "update pid gains";
    } else if (*user_command == DOWNLOAD_LOGS) {
        return "Download flight logs";
    } else if (*user_command == UNLOCK){
        return "unlock disarm state (quad can re-arm in ten seconds if arm switch flipped down)";
    } else {
        //should never actually be called; just prevents compiler warning
        return "invalid";
    }
}

void unlock_state(bool* disarm_locked) {
    if (confirm_user_action("unlock from hard disarm state")) {
        *disarm_locked = false;
        printf("Unlocked hard disarm. After 10 second delay, quad behavior will reflect arm switch state.\n"
                "To remain in hard_disarm mode, ensure hard_disarm switch remains flipped down after the delay\n");
    } else {
        printf("Unlock canceled. Quadcopter will remain in hard_disarm mode.\n");
    }
    // else: do nothing
}
//"Are you sure you want to %s? Type Y to confirm or N to cancel\n", user_command_string"
bool confirm_user_action(const char* user_action_string) {

    send_confirmation_msg(user_action_string);
    wait_for_user_input(10);
    uart_data_ready = false;
    // no point copying uart data to a separate array since i'm just doing a simple comparison on it in one step. it's 
    // unlikely to change during the time it takes to do that one action
    
    switch (check_user_confirmation_input(UserRxBufferFS, uart_receive_len)) {
        case CONFIRM:
            return true;
        case CANCEL:
            printf("Cancelling request\n");
            return false;
        case INVALID_INPUT:
            printf("Invalid confirmation message --> Cancelling request. Please try again\n");
            return false;
        default: 
            return false;
    }
}

static User_Confirmation_Result_t check_user_confirmation_input(uint8_t uart_buffer[APP_RX_DATA_SIZE], const uint32_t Len) {
    //make input all lowercase
    normalize_input(uart_buffer, &Len);
    //allow space for newline character but don't require it
    if (Len > 1) {
        return INVALID_INPUT;
    }

    if (uart_buffer[0] == 'y') {
        return CONFIRM;
    } else if (uart_buffer[0] == 'n') {
        return CANCEL;
    } else {
        return INVALID_INPUT;
    }
}

// messages
static void prompt_for_usb_commands(void) {
    printf("\n------------------------------------------------------------------------------------------------\n\n"
            "Please enter one of the following commands: \n\n"
            "Flight Controller USB Commands:\n"
            "'update_pid': Allows user to update PID gain values\n"
            "'download_logs': Streams flight log data over serial to computer\n"
            "'unlock': unlocks quad from hard disarm state. DANGER: AFTER TEN SECONDS, QUAD CAN POTENTIALLY BE"
            "RE-ARMED. BE PREPARED TO MOVE AWAY QUICKLY\n\n");
}


static void send_confirmation_msg(const char* user_command_string) {

    printf("\nAre you sure you want to %s? Type Y to confirm or N to cancel\n", user_command_string);
}


//utility
bool add_null_terminator(uint8_t* uart_buffer, const uint32_t* Len) {
    // indexing to len, not len-1 gives one more than last index of user input
    // uart buffer is much larger than the string should ever be, so little overflow risk

    // catch edge case where user inputs exactly the number of bytes that fit in the rx buffer. don't add null character 
    // but return false
    if (*Len >= APP_RX_DATA_SIZE) {
        return false;
    }

    uart_buffer[*Len] = '\0';
    return true;
}


void wait_for_user_input(uint32_t delay_ms_between_checks) {
    while (!uart_data_ready) {
    HAL_Delay(delay_ms_between_checks);
    }
}

static void normalize_input(uint8_t* input, const uint32_t* Len) {
    
    for (int i = 0; i < *Len; i++) {
        // if current index is a capital letter, make it lowercase
        if (input[i] >= 'A' && input[i] <= 'Z') {
            input[i] += UPPERCASE_TO_LOWERCASE_DIFFERENCE;  
        }
        //TODO: Remove all whitespace and newline characters
    }
}

static uint32_t select_log_to_download(const uint32_t* chunk_map_list) {
    uint8_t input_buffer[APP_RX_DATA_SIZE];
    printf("Which log would you like to download?");
    wait_for_user_input(10);\
    uart_data_ready = false;
    //use memcpy to prevent volatile UserRxBuffer changing while using it
    uint32_t len = uart_receive_len;
    memcpy(input_buffer, UserRxBufferFS, uart_receive_len);
    add_null_terminator(input_buffer, &len);
    // cast from unsigned char to char
    int user_choice = atoi((char*)input_buffer);
    
    int max_log_number = max_in_list(chunk_map_list, FLASH_NUM_CHUNKS);
    int min_log_number = min_in_list(chunk_map_list, FLASH_NUM_CHUNKS);

    if (user_choice > max_log_number || user_choice < min_log_number) {
        //replace with named constant
        return INVALID_LOG_CHOICE;
    } else {
        return user_choice;
    }

}

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
 //TODO: implement
static void transmit_logs(uint32_t log_number) {
    //read selected log into an array of type FlightLog_packet_t with size FLIGHTLOG_MAX_ENTRIES
    FlightLog_Packet_t flight_data[FLIGHTLOG_MAX_ENTRIES];
    FlightLogger_Metadata_t metadata;
    uint32_t log_start_addr = log_number_to_chunk_addr(log_number);
    flash_read_metadata(&log_start_addr, &metadata);
    print_flightlog_metadata_header();
    print_metadata(&metadata);
    print_flightlog_data_header();
    
    for (int i = 0; i < FLIGHTLOG_MAX_ENTRIES; i++) {
        FlightLog_Packet_t datapoint;
        flash_read_datapoint(log_number, i, &datapoint);
        print_flightlog_datapoint(&flight_data[i]);
    }
}


void download_logs(void) {
    uint32_t chunk_map_list[FLASH_NUM_CHUNKS];
    flash_print_memory_map(chunk_map_list);

    uint32_t user_choice = select_log_to_download(chunk_map_list);
    //wait for valid input
    while (user_choice == INVALID_LOG_CHOICE) {
        uint32_t min = min_in_list(chunk_map_list, FLASH_NUM_CHUNKS);
        uint32_t max = max_in_list(chunk_map_list, FLASH_NUM_CHUNKS);
        printf("Invalid log selection. Choice must be between %" PRIu32 " and %" PRIu32 "\n", min, max);
        user_choice = select_log_to_download(chunk_map_list);
    }

    transmit_logs(user_choice);
}

static void print_flightlog_metadata_header(void) {
    printf("Log Counter, packet_version, Pitch_P, Pitch_I, Pitch_D, Roll_P, Roll_I, Roll_D, Yaw_P, Yaw_I, Yaw_D \n");
}

//TODO: Verify Units
static void print_flightlog_data_header(void) {
    printf("Current state, Entry Counter, Pitch Angle (deg), Roll Angle (deg), Pitch Rate(dps), Roll Rate (dps)," 
        "Yaw Rate (dps), Accel X (g), Accel Y (g), Accel Z (g), Pilot Roll Input (%%), Pilot Pitch Input (%%), "
        "Pilot Throttle Input (%%), Pilot Yaw Input (%%), Raw Motor Command 1, Raw Motor Command 2, Raw Motor Command 3, Raw Motor Command 4, Normalized Motor Command 1, Normalized Motor Command 2, Normalized Motor Command 3, Normalized Motor Command 4 \n");
}

static void print_metadata(const FlightLogger_Metadata_t* metadata) {
    printf("%" PRIu32 ", %" PRIu32 ", %3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f \n", metadata->log_counter, metadata->packet_version, 
            metadata->pitch_proportional_gain, metadata->pitch_integrator_gain, metadata->pitch_derivative_gain, 
            metadata->roll_proportional_gain, metadata->roll_integrator_gain, metadata->roll_derivative_gain,
            metadata->yaw_proportional_gain, metadata->yaw_integrator_gain, metadata->yaw_derivative_gain);
}

static void print_flightlog_datapoint(const FlightLog_Packet_t* datapoint) {
    printf("%s, %" PRIu16 ", %3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f, %" PRIu16 ", %" PRIu16 ", %" PRIu16 ", %" PRIu16 ", %" PRIu16 ", %" PRIu16 ", %" PRIu16 ", %" PRIu16 "\n", flight_state_to_string(&datapoint->current_state), datapoint->entry_counter,
            datapoint->pitch_angle, datapoint->roll_angle, datapoint->pitch_rate, datapoint->roll_rate, datapoint->yaw_rate,
            datapoint->pitch_command_angle, datapoint->roll_command_angle, datapoint->yaw_command_rate, 
            datapoint->throttle_command, datapoint->m1_output_raw, datapoint->m2_output_raw, datapoint->m3_output_raw, 
            datapoint->m4_output_raw, datapoint->m1_output_normalized, datapoint->m2_output_normalized,
            datapoint->m3_output_normalized, datapoint->m1_output_normalized); 
}

static const char* flight_state_to_string(const Quadcopter_State_t* state) {
    switch (*state) {
        case ARMED: return "ARMED";
        
        case SOFT_DISARM: return "SOFT_DISARM";
        
        case HARD_DISARM: return "HARD_DISARM";
        default: return "UNKNOWN";
    }
}