/*
 * USB_Handler.c
 *
 * Created on: August 19, 2026
 * Author: Evan McGinnis
 * 
 */

#include "USB_Handler.h"
#include "State.h"
#include <string.h>
#include "usbd_cdc_if.h"





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

/*
 * Requires: nothing
 * Modifies: USB TX buffer
 * Effects: Sends confirmation message over USB virtual COM to user
 */
static void send_confirmation_msg(const char* user_command_string);

User_Confirmation_Result_t check_user_confirmation_input(uint8_t uart_buffer[APP_RX_DATA_SIZE], const uint32_t Len);



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
        printf("invalid command\n");
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
            //fallthrough
        case INVALID_INPUT:
            printf("Invalid confirmation message --> Cancelling request. Please try again\n");
            //fallthrough
        default: 
            return false;
    }
}

static User_Confirmation_Result_t check_user_confirmation_input(uint8_t uart_buffer[APP_RX_DATA_SIZE], const uint32_t Len) {
    //make input all lowercase
    normalize_input(uart_buffer, &Len);
    //allow space for newline character but don't require it
    if (Len > 2) {
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
    printf("\nFlight Controller USB Commands:\n"
            "'update_pid': Allows user to update PID gain values\n"
            "'download_logs': Streams flight log data over serial to computer\n"
            "'unlock': unlocks quad from hard disarm state. DANGER: AFTER TEN SECONDS, QUAD CAN POTENTIALLY BE"
            "RE-ARMED. BE PREPARED TO MOVE AWAY QUICKLY\n");
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

