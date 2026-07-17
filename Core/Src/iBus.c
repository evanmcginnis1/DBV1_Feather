/*
 * iBus.c
 *
 * Created on: July 15, 2026
 * Author: Evan McGinnis
 * 
 * Code to decode iBus protocol from reciever in order to be read by flight controller
 */
#include <ibus.h>

static uint8_t uart_buf[IBUS_FRAME_SIZE] = {0};
static uint8_t failsafe_flag_count = 0;

//static functions
static void ibus_update(uint16_t* ibus_data);
static bool ibus_verify_start(void);
static bool ibus_checksum(void);

// Tell DMA to target uart_buf variable
void ibus_init(UART_HandleTypeDef* huart) {
    HAL_UART_Receive_DMA(huart, uart_buf, IBUS_FRAME_SIZE);
}

bool ibus_read(uint16_t* ibus_data) {
    if (!ibus_verify_start()) {
        return false;
    }

    if (!ibus_checksum()) {
        return false;
    }

    ibus_update(ibus_data);

    return true; 
}

// ibus_data is an array where each index in the array represents a channel whose value is between 1000 and 2000
static void ibus_update(uint16_t* ibus_data) {
    for (int channel_idx = 0, buf_idx = 2; channel_idx < IBUS_NUM_CHANNELS; channel_idx++, buf_idx += 2) {
        ibus_data[channel_idx] = (uart_buf[buf_idx] | uart_buf[buf_idx + 1] << 8);
    }
}

void ibus_failsafe_check(uint16_t* ibus_data) {

    failsafe_flag_count++;

    if (failsafe_flag_count <= IBUS_FAILSAFE_MAX) {
        return;
    } else {
        for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
            ibus_data[i] = 0;
        }
    }
}

static bool ibus_verify_start(void) {
    return (uart_buf[0] == IBUS_LENGTH && uart_buf[1] == IBUS_THROTTLE_COMMAND);
}

// checksum = sum of all bytes besides the checksum itself
static bool ibus_checksum(void) {
    uint16_t checksum_theo = 0xFFFF;

    // last two bytes are checksum, so ignore
    for (int i = 0; i < (IBUS_LENGTH - 2); i++) {
        checksum_theo -= uart_buf[i];
    }

    uint16_t checksum_actual = (uart_buf[31] << 8 | uart_buf[30]);

    return (checksum_actual == checksum_theo);
}

void ibus_reset_failsafe(void) {
    failsafe_flag_count = 0;
}

//convert ibus channel data into a number between 0 and 1000 (effectively a percent without using floats)
void ibus_read_as_percents(uint16_t* ibus_data_percents) {
    uint16_t ibus_data[IBUS_NUM_CHANNELS];
    ibus_read(ibus_data);
    for (int i = 0; i < IBUS_NUM_CHANNELS; i++) {
        ibus_data[i] -= 1000;
    }
}