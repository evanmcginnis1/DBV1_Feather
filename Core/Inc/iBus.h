/*
 * iBus.h
 *
 * Created on: July 15, 2026
 * Author: Evan McGinnis
 * 
 * Header file for ibus decode functions
 */
 
#ifndef IBUS_H
#define IBUS_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define IBUS_UART &huart1
#define IBUS_NUM_CHANNELS 14
#define IBUS_FRAME_SIZE 32
//adjust so that max delay between dead reciever and full shutdown is 0.1s
// counter increments up every 0.01 seconds
#define IBUS_FAILSAFE_MAX 10
//length of transmission is 32 bytes
#define IBUS_LENGTH 0x20
//0x40 denotes throttle signal
#define IBUS_THROTTLE_COMMAND 0x40

/*
 * Requires: UART object has been initialized. Must be called before main(). UART DMA is set to circular mode
 * Modifies: DMA registers, uart_buf
 * Effects: points DMA to put uart data in buffer, starts circular transfers
 */
void ibus_init(UART_HandleTypeDef* huart);


/*
 * Requires: ibus_data is an array of 16-bit variables with size equal to the number of channels. 
 * Modifies: 
 * Effects: 
 */
bool ibus_read_raw(uint16_t* ibus_data);

/*
Does same thing as ibus_read, but outputs data for each channel as a number between 0 and 1000, rather than between 
1000 and 2000
 * Requires: new ibus data is available in buffer. 
 * Modifies: ibus_data_percents array
 * Effects: returns false if ibus data is invalid. returns true otherwise. Converts raw ibus data into number between 0 
            and 1000
*/
bool ibus_read_as_percents(uint16_t* ibus_data_percents);

/* 
 * Requires: ibus_data has filled with ibus data in *percent* form
 * Modifies: nothing
 * Effects: checks if channel 5 of ibus transmission is low or high. If it is low, then the quadcopter is armed so
            return true. Otherwise, return false.
 */
bool ibus_is_armed(const uint16_t* ibus_data_percents);
/*
 * Requires: HAL_UART_RxCpltCallback function is implemented to reset failsafe flag every time it is triggered
 * Modifies: ibus_data
 * Effects: Failsafe for if reciever packets are stale (does not cover transmitter power loss). Sets all channels to zero
 */
bool ibus_failsafe_check(uint16_t* ibus_data);

/*
 * Requires: failsafe flag counter has been initialized
 * Modifies: failsafe flag counter
 * Effects: Resets failsafe flag count to zero. Call from within HAL_UART_RxCpltCallback
 */
void ibus_reset_failsafe(void);



#endif