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
//adjust so that max delay between dead reciever and full shutdown is <0.3s
#define IBUS_FAILSAFE_MAX 10
//length of transmission is 32 bytes
#define IBUS_LENGTH 0x20
//0x40 denotes throttle signal
#define IBUS_THROTTLE_COMMAND 0x40

/*
 * Requires: UART object has been initialized. Must be called before main().
 * Modifies: 
 * Effects: points DMA to put uart data in buffer
 */
void ibus_init(UART_HandleTypeDef* huart);

/*
 * Requires: ibus_data is an array of 16-bit variables with size equal to the number of channels. 
 * idx map: 
 1: 
 * Modifies: 
 * Effects: 
 */
bool ibus_read(uint16_t* ibus_data);


/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
void ibus_failsafe_check();

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
void ibus_reset_failsafe(void);

// Does same thing as ibus_read, but outputs data for each channel as a number between 0 and 1000, rather than between 
// 1000 and 2000
// 
void ibus_read_as_percents(uint16_t* ibus_data_percents);

#endif