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
 * Requires: ibus_data is an array of 16-bit variables with size equal to the number of channels
 * Modifies: 
 * Effects: 
 */
bool ibus_read(uint16_t* ibus_data);

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
void ibus_update(uint16_t* ibus_data);

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
void ibus_failsafe_check();

/*
 * Requires: Nothing
 * Modifies: Nothing
 * Effects: Checks that the first two bytes of the frame are correct
 */
bool ibus_verify_start(void);

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
bool ibus_checksum(void);

/*
 * Requires: 
 * Modifies: 
 * Effects: 
 */
void ibus_reset_failsafe(void);

#endif