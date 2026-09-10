/*
 * GD25Q16_Conductor.h
 *
 * Created on: September 09, 2026
 * Author: Evan McGinnis
 * 
 */

//function to read metadata, store it, then skip to first page with data and start reading there. Should read a set 
// number of blocks (2?) and stop when it gets to the next block
//should just read all the data at once

#include "GD25Q16.h"
#include "FlightLogger_Model.h"
#include "IMU_Model.h"
#include "State.h"

#define ENTRIES_PER_PAGE 4
#define FLASH_METADATA_SIZE_PAGES ((sizeof(FlightLogger_Metadata_t) + FLASH_PAGE_SIZE_BYTES - 1) / FLASH_PAGE_SIZE_BYTES)
#define FLASH_METADATA_SIZE_BYTES (FLASH_METADATA_SIZE_PAGES * FLASH_PAGE_SIZE_BYTES)
#define FLIGHTLOG_MAX_ENTRIES ((FLASH_CHUNK_SIZE_64 - FLASH_METADATA_SIZE_BYTES) / sizeof(FlightLog_Packet_t))

/* 
 * Requires: Flash chip 
 * Modifies: current_page_addr and current_block_addr
 * Effects: Adds 5ms delay for flash chip power up time, finds chunk of two blocks to write to
 */
void flash_init(PID_t* pitch_info, PID_t* roll_info, PID_t* yaw_info);


/*
* Requires: data counter is defined as a static variable in SPIFlash_Conductor.c, motor commands and normalized motor 
            commands each have four elements, pilot_inputs in percent form and are ordered such that: 
            [0] = roll
            [1] = pitch
            [2] = throttle
            [3] = yaw
* Modifies: 
* Effects: compiles raw datapoints into flightlog packet. Stores four at a time in buffer, than transmits them all 
            at once to flash chip (so that write entire page at a time)
*/
bool flash_add_entry(const Quadcopter_State_t* current_state, const IMU_Model_t* imu_data, 
                    const float* pilot_setpoint, const uint16_t* raw_motor_commands, 
                    const uint16_t* normalized_motor_commands);

