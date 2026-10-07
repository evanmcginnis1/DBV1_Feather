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
#include "pid.h"
#include "stm32f405xx.h"
#include <sys/_intsup.h>

#define ENTRIES_PER_PAGE 4
#define FLASH_METADATA_SIZE_PAGES ((sizeof(FlightLogger_Metadata_t) + FLASH_PAGE_SIZE_BYTES - 1) / FLASH_PAGE_SIZE_BYTES)
#define FLASH_METADATA_SIZE_BYTES (FLASH_METADATA_SIZE_PAGES * FLASH_PAGE_SIZE_BYTES)
#define FLIGHTLOG_MAX_ENTRIES ((FLASH_CHUNK_SIZE_64 - FLASH_METADATA_SIZE_BYTES) / sizeof(FlightLog_Packet_t))
#define MEMORY_MAP_CHUNKS_PER_ROW 4

//entry counter is 16 bits, and 0xFFFF is reserved to mark erased (unused) entries
_Static_assert(FLIGHTLOG_MAX_ENTRIES < UINT16_MAX, "Log entries must fit in 16-bit entry counter");



/* 
 * Requires: Flash chip 
 * Modifies: chunk_map_list, pid_out_pcts
 * Effects: Creates chunk map list, 
 */
void flash_init(const Pid_Output_t* pid_outputs);

void flash_erase_chip(void);

/*
 * Requires: chunk map is up to date, not WIP
 * Modifies: chunk that holds log_number
 * Effects: Erases the chunk holding the given log. Returns false and erases nothing if log_number is not in chunk map
 */
bool flash_erase_log(uint32_t log_number);

void flash_new_log(const PID_t* pitch_info, const PID_t* roll_info, const PID_t* yaw_info);
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
                    const float* pilot_setpoint, const uint16_t* synthesized_motor_commands);



void flash_get_metadata(uint32_t log_number, FlightLogger_Metadata_t* metadata);

/*
 * Requires: chunk map is up to date, not WIP
 * Modifies: metadata
 * Effects: Reads metadata of the log with the highest log number. Returns false and leaves metadata untouched if
 *          there are no logs on the chip
 */
bool flash_get_newest_metadata(FlightLogger_Metadata_t* metadata);
/*
 * Requires: WIP flag not set
 * Modifies: UART
 * Effects: prints visual depiction of which memory chunks have data in them, and which data is there. 
 NOTE: chunk_map_list must be of size FLASH_NUM_CHUNKS, and it will be overwritten
 */

void flash_print_memory_map(void);

void flash_update_memory_map(void);

const uint32_t* flash_get_chunk_map_list(void);
/*
 * Requires: log_number is between 0 and UINT32_MAX, log_number is valid (i.e. appears in chunk map)
 * Modifies: nothing
 * Effects: returns chunk number that corresponds to given log
 */
int get_chunk_number_by_log(uint32_t log_number);

/*
 * Requires: log_number is a reference to an existing log, not WIP, metadata is write-only, data is write-only
 * Modifies: metadata, data
 * Effects: reads chunk into metadata and data arrays. Returns false if invalid log number is given
 */
bool flash_read_datapoint(uint32_t log_number, uint16_t entry_number, FlightLog_Packet_t* data);

/*
 * Requires: chunk map is up to date, not WIP
 * Modifies: nothing
 * Effects: Returns true if log has no datapoints (only metadata) or log_number is not in chunk map
 */
bool flash_log_is_empty(uint32_t log_number);

uint32_t get_max_log_number(void);
uint32_t get_min_log_number(void);
