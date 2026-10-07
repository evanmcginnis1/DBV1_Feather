/*
 * GD25Q16_Conductor.c
 *
 * Created on: September 09, 2026
 * Author: Evan McGinnis
 * 
 */

 //function to read a page of data 

#include "SPIFlash_Conductor.h"
#include "GD25Q16.h"
#include "FlightLogger_Model.h"
#include "IMU_Model.h"
#include "State.h"
#include <inttypes.h>
#include <string.h>
#include <sys/reent.h>

static uint16_t datapoint_counter = 0;
static uint32_t new_chunk_base_addr;
static FlightLog_Packet_t data_buffer[ENTRIES_PER_PAGE];
static uint32_t chunk_map_list[FLASH_NUM_CHUNKS];
static const Pid_Output_t* pid_out_pcts;

/*
 * Requires: Flash chip has booted up
 * Modifies: next_block_addr, next_block_counter
 * Effects: Searches through flash memory metadata to find latest entry, then updates 
 *          next_chunk_addr to one block after that and next_chunk_counter to one more than the previous counter
 */
static void find_next_chunk(uint8_t* next_chunk_addr, uint32_t* next_chunk_counter);

/*
 * Requires: Flash chip has booted up. Target is an even numbered block
 * Modifies: Updates counter value based on metadata in block
 * Effects: Reads first four bytes of block for counter value. Returns true and updates counter if block has metadata. 
            returns false if block is completely erased(has no metadata) (Careful: could also be caused by alignment issue). 
 */
static bool flash_get_metadata_counter(uint8_t chunk_idx, uint32_t* counter);

/*
 * Requires: new_block_addr is the address of the block to write current log to, points to chunk of two erased blocks
             WIP flag is cleared
 * Modifies: first page of new block, CS pin, SPI bus
 * Effects: Writes metadata on first page of new chunk of blocks used for flight log. Only writes to first page of first 
 *          block, not second block. 
 */
static void write_metadata(uint8_t new_chunk_idx, uint32_t new_chunk_counter, const PID_t* pitch_info, 
                                const PID_t* roll_info, const PID_t* yaw_info);
/*
 * Requires: Nothing
 * Modifies: Nothing
 * Effects: Reads metadata. Returns true if data found and updates metadata object. Otherwise, 
            returns false if chunk is empty or invalid start addr given
 */
static bool flash_read_metadata(uint8_t chunk_idx, FlightLogger_Metadata_t* metadata);
/*
 * Requires: Nothing
 * Modifies: CS pin
 * Effects: Returns true if block has not been written to, otherwise returns false
 */
static bool chunk_is_erased(uint8_t chunk_idx);


/*
 * Requires: chunk_map_list is an array with size FLASH_NUM_CHUNKS, WIP flag not set
 * Modifies: chunk_map_list, flash read
 * Effects: each index in chunk_map_list represents a chunk. Writes -1 to index if no data found. If data found, writes 
            its metadata number at the given index
 */
static void create_chunk_map_list(void);
/*
 * Requires: log_number is a valid log entry
 * Modifies: 
 * Effects: returns UINT8_MAX if log cannot be found
 */
static uint8_t log_number_to_chunk_idx(uint32_t log_number);
static uint32_t chunk_idx_to_addr(uint8_t chunk_idx);

void flash_init(const Pid_Output_t* pid_outputs) {
    //give chip time to power on
    HAL_Delay(5);
    create_chunk_map_list();
    pid_out_pcts = pid_outputs;
    return;
}

void flash_update_memory_map(void) {
    create_chunk_map_list();
}

/*
 * Requires: new_chunk_base_addr points to current log if any entries are buffered
 * Modifies: flash page after last full page of current log
 * Effects: Writes entries that are buffered but not yet written (fewer than ENTRIES_PER_PAGE). Rest of page stays
 *          erased, so reads still stop at first unused entry. Does nothing if buffer is empty
 */
static void flush_partial_page(void) {
    uint16_t buffered_entries = datapoint_counter % ENTRIES_PER_PAGE;
    if (buffered_entries == 0) {
        return;
    }
    uint32_t addr_to_write = (new_chunk_base_addr + (datapoint_counter / ENTRIES_PER_PAGE) *
                                FLASH_PAGE_SIZE_BYTES);
    page_program(&addr_to_write, buffered_entries * sizeof(FlightLog_Packet_t), (uint8_t*)data_buffer);
}

//in case pid parameters have changed, update them
void flash_new_log(const PID_t* pitch_info, const PID_t* roll_info, const PID_t* yaw_info) {
    uint32_t new_log_counter;
    uint8_t new_chunk_idx;
    //finish previous log before starting a new one
    flush_partial_page();
    find_next_chunk(&new_chunk_idx, &new_log_counter);
    write_metadata(new_chunk_idx, new_log_counter, pitch_info, roll_info, yaw_info);
    // start on first page after metadata
    new_chunk_base_addr = chunk_idx_to_addr(new_chunk_idx) + FLASH_METADATA_SIZE_BYTES;
    //new log starts writing from its first entry
    datapoint_counter = 0;
    flash_update_memory_map();
}
void flash_erase_chip(void) {
    for (int idx = 0; idx < FLASH_NUM_CHUNKS; idx++) {
        uint32_t addr = chunk_idx_to_addr(idx);
        erase_chunk(&addr);
    }
    //chunk map is stale after erase
    create_chunk_map_list();
}

bool flash_erase_log(uint32_t log_number) {
    uint8_t chunk_idx = log_number_to_chunk_idx(log_number);
    //log not found; don't erase anything
    if (chunk_idx == UINT8_MAX) {
        return false;
    }
    uint32_t chunk_start_addr = chunk_idx_to_addr(chunk_idx);
    erase_chunk(&chunk_start_addr);
    return true;
}

static void find_next_chunk(uint8_t* next_chunk_idx, uint32_t* next_chunk_log_counter) {

    //counter is total number of datapoints that have been written
    uint32_t max_found_log_number = 0;
    //index is just a number that loops from 0 to FLASH_NUM_CHUNKS (idx = newest_counter % FLASH_NUM_CHUNKS)
    uint32_t newest_chunk_idx = 0;
    bool found_data = false;


    //check all chunks even if first one is empty just in case
    for (int chunk = 0; chunk < FLASH_NUM_CHUNKS; chunk += 1) {
        uint32_t current_log_number;
        bool current_chunk_is_erased;

        //update counter if find metadata
        current_chunk_is_erased = !flash_get_metadata_counter(chunk, &current_log_number);

        //if this chunk is erased, then don't update newest_found_counter variable
        //continue instead of break in case there is a later chunk that does have data (not expected). maintain ordering
        if (current_chunk_is_erased) {
            //found free block; ;
            continue;
        } 

        //search for max log number value. log number increases monotonically, so max log number must be most recenlty 
        //written log. 
        //use first found data as seed for finding max, regardless of what chunk it is in. 
        //!found_data parameter addresses edge case where 0th log is not at 0th chunk. 
        if (!found_data || current_log_number > max_found_log_number) {
            // counter is total number of datapoints that have been written
            found_data = true;
            max_found_log_number = current_log_number;
            newest_chunk_idx = chunk;
        }
    }
    
    //case where chip is completely erased
    if (!found_data) {
        //means flash chip is totally erased, so start at zero
        *next_chunk_log_counter = 0;
        *next_chunk_idx = 0;
        return;
    }

    // modulo only ever has an effect when flash chip wants to write to very last index; it will then write to the zeroeth index instead
    *next_chunk_idx = (newest_chunk_idx + 1) % FLASH_NUM_CHUNKS;

    // all cases update next chunk counter in the same way, so just have this be a fallthrough for all
    *next_chunk_log_counter = max_found_log_number + 1;

    //after wraparound, next slot holds oldest log, so erase it
    if (!chunk_is_erased(*next_chunk_idx)) {
        uint32_t next_chunk_addr = chunk_idx_to_addr(*next_chunk_idx);
        erase_chunk(&next_chunk_addr);
        
        //printf("Erased chunk at index %" PRIu32 "\n", *next_chunk_idx);
    }
    return;
}

static bool flash_read_metadata(uint8_t chunk_idx, FlightLogger_Metadata_t* metadata) {
    uint8_t buf[sizeof(*metadata)];
    bool data_found = false;
    uint32_t chunk_start_addr = chunk_idx_to_addr(chunk_idx);
    flash_read_raw(&chunk_start_addr, sizeof(FlightLogger_Metadata_t), buf);
    
    // check if there is any data written to metadata section of current block
    for (size_t current_byte = 0; current_byte < sizeof(FlightLogger_Metadata_t); current_byte++) {
        // 0xFF is erased state for NOR flash
        if (buf[current_byte] != 0xFF) {
            data_found = true;
            break;
        }
    }
    if (data_found) {
        memcpy(metadata, buf, sizeof(*metadata));
    }
    //return true if there's data, false if there's not
    return data_found;
}

static bool flash_get_metadata_counter(uint8_t chunk_idx, uint32_t* counter) {
    FlightLogger_Metadata_t metadata;
    if (flash_read_metadata(chunk_idx, &metadata)) {
        *counter = metadata.log_counter;
        return true;
    }
    return false;
}

static uint32_t chunk_idx_to_addr(uint8_t chunk_idx) {
    return chunk_idx * FLASH_CHUNK_SIZE_64;
}

static void write_metadata(uint8_t new_chunk_idx, uint32_t new_chunk_counter, const PID_t* pitch_info, 
                                const PID_t* roll_info, const PID_t* yaw_info) {
    //idle and max are static to pid.c, so get a copy of their current values
    uint16_t motor_output_idle;
    uint16_t motor_output_max;
    pid_get_output_limits(&motor_output_idle, &motor_output_max);

    FlightLogger_Metadata_t metadata = {
                                        .log_counter = new_chunk_counter,
                                        .packet_version = PACKET_VERSION,
                                        .motor_output_idle = motor_output_idle,
                                        .motor_output_max = motor_output_max,
                                        .pitch_proportional_gain = pitch_info->kp, 
                                        .pitch_integrator_gain = pitch_info->ki, 
                                        .pitch_derivative_gain = pitch_info->kd,
                                        .roll_proportional_gain = roll_info->kp,
                                        .roll_integrator_gain = roll_info->ki,
                                        .roll_derivative_gain = roll_info->kd,
                                        .yaw_proportional_gain = yaw_info->kp,
                                        .yaw_integrator_gain = yaw_info->ki,
                                        .yaw_derivative_gain = yaw_info->kd
                                        };
                                        //todo: add crc calculation
    //&metadata is a pointer to metadata object. Cast tells compiler to treat that pointer as though it were pointing to
    //  a group of bytes instead
    uint32_t new_chunk_addr = chunk_idx_to_addr(new_chunk_idx);
    page_program(&new_chunk_addr, sizeof(metadata), (uint8_t*)&metadata);
    return; 
}

static bool chunk_is_erased(uint8_t chunk_idx) {
    FlightLogger_Metadata_t dummy;
    return !flash_read_metadata(chunk_idx, &dummy);
}

static void create_chunk_map_list(void) {
    // populate chunk_map_list
    for (int chunk = 0; chunk < FLASH_NUM_CHUNKS; chunk++) {
        //updates the value in the array if data found
        if (!flash_get_metadata_counter(chunk, &chunk_map_list[chunk])) {
            chunk_map_list[chunk] = FLASH_CHUNK_EMPTY;
        }
    }
}

const uint32_t* flash_get_chunk_map_list(void) {
    return chunk_map_list;
}

static uint8_t log_number_to_chunk_idx(uint32_t log_number) {
    
    for (int i = 0; i < FLASH_NUM_CHUNKS; i++) {
        if (chunk_map_list[i] == log_number) {
            return i;
        }
    }
    //return flash_chunk_empty if can't find the chunk
    //not good
    return UINT8_MAX;
}
//output is written to output parameter because copying 64 byte packet is not efficient
//checksum unused for now, but could be implemented in future
static void make_flightlog_packet(const Quadcopter_State_t* current_state, const IMU_Model_t* imu_data, 
                                                const float* pilot_setpoint, const uint16_t* synthesized_motor_commands, 
                                                                                    FlightLog_Packet_t* packet_output) {
                        //creates temporary flightlog_packet object, then copies it to output parameter.
                        //cleaner than having a bunch of individual assignents to packet_output object
    *packet_output = (FlightLog_Packet_t) {
                                      .current_state = *current_state,
                                      .entry_counter = datapoint_counter,
                                      .loop_dt_us = 1000000 / PID_LOOP_RATE_HZ,
                                      .batt_voltage_mV = 0,

                                      //unused for now since in rate mode...
                                      .pitch_angle = 0,
                                      .roll_angle = 0,

                                      .pitch_rate = imu_data->pitch_rate,
                                      .roll_rate = imu_data->roll_rate,
                                      .yaw_rate = imu_data->yaw_rate,

                                      .pilot_pitch_command_rate = pilot_setpoint[1],
                                      .pilot_roll_command_rate = pilot_setpoint[0],
                                      .pilot_yaw_command_rate = pilot_setpoint[3],
                                      .pilot_throttle_command = pilot_setpoint[2],

                                      .pid_pitch_out_pct = pid_out_pcts->pitch_pct,
                                      .pid_roll_out_pct = pid_out_pcts->roll_pct,
                                      .pid_yaw_out_pct = pid_out_pcts->yaw_pct,

                                      .m1_output_synthesized = synthesized_motor_commands[0],
                                      .m2_output_synthesized = synthesized_motor_commands[1],
                                      .m3_output_synthesized = synthesized_motor_commands[2],
                                      .m4_output_synthesized = synthesized_motor_commands[3],

                                      .crc = 0,
    };
                    }


bool flash_add_entry(const Quadcopter_State_t* current_state, const IMU_Model_t* imu_data, 
                    const float* pilot_setpoint, const uint16_t* synthesized_motor_commands) {
    //stop adding new entries if log is full
    if (datapoint_counter >= FLIGHTLOG_MAX_ENTRIES) {
        return false;
    }

    make_flightlog_packet(current_state, imu_data, pilot_setpoint, synthesized_motor_commands, 
                        &data_buffer[datapoint_counter % ENTRIES_PER_PAGE]);

    //entry counter is zero-indexed, so add 1. every four entries, write to flash
    if ((datapoint_counter + 1) % ENTRIES_PER_PAGE == 0) {
        //shift left since it's a 24 bit address
        uint32_t addr_to_write = (new_chunk_base_addr + (datapoint_counter / ENTRIES_PER_PAGE) * 
                                    FLASH_PAGE_SIZE_BYTES);
        page_program(&addr_to_write, FLASH_PAGE_SIZE_BYTES, (uint8_t*)data_buffer);
    }
    datapoint_counter++;
    return true;
}

// 16 chunks total; break into 4 x 4 grid
// read first chunk -> write its metadata number
void flash_print_memory_map(void) {
    printf("Flash Memory Map: (Numbers represent log number in each chunk)\n");

    for (int row = 0; row < 4; row++) {
        printf("| ");
        for (int col = 0; col < 4; col++) {
            uint32_t val = chunk_map_list[row * 4 + col];
            if (val == FLASH_CHUNK_EMPTY) {
                printf("%8s | ", "--");
            } else if (flash_log_is_empty(val)) {
                //log has metadata but no datapoints (e.g. currently open log)
                printf("%7" PRIu32 "* | ", val);
            } else {
                printf("%8" PRIu32 " | ", val);
            }
        }
        printf("\n");
    }
    printf("* = no flight data\n");
}

bool flash_read_datapoint(uint32_t log_number, uint16_t entry_number, FlightLog_Packet_t* data) {
    uint8_t idx = log_number_to_chunk_idx(log_number);
    //log not found; nothing to read
    if (idx == UINT8_MAX) {
        return false;
    }

    uint32_t addr = chunk_idx_to_addr(idx) + FLASH_METADATA_SIZE_BYTES + entry_number * sizeof(FlightLog_Packet_t);
    flash_read_raw(&addr, sizeof(FlightLog_Packet_t), (uint8_t*)data);
    //if reach erased memory, stop
    if (data->entry_counter == 0xFFFF) {
        return false;
    }
    return true;
}

bool flash_log_is_empty(uint32_t log_number) {
    FlightLog_Packet_t first_entry;
    return !flash_read_datapoint(log_number, 0, &first_entry);
}

void flash_get_metadata(uint32_t log_number, FlightLogger_Metadata_t* metadata) {
    uint8_t chunk_idx = log_number_to_chunk_idx(log_number);
    flash_read_metadata(chunk_idx, metadata);
}

bool flash_get_newest_metadata(FlightLogger_Metadata_t* metadata) {
    uint32_t max_log_number = 0;
    uint8_t newest_chunk_idx = 0;
    bool found_data = false;

    //can't use get_max_log_number here; it returns 0 both for "no logs" and "only log 0 exists"
    for (int i = 0; i < FLASH_NUM_CHUNKS; i++) {
        if (chunk_map_list[i] == FLASH_CHUNK_EMPTY) {
            continue;
        }
        if (!found_data || chunk_map_list[i] > max_log_number) {
            found_data = true;
            max_log_number = chunk_map_list[i];
            newest_chunk_idx = i;
        }
    }

    if (!found_data) {
        return false;
    }
    return flash_read_metadata(newest_chunk_idx, metadata);
}


uint32_t get_max_log_number(void) {
    uint32_t max = 0;
    bool found_data = false;
    
    for(int i = 0; i < FLASH_NUM_CHUNKS; i++) {
        if (chunk_map_list[i] == FLASH_CHUNK_EMPTY) {
            continue;
        }
        if (!found_data || chunk_map_list[i] > max) {
            found_data = true;
            max = chunk_map_list[i];
        }
    }
    return max;
}

uint32_t get_min_log_number(void) {
    uint32_t min = 0;
    bool found_data = false;
        for(int i = 0; i < FLASH_NUM_CHUNKS; i++) {
            //skip over empty chunk
            if (chunk_map_list[i] == FLASH_CHUNK_EMPTY) {
                continue;
            }
            if (!found_data || chunk_map_list[i] < min) {
                found_data = true;
                min = chunk_map_list[i];
            }
    }
    return min;
}