/*
 * GD25Q16.c
 *
 * Created on: August 20, 2026
 * Author: Evan McGinnis
 * 
 */

 #include "GD25Q16.h"
 #include <stdbool.h>

#include "GD25Q16.h"
#include "FlightLogger_Model.h"
#include "pid.h"
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal_spi.h"
#include <inttypes.h>

 
// Implementation note: Only call wait_for_WIP_reset when a new write/erase cycle actually needs to be performed. 
// Calling it too early (i.e. inside the write/erase function itself, before releasing) is pointless and prevents CPU doing other things

//static function declarations



/* 
 * Requires: WEL bit has been set
 * Modifies: chunk that address points to
 * Effects: Erases chunk (set of 2 consecutive blocks)
 */
static void erase_chunk(const uint32_t* chunk_start_addr);

/*
 * Requires: WIP flag in reset state (i.e. flash chip not busy)
 * Modifies: block on flash chip
 * Effects: sets CS pin low, sends erase command, sets CS pin high 
 * NOTE: does not wait for erase to complete
 */
static void erase_block(const uint32_t* block_addr);

/*
 * Requires: Flash chip has booted up. Target is an even numbered block
 * Modifies: Updates counter value based on metadata in block
 * Effects: Reads first four bytes of block for counter value. Returns true and updates counter if block has metadata. 
            returns false if block is completely erased(has no metadata) (Careful: could also be caused by alignment issue). 
 */
static bool get_metadata_counter(const uint32_t* block_start_addr, uint32_t* counter);

/*
 * Requires: Nothing
 * Modifies: Nothing
 * Effects: Reads metadata. Returns true if data found and updates metadata object. Otherwise, 
            returns false if block is empty
 */
static bool read_metadata(const uint32_t* block_start_addr, FlightLogger_Metadata_t* metadata);

//helper functions
/*
 * Requires: flash chip is initialized
 * Modifies: Status register on flash chip
 * Effects: Sets write enable bit
 */
static void write_enable(void);


/*
 * Requires: command is a valid spiflash command defined in GD25Q16.h
 * Modifies: SPIFlash chip
 * Effects: sends command to flash chip
 * Note: Does not set CS pin since some commands like continuous read require cs pin to remain low afterwards
 */
static void send_flash_command_only(uint8_t command);

/*
 * Requires: command is a one-bit valid flash command, addr is the 24-byte address that the command will target
 * Modifies: SI bus
 * Effects: concacetates command and addr to make a full command packet, then sends that packet to flash chip
 * NOTE: Does not set/reset CS pin (allows for continuous read if necessary)
 */
static void send_flash_command_and_addr_packet(uint8_t command, const uint32_t* addr);

/*
 * Requires: flash chip initialized
 * Modifies: Nothing
 * Effects: Continuously checks WIP flag and blocks until it is reset by flash chip
 * (use to check if write/erase is still in progress)
 */
static void wait_for_WIP_reset(void);

/*
 * Requires: Nothing
 * Modifies: CS pin
 * Effects: Returns true if block has not been written to, otherwise returns false
 */
static bool chunk_is_erased(const uint32_t* chunk_addr);

/*
 * Requires: CS pin is low already, status register lower byte read command has been sent
 * Modifies: WIP_flag_set flag
 * Effects: sets WIP_flag_set if WIP flag in status register is set. If WIP flag in register not set, resets WIP flag
 */
static void update_WIP_flag(bool* WIP_flag_set);

static void cs_pin_high(void);
static void cs_pin_low(void);



static void test_find_next_chunk(uint32_t* next_chunk_addr, uint32_t* next_chunk_counter);
static void test_write_header(uint32_t* next_chunk_addr, uint32_t* next_chunk_counter);



static void test_write_header(uint32_t* next_chunk_addr, uint32_t* next_chunk_counter) {
    PID_t pitch_info = {5};
    PID_t roll_info = {10.1};
    PID_t yaw_info = {15.5};
    FlightLogger_Metadata_t metadata;
    printf("Testing SPIFlash function write_header():\n");
    wait_for_WIP_reset();
    write_metadata(next_chunk_addr, next_chunk_counter, &pitch_info, &roll_info, &yaw_info);
    if (!read_metadata(next_chunk_addr, &metadata)) {
        printf("Metadata not found at %" PRIu32 "\n", *next_chunk_addr);
    } else {
        if (metadata.chunk_counter == *next_chunk_counter) {
            printf("Metadata found and counter matches!\n");
            printf("Pid pitch: %3f , roll: %3f, yaw: %3f \n", metadata.pitch_proportional_gain, 
                    metadata.roll_proportional_gain, metadata.yaw_proportional_gain);
        } else {
            printf("Metadata corrupted...\n");
        }
    }

    return;
}


void test_flash_functions(void) {
    uint32_t next_chunk_addr;
    uint32_t next_chunk_counter;
    test_find_next_chunk(&next_chunk_addr, &next_chunk_counter);
    printf("\n");
    test_write_header(&next_chunk_addr, &next_chunk_counter);
    printf("\n");
}

static void test_find_next_chunk(uint32_t* next_chunk_addr, uint32_t* next_chunk_counter) {
    printf("Testing SPIFlash function find_next_chunk()\n");
    wait_for_WIP_reset();
    find_next_chunk(next_chunk_addr, next_chunk_counter);
    printf("Found next chunk at address: %" PRIu32 "\n", *next_chunk_addr);
    printf("Next counter value: %" PRIu32 "\n", *next_chunk_counter);
}



void page_program(const uint32_t* addr, const uint16_t size_bytes, const uint8_t* data) {
    cs_pin_low();
    wait_for_WIP_reset();
    send_flash_command_and_addr_packet(COMMAND_PAGE_PROGRAM, addr);
    HAL_SPI_Transmit(FLASH_SPI, data, size_bytes, FLASH_SPI_TIMEOUT_MS);
    cs_pin_high();
    return;
}

void read_flash_data(const uint32_t* start_addr, const uint16_t num_bytes_to_read, uint8_t* data) {
    
    wait_for_WIP_reset();

    cs_pin_low();
    send_flash_command_and_addr_packet(COMMAND_READ_DATA, start_addr);
    HAL_SPI_Receive(FLASH_SPI, data, num_bytes_to_read, FLASH_SPI_TIMEOUT_MS);
    cs_pin_high();
    return;
}

void find_next_chunk(uint32_t* next_chunk_base_addr, uint32_t* next_chunk_counter) {

    //counter is total number of datapoints that have been written
    uint32_t newest_counter = 0;
    //index is just a number that loops from 0 to FLASH_NUM_CHUNKS (idx = newest_counter % FLASH_NUM_CHUNKS)
    uint32_t newest_chunk_idx = 0;
    bool found_data = false;


    //check all chunks even if first one is empty just in case
    for (int chunk = 0; chunk < FLASH_NUM_CHUNKS; chunk += 1) {
        uint32_t counter;
        uint32_t current_chunk_start_addr = FLASH_CHUNK_SIZE_64 * chunk;
        bool current_chunk_is_erased;

        //clean up
        current_chunk_is_erased = !get_metadata_counter(&current_chunk_start_addr, &counter);

        // update counter from current block and check if the current block is erased or not
        if (current_chunk_is_erased) {
            //found free block; ;
            continue;
        } 
        // if haven't found any data yet (0th index case) or found a newer datapoint, set the focus on the new chunk
        if (!found_data || counter > newest_counter) {
            found_data = true;
            // counter is total number of datapoints that have been written
            newest_counter = counter;
            newest_chunk_idx = chunk;
        }
    }
    
    //case where chip is completely erased
    if (!found_data) {
        //means flash chip is totally erased, so start at zero
        *next_chunk_counter = 0;
        *next_chunk_base_addr = 0x00;
        return;
    }

    // modulo only ever has an effect when flash chip wants to write to very last index; it will then write to the zeroeth index instead
    uint32_t next_chunk_idx = (newest_chunk_idx + 1) % FLASH_NUM_CHUNKS;

    *next_chunk_base_addr = next_chunk_idx * FLASH_CHUNK_SIZE_64;
    // all cases update next chunk counter in the same way, so just have this be a fallthrough for all
    *next_chunk_counter = newest_counter + 1;

    // once find a non-erased chunk, all of the following chunks will be also not erased, so the flag stays valid
    if (!chunk_is_erased(next_chunk_base_addr)) {
        erase_chunk(next_chunk_base_addr);
        printf("Erased chunk at address %" PRIu32 "\n", *next_chunk_base_addr);
    }
    return;
}


static bool read_metadata(const uint32_t* block_start_addr, FlightLogger_Metadata_t* metadata) {
    uint8_t buf[sizeof(*metadata)];
    bool data_found = false;
    read_flash_data(block_start_addr, sizeof(FlightLogger_Metadata_t), buf);
    
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

static bool get_metadata_counter(const uint32_t* block_start_addr, uint32_t* counter) {
    FlightLogger_Metadata_t metadata;
    if (read_metadata(block_start_addr, &metadata)) {
        *counter = metadata.chunk_counter;
        return true;
    }
    return false;

}

void write_metadata(const uint32_t* new_chunk_addr, const uint32_t* new_chunk_counter, const PID_t* pitch_info, 
                                const PID_t* roll_info, const PID_t* yaw_info) {
    FlightLogger_Metadata_t metadata = {
                                        .chunk_counter = *new_chunk_counter, 
                                        .packet_version = PACKET_VERSION, 
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
    write_enable();
    //&metadata is a pointer to metadata object. Cast tells compiler to treat that pointer as though it were pointing to
    //  a group of bytes instead
    page_program(new_chunk_addr, sizeof(metadata), (uint8_t*)&metadata);
    return; 
}

static void erase_chunk(const uint32_t* chunk_start_addr){
    uint32_t block_1 = *chunk_start_addr;
    uint32_t block_2 = *chunk_start_addr + FLASH_BLOCK_SIZE_64;

    wait_for_WIP_reset();
    erase_block(&block_1);

    wait_for_WIP_reset();
    erase_block(&block_2);

    return;
}


static void wait_for_WIP_reset(void) {
    bool WIP_flag_set;
    cs_pin_low();
    send_flash_command_only(COMMAND_STATUS_REGISTER_READ_LOWER);
    //use do while loop because want to check flag at least once everytime this function is called

    do {
        update_WIP_flag(&WIP_flag_set);
    } while (WIP_flag_set);
    cs_pin_high();
}

//could make this boolean and check if WEL actually set.. (but takes more time)
static void write_enable(void) {
    cs_pin_low();
    send_flash_command_only(COMMAND_WRITE_ENABLE);
    cs_pin_high();
    return;
}

//not efficient for read commands (besides status); better to combine command and address into one packet
static void send_flash_command_only(uint8_t command) {
    HAL_SPI_Transmit(FLASH_SPI, &command, FLASH_COMMAND_SIZE, FLASH_SPI_TIMEOUT_MS);
}

static void send_flash_command_and_addr_packet(uint8_t command, const uint32_t* addr) {
    uint8_t packet_output[4];
    packet_output[0] = command;
    packet_output[1] = (uint8_t)(*addr >> 16);
    packet_output[2] = (uint8_t)(*addr >> 8);
    packet_output[3] = (uint8_t)(*addr);
    HAL_SPI_Transmit(FLASH_SPI, packet_output, 4, FLASH_SPI_TIMEOUT_MS);
}

//block_addr is a 24-bit address
static void erase_block(const uint32_t* block_addr) {

    write_enable();
    cs_pin_low();
    send_flash_command_and_addr_packet(COMMAND_BLOCK_ERASE_64, block_addr);
    cs_pin_high();

    return;
}

//status checking functions
static bool chunk_is_erased(const uint32_t* chunk_addr) {
    FlightLogger_Metadata_t dummy;
    return !read_metadata(chunk_addr, &dummy);
}

static void update_WIP_flag(bool* WIP_flag_set) {
    uint8_t status_lower_byte;

    //use blocking SPI transaction since only sending one byte?
    HAL_SPI_Receive(FLASH_SPI, &status_lower_byte, FLASH_STATUS_LOWER_SIZE, FLASH_SPI_TIMEOUT_MS);
    // 0x00 is reset state, 0x01 is set state
    *WIP_flag_set = (status_lower_byte & 0x01) == 0x01;

    return;
}


static void cs_pin_low(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void cs_pin_high(void) {
    HAL_GPIO_WritePin(FLASH_CS_PORT, FLASH_CS_PIN_NUMBER, GPIO_PIN_SET);
}

