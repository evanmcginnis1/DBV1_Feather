/*
 * GD25Q16.h
 *
 * Created on: August 20, 2026
 * Author: Evan McGinnis
 * 
 * 
 */

 #ifndef GD25Q16_H
 #define GD25Q16_H

#include <stdint.h>
#include <stdbool.h>
#include "SPI.h"
#include "pid.h"
#include "FlightLogger_Model.h"


//hardware definitions
#define FLASH_CS_PORT GPIOA
#define FLASH_CS_PIN_NUMBER GPIO_PIN_15
#define FLASH_SPI (&hspi1)


//Misc defines
//flash chip has either 32 sector or 64 sector blocks
#define FLASH_BLOCK_SIZE_64 0x010000
#define FLASH_CHUNK_SIZE_64 (2 * FLASH_BLOCK_SIZE_64)
#define FLASH_PAGE_SIZE_BYTES 256

//#define FLASH_METADATA_CUTOUT_BYTES (1 * FLASH_PAGE_SIZE_BYTES)
#define FLASH_CHUNK_EMPTY UINT32_MAX



#define FLASH_SPI_TIMEOUT_MS 2
//each chunk is two 64k byte blocks
#define FLASH_NUM_CHUNKS 16
// number of 64k byte blocks on flash chip
#define FLASH_NUM_BLOCKS_64 32
#define FLASH_COMMAND_SIZE 1
#define FLASH_STATUS_LOWER_SIZE 1


//commands
#define COMMAND_STATUS_REGISTER_READ_UPPER 0x35
#define COMMAND_STATUS_REGISTER_READ_LOWER 0x05
#define COMMAND_WRITE_ENABLE 0x06
#define COMMAND_BLOCK_ERASE_64 0xD8
#define COMMAND_PAGE_PROGRAM 0x02
#define COMMAND_READ_DATA 0x03

/*
 * Requires: Flash chip has booted up
 * Modifies: next_block_addr, next_block_counter
 * Effects: Searches through flash memory metadata to find latest entry, then updates 
 *          next_chunk_addr to one block after that and next_chunk_counter to one more than the previous counter
 */
void find_next_chunk(uint32_t* next_chunk_addr, uint32_t* next_chunk_counter);


/*
 * Requires: Page has been erased already. data MUST be an array of 256 bytes
 * Modifies: Page on flash chip that addr points to
 * Effects: Writes data to selected page (intended to write a full page worth of data). 
 */
void page_program(const uint32_t* addr, const uint16_t size, const uint8_t* data);

/*
 * Requires: 
 * Modifies: 
 * Effects: updates data with the data that is read from flash
 */
void flash_read(const uint32_t* start_addr, const uint16_t num_bytes_to_read, uint8_t* data);

/*
 * Requires: Nothing
 * Modifies: Nothing
 * Effects: Reads metadata. Returns true if data found and updates metadata object. Otherwise, 
            returns false if chunk is empty or invalid start addr given
 */
bool flash_read_metadata(const uint32_t* block_start_addr, FlightLogger_Metadata_t* metadata);

/*
 * Requires: Flash chip has booted up. Target is an even numbered block
 * Modifies: Updates counter value based on metadata in block
 * Effects: Reads first four bytes of block for counter value. Returns true and updates counter if block has metadata. 
            returns false if block is completely erased(has no metadata) (Careful: could also be caused by alignment issue). 
 */
bool flash_get_metadata_counter(uint8_t chunk_number, uint32_t* counter);


/*
 * Requires: new_block_addr is the address of the block to write current log to, points to chunk of two erased blocks
             WIP flag is cleared
 * Modifies: first page of new block, CS pin, SPI bus
 * Effects: Writes metadata on first page of new chunk of blocks used for flight log. Only writes to first page of first 
 *          block, not second block. 
 */
void write_metadata(const uint32_t* new_chunk_addr, const uint32_t* new_chunk_counter, const PID_t* pitch_info, 
                                const PID_t* roll_info, const PID_t* yaw_info);


void test_flash_functions(void);

/*
 * Requires: chunk_map_list is an array with size FLASH_NUM_CHUNKS, WIP flag not set
 * Modifies: chunk_map_list, flash read
 * Effects: each index in chunk_map_list represents a chunk. Writes -1 to index if no data found. If data found, writes 
            its metadata number at the given index
 */
void get_chunk_map_list(uint32_t* chunk_map_list);

uint32_t log_number_to_chunk_addr(uint32_t log_number);
 #endif