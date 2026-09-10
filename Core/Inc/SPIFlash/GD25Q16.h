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

//hardware definitions
#define FLASH_CS_PORT GPIOA
#define FLASH_CS_PIN_NUMBER GPIO_PIN_15
#define FLASH_SPI (&hspi1)


//Misc defines
//flash chip has either 32 sector or 63 sector blocks
#define FLASH_BLOCK_SIZE_64 0x010000
#define FLASH_CHUNK_SIZE_64 (2 * FLASH_BLOCK_SIZE_64)
#define FLASH_PAGE_SIZE_BYTES 256

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
 * Requires: WEL bit has been set. Page has been erased already. WIP bit is not set. data MUST be an array of 256 bytes
 * Modifies: Page on flash chip that addr points to
 * Effects: Writes data to selected page (intended to write a full page worth of data). 
 */
void page_program(const uint32_t* addr, const uint16_t size, const uint8_t* data);

/*
 * Requires: 
 * Modifies: 
 * Effects: updates data with the data that is read from flash
 */
void read_flash_data(const uint32_t* start_addr, const uint16_t num_bytes_to_read, uint8_t* data);

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


 #endif