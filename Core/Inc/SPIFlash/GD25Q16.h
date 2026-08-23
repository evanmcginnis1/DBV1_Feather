/*
 * GD25Q16.h
 *
 * Created on: August 20, 2026
 * Author: Evan McGinnis
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
#define FLASH_METADATA_SIZE_PAGES 1

#define FLASH_METADATA_SIZE_BYTES (FLASH_METADATA_SIZE_PAGES * FLASH_PAGE_SIZE_BYTES)
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
 * Requires: Flash chip 
 * Modifies: current_page_addr and current_block_addr
 * Effects: Adds 5ms delay for flash chip power up time, finds chunk of two blocks to write to
 */
void flash_init(PID_t* pitch_info, PID_t* roll_info, PID_t* yaw_info);

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

void test_flash_functions(void);


 #endif