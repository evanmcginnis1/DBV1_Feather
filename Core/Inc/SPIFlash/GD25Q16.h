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
 * Requires: Page has been erased already. data MUST be an array of 256 bytes
 * Modifies: Page on flash chip that addr points to
 * Effects: Writes data to selected page (intended to write a full page worth of data). 
 */
void page_program(const uint32_t* addr, const uint16_t size, const uint8_t* data);

/* 
 * Requires: 
 * Modifies: chunk that address points to
 * Effects: Erases chunk (set of 2 consecutive blocks)
 */
void erase_chunk(const uint32_t* chunk_start_addr);


/*
 * Requires: 
 * Modifies: 
 * Effects: updates data with the data that is read from flash
 */
void flash_read_raw(const uint32_t* start_addr, const uint16_t num_bytes_to_read, uint8_t* data);


 #endif