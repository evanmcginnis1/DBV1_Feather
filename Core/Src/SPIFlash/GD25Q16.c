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
 * Requires: WIP flag in reset state (i.e. flash chip not busy)
 * Modifies: block on flash chip
 * Effects: sets CS pin low, sends erase command, sets CS pin high 
 * NOTE: does not wait for erase to complete
 */
static void erase_block(const uint32_t* block_addr);

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
 * Requires: CS pin is low already, status register lower byte read command has been sent
 * Modifies: WIP_flag_set flag
 * Effects: sets WIP_flag_set if WIP flag in status register is set. If WIP flag in register not set, resets WIP flag
 */
static void update_WIP_flag(bool* WIP_flag_set);

static void cs_pin_high(void);
static void cs_pin_low(void);

void page_program(const uint32_t* addr, const uint16_t size_bytes, const uint8_t* data) {
    wait_for_WIP_reset();
    write_enable();
    cs_pin_low();
    send_flash_command_and_addr_packet(COMMAND_PAGE_PROGRAM, addr);
    HAL_SPI_Transmit(FLASH_SPI, data, size_bytes, FLASH_SPI_TIMEOUT_MS);
    cs_pin_high();
    return;
}

void flash_read_raw(const uint32_t* start_addr, const uint16_t num_bytes_to_read, uint8_t* data) {
    
    wait_for_WIP_reset();

    cs_pin_low();
    send_flash_command_and_addr_packet(COMMAND_READ_DATA, start_addr);
    HAL_SPI_Receive(FLASH_SPI, data, num_bytes_to_read, FLASH_SPI_TIMEOUT_MS);
    cs_pin_high();
    return;
}

//block_addr is a 24-bit address
static void erase_block(const uint32_t* block_addr) {

    write_enable();
    cs_pin_low();
    send_flash_command_and_addr_packet(COMMAND_BLOCK_ERASE_64, block_addr);
    cs_pin_high();

    return;
}

void erase_chunk(const uint32_t* chunk_start_addr) {
    for (uint32_t block = 0; block < FLASH_BLOCKS_PER_CHUNK; block++) {
        uint32_t block_addr = *chunk_start_addr + block * FLASH_BLOCK_SIZE_64;

        wait_for_WIP_reset();
        erase_block(&block_addr);
    }
    wait_for_WIP_reset();
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



//status checking functions


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

