/*
 * DShot.c
 *
 * Created on: July 16, 2026
 * Author: Evan McGinnis
 * 
 * This file includes code partially derived from: 
 *  stm32_hal_dshot
 *  Copyright (c) 2023 Eunhye Seok 
 *  Licensed under The MIT License
 *  Source: https://github.com/mokhwasomssi/stm32_hal_dshot
 */

#include "DShot.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>
#include <math.h>


static uint32_t motor_1_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_2_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_3_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_4_dma_buf[DSHOT_DMA_BUFFER_SIZE];

// Static Functions
/*
 * Requires: dshot_type is a valid member of dshot_type_e
 * Modifies: Nothing
 * Effects: Converts from dshot bitrate into tick frequency, then returns the result
*/
static uint32_t get_dshot_tick_freq_hz(dshot_type_e dshot_type);

/*
 * Requires: dshot_type is a valid member of dshot_type_e. Timers used for dshot have already been initialized. 
 * Modifies: Prescaler and ARR values for chosen timers
 * Effects: Calculates prescaler value, sets prescaler and ARR value using STM32 HAL
 */
static void dshot_set_timers(dshot_type_e dshot_type);

/*
 * Requires: motor_command is an array of the actual throttle values (range from 48-2048) to send to the ESC
 * Modifies: Nothing
 * Effects: Sets telemetry bit to 0 (disabled), calculates DShot checksum & appends it to end of packet
 */
static uint16_t dshot_make_packet(const uint16_t* motor_command);
/*
 * Requires: motor_command is a complete DShot frame (in binary format, not converted to DShot protocol yet)
 * Modifies: motor_dma_buf
 * Effects: Converts packet into an array of capture/compare values, with each member representing one byte. Sets last 
            two members of array to zero, so that the following ARR cycles have zero output while waiting for next command.
 */
static void dshot_prepare_dma(uint32_t* motor_dma_buf, const uint16_t* motor_command);
/*
 * Requires: motor_throttles are either 0 (disarmed, zero motor spin command) or a value between 48 and 2048
 * Modifies: All four motor buffers
 * Effects:  Calls dshot_prepare_dma for each motor
 */
static void dshot_prepare_dma_all(const uint16_t* motor_throttles);
/*
 * Requires: Timers and DMA have all been initialized. 
 * Modifies: DMA bus
 * Effects: Enables DMA transaction with interrupt when complete (does not actually start transaction)
 */
static void dshot_start_dma(void);

/*
 * Requires: Timer(s) used for DShot signal generation have been initialized. CCR is set to zero in CubeMX
             (to prevent any signal being actually sent)
 * Modifies: TIMx CCx register
 * Effects: Starts PWM on TIMx
 */
static void dshot_start_pwm(void);

/*
 * Requires: DMA & Timers have been initialized
 * Modifies: TIMx channel x DMA register
 * Effects: Makes timer start responding to DMA requests
 */
static void dshot_enable_dma_request(void);

/*
 * Requires: dshot_dma_tc_callback is a valid function
 * Modifies: TIM XferCpltCallback register
 * Effects: points XferCpltCallback member of TIMx to callback function
 */
static void dshot_put_tc_callback_function(void);

/*
 * Requires: hdma is a member of a TIM object on an index corresponding to a channel between 1 and 4
 * Modifies: DMA bus
 * Effects: Disables DMA transactions for the given timer channel
 */

static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma);

void dshot_write_from_percents(const uint16_t* motor_throttles_pcts) {
    uint16_t motor_throttles_raw[4];

    for (int i = 0; i < 4; i++) {
        motor_throttles_raw[i] = ((motor_throttles_pcts[i] * PCT_TO_DSHOT_RATIO) + DSHOT_OFFSET);
    }
    dshot_write_raw(motor_throttles_raw);
}

void dshot_init(dshot_type_e dshot_type) {
    dshot_set_timers(dshot_type);
    dshot_put_tc_callback_function();
    dshot_start_pwm();
}

// test if prescaler settings are correct
void dshot_write_raw(const uint16_t* motor_throttles) {
    dshot_prepare_dma_all(motor_throttles);
    dshot_start_dma();
    dshot_enable_dma_request();
}

void dshot_disarm(void) {
    uint16_t throttles[4] = {0};
    dshot_write_raw(throttles);
}

//tick frequency = bit rate (same as baud rate) * tick length
static uint32_t get_dshot_tick_freq_hz(dshot_type_e dshot_type) {
    uint32_t dshot_bitrate = (uint32_t)dshot_type * 1000;
    uint32_t dshot_tick_freq = (uint32_t)dshot_bitrate * DSHOT_BIT_LENGTH;
    return dshot_tick_freq;
}

// update so that timer prescale variables are not specific to the timers; move specific timer clock info into a #define
static void dshot_set_timers(dshot_type_e dshot_type) {

    // easier to use defined clock frequency instead of reading from stm32 clock register since timers are on different 
    // buses. 
    uint32_t tim3_clk = TIM3_CLK;
    uint32_t tim8_clk = TIM8_CLK;

    uint32_t dshot_tick_freq_hz = get_dshot_tick_freq_hz(dshot_type);

    //consider adding 0.01 to lrintf operand for potential greater accuracy; doesn't seem necessary
    //prescaler register is 0-indexed, so need to subtract 1 for accurate prescaler value
    uint16_t dshot_prescaler_tim3 = lrintf((float) tim3_clk / dshot_tick_freq_hz) - 1;
    uint16_t dshot_prescaler_tim8 = lrintf((float) tim8_clk / dshot_tick_freq_hz) - 1; 

    //set Prescaler
    __HAL_TIM_SET_PRESCALER(MOTOR_1_TIM, dshot_prescaler_tim3);
    __HAL_TIM_SET_PRESCALER(MOTOR_2_TIM, dshot_prescaler_tim8);
    __HAL_TIM_SET_PRESCALER(MOTOR_3_TIM, dshot_prescaler_tim3);
    __HAL_TIM_SET_PRESCALER(MOTOR_4_TIM, dshot_prescaler_tim8);

    //set ARR
    __HAL_TIM_SET_AUTORELOAD(MOTOR_1_TIM, DSHOT_BIT_LENGTH);
    __HAL_TIM_SET_AUTORELOAD(MOTOR_2_TIM, DSHOT_BIT_LENGTH);
    __HAL_TIM_SET_AUTORELOAD(MOTOR_3_TIM, DSHOT_BIT_LENGTH);
    __HAL_TIM_SET_AUTORELOAD(MOTOR_4_TIM, DSHOT_BIT_LENGTH);
}

static void dshot_start_pwm(void) {
    HAL_TIM_PWM_Start(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_4_TIM, MOTOR_4_TIM_CHANNEL);
}

// Add no-telemetry bit to throttle data
// calculate crc and append to throttle data.
static uint16_t dshot_make_packet(const uint16_t* motor_command) {
    // Clear twelfth bit to represent no-telemetry signal
    uint16_t packet = (*motor_command << 1);
    
    uint16_t crc = (packet ^ (packet >> 4) ^ packet >> 8) & 0x0F;

    packet = (packet << 4) | crc;

    return packet;
}

//go through each bit of dshot packet
static void dshot_prepare_dma(uint32_t* motor_dma_buf, const uint16_t* motor_command) {

    uint16_t packet = dshot_make_packet(motor_command);

    // not cleanest, but proud of coming up with this myself so keep it
    for (int i = 0; i < 16; i++) {
        motor_dma_buf[15 - i] = (packet & (1 << i)) ? (DSHOT_T1H_TICKS) : DSHOT_T0H_TICKS;
        //packet <<= 1;
    }
        motor_dma_buf[16] = 0;
        motor_dma_buf[17] = 0;

    // stm32_hal_dshot repo sets 17th and 18th items in array to zero to add a delay; skip because my pid loop will slow
    // dshot down enough that manually inserting a delay is pointless
}

static void dshot_prepare_dma_all(const uint16_t* motor_throttles) {
    dshot_prepare_dma(motor_1_dma_buf, &motor_throttles[0]);
    dshot_prepare_dma(motor_2_dma_buf, &motor_throttles[1]);
    dshot_prepare_dma(motor_3_dma_buf, &motor_throttles[2]);
    dshot_prepare_dma(motor_4_dma_buf, &motor_throttles[3]);
}

static void dshot_start_dma(void) {
    HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t) motor_1_dma_buf, (uint32_t) &MOTOR_1_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_2_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor_2_dma_buf, (uint32_t) &MOTOR_2_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_3_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor_3_dma_buf, (uint32_t) &MOTOR_3_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_4_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t) motor_4_dma_buf, (uint32_t) &MOTOR_4_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
}


static void dshot_enable_dma_request(void) {
	__HAL_TIM_ENABLE_DMA(MOTOR_1_TIM, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(MOTOR_2_TIM, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(MOTOR_3_TIM, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(MOTOR_4_TIM, TIM_DMA_CC2);
}


static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}


static void dshot_put_tc_callback_function(void) {
	// TIM_DMA_ID_CCx depends on timer channel
	MOTOR_1_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_2_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_3_TIM->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_4_TIM->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshot_dma_tc_callback;
}
