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
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>
#include <math.h>


static uint32_t motor_1_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_2_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_3_dma_buf[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor_4_dma_buf[DSHOT_DMA_BUFFER_SIZE];

// Static Functions
static uint16_t get_dshot_tick_freq_hz(dshot_type_e dshot_type);
static void dshot_set_timers(dshot_type_e dshot_type);
static uint16_t dshot_make_packet(const uint16_t* motor_command);
static void dshot_prepare_dma(uint32_t* motor_dma_buf, const uint16_t* motor_command);
static void dshot_prepare_dma_all(const uint16_t* motor_throttles);
static void dshot_start_dma(void);
static void dshot_start_pwm(void);


void dshot_init(dshot_type_e dshot_type) {
    dshot_set_timers(dshot_type);
    dshot_start_pwm();
}

void dshot_write(const uint16_t* motor_throttles) {
    dshot_prepare_dma_all(motor_throttles);
    dshot_start_dma();
}

//tick frequency = bit rate (same as baud rate) * tick length
static uint16_t get_dshot_tick_freq_hz(dshot_type_e dshot_type) {
    uint16_t dshot_tick_freq = (uint16_t)dshot_type * DSHOT_BIT_LENGTH;
    return dshot_tick_freq;
}

// update so that timer prescale variables are not specific to the timers; move specific timer clock info into a #define
static void dshot_set_timers(dshot_type_e dshot_type) {

    uint32_t tim3_clk = TIM3_CLK;
    uint32_t tim8_clk = TIM8_CLK;

    uint16_t dshot_tick_freq = get_dshot_tick_freq_hz(dshot_type);

    //consider adding 0.01 to lrintf operand for potential greater accuracy; doesn't seem necessary
    uint16_t dshot_prescaler_tim3 = lrintf(tim3_clk / dshot_tick_freq) - 1;
    uint16_t dshot_prescaler_tim8 = lrintf(tim8_clk / dshot_tick_freq) - 1; 

    //set Prescaler
    __HAL_TIM_SET_PRESCALER(MOTOR_1_TIM, dshot_prescaler_tim3);
    __HAL_TIM_SET_PRESCALER(MOTOR_2_TIM, dshot_prescaler_tim3);
    __HAL_TIM_SET_PRESCALER(MOTOR_3_TIM, dshot_prescaler_tim8);
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
        packet <<= 1;
    }

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
    HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor_1_dma_buf, (uint32_t) MOTOR_1_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_2_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t) motor_2_dma_buf, (uint32_t) MOTOR_2_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_3_TIM->hdma[TIM_DMA_ID_CC1], (uint32_t) motor_3_dma_buf, (uint32_t) MOTOR_3_TIM->Instance->CCR1, DSHOT_DMA_BUFFER_SIZE);
    HAL_DMA_Start_IT(MOTOR_4_TIM->hdma[TIM_DMA_ID_CC2], (uint32_t) motor_4_dma_buf, (uint32_t) MOTOR_4_TIM->Instance->CCR2, DSHOT_DMA_BUFFER_SIZE);
}