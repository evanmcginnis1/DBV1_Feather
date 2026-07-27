/*
 * DShot.h
 *
 * Created on: July 16, 2026
 * Author: Evan McGinnis
 * 
 * Implementation note: MUST check box in STM32CubeMX->Project Manager->Code Generator->Generated Files for "generate
   peripheral initialization as a pair of .c and .h files." That way, timer objects are created as extern, so they can
   be used here without having to pass them to every function
 */

 #ifndef DSHOT_H
 #define DSHOT_H

#include "stm32f4xx_hal.h"
#include "tim.h"

// TIM1 is on slower APB1 bus, while TIM8 is on faster APB2 bus.
#define TIM3_CLK 84000000
#define TIM8_CLK 168000000

#define MOTOR_1_TIM  (&htim3)
#define MOTOR_1_TIM_CHANNEL TIM_CHANNEL_1
#define MOTOR_1_DMA()

#define MOTOR_2_TIM (&htim3)
#define MOTOR_2_TIM_CHANNEL TIM_CHANNEL_2

#define MOTOR_3_TIM (&htim8)
#define MOTOR_3_TIM_CHANNEL TIM_CHANNEL_1

#define MOTOR_4_TIM (&htim8)
#define MOTOR_4_TIM_CHANNEL TIM_CHANNEL_2

#define DSHOT_BIT_LENGTH 140 //ticks per bit
#define DSHOT_T1H_TICKS 105
#define DSHOT_T0H_TICKS 53

#define DSHOT_DMA_BUFFER_SIZE 18

#define MHZ_TO_HZ(MHZ) ((MHZ) * 1000000)

// enum is based on baud rate of each type of DShot signal.
typedef enum {
    DSHOT150 = 150,
    DSHOT300 = 300,
    DSHOT600 = 600,
} dshot_type_e;

/*
Requires: DShot_type is a member of dshot_type_e. APB1 Timer clock is set to 84MHz, APB2 Timer clock is set to 168MHz. 
          Motors are each connected to a unique PWM output channel of a TIM. TIM & channel selection for each motor is
          defined in DShot.h. Motors 1 & 2 connected to TIM3 Channel 1 and 2, respectively. Motors 3 and 4 connected to 
          TIM8 channels 1 and 2, respectively.
Modifies: TIM1 and TIM3 prescaler value. Enables PWM on TIM3 ch. 1 & 2, and on TIM8 ch. 1 & 2. 
Effects: Initializes DMA & timer settings, sets prescaler & ARR so that timings are correct based on chosen dshot protocol
*/
void dshot_init(dshot_type_e dshot_type);

/*
Requires: Motor_throttles is an array of type uint16_t, where each value is between 0 and 2000
Modifies: All motor_dma_buf arrays
Effects:  Converts motor_throttles into DShot CCR values. Then, passes them into their respective TIM CCR registers & 
          starts a single DMA transaction
*/
void dshot_write(const uint16_t* motor_throttles);

//Rest of functions are static in DShot.c
/*
Requires: dshot_type is a valid member of dshot_type_e
Modifies: Nothing
Effects: Returns the tick frequency for a given DShot type
*/
//static uint16_t get_dshot_tick_freq_hz(dshot_type_e dshot_type);

/*
Requires: dshot_type is a valid member of dshot_type_e
Modifies: Prescaler and ARR registers
Effects:  Sets prescaler and ARR for each pwm channel used so that timings will be correct for chosen DShot speed
*/
//static void dshot_set_timers(dshot_type_e dshot_type);
//static void dshot_start_pwm(void);
//static uint16_t dshot_make_packet(const uint16_t* motor_command);
//static void dshot_prepare_dma(uint32_t* motor_dma_buf, const uint16_t* motor_command);
//static void dshot_prepare_dma_all(const uint16_t* motor_throttles);
//static void dshot_start_dma(void);




 #endif