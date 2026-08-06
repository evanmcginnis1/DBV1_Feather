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

//change this value to limit max throttle
#define DSHOT_THROTTLE_RANGE 2000
//percents defined as being from 0 to 1000 to avoid excess floating point math
#define PCT_TO_DSHOT_RATIO ((DSHOT_THROTTLE_RANGE / 1000))
//dshot range is 48-2028, so need to offset
#define DSHOT_OFFSET 48

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
Requires: motor_throttles_pcts is an array of type uint16_t whose values are between 0 and 1000
Modifies: PWM timers' given CCR value to output bits in dshot format
Effects: converts percents into a value between 48 and 2048, then passes that value to dshot_write_raw
*/
void dshot_write_from_percents(const uint16_t* motor_throttles_pcts);
/*
Requires: Motor_throttles is an array of type uint16_t, where each value is between 0 and 2000
Modifies: All motor_dma_buf arrays
Effects:  Converts motor_throttles into DShot CCR values. Then, passes them into their respective TIM CCR registers & 
          starts a single DMA transaction
*/
void dshot_write_raw(const uint16_t* motor_throttles);

void dshot_disarm(void);

 #endif