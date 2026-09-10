/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU_Model.h"
#include "IMU_Conductor.h"
#include "DShot.h"
#include "iBus.h"
#include "pid.h"
#include "state.h"
#include "USB_Handler.h"
#include "FlightLogger_Model.h"
#include "GD25Q16.h"
#include <math.h>
#include <usbd_cdc_if.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile bool loop_ready_flag = 0;

//USB virtual COM variables

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_loop_rate(uint32_t loop_rate_hz) {
  uint32_t tim_clk = MAIN_LOOP_TIM_CLK;

  uint16_t tim_PSC = lrintf((float) tim_clk / LOOP_TIM_TICK_RATE_HZ) - 1;
  uint32_t tim_ARR =  LOOP_TIM_TICK_RATE_HZ / loop_rate_hz;

  __HAL_TIM_SET_PRESCALER(MAIN_LOOP_TIM, tim_PSC);
  __HAL_TIM_SET_AUTORELOAD(MAIN_LOOP_TIM, tim_ARR);
}

//overwrite weak function so that printf works with serial port
int _write(int file, char *ptr, int len) {
  while (CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY) {
        HAL_Delay(1);
    }
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// for ibus software failsafe
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//for timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  IMU_Model_t imu_model;
  //write all ones to ibus_data array so that 
  uint16_t ibus_data_pcts[IBUS_NUM_CHANNELS];
  uint16_t esc_commands_pcts[4] = {0};
  Quadcopter_State_t state = SOFT_DISARM;
  bool disarm_locked = false;
  User_USB_Commands_t user_command;
  float setpoint_output[NUM_MOTORS] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


  set_loop_rate(100);
  /*
  dshot_init(DSHOT300);
  ibus_init(IBUS_UART);
  IMU_init(&hi2c1);
  pid_init();
  */
  while(1) {
    test_flash_functions();
    HAL_Delay(10000);
  }
  //need to start timer explicitly to run interrupt-based main loop 
  HAL_TIM_Base_Start_IT(MAIN_LOOP_TIM);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (loop_ready_flag) {
      loop_ready_flag = 0;
      //IMU_update_model(&imu_model);
      //ibus_read_as_percents(ibus_data);

      //update_state(ibus_data, &imu_model, &disarm_locked, &state);

      switch(state) {
        case ARMED: 
            if (ibus_failsafe_check(ibus_data_pcts)) {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
              pid_update(&imu_model, ibus_data_pcts, esc_commands_pcts);
              dshot_write_from_percents(esc_commands_pcts);
            }
          break;

        case HARD_DISARM:
          // if just waiting for user to flip arm switch to disarmed, don't enforce 10s delay or check serial input
          if (!disarm_locked) {
            break;
          }
          user_command = INVALID_COMMAND;
          while (disarm_locked) {
            //dshot_disarm();
            //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
            user_command = get_usb_command();
            uart_data_ready = false;

            switch (user_command) {
              case DOWNLOAD_LOGS:
              //TODO: Implement
                ;
                break;
              case UPDATE_PID_GAINS:
                pid_update_gains();
                break;
              case UNLOCK:
                unlock_state(&disarm_locked);
                break;
              case INVALID_COMMAND: 
                printf("\nTry again\n");
                break;
            }

          // require affirmative user input to re-arm
          // print list of options every 3 seconds: 
          // re-arm (10s delay)
          // download logs
          // update PID gains
          // check UART buffer for user command
          // if user commands to release disarm, reset disarm_locked flag

          }
          HAL_Delay(10000);
          break;
        case SOFT_DISARM: 
          //fall through (default to disarmed)
          //only requires flipping ARM switch to re-arm
        default: 
          dshot_disarm();
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
          break;
      }
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Resets ibus failsafe flag counter every time a new packet is recieved. prevents stale data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == IBUS_UART)
		ibus_reset_failsafe();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == MAIN_LOOP_TIM) {
    loop_ready_flag = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
