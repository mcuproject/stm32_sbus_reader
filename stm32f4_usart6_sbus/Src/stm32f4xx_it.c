/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static void process_sbus(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim12;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */

uint32_t usart1_int_cnt, usart1_idle_cnt;
uint8_t usart1_data_buffer[30];
void USART6_IRQHandler(void) {
    /* USER CODE BEGIN USART1_IRQn 0 */
    usart1_int_cnt++;
    if (USART6->SR & 0x0010) {
        
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
        /* This part is important */
        /* Clear IDLE flag by reading status register first */
        /* And follow by reading data register */
        volatile uint32_t tmp; /* Must be volatile to prevent optimizations */
        tmp = USART6->SR;      /* Read status register */
        tmp = USART6->DR;      /* Read data register */

        DMA2_Stream1->M0AR = (uint32_t)usart1_data_buffer; /* Set memory address for DMA again */
        DMA2_Stream1->NDTR = 30;               /* Set number of bytes to receive */
        __HAL_DMA_ENABLE(&hdma_usart6_rx);
			if (usart1_data_buffer[0] == 0x0f) {
				usart1_idle_cnt++;
				process_sbus();	
			}
				
    }
    /* USER CODE END USART1_IRQn 0 */
    //HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}
/* sbus channels */
uint16_t channels[16];
uint8_t lost_frame, failsafe;
/* USER CODE BEGIN 1 */
static void process_sbus(void) {
			channels[0]  = (uint16_t)((usart1_data_buffer[1] | usart1_data_buffer[2] << 8) & 0x07FF);
			channels[1]  = (uint16_t)((usart1_data_buffer[2] >> 3 | usart1_data_buffer[3] << 5) & 0x07FF);
			channels[2]  = (uint16_t)((usart1_data_buffer[3] >> 6 | usart1_data_buffer[4] << 2 | usart1_data_buffer[5] << 10) & 0x07FF);
			channels[3]  = (uint16_t)((usart1_data_buffer[5] >> 1 | usart1_data_buffer[6] << 7) & 0x07FF);
			channels[4]  = (uint16_t)((usart1_data_buffer[6] >> 4 | usart1_data_buffer[7] << 4) & 0x07FF);
			channels[5]  = (uint16_t)((usart1_data_buffer[7] >> 7 | usart1_data_buffer[8] << 1 | usart1_data_buffer[9] << 9) & 0x07FF);
			channels[6]  = (uint16_t)((usart1_data_buffer[9] >> 2 | usart1_data_buffer[10] << 6) & 0x07FF);
			channels[7]  = (uint16_t)((usart1_data_buffer[10] >> 5 | usart1_data_buffer[11]<< 3) & 0x07FF);
			channels[8]  = (uint16_t)((usart1_data_buffer[12] | usart1_data_buffer[13] << 8) & 0x07FF);
			channels[9]  = (uint16_t)((usart1_data_buffer[13] >> 3 | usart1_data_buffer[14] << 5) & 0x07FF);
			channels[10] = (uint16_t)((usart1_data_buffer[14] >> 6 | usart1_data_buffer[15] << 2 | usart1_data_buffer[16] << 10) & 0x07FF);
			channels[11] = (uint16_t)((usart1_data_buffer[16] >> 1 | usart1_data_buffer[17] << 7) & 0x07FF);
			channels[12] = (uint16_t)((usart1_data_buffer[17] >> 4 | usart1_data_buffer[18] << 4) & 0x07FF);
			channels[13] = (uint16_t)((usart1_data_buffer[18] >> 7 | usart1_data_buffer[19] << 1 | usart1_data_buffer[20] <<9 ) & 0x07FF);
			channels[14] = (uint16_t)((usart1_data_buffer[20] >> 2 | usart1_data_buffer[21] << 6) & 0x07FF);
			channels[15] = (uint16_t)((usart1_data_buffer[21] >> 5 | usart1_data_buffer[22] << 3) & 0x07FF);
			lost_frame = usart1_data_buffer[23] && (1<<2);
			failsafe = usart1_data_buffer[23] && (1<<3);
}

/* USER CODE END 1 */
