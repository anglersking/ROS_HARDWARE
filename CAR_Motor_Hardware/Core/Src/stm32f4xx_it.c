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
#include "ROS_USART_DEVICE.h"
#include "cmsis_os.h"
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
unsigned char Rcount = 0;
extern osMessageQId Ubuntu_data_QueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

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
  while (1)
  {
  }
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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
   
	//�жϽ��ձ�־��λ
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) == SET){
		//��ȡ���ռĴ���
		Recive_Data.buffer[Rcount] = huart1.Instance->DR;
		(Recive_Data.buffer[0] == 0xFe)?(Rcount++):(Rcount = 0);
		if (Rcount == PROTOCL_DATA_SIZE)	//��֤���ݰ��ĳ���
		{
			if(Recive_Data.Sensor_Str.Header == PROTOCOL_HEADER)	//��֤���ݰ���ͷ��У����Ϣ
			{
				if(Recive_Data.Sensor_Str.End_flag == PROTOCOL_END)	//��֤���ݰ���β��У����Ϣ
				{
					//������λ���������ʹ�����˲�����Ӧ���˶�
					Kinematics_Positive(Recive_Data.Sensor_Str.X_speed, Recive_Data.Sensor_Str.Z_speed);
				}
			}
			Rcount = 0;
		}
		
		//������Ӳ���
		//xQueueSendFromISR(Ubuntu_data_QueueHandle,&Recive_Data,NULL);
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/*unsigned char Rcount = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)"�Ѿ��ж�",10, 100);
	if(huart->Instance == USART1)
	{
		printf("�Ѿ����봮���жϣ�����������������");
		HAL_UART_Receive_IT(&huart1, Recive_Data.buffer, sizeof(Recive_Data.buffer));
		HAL_UART_Transmit(&huart1,Recive_Data.buffer, sizeof(Recive_Data.buffer), 100);
		(Recive_Data.buffer[0] == 0xFe)?(Rcount++):(Rcount = 0);
	
		if (Rcount !=0)	//��֤���ݰ��ĳ���
		{
			printf("INTERUPT %x,%x",Recive_Data.Sensor_Str.Header,PROTOCOL_HEADER);
			if(Recive_Data.Sensor_Str.Header == PROTOCOL_HEADER)	//��֤���ݰ���ͷ��У����Ϣ
			{
				printf("INTERUPTendend %x,%x",Recive_Data.Sensor_Str.End_flag,PROTOCOL_END);
				if(Recive_Data.Sensor_Str.End_flag == PROTOCOL_END)	//��֤���ݰ���β��У����Ϣ
				{
					printf("INTERUPT vvv %f,%f",Recive_Data.Sensor_Str.X_speed,Recive_Data.Sensor_Str.Z_speed);
					//������λ���������ʹ�����˲�����Ӧ���˶�
					Kinematics_Positive(Recive_Data.Sensor_Str.X_speed, Recive_Data.Sensor_Str.Z_speed);
				}
			}
			Rcount = 0;
		}
	} 
}

*/
/* USER CODE END 1 */
