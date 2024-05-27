/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "comminicate.h"
#include "contrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
unsigned char Rcount = 0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
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
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
    /* USER CODE BEGIN TIM4_IRQn 0 */

    /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4);
    /* USER CODE BEGIN TIM4_IRQn 1 */

    /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    HAL_UART_Receive_IT(&huart1, Recive_Data.buffer, sizeof(Recive_Data.buffer));
//    HAL_UART_Transmit(&huart1,(uint8_t *) "init hahaha\n", 9, 100);
//
//    HAL_UART_Receive_IT(&huart1, Recive_Data.buffer, sizeof(Recive_Data.buffer));

    //判断接收标志置位
//    if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) == SET){
//        //读取接收寄存器
//        Recive_Data.buffer[Rcount] = huart1.Instance->DR;
//        (Recive_Data.buffer[0] == 0xFe)?(Rcount++):(Rcount = 0);
//        if (Rcount == PROTOCL_DATA_SIZE)	//��֤���ݰ��ĳ���
//        {
//            if(Recive_Data.Sensor_Str.Header == PROTOCOL_HEADER)	//��֤���ݰ���ͷ��У����Ϣ
//            {
//                if(Recive_Data.Sensor_Str.End_flag == PROTOCOL_END)	//��֤���ݰ���β��У����Ϣ
//                {
//                    //������λ���������ʹ�����˲�����Ӧ���˶�
//                    Kinematics_Positive(Recive_Data.Sensor_Str.X_speed, Recive_Data.Sensor_Str.Z_speed);
//                }
//            }
//            Rcount = 0;
//        }

    //������Ӳ���
    //xQueueSendFromISR(Ubuntu_data_QueueHandle,&Recive_Data,NULL);
    // }

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */

    /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
    /* USER CODE BEGIN TIM5_IRQn 0 */

    /* USER CODE END TIM5_IRQn 0 */
    HAL_TIM_IRQHandler(&htim5);
    /* USER CODE BEGIN TIM5_IRQn 1 */

    /* USER CODE END TIM5_IRQn 1 */
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
#include <stdio.h>
#include <string.h>
void SendFloatOverUART(UART_HandleTypeDef *huart, float value) {
    char buffer[32]; // 足够大的缓冲区来存储浮点数和终止符
    int length;

    // 使用sprintf将浮点数转换为字符串，并指定小数点后的位数（例如2位）
    length = sprintf(buffer, "%.2f", value);

    // 确保字符串以null字符结束
    buffer[length] = '\0';

    // 发送字符串
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    HAL_UART_Transmit(&huart1,(uint8_t *) "init callbak\n", 9, 100);

    if(huart->Instance == USART1)
    {
//        HAL_UART_Transmit(&huart1,(uint8_t *) "init callback\n", 14, 100);
        HAL_UART_Receive_IT(&huart1, Recive_Data.buffer, sizeof(Recive_Data.buffer));
        (Recive_Data.buffer[0] == 0xFe)?(Rcount++):(Rcount = 0);

        if (Rcount ==PROTOCL_DATA_SIZE)	//验证数据包的长度 之前是 !=0
        {
            //	printf("INTERUPT %x,%x",Recive_Data.Sensor_Str.Header,PROTOCOL_HEADER);
            if(Recive_Data.Sensor_Str.Header == PROTOCOL_HEADER)	//验证数据包的头部校验信息
            {
                //	printf("INTERUPTendend %x,%x",Recive_Data.Sensor_Str.End_flag,PROTOCOL_END);
                if(Recive_Data.Sensor_Str.End_flag == PROTOCOL_END)	//验证数据包的尾部校验信息
                {

                    Send_Data.Sensor_Str.Source_Voltage = Recive_Data.Sensor_Str.X_speed;//Source_Valtage;

//                    	printf("INTERUPT vvv %f,%f",Recive_Data.Sensor_Str.X_speed,Recive_Data.Sensor_Str.Z_speed);
                    //接收上位机控制命令，使机器人产生相应的运动
//                    SendFloatOverUART(&huart1, Left_moto.Target_Speed  );
//                    SendFloatOverUART(&huart1, Right_moto.Target_Speed  );
//                    HAL_UART_Transmit(&huart1,(uint8_t *) "in control\n", 11, 100);
//                    SendFloatOverUART(&huart1, Recive_Data.Sensor_Str.X_speed);

//                    HAL_UART_Transmit(&huart1,(uint8_t *) a, 12, 100);
//                    HAL_UART_Transmit(&huart1,Recive_Data.Sensor_Str.X_speed, 12, 100);
                    Kinematics_Positive(Recive_Data.Sensor_Str.X_speed, Recive_Data.Sensor_Str.Z_speed);
//                    SendFloatOverUART(&huart1, Left_moto.Target_Speed  );
//                    SendFloatOverUART(&huart1, Right_moto.Target_Speed  );


                }
            }
            Rcount = 0;
        }
        HAL_UART_Receive_IT(&huart1, Recive_Data.buffer, sizeof(Recive_Data.buffer));
    }
}

/* USER CODE END 1 */
