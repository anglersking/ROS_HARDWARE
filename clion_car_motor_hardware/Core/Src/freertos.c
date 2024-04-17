/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "contrl.h"
#include "comminicate.h"
#include "car_task.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Task_uart2Handle;
osThreadId Task_uart1Handle;
osThreadId Task_PIDHandle;
osThreadId Task_IMUHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_uart2(void const * argument);
void StartTask_uart1(void const * argument);
void StartTask_PID(void const * argument);
void StartTask_IMU(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_uart2 */
  osThreadDef(Task_uart2, StartTask_uart2, osPriorityNormal, 0, 128);
  Task_uart2Handle = osThreadCreate(osThread(Task_uart2), NULL);

  /* definition and creation of Task_uart1 */
  osThreadDef(Task_uart1, StartTask_uart1, osPriorityIdle, 0, 128);
  Task_uart1Handle = osThreadCreate(osThread(Task_uart1), NULL);

  /* definition and creation of Task_PID */
  osThreadDef(Task_PID, StartTask_PID, osPriorityIdle, 0, 128);
  Task_PIDHandle = osThreadCreate(osThread(Task_PID), NULL);

  /* definition and creation of Task_IMU */
  osThreadDef(Task_IMU, StartTask_IMU, osPriorityIdle, 0, 128);
  Task_IMUHandle = osThreadCreate(osThread(Task_IMU), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask_uart2 */
/**
  * @brief  Function implementing the Task_uart2 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_uart2 */
void StartTask_uart2(void const * argument)
{
  /* USER CODE BEGIN StartTask_uart2 */
  /* Infinite loop */
  for(;;)
  {
//    HAL_UART_Transmit(&huart1,(uint8_t *) "GPS TASK\n", 9, 100);
    osDelay(1000);
  }
  /* USER CODE END StartTask_uart2 */
}

/* USER CODE BEGIN Header_StartTask_uart1 */
/**
* @brief Function implementing the Task_uart1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_uart1 */
void StartTask_uart1(void const * argument)
{
  /* USER CODE BEGIN StartTask_uart1 */
//  printf("uart1_init");

  /* Infinite loop */
  for(;;)
  {

//    printf("Task uart");
//    HAL_UART_Transmit(&huart1,(uint8_t *) "UART SEND\n", 10, 100);
    SendTo_UbuntuPC();
    osDelay(1000);
  }
  /* USER CODE END StartTask_uart1 */
}

/* USER CODE BEGIN Header_StartTask_PID */
/**
* @brief Function implementing the Task_PID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_PID */
void StartTask_PID(void const * argument)
{
  /* USER CODE BEGIN StartTask_PID */
  /* Infinite loop */
  for(;;)
  {
//      Car_Task_100HZ();
//      Moto_Control_speed(Right_moto.Current_Speed, Right_moto.Target_Speed ,MOTO_RIGHT);
//      Moto_Control_speed(Left_moto.Current_Speed,  Left_moto.Target_Speed  ,MOTO_LEFT );
//    HAL_UART_Transmit(&huart1,(uint8_t *) "control Task\n", 13, 100);
    osDelay(1000);
  }
  /* USER CODE END StartTask_PID */
}

/* USER CODE BEGIN Header_StartTask_IMU */
/**
* @brief Function implementing the Task_IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_IMU */
void StartTask_IMU(void const * argument)
{
  /* USER CODE BEGIN StartTask_IMU */
  /* Infinite loop */
  for(;;)
  {
//    HAL_UART_Transmit(&huart1,(uint8_t *) "IMU TASK\n", 9, 100);
    osDelay(1000);
  }
  /* USER CODE END StartTask_IMU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
