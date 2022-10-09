/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "car_task.h"
#include "inv_mpu_user.h"
#include "ROS_USART_DEVICE.h"
#include "contrl.h"
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
//unsigned char Rcount = 0;
/* USER CODE END Variables */
osThreadId Task_PID_Ctrol_Handle;
osThreadId Task_IMU_200HZHandle;
osThreadId Task_USART_5HZHandle;
osThreadId Task_InteractioHandle;
osMessageQId Ubuntu_data_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Task_PID_Ctrol_Task_100HZ(void const * argument);
void Start_Task_IMU_200HZ(void const * argument);
void Start_Task_USART_5HZ(void const * argument);
void Start_Task_Interactio(void const * argument);

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

  /* Create the queue(s) */
  /* definition and creation of Ubuntu_data_Queue */
  osMessageQDef(Ubuntu_data_Queue, 33, unsigned char);
  Ubuntu_data_QueueHandle = osMessageCreate(osMessageQ(Ubuntu_data_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_PID_Ctrol_ */
  osThreadDef(Task_PID_Ctrol_, Start_Task_PID_Ctrol_Task_100HZ, osPriorityIdle, 0, 128);
  Task_PID_Ctrol_Handle = osThreadCreate(osThread(Task_PID_Ctrol_), NULL);

  /* definition and creation of Task_IMU_200HZ */
  osThreadDef(Task_IMU_200HZ, Start_Task_IMU_200HZ, osPriorityNormal, 0, 128);
  Task_IMU_200HZHandle = osThreadCreate(osThread(Task_IMU_200HZ), NULL);

  /* definition and creation of Task_USART_5HZ */
  osThreadDef(Task_USART_5HZ, Start_Task_USART_5HZ, osPriorityIdle, 0, 128);
  Task_USART_5HZHandle = osThreadCreate(osThread(Task_USART_5HZ), NULL);

  /* definition and creation of Task_Interactio */
  osThreadDef(Task_Interactio, Start_Task_Interactio, osPriorityIdle, 0, 128);
  Task_InteractioHandle = osThreadCreate(osThread(Task_Interactio), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Task_PID_Ctrol_Task_100HZ */
/**
  * @brief  Function implementing the Task_PID_Ctrol_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Task_PID_Ctrol_Task_100HZ */
void Start_Task_PID_Ctrol_Task_100HZ(void const * argument)
{
  /* USER CODE BEGIN Start_Task_PID_Ctrol_Task_100HZ */
  /* Infinite loop */
  for(;;)
  {
		Car_Task_100HZ();
		Moto_Control_speed(Right_moto.Current_Speed, Right_moto.Target_Speed ,MOTO_RIGHT);
		Moto_Control_speed(Left_moto.Current_Speed,  Left_moto.Target_Speed  ,MOTO_LEFT );
	//	printf("Task_PID_Ctrol\n");
    osDelay(1000);
  }
  /* USER CODE END Start_Task_PID_Ctrol_Task_100HZ */
}

/* USER CODE BEGIN Header_Start_Task_IMU_200HZ */
/**
* @brief Function implementing the Task_IMU_200HZ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_IMU_200HZ */
void Start_Task_IMU_200HZ(void const * argument)
{
  /* USER CODE BEGIN Start_Task_IMU_200HZ */
  /* Infinite loop */
	MPU_Init();
	
	while(mpu_dmp_init());
	printf("Task_IMU_get\n");
  for(;;)
  {
		Car_Task_200HZ();
		
    osDelay(200);
  }
  /* USER CODE END Start_Task_IMU_200HZ */
}

/* USER CODE BEGIN Header_Start_Task_USART_5HZ */
/**
* @brief Function implementing the Task_USART_5HZ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_USART_5HZ */
void Start_Task_USART_5HZ(void const * argument)
{
  /* USER CODE BEGIN Start_Task_USART_5HZ */
  /* Infinite loop */
  for(;;)
  {
		Car_Task_5HZ();
		//printf("Task_ROS_communiate\n");
    osDelay(1000);
  }
  /* USER CODE END Start_Task_USART_5HZ */
}

/* USER CODE BEGIN Header_Start_Task_Interactio */
/**
* @brief Function implementing the Task_Interactio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task_Interactio */
void Start_Task_Interactio(void const * argument)
{
  /* USER CODE BEGIN Start_Task_Interactio */
	uint8_t u8Index;
  /* Infinite loop */
  for(;;)
  {
		//每次读取消息之前，把索引初始化为0
	 // u8Index = 0;
	  //1、一直等待接收消息,第一个消息应该放在消息缓冲区的第一个元素上
		//if(xQueueReceive(Ubuntu_data_QueueHandle,&Recive_Data.buffer[u8Index++],portMAX_DELAY)==pdPASS){
		//	while(xQueueReceive(Ubuntu_data_QueueHandle,&Recive_Data.buffer[u8Index++],50)){}
			//Recive_Data.buffer[u8Index] = '\0';//保证一包完整字符串信息
		
				
		
		//}
				
			//完成解析以后，要清空接收缓冲区，不然会出现问题
		  //  memset(Recive_Data.buffer,0,33);
		//printf("Task_Interactio\n");
    osDelay(100);
  }

  /* USER CODE END Start_Task_Interactio */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
