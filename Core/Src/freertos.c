/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <stdio.h>
#include "atkled.h"
#include "tftlcd.h"
#include "touch.h"
#include "spi_flash.h"
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
osThreadId xTask00Handle;
osThreadId xTask01Handle;
osThreadId xTask02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void vUserTask00(void const * argument);
void vUserTask01(void const * argument);
void vUserTask03(void const * argument);

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
  /* definition and creation of xTask00 */
  osThreadDef(xTask00, vUserTask00, osPriorityIdle, 0, 1024);
  xTask00Handle = osThreadCreate(osThread(xTask00), NULL);

  /* definition and creation of xTask01 */
  osThreadDef(xTask01, vUserTask01, osPriorityIdle, 0, 1024);
  xTask01Handle = osThreadCreate(osThread(xTask01), NULL);

  /* definition and creation of xTask02 */
  osThreadDef(xTask02, vUserTask03, osPriorityIdle, 0, 1024);
  xTask02Handle = osThreadCreate(osThread(xTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_vUserTask00 */
uint8_t ExtSRAM[1024*1024] __attribute__((section(".sram")));
/**
  * @brief  Function implementing the xTask00 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_vUserTask00 */
void vUserTask00(void const * argument)
{
  /* USER CODE BEGIN vUserTask00 */
  /* Infinite loop */
  for(;;)
  {
    LED0=LEDON;
    LED1=LEDOFF;
    osDelay(500);
    LED0=LEDOFF;
    LED1=LEDON;
    osDelay(500);
  }
  /* USER CODE END vUserTask00 */
}

/* USER CODE BEGIN Header_vUserTask01 */
/**
* @brief Function implementing the xTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vUserTask01 */
void vUserTask01(void const * argument)
{
  /* USER CODE BEGIN vUserTask01 */
  uint8_t t=0;
	uint8_t i=0;	  	    
 	uint16_t lastpos[5][2];		//记录最后一次的数据   
  /* Infinite loop */
  for(;;)
  {
		tp_dev.scan(0);
		for(t=0;t<CT_MAX_TOUCH;t++){
			if((tp_dev.sta)&(1<<t)){
				if(tp_dev.x[t]<lcddev.width&&tp_dev.y[t]<lcddev.height){
					if(lastpos[t][0]==0XFFFF){
						lastpos[t][0] = tp_dev.x[t];
						lastpos[t][1] = tp_dev.y[t];
					}
					lcd_draw_bline(lastpos[t][0],lastpos[t][1],tp_dev.x[t],tp_dev.y[t],2,POINT_COLOR_TBL[t]);//画线
					lastpos[t][0]=tp_dev.x[t];
					lastpos[t][1]=tp_dev.y[t];
					if(tp_dev.x[t]>(lcddev.width-24)&&tp_dev.y[t]<16){
						Load_Drow_Dialog();//清除
					}
				}
			}else{
        lastpos[t][0]=0XFFFF;
      }
		}
		osDelay(5);i++;
		if(i%20==0){
      LED0=!LED0;
    }
  }
  /* USER CODE END vUserTask01 */
}

/* USER CODE BEGIN Header_vUserTask03 */
/**
* @brief Function implementing the xTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vUserTask03 */
void vUserTask03(void const * argument)
{
  /* USER CODE BEGIN vUserTask03 */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vUserTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
