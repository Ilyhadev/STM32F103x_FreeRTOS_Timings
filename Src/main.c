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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
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
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t uartMutex;      // Protects UART
SemaphoreHandle_t uartSemaphore;  // Signals TX complete from ISR

/* Task handles - global so trace hooks can identify them */
TaskHandle_t hTaskHigh   = NULL;
TaskHandle_t hTaskMed    = NULL;
TaskHandle_t hTaskMonitor = NULL;

/* Pure CPU cycle measurement via trace hooks */
volatile uint32_t task_cpu_start_cycles = 0;
volatile uint32_t task_cpu_cycles[2] = {0};   // [0]=High, [1]=Med

/* Statistics per task (reset every 10 s) */
typedef struct {
    int32_t  min_jitter_us;
    int32_t  max_jitter_us;
    int64_t  sum_jitter_us;
    uint32_t job_count;
    uint32_t deadline_misses;
    uint32_t max_response_us;
    uint32_t max_pure_c_us;
    int32_t  min_pure_c_us;
    uint64_t sum_pure_c_us;
} TaskStats_t;

TaskStats_t stats[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// timing
static volatile uint64_t dwt_overflows = 0;
static volatile uint32_t last_dwt = 0;

static inline uint64_t get_micros64(void) {
    uint32_t now;
    uint64_t overflows;

    taskENTER_CRITICAL();

    now = DWT->CYCCNT;
    if (now < last_dwt) {
        dwt_overflows += (1ULL << 32);
    }
    last_dwt = now;
    overflows = dwt_overflows;

    taskEXIT_CRITICAL();

    return (overflows + now) / 64ULL;
}
void App_TraceSwitchedIn(void) {
    task_cpu_start_cycles = DWT->CYCCNT;
}

void App_TraceSwitchedOut(void) {
    uint32_t delta = DWT->CYCCNT - task_cpu_start_cycles;
    TaskHandle_t curr = xTaskGetCurrentTaskHandle();

    if (curr == hTaskHigh) {
        task_cpu_cycles[0] += delta;
    } else if (curr == hTaskMed) {
        task_cpu_cycles[1] += delta;
    }
}
void CalibrateWorkload(void) {
	char buf[64];
    uint32_t start = get_micros64();
    for (volatile uint32_t i = 0; i < 100000; i++);
    uint32_t end = get_micros64();

    // This prints how many microseconds 100,000 iterations take.
    printf("100k loops = %lu us\n", end - start);
    int len = snprintf(buf, sizeof(buf), "100k loops = %lu us\n", end - start);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 10);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Signal the task that transmission is finished
        xSemaphoreGiveFromISR(uartSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

typedef struct {
    uint32_t period_ms;
    uint32_t workload_loops;
    uint8_t  idx;          // 0 = High, 1 = Med
} TaskParams_t;


void PeriodicTask(void *argument) {
    TaskParams_t *p = (TaskParams_t *)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint64_t theoretical_us = get_micros64();

    while (1) {
    	uint64_t release_us = get_micros64();
        int32_t  jitter_us  = (int32_t)(release_us - theoretical_us);

        /* Start fresh measurement for this job */
        task_cpu_cycles[p->idx] = 0;

        for (volatile uint32_t i = 0; i < p->workload_loops; i++){
        	if (i % (rand() % 10 + 1) == 0) {
        	        volatile uint32_t dummy = i * 2;
        	} else if (rand() % 3) {
        		volatile uint32_t dummy = i * 2;
        	} else if (rand() % 2) {
        		volatile uint32_t dummy = i * 2;
        		volatile uint32_t dummy2 = i * 2;
        	}
        }





        /* Update stats */
        if (jitter_us < stats[p->idx].min_jitter_us) stats[p->idx].min_jitter_us = jitter_us;
        if (jitter_us > stats[p->idx].max_jitter_us) stats[p->idx].max_jitter_us = jitter_us;
        stats[p->idx].sum_jitter_us += jitter_us;
        stats[p->idx].job_count++;

        /* Pure execution time C = all CPU slices + current slice */
        uint32_t current_slice = DWT->CYCCNT - task_cpu_start_cycles;
        uint32_t pure_c_us     = (task_cpu_cycles[p->idx] + current_slice) / 64U;

        if (pure_c_us > stats[p->idx].max_pure_c_us)     stats[p->idx].max_pure_c_us = pure_c_us;
        if (pure_c_us < stats[p->idx].min_pure_c_us)     stats[p->idx].min_pure_c_us = pure_c_us;
        stats[p->idx].sum_pure_c_us += pure_c_us;
        uint64_t finish_us = get_micros64();
        uint32_t response_us = (uint32_t)(finish_us - release_us);
        if (response_us > stats[p->idx].max_response_us) stats[p->idx].max_response_us = response_us;
        if (response_us > (p->period_ms * 1000UL)){
        	stats[p->idx].deadline_misses++;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(p->period_ms));
        theoretical_us += (uint64_t)p->period_ms * 1000ULL;
    }
}


/* Monitor Task*/
void MonitorTask(void *arg) {
    char buf[170];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10000); // X0 s window

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        for (int i = 0; i < 2; i++) {
            if (stats[i].job_count == 0) continue;

            int32_t  avg_jitter = (int32_t)(stats[i].sum_jitter_us / stats[i].job_count);
            uint32_t avg_c_us   = (uint32_t)(stats[i].sum_pure_c_us / stats[i].job_count);

            int len = snprintf(buf, sizeof(buf),
                "Task %d | Jobs:%lu | Jitter min/avg/max: %ld/%ld/%ld us | "
                "Max R: %lu us | Max C: %lu us | Min C: %ld us | Avg C: %lu us | Misses: %lu\r\n",
                i,
                stats[i].job_count,
                stats[i].min_jitter_us,
                avg_jitter,
                stats[i].max_jitter_us,
                stats[i].max_response_us,
                stats[i].max_pure_c_us,
				stats[i].min_pure_c_us,
                avg_c_us,
                stats[i].deadline_misses);

            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
                HAL_UART_Transmit_IT(&huart1, (uint8_t*)buf, len);
                xSemaphoreTake(uartSemaphore, portMAX_DELAY);
                xSemaphoreGive(uartMutex);
            }

            /* Reset for next window */
            stats[i].min_jitter_us = INT32_MAX;
            stats[i].max_jitter_us = INT32_MIN;
            stats[i].sum_jitter_us = 0;
            stats[i].job_count     = 0;
            stats[i].deadline_misses = 0;
            stats[i].max_response_us = 0;
            stats[i].max_pure_c_us   = 0;
            stats[i].sum_pure_c_us   = 0;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  last_dwt = 0;
  dwt_overflows = 0;
  /* Calibration (uses blocking TX) */
  CalibrateWorkload();

  /* Init stats */
  for (int i = 0; i < 2; i++) {
      stats[i].min_jitter_us = INT32_MAX;
      stats[i].max_jitter_us = INT32_MIN;
      stats[i].sum_jitter_us = 0;
      stats[i].job_count     = 0;
      stats[i].deadline_misses = 0;
      stats[i].max_response_us = 0;
      stats[i].max_pure_c_us   = 0;
      stats[i].min_pure_c_us   = INT32_MAX;
      stats[i].sum_pure_c_us   = 0;
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uartMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  uartSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadAttr_t attr = { .stack_size = 512 * 4 };

  static TaskParams_t highParams = { .period_ms = 13, .workload_loops = 3000, .idx = 0 };
  attr.name = "HighTask";
  attr.priority = (osPriority_t) osPriorityAboveNormal;
  hTaskHigh = osThreadNew(PeriodicTask, &highParams, &attr);

  static TaskParams_t medParams = { .period_ms = 40, .workload_loops = 1000, .idx = 1 };
  attr.name = "MedTask";
  attr.priority = (osPriority_t) osPriorityNormal;
  hTaskMed = osThreadNew(PeriodicTask, &medParams, &attr);

  attr.name = "Monitor";
  attr.priority = (osPriority_t) osPriorityNormal;
  attr.stack_size = 384 * 4;
  hTaskMonitor = osThreadNew(MonitorTask, NULL, &attr);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  HAL_UART_Transmit(&huart1, (uint8_t*)"=== All tasks created - starting scheduler ===\r\n", 48, 100);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
