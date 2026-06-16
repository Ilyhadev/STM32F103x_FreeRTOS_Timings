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
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "mpu6050.h"
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
I2C_HandleTypeDef hi2c1;

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

/* Task handles - global so trace hooks can identify them */
TaskHandle_t hTaskI2C     = NULL;
TaskHandle_t hTaskProcess = NULL;
TaskHandle_t hTaskUart    = NULL;
TaskHandle_t hTaskMonitor = NULL;

mpu6050_t Mpu6050 = {0};

/* Pure CPU cycle measurement via trace hooks */
volatile uint32_t task_cpu_start_cycles = 0;

#define MEASURED_TASK_COUNT 3
#define TASK_I2C_IDX        0
#define TASK_PROCESS_IDX    1
#define TASK_UART_IDX       2

volatile uint32_t task_cpu_cycles[MEASURED_TASK_COUNT] = {0};

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

TaskStats_t stats[MEASURED_TASK_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
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
    BaseType_t scheduler_running = (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
    uint32_t primask = 0;

    if (scheduler_running) {
        taskENTER_CRITICAL();
    } else {
        primask = __get_PRIMASK();
        __disable_irq();
    }

    now = DWT->CYCCNT;
    if (now < last_dwt) {
        dwt_overflows += (1ULL << 32);
    }
    last_dwt = now;
    overflows = dwt_overflows;

    if (scheduler_running) {
        taskEXIT_CRITICAL();
    } else if (primask == 0U) {
        __enable_irq();
    }

    return (overflows + now) / 64ULL;
}
void App_TraceSwitchedIn(void) {
    task_cpu_start_cycles = DWT->CYCCNT;
}

void App_TraceSwitchedOut(void) {
    uint32_t delta = DWT->CYCCNT - task_cpu_start_cycles;
    TaskHandle_t curr = xTaskGetCurrentTaskHandle();

    if (curr == hTaskI2C) {
        task_cpu_cycles[TASK_I2C_IDX] += delta;
    } else if (curr == hTaskProcess) {
        task_cpu_cycles[TASK_PROCESS_IDX] += delta;
    } else if (curr == hTaskUart) {
        task_cpu_cycles[TASK_UART_IDX] += delta;
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

static void DebugLog(const char *msg) {
    if ((uartMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        if (xSemaphoreTake(uartMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
            xSemaphoreGive(uartMutex);
        }
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
    }
}

static void CheckCreated(const void *handle, const char *name) {
    if (handle == NULL) {
        DebugLog("create failed: ");
        DebugLog(name);
        DebugLog("\r\n");
        Error_Handler();
    }
}

#define I2C_TASK_PERIOD_MS     10U
#define RAW_RING_LEN           8U
#define PROCESSED_RING_LEN     8U
#define MPU_AXIS_COUNT         FIFO_SAMPLE_VALUES

static const char *task_names[MEASURED_TASK_COUNT] = {
    "I2C",
    "Process",
    "UART"
};

typedef struct {
    uint32_t seq;
    uint64_t capture_us;
    uint64_t ready_us;
    float value[MPU_AXIS_COUNT]; // ax, ay, az, temp, gx, gy, gz
} SensorSample_t;

typedef struct {
    uint32_t seq;
    uint64_t capture_us;
    uint64_t ready_us;
    float value[MPU_AXIS_COUNT];
} ProcessedSample_t;

static QueueHandle_t rawRing;
static QueueHandle_t processedRing;
static volatile uint32_t raw_drop_count = 0;
static volatile uint32_t processed_drop_count = 0;
static volatile uint32_t fifo_empty_count = 0;
static volatile uint32_t fifo_overflow_count = 0;
static volatile uint16_t last_fifo_count = 0;
static volatile uint16_t max_fifo_count = 0;

static void ResetTaskStats(uint8_t idx) {
    stats[idx].min_jitter_us = INT32_MAX;
    stats[idx].max_jitter_us = INT32_MIN;
    stats[idx].sum_jitter_us = 0;
    stats[idx].job_count = 0;
    stats[idx].deadline_misses = 0;
    stats[idx].max_response_us = 0;
    stats[idx].max_pure_c_us = 0;
    stats[idx].min_pure_c_us = INT32_MAX;
    stats[idx].sum_pure_c_us = 0;
}

static void BeginMeasuredJob(uint8_t idx, int32_t jitter_us) {
    task_cpu_cycles[idx] = 0;

    if (jitter_us < stats[idx].min_jitter_us) stats[idx].min_jitter_us = jitter_us;
    if (jitter_us > stats[idx].max_jitter_us) stats[idx].max_jitter_us = jitter_us;
    stats[idx].sum_jitter_us += jitter_us;
    stats[idx].job_count++;
}

static void EndMeasuredJob(uint8_t idx, uint64_t release_us, uint32_t deadline_us) {
    uint32_t current_slice = DWT->CYCCNT - task_cpu_start_cycles;
    uint32_t pure_c_us = (task_cpu_cycles[idx] + current_slice) / 64U;
    uint64_t finish_us = get_micros64();
    uint32_t response_us = (uint32_t)(finish_us - release_us);

    if (pure_c_us > stats[idx].max_pure_c_us) stats[idx].max_pure_c_us = pure_c_us;
    if ((int32_t)pure_c_us < stats[idx].min_pure_c_us) stats[idx].min_pure_c_us = (int32_t)pure_c_us;
    stats[idx].sum_pure_c_us += pure_c_us;

    if (response_us > stats[idx].max_response_us) stats[idx].max_response_us = response_us;
    if ((deadline_us > 0U) && (response_us > deadline_us)) {
        stats[idx].deadline_misses++;
    }
}

static bool ReadFifoIntoSample(SensorSample_t *sample) {
    uint16_t fifo_count = MPU6050_Get_FIFO_Count(&Mpu6050);
    last_fifo_count = fifo_count;
    if (fifo_count > max_fifo_count) {
        max_fifo_count = fifo_count;
    }

    if (fifo_count >= 1024U) {
        fifo_overflow_count++;
        MPU6050_Read_Fifo(&Mpu6050);
        return false;
    }

    if (fifo_count < FIFO_SAMPLE_SIZE) {
        fifo_empty_count++;
        return false;
    }

    sample->capture_us = get_micros64();
    MPU6050_Read_Fifo(&Mpu6050);
    MPU6050_Get_Current_Sample(&Mpu6050, sample->value);
    sample->ready_us = get_micros64();

    return true;
}

void I2CReadTask(void *argument) {
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint64_t theoretical_us = get_micros64();
    uint32_t seq = 0;

    while (1) {
        uint64_t release_us = get_micros64();
        int32_t jitter_us = (int32_t)(release_us - theoretical_us);
        SensorSample_t sample = {0};

        BeginMeasuredJob(TASK_I2C_IDX, jitter_us);

        if (ReadFifoIntoSample(&sample)) {
            sample.seq = seq++;
            if (xQueueSend(rawRing, &sample, 0) != pdPASS) {
                raw_drop_count++;
            }
        }

        theoretical_us += (uint64_t)I2C_TASK_PERIOD_MS * 1000ULL;
        EndMeasuredJob(TASK_I2C_IDX, release_us, I2C_TASK_PERIOD_MS * 1000U);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(I2C_TASK_PERIOD_MS));
    }
}

void ProcessDataTask(void *argument) {
    (void)argument;

    while (1) {
        SensorSample_t raw = {0};
        ProcessedSample_t processed = {0};

        xQueueReceive(rawRing, &raw, portMAX_DELAY);

        uint64_t release_us = get_micros64();
        int32_t jitter_us = (int32_t)(release_us - raw.ready_us);
        BeginMeasuredJob(TASK_PROCESS_IDX, jitter_us);

        processed.seq = raw.seq;
        processed.capture_us = raw.capture_us;
        for (uint8_t i = 0; i < MPU_AXIS_COUNT; i++) {
            processed.value[i] = raw.value[i] * 200.0f;
        }
        processed.ready_us = get_micros64();

        if (xQueueSend(processedRing, &processed, 0) != pdPASS) {
            processed_drop_count++;
        }

        EndMeasuredJob(TASK_PROCESS_IDX, release_us, 0);
    }
}

void UartSendTask(void *argument) {
    (void)argument;

    while (1) {
        ProcessedSample_t processed = {0};

        xQueueReceive(processedRing, &processed, portMAX_DELAY);

        uint64_t release_us = get_micros64();
        int32_t jitter_us = (int32_t)(release_us - processed.ready_us);
        BeginMeasuredJob(TASK_UART_IDX, jitter_us);

        EndMeasuredJob(TASK_UART_IDX, release_us, 0);
    }
}


/* Monitor Task*/
void MonitorTask(void *arg) {
    (void)arg;

    char buf[220];
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10000); // X0 s window

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        for (int i = 0; i < MEASURED_TASK_COUNT; i++) {
            if (stats[i].job_count == 0) continue;

            int32_t  avg_jitter = (int32_t)(stats[i].sum_jitter_us / stats[i].job_count);
            uint32_t avg_c_us   = (uint32_t)(stats[i].sum_pure_c_us / stats[i].job_count);

            int len = snprintf(buf, sizeof(buf),
                "%s | Jobs:%lu | Jitter min/avg/max: %ld/%ld/%ld us | "
                "Max R: %lu us | Max C: %lu us | Min C: %ld us | Avg C: %lu us | Misses: %lu\r\n",
                task_names[i],
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
                HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
                xSemaphoreGive(uartMutex);
            }

            /* Reset for next window */
            ResetTaskStats((uint8_t)i);
        }

        int len = snprintf(buf, sizeof(buf),
            "Buffers | raw drops:%lu | processed drops:%lu | fifo empty:%lu | fifo overflow:%lu | fifo last/max:%u/%u\r\n",
            raw_drop_count,
            processed_drop_count,
            fifo_empty_count,
            fifo_overflow_count,
            last_fifo_count,
            max_fifo_count);

        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
            xSemaphoreGive(uartMutex);
        }
        max_fifo_count = 0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  last_dwt = 0;
  dwt_overflows = 0;
  /* Calibration (uses blocking TX) */
  CalibrateWorkload();
  MPU6050_Init(&Mpu6050, &hi2c1);
  float offsetXYZ[3] = {188.75, 240, 3361.5};
  float scaleXYZ[3] = {16358.75, 16312.67, 16721.5};
  MPU6050_Set_Accel_Offset_Scale(&Mpu6050, offsetXYZ, scaleXYZ);
  /* Init stats */
  for (int i = 0; i < MEASURED_TASK_COUNT; i++) {
      ResetTaskStats((uint8_t)i);
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  uartMutex = xSemaphoreCreateMutex();
  CheckCreated(uartMutex, "uartMutex");
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  rawRing = xQueueCreate(RAW_RING_LEN, sizeof(SensorSample_t));
  processedRing = xQueueCreate(PROCESSED_RING_LEN, sizeof(ProcessedSample_t));
  CheckCreated(rawRing, "rawRing");
  CheckCreated(processedRing, "processedRing");
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = NULL;

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadAttr_t attr = { .stack_size = 384 * 4 };

  attr.name = "I2CRead";
  attr.priority = (osPriority_t) osPriorityAboveNormal;
  attr.stack_size = 384 * 4;
  hTaskI2C = osThreadNew(I2CReadTask, NULL, &attr);
  CheckCreated(hTaskI2C, "I2CRead");

  attr.name = "ProcessData";
  attr.priority = (osPriority_t) osPriorityNormal;
  attr.stack_size = 256 * 4;
  hTaskProcess = osThreadNew(ProcessDataTask, NULL, &attr);
  CheckCreated(hTaskProcess, "ProcessData");

  attr.name = "UartSend";
  attr.priority = (osPriority_t) osPriorityNormal;
  attr.stack_size = 512 * 4;
  hTaskUart = osThreadNew(UartSendTask, NULL, &attr);
  CheckCreated(hTaskUart, "UartSend");

  attr.name = "Monitor";
  attr.priority = (osPriority_t) osPriorityNormal;
  attr.stack_size = 384 * 4;
  hTaskMonitor = osThreadNew(MonitorTask, NULL, &attr);
  CheckCreated(hTaskMonitor, "Monitor");

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  static const char startMsg[] = "=== I2C/process/UART jitter pipeline starting ===\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)startMsg, sizeof(startMsg) - 1U, 100);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
