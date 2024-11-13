/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdbool.h>
#include "semphr.h"
#include "filter.h"
#include "new_funcs.h"
#include "task.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const float rThreshold = 0.7;

const float kLowPassCutoff = 5.0;   // 5 hz
const float kHighPassCutoff = 0.5;  // 0.5 hz
const float kSamplingFrequency = 400.0;
const float kEdgeThreshold = -2000.0;

const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;




const float decayRate = 0.02;
const float thrRate = 0.05;
const int minDiff = 50;
const uint32_t fingerThreshold = 10000;  // threshold for detecting a finger on the sensor

float maxValue = 0;
float minValue = 0;
float threshold = 0;
long lastHeartbeat = 0;
float lastValue = 0;
int fingerDetected = 0;  // variable to track whether the finger is detected

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;
/* variables */
LowPassFilter low_pass_filter_red;
LowPassFilter low_pass_filter_ir;
HighPassFilter high_pass_filter;
LowPassFilter low_pass_filter;
Differentiator differentiator;
MovingAverageFilter moving_avg_filter;
MovingAverageFilter moving_avg_bpm;
MovingAverageFilter moving_avg_spo2;
/* statistic Variables */
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;


/* finger Detection */
long finger_timestamp = 0;
bool finger_detected = false;


//some variables for FatFs
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

TaskHandle_t xTemperatureTaskHandle = NULL;

SemaphoreHandle_t xI2CSemaphore;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint32_t redLED;
    uint32_t irLED;
} LEDOutput;

typedef struct {
    int bpm;
    float spo2;
} FilterOutput;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// interrupt callback for PA1 external interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (GPIO_Pin == GPIO_PIN_1) {
//    	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
//    	myprintf("Interrupt has been triggered\r\n");
    	vTaskNotifyGiveFromISR(xTemperatureTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

QueueHandle_t xQueueLEDOutput;
QueueHandle_t xQueueFilterOutput;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, -1);

}

void vTaskLEDInput(void *pvParameters) {
    uint32_t redLED, irLED;

    for (;;)
    {
    	xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
        MAX30102_ReadRawData(&redLED, &irLED, &hi2c1, &huart2);
        xSemaphoreGive(xI2CSemaphore);

        LEDOutput sensorData;
        sensorData.redLED = redLED;
        sensorData.irLED = irLED;

//    	myprintf("Inside sensing task\r\n");
        xQueueSend(xQueueLEDOutput, &sensorData, portMAX_DELAY);
//    	myprintf("Exiting loop\r\n");
    }
}

void vTaskFilter(void *pvParameters) {
    LEDOutput sensorData;
    FilterOutput processedData;
//    long counter = 0;

    for(;;)
    {
        if(xQueueReceive(xQueueLEDOutput, &sensorData, portMAX_DELAY) == pdPASS)
        {
//        	myprintf("Received readings from sensing task\r\n");
            float current_value_red = (float)sensorData.redLED;
			float current_value_ir = (float)sensorData.irLED;

            if (current_value_red > kFingerThreshold) {
                if (xTaskGetTickCount() - finger_timestamp > pdMS_TO_TICKS(kFingerCooldownMs)) {
                    finger_detected = true;
                }
            } else {
                // Reset all values if the finger is removed
                if (finger_detected && (xTaskGetTickCount() - finger_timestamp > pdMS_TO_TICKS(1000))) {
                    Differentiator_Reset(&differentiator);
                    MovingAverageFilter_Reset(&moving_avg_bpm);
                    MovingAverageFilter_Reset(&moving_avg_spo2);
                    LowPassFilter_Reset(&low_pass_filter_red);
                    LowPassFilter_Reset(&low_pass_filter_ir);
                    HighPassFilter_Reset(&high_pass_filter);
                    MinMaxAvgStatistic_Reset(&stat_red);
                    MinMaxAvgStatistic_Reset(&stat_ir);

                    finger_detected = false;
                    finger_timestamp = xTaskGetTickCount();
                }
            }

            if (finger_detected) {
//            	if(counter % 100 == 0) myprintf("Finger has been detected\r\n");
//            	counter++;

                // Apply Filters to Red and IR signals
                current_value_red = LowPassFilter_Process(&low_pass_filter_red, current_value_red);
                current_value_ir = LowPassFilter_Process(&low_pass_filter_ir, current_value_ir);

                // Update statistics for both Red and IR signals
                MinMaxAvgStatistic_Process(&stat_red, current_value_red);
                MinMaxAvgStatistic_Process(&stat_ir, current_value_ir);

                // Apply High Pass Filter for heartbeat detection on the Red LED
                float current_value = HighPassFilter_Process(&high_pass_filter, current_value_red);
                float current_diff = Differentiator_Process(&differentiator, current_value);

                if (!isnan(current_diff) && !isnan(last_diff)) {
                    // Detect heartbeat via zero-crossing
                    if (last_diff > 0 && current_diff < 0) {
                        crossed = true;
                        crossed_time = xTaskGetTickCount();
                    }

                    if (crossed && current_diff < kEdgeThreshold) {
                        if (lastHeartbeat != 0 && crossed_time - lastHeartbeat > 300) {
                        	myprintf("Inside time threshold\r\n");

                            // Calculate heart rate (BPM)
                            int bpm = 60000 / (crossed_time - lastHeartbeat);

                            // Calculate R-values for SpO2 calculation
                            float rred = (MinMaxAvgStatistic_Maximum(&stat_red) - MinMaxAvgStatistic_Minimum(&stat_red)) / MinMaxAvgStatistic_Average(&stat_red);
                            float rir = (MinMaxAvgStatistic_Maximum(&stat_ir) - MinMaxAvgStatistic_Minimum(&stat_ir)) / MinMaxAvgStatistic_Average(&stat_ir);
                            float spo2 = calculate_SpO2(rred, rir);

                            myprintf("Bpm: %i, Spo2 : %.2f\r\n", bpm, spo2);
                            if (bpm > 5 && bpm < 250) {
                                int avg_bpm = MovingAverageFilter_Process(&moving_avg_bpm, bpm);
                                int avg_spo2 = MovingAverageFilter_Process(&moving_avg_spo2, spo2);

                                // Prepare processed data
                                processedData.bpm = avg_bpm;
                                processedData.spo2 = avg_spo2;

                                // Send processed data to vTaskWrite
                                xQueueSend(xQueueFilterOutput, &processedData, portMAX_DELAY);
                            }
                        }
                        crossed = false;
                        lastHeartbeat = crossed_time;
                    }
                }
                last_diff = current_diff;
            }
        }
    }
}

void vTaskWrite(void *pvParameters) {
    FilterOutput processedData;
	char uartBuf[100];  // buffer for  messages

    for(;;)
    {
		myprintf("Inside sd card writing task\r\n");
        if(xQueueReceive(xQueueFilterOutput, &processedData, portMAX_DELAY) == pdPASS)
        {
        	myprintf("Received processed data from queue\r\n");
            sprintf(uartBuf, "Heart Rate: %d bpm, SpO2: %.2f%%\r\n", processedData.bpm, (double)processedData.spo2);
            myprintf(uartBuf);

            FRESULT fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_APPEND);
            if(fres == FR_OK) {
                myprintf("I was able to open 'write.txt' for measurements\r\n");
            } else {
                myprintf("f_open error (%i)\r\n", fres);
            }

            UINT bytesWrote;
            fres = f_write(&fil, uartBuf, strlen(uartBuf), &bytesWrote);
            if(fres == FR_OK) {
                myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
            } else {
                myprintf("f_write error (%i)\r\n");
            }

            f_close(&fil);
        }
    }
}

void vTaskTemperature(void *pvParameters) {
    char uartBuf[100];  // buffer for  messages

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        myprintf("Inside temperature task\r\n");

        uint8_t intStatus = 0;
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
        HAL_I2C_Mem_Read(&hi2c1, MAX30102_ADDRESS << 1, MAX30105_INTSTAT2, I2C_MEMADD_SIZE_8BIT, &intStatus, 1, HAL_MAX_DELAY);
        xSemaphoreGive(xI2CSemaphore);

        if (intStatus & MAX30105_INT_DIE_TEMP_RDY_ENABLE) {  // if temperature ready
        	xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
            float temperature = MAX30102_ReadTemperature(&hi2c1);  // read temperature
            xSemaphoreGive(xI2CSemaphore);
            sprintf(uartBuf, "Temperature: %.2f°C\r\n", temperature);

            myprintf(uartBuf);

            FRESULT fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_APPEND);
            if(fres == FR_OK) {
                myprintf("I was able to open 'write.txt' for measurements\r\n");
            } else {
                myprintf("f_open error (%i)\r\n", fres);
            }

            UINT bytesWrote;
            fres = f_write(&fil, uartBuf, strlen(uartBuf), &bytesWrote);
            if(fres == FR_OK) {
                myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
            } else {
                myprintf("f_write error (%i)\r\n");
            }

            f_close(&fil);
        }

        // new temperature measurement
        xSemaphoreTake(xI2CSemaphore, portMAX_DELAY);
        MAX30102_StartTemperatureMeasurement(&hi2c1);
        xSemaphoreGive(xI2CSemaphore);
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    // Initialize the MAX30102 sensor
	MAX30102_Init(&hi2c1);
    // Initialize Filters
	initialize_filters();

	// enable the temperature interrupt on MAX30102
	MAX30102_EnableTemperatureInterrupt(&hi2c1,&huart2);

    // start the first temperature measurement
    MAX30102_StartTemperatureMeasurement(&hi2c1);

  	// SD Card funtionality
  myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

  HAL_Delay(1000); //a short delay is important to let the SD card settle

  myprintf("HAL Delay print statement\r\n");

  //Open the file system
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
	myprintf("f_mount error (%i)\r\n", fres);
	while(1);
  }

  //Let's get some statistics from the SD card
  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
	myprintf("f_getfree error (%i)\r\n", fres);
	while(1);
  }

  //Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  //Now let's try to open file "test.txt"
  fres = f_open(&fil, "test.txt", FA_READ);
  if (fres != FR_OK) {
	myprintf("f_open error (%i)\r\n");
	while(1);
  }
  myprintf("I was able to open 'test.txt' for reading!\r\n");

  //Read 30 bytes from "test.txt" on the SD card
  BYTE readBuf[30];

  //We can either use f_read OR f_gets to get data out of files
  //f_gets is a wrapper on f_read that does some string formatting for us
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if(rres != 0) {
	myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  } else {
	myprintf("f_gets error (%i)\r\n", fres);
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  //Now let's try and write a file "write.txt"
  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(fres == FR_OK) {
	myprintf("I was able to open 'write.txt' for writing\r\n");
  } else {
	myprintf("f_open error (%i)\r\n", fres);
  }

  //Copy in a string
  strncpy((char*)readBuf, "a new file is made!\r\n", strlen("a new file is made!\r\n"));
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if(fres == FR_OK) {
	myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
	myprintf("f_write error (%i)\r\n");
  }

  //Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

// //We're done, so de-mount the drive
// f_mount(NULL, "", 0);


	// Initialize semaphore
	xI2CSemaphore = xSemaphoreCreateMutex();
	if(xI2CSemaphore == NULL) {
		myprintf("Semaphore was not created\r\n");
		while(1);
	}

 	 // Create the queues
    xQueueLEDOutput = xQueueCreate(100, sizeof(LEDOutput));
    xQueueFilterOutput = xQueueCreate(100, sizeof(FilterOutput));

    BaseType_t created;

    // Create the tasks
    created = xTaskCreate(vTaskLEDInput, "vTaskLEDInput", 128, NULL, 2, NULL);
    if(created != pdPASS) {
		myprintf("Input task was not created\r\n");
		while(1);
	}
    created = xTaskCreate(vTaskFilter, "vTaskFilter", 256, NULL, 2, NULL);
    if(created != pdPASS) {
		myprintf("Filtering task was not created\r\n");
		while(1);
	}
    created = xTaskCreate(vTaskWrite, "vTaskWrite", 256, NULL, 2, NULL);
    if(created != pdPASS) {
    	myprintf("Writing task was not created\r\n");
    	while(1);
    }
    created = xTaskCreate(vTaskTemperature, "vTaskTemperature", 256, NULL, 2, &xTemperatureTaskHandle);
    if(created != pdPASS) {
    	myprintf("Temperature task was not created\r\n");
    	while(1);
    }
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000608;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
void StartDefaultTask(void const * argument)
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

#ifdef  USE_FULL_ASSERT
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
