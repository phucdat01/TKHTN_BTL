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
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "lcd_con.h"
#include "sht3x.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* Definitions for Task1_SHT30 */
osThreadId_t Task1_SHT30Handle;
const osThreadAttr_t Task1_SHT30_attributes = {
  .name = "Task1_SHT30",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2_Soil */
osThreadId_t Task2_SoilHandle;
const osThreadAttr_t Task2_Soil_attributes = {
  .name = "Task2_Soil",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task3_Cmd */
osThreadId_t Task3_CmdHandle;
const osThreadAttr_t Task3_Cmd_attributes = {
  .name = "Task3_Cmd",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task4_LCD */
osThreadId_t Task4_LCDHandle;
const osThreadAttr_t Task4_LCD_attributes = {
  .name = "Task4_LCD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myUartMutex */
osMutexId_t myUartMutexHandle;
const osMutexAttr_t myUartMutex_attributes = {
  .name = "myUartMutex"
};
/* Definitions for semDataReady */
osSemaphoreId_t semDataReadyHandle;
const osSemaphoreAttr_t semDataReady_attributes = {
  .name = "semDataReady"
};
/* Definitions for semUartCmd */
osSemaphoreId_t semUartCmdHandle;
const osSemaphoreAttr_t semUartCmd_attributes = {
  .name = "semUartCmd"
};
/* USER CODE BEGIN PV */
// --- BIẾN TOÀN CỤC (Tên ngắn gọn theo yêu cầu) ---
float temp = 0.0f;  // Nhiệt độ từ SHT30
float soil = 0.0f;  // Độ ẩm đất từ ADC

// Buffer nhận UART
static char rx_buffer[50];
static uint8_t rx_byte;

sht3x_handle_t sht3x;

uint32_t cycle_SHT30 = 1000;
uint32_t cycle_Soil  = 2000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void StartTask1_SHT30(void *argument);
void StartTask2_soil(void *argument);
void StartTask3_Cmd(void *argument);
void StartTask4_LCD(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Hàm này được thư viện chuẩn C gọi mỗi khi bạn dùng printf
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{

	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
    return len;
}
#endif

float SoilMoisture_Convert(uint16_t adc_value) {
    if (adc_value > 2500) adc_value = 2500;
    if (adc_value < 830)  adc_value = 830;
    return (float)(2500 - adc_value) * 100.0f / (2500 - 830);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  sht3x.i2c_handle = &hi2c2;
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  lcd_init(&hi2c1);
  lcd_puts(&hi2c1, "System Init...");
  HAL_Delay(1000);
  lcd_clear(&hi2c1);

  setvbuf(stdout, NULL, _IONBF, 0);
  // Bật ngắt UART nhận
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myUartMutex */
  myUartMutexHandle = osMutexNew(&myUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semDataReady */
  semDataReadyHandle = osSemaphoreNew(1, 0, &semDataReady_attributes);

  /* creation of semUartCmd */
  semUartCmdHandle = osSemaphoreNew(1, 0, &semUartCmd_attributes);

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
  /* creation of Task1_SHT30 */
  Task1_SHT30Handle = osThreadNew(StartTask1_SHT30, NULL, &Task1_SHT30_attributes);

  /* creation of Task2_Soil */
  Task2_SoilHandle = osThreadNew(StartTask2_soil, NULL, &Task2_Soil_attributes);

  /* creation of Task3_Cmd */
  Task3_CmdHandle = osThreadNew(StartTask3_Cmd, NULL, &Task3_Cmd_attributes);

  /* creation of Task4_LCD */
  Task4_LCDHandle = osThreadNew(StartTask4_LCD, NULL, &Task4_LCD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Callback ngắt UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t i = 0;
  if (huart->Instance == USART1)
  {
    if (rx_byte == '!') {
        rx_buffer[i] = 0;
        osSemaphoreRelease(semUartCmdHandle);
        i = 0;
    } else {
        if (i < 49) rx_buffer[i++] = rx_byte;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1_SHT30 */
/**
  * @brief  Function implementing the Task1_SHT30 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1_SHT30 */
void StartTask1_SHT30(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t tick = osKernelGetTickCount();
  float dummy_hum;
  /* Infinite loop */
  for(;;)
  {
    sht3x_read_temperature_and_humidity(&sht3x, &temp, &dummy_hum);
	osSemaphoreRelease(semDataReadyHandle);
	if (osMutexAcquire(myUartMutexHandle, osWaitForever) == osOK) {
	    printf("Task 1 in: %lu\n", osKernelGetTickCount());
	    printf("Data SHT30: Temp: %.1f C\n\n", temp);
	    osMutexRelease(myUartMutexHandle);
	}
	tick += cycle_SHT30;
	osDelayUntil(tick);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask2_soil */
/**
* @brief Function implementing the Task2_Soil thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask2_soil */
void StartTask2_soil(void *argument)
{
  /* USER CODE BEGIN StartTask2_soil */
  uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	   soil = SoilMoisture_Convert(HAL_ADC_GetValue(&hadc1));
	}
	HAL_ADC_Stop(&hadc1);
	osSemaphoreRelease(semDataReadyHandle);
	if (osMutexAcquire(myUartMutexHandle, osWaitForever) == osOK) {
	    printf("Task 2 in: %lu\n", osKernelGetTickCount());
	    printf("Data Soil: %.1f %%\n\n", soil);
	    osMutexRelease(myUartMutexHandle);
	}
	tick += cycle_Soil;
	osDelayUntil(tick);
  }
  /* USER CODE END StartTask2_soil */
}

/* USER CODE BEGIN Header_StartTask3_Cmd */
/**
* @brief Function implementing the Task3_Cmd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask3_Cmd */
void StartTask3_Cmd(void *argument)
{
  /* USER CODE BEGIN StartTask3_Cmd */
  int new_value = 0; // Biến tạm để lưu số tách được
  /* Infinite loop */
  for(;;)
  {
	  // Chờ lệnh từ UART (Event-Driven)
	  if (osSemaphoreAcquire(semUartCmdHandle, osWaitForever) == osOK) {
		  osMutexAcquire(myUartMutexHandle, osWaitForever);
	      // --- 1. Xử lý lệnh chỉnh tốc độ SHT30 ---
	      // Cú pháp: RATE_TEMP:xxxx (Ví dụ: RATE_TEMP:2000)
	      if (strstr(rx_buffer, "T:") != NULL) {

	          // Lấy số sau dấu hai chấm bỏ vào biến new_value
	          sscanf(rx_buffer, "T:%d", &new_value);

	          // Bảo vệ: Không cho đặt nhanh quá (nhỏ hơn 100ms) gây treo
	          if (new_value >= 100) {
	              cycle_SHT30 = new_value;
	              printf("\r\nOK -> Temp sampling rate changed to: %lu ms\r\n", cycle_SHT30);
	          }
	      }

	      // --- 2. Xử lý lệnh chỉnh tốc độ Soil ---
	      // Cú pháp: RATE_SOIL:xxxx (Ví dụ: RATE_SOIL:500)
	      else if (strstr(rx_buffer, "S:") != NULL) {

	           sscanf(rx_buffer, "S:%d", &new_value);

	           if (new_value >= 100) {
	               cycle_Soil = new_value;
	               printf("\r\nOK -> Soil sampling rate changed to: %lu ms\r\n", cycle_Soil);
	           }
	      }

	      // --- 3. Lệnh không hợp lệ ---
	      else {
	            // Chỉ báo lỗi nếu chuỗi không rỗng (tránh in rác)
	            if(strlen(rx_buffer) > 1) {
	               printf("ERROR: Unknown command!\r\n");
	            }
	      }
	      osMutexRelease(myUartMutexHandle);
	      // Xóa sạch buffer sau khi xử lý xong
	      memset(rx_buffer, 0, sizeof(rx_buffer));
	  }
  }
  /* USER CODE END StartTask3_Cmd */
}

/* USER CODE BEGIN Header_StartTask4_LCD */
/**
* @brief Function implementing the Task4_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask4_LCD */
void StartTask4_LCD(void *argument)
{
  /* USER CODE BEGIN StartTask4_LCD */
  char lcd_buf[20];
  /* Infinite loop */
  for(;;)
  {
	  // Dùng Semaphore để chờ dữ liệu mới từ sensor
	  if (osSemaphoreAcquire(semDataReadyHandle, osWaitForever) == osOK) {
	      sprintf(lcd_buf, "Temp: %.1f C     ", temp);
	      lcd_gotoxy(&hi2c1, 0, 0);
	      lcd_puts(&hi2c1, lcd_buf);

	      sprintf(lcd_buf, "Soil: %.1f %%     ", soil);
	      lcd_gotoxy(&hi2c1, 0, 1);
	      lcd_puts(&hi2c1, lcd_buf);
	  }
  }
  /* USER CODE END StartTask4_LCD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
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
