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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_con.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sht3x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_DURATION_MS   100
#define RX_BUFFER_SIZE      50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

sht3x_handle_t sht3x;
float temp = 0.0f;
float soil = 0.0f;

uint32_t frame_idx = 0;

uint32_t major_cycle_ms = 2000; // Mặc định 2000ms
uint32_t total_frames = 20;     // Mặc định 20 frames

uint8_t rx_byte_irq;
char cmd_buffer[RX_BUFFER_SIZE];
uint8_t rx_indx = 0;
bool is_system_running = false;

volatile uint8_t timer_flag = 0;
volatile uint8_t cmd_idx = 0;
volatile uint8_t cmd_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// --- KHAI BÁO TÊN TASK ---
void Task_SHT30(void);
void Task_Soil(void);
void Task_Cmd(void);
void Task_LCD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100); return len;
}
#endif

float SoilMoisture_Convert(uint16_t adc_value) {
    if (adc_value > 2370) adc_value = 2370;
    if (adc_value < 470)  adc_value = 470;
    return (float)(2370 - adc_value) * 100.0f / (2370 - 470);
}

/* 1. Task SHT30 */
void Task_SHT30(void) {
    uint32_t t_start = HAL_GetTick(); // Bắt đầu đo thời gian thực thi
    float dummy;
    sht3x_read_temperature_and_humidity(&sht3x, &temp, &dummy);
    printf("Task 1 in: %lu\r\n", t_start);
    printf("Data SHT30: Temp: %.1f C\r\n", temp);
    uint32_t t_exec = HAL_GetTick() - t_start;
    printf("[EXEC] Task_SHT30 time: %lu ms\r\n\r\n", t_exec);
}

/* 2. Task Soil */
void Task_Soil(void) {
    uint32_t t_start = HAL_GetTick();
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        soil = SoilMoisture_Convert(HAL_ADC_GetValue(&hadc1));
    HAL_ADC_Stop(&hadc1);
    printf("Task 2 in: %lu\r\n", t_start);
    printf("Data Soil: %.1f %%\r\n", soil);
    uint32_t t_exec = HAL_GetTick() - t_start;
    printf("[EXEC] Task_Soil time: %lu ms\r\n\r\n", t_exec);
}

/* 3. Task LCD */
void Task_LCD(void) {
    uint32_t t_start = HAL_GetTick();
    char LcdBuf[20];
    snprintf(LcdBuf, 20, "Temp: %.1f C    ", temp);
    lcd_gotoxy(&hi2c1, 0, 0); lcd_puts(&hi2c1, LcdBuf);
    snprintf(LcdBuf, 20, "Soil: %.1f %%    ", soil);
    lcd_gotoxy(&hi2c1, 0, 1); lcd_puts(&hi2c1, LcdBuf);
    printf("Task 3 in: %lu\r\n", t_start); // Thêm dòng này cho đồng bộ
    uint32_t t_exec = HAL_GetTick() - t_start;
    printf("[EXEC] Task_LCD time: %lu ms\r\n\r\n", t_exec);
}

void Process_Command(void) {
    // 1. START
    if (cmd_buffer[0] == '0') {
        if (!is_system_running) {
            is_system_running = true;
            printf("\r\n>>> SYSTEM STARTED <<<\r\n");
            lcd_clear(&hi2c1);
        }
    }
    // 2. STOP
    else if (cmd_buffer[0] == '1') {
        if (is_system_running) {
            is_system_running = false;
            printf("\r\n>>> SYSTEM STOPPED <<<\r\n");
            lcd_clear(&hi2c1);
            lcd_puts(&hi2c1, "STOPPED");
        }
    }
    // 3. CHANGE CYCLE (C:xxxx)
    else if (cmd_buffer[0] == 'C' && cmd_buffer[1] == ':') {
        int new_cycle = atoi(&cmd_buffer[2]);
        if (new_cycle >= 1600) {
            major_cycle_ms = new_cycle;
            total_frames = major_cycle_ms / FRAME_DURATION_MS;
            frame_idx = 0; // Reset vòng lặp
            printf("\r\n>>> OK: New Cycle = %lu ms (%lu frames) <<<\r\n", major_cycle_ms, total_frames);
        } else {
            printf("\r\n>>> ERROR: Min cycle is 1600ms <<<\r\n");
        }
    }
    else {
        printf("\r\n>>> Unknown Command: %s <<<\r\n", cmd_buffer);
    }

    // Xóa buffer
    memset(cmd_buffer, 0, RX_BUFFER_SIZE);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  sht3x.i2c_handle = &hi2c2;
  sht3x.device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW;
  sht3x_init(&sht3x);

  lcd_init(&hi2c1);          // Khởi tạo LCD qua I2C1
  lcd_puts(&hi2c1, "System Init...");
  HAL_Delay(1000);
  lcd_clear(&hi2c1);
  printf("Init Done. Send '0' to start, '1' to stop.\r\n");

  HAL_UART_Receive_IT(&huart1, &rx_byte_irq, 1);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   if (timer_flag == 1) {
	       timer_flag = 0;

	       if (cmd_flag == 1) {
	           cmd_flag = 0;
	           Process_Command();
	       }

	       if (is_system_running) {
	           switch (frame_idx) {
	                case 0:  Task_SHT30(); break;
	                case 5:  Task_Soil();  break;
	                case 10: Task_SHT30(); break;
	                case 15: Task_LCD();   break;
	                default: break;
	           }
	           frame_idx++;
	           if (frame_idx >= total_frames) frame_idx = 0;
	        }

	        if (timer_flag == 1) {
	            printf("ERROR: Overrun! Task took > 100ms\r\n");
	        }
	        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	   }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Kiểm tra xem có phải Timer 2 ngắt không
  if (htim->Instance == TIM2) {
    timer_flag = 1; // Bật cờ báo hiệu đã hết 100ms
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {

    // [FIX LỖI] Chỉ xử lý '0', '1' nhanh nếu nó là ký tự đầu tiên
    if ((rx_byte_irq == '0' || rx_byte_irq == '1') && cmd_idx == 0) {
        cmd_buffer[0] = rx_byte_irq;
        cmd_buffer[1] = '\0';
        cmd_flag = 1;
        cmd_idx = 0;
    }
    // Kiểm tra ký tự kết thúc lệnh
    else if (rx_byte_irq == '\n' || rx_byte_irq == '\r' || rx_byte_irq == '!') {
        cmd_buffer[cmd_idx] = '\0'; // Kết thúc chuỗi
        cmd_flag = 1;
        cmd_idx = 0; // Reset index cho lệnh sau
    }
    else {
        // Nhận dữ liệu bình thường (bao gồm cả số 0 trong 5000)
        if (cmd_idx < (RX_BUFFER_SIZE - 1)) {
            cmd_buffer[cmd_idx++] = rx_byte_irq;
        } else {
            cmd_idx = 0; // Tràn buffer thì reset
        }
    }

    // Bật lại ngắt
    HAL_UART_Receive_IT(&huart1, &rx_byte_irq, 1);
  }
}

/* USER CODE END 4 */

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
