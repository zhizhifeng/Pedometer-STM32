/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> /* strlen */
#include <stdio.h>  /* snprintf */
#include <math.h>   /* trunc */
#include "pedometer.h"
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct displayFloatToInt_s
{
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int RTC_SYNCH_PREDIV = 255;
static volatile uint8_t acquire_data_enable_request = 1;
static volatile uint8_t acquire_data_disable_request = 0;

static uint8_t acquire_data_enabled = 0;
static uint8_t verbose = 0; /* Verbose output to UART terminal ON/OFF. */

static char dataOut[MAX_BUF_SIZE];

static void *LSM6DS0_X_0_handle = NULL;
static void *HTS221_H_0_handle  = NULL;
static void *HTS221_T_0_handle  = NULL;

AccVector acc;
Acc data;
FilterAccBuffer coord_data;
StepDetectHandler hdetect;
int step_temp[2];
float temperature_temp[2];
float humidity_temp[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void initializeAllSensors(void);
static void enableAllSensors(void);
static void disableAllSensors(void);
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
// static void RTC_Handler( void );

static void Accelero_Sensor_Handler(void *handle);
static void Humidity_Sensor_Handler( void *handle );
static void Temperature_Sensor_Handler( void *handle );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  initializeAllSensors();
  Display_Page_Initialize();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (acquire_data_enable_request == 1)
    {
      enableAllSensors();
      acquire_data_enabled = 1;
      acquire_data_enable_request = 0;
    }

    if (acquire_data_disable_request == 1)
    {
      disableAllSensors();
      acquire_data_enabled = 0;
      acquire_data_disable_request = 0;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
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
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if (in >= 0.0f)
  {
    out_value->sign = 0;
  }
  else
  {
    out_value->sign = 1;
    in = -in;
  }

  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}
/**
 * @brief  Handles the time+date getting/sending
 * @param  None
 * @retval None
 */
static void RTC_Handler(void)
{

  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime(&hrtc, &stimestructure, FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, FORMAT_BIN);
  subSec = ((((((int)RTC_SYNCH_PREDIV) - ((int)stimestructure.SubSeconds)) * 100) /
             (RTC_SYNCH_PREDIV + 1)) &
            0xff);

  snprintf(dataOut, MAX_BUF_SIZE, "TimeStamp: %02d:%02d:%02d.%02d ", stimestructure.Hours, stimestructure.Minutes,
           stimestructure.Seconds, subSec);
  // snprintf( dataOut, MAX_BUF_SIZE, "\r\n\r\n\r\nTimeStamp: %02d:%02d:%02d.%02d\r\n", stimestructure.Hours, stimestructure.Minutes,
  //          stimestructure.Seconds, subSec );

  HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
}

/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Accelero_Sensor_Handler(void *handle)
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;
  displayFloatToInt_t out_value;
  AccVector data_in;  // For pedometer

  BSP_ACCELERO_Get_Instance(handle, &id);

  BSP_ACCELERO_IsInitialized(handle, &status);

  if (status == 1)
  {
    if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR)
    {
      acceleration.AXIS_X = 0;
      acceleration.AXIS_Y = 0;
      acceleration.AXIS_Z = 0;
    }

    // For pedometer
    data_in.AccX = (float)acceleration.AXIS_X / 1000.0f;
    data_in.AccY = (float)acceleration.AXIS_Y / 1000.0f;
    data_in.AccZ = (float)acceleration.AXIS_Z / 1000.0f;
    pedometer_update(data_in, &data, &coord_data, &hdetect);
    step_temp[0] = hdetect.step;

    // snprintf( dataOut, MAX_BUF_SIZE, "\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)id, (int)acceleration.AXIS_X, (int)id,
    //          (int)acceleration.AXIS_Y, (int)id, (int)acceleration.AXIS_Z );

    // HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if (verbose == 1)
    {
      if (BSP_ACCELERO_Get_WhoAmI(handle, &who_am_i) == COMPONENT_ERROR)
      {
        snprintf(dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id);
      }
      else
      {
        snprintf(dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i);
      }

      HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);

      if (BSP_ACCELERO_Get_ODR(handle, &odr) == COMPONENT_ERROR)
      {
        snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id);
      }
      else
      {
        floatToInt(odr, &out_value, 3);
        snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec);
      }

      HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);

      if (BSP_ACCELERO_Get_FS(handle, &fullScale) == COMPONENT_ERROR)
      {
        snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id);
      }
      else
      {
        floatToInt(fullScale, &out_value, 3);
        snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d g\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec);
      }

      HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
    }
  }
}
/**
 * @brief  Handles the humidity data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Humidity_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float humidity;
  uint8_t status;
  displayFloatToInt_t out_value;

  BSP_HUMIDITY_Get_Instance( handle, &id );

  BSP_HUMIDITY_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_HUMIDITY_Get_Hum( handle, &humidity ) == COMPONENT_ERROR )
    {
      humidity = 0.0f;
    }

    humidity_temp[0] = humidity;
    // floatToInt( humidity, &out_value, 2 );
    // snprintf( dataOut, MAX_BUF_SIZE, "\r\nHUM[%d]: %d.%02d\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
    // HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_HUMIDITY_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_HUMIDITY_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}
/**
 * @brief  Handles the temperature data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Temperature_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float temperature;
  uint8_t status;
  displayFloatToInt_t out_value;

  BSP_TEMPERATURE_Get_Instance( handle, &id );

  BSP_TEMPERATURE_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_TEMPERATURE_Get_Temp( handle, &temperature ) == COMPONENT_ERROR )
    {
      temperature = 0.0f;
    }

    temperature_temp[0] = temperature;
    // floatToInt( temperature, &out_value, 2 );
    // snprintf( dataOut, MAX_BUF_SIZE, "\r\nTEMP[%d]: %c%d.%02d\r\n", (int)id, ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec );
    // HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_TEMPERATURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_TEMPERATURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}
/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void)
{

  BSP_ACCELERO_Init(LSM6DS0_X_0, &LSM6DS0_X_0_handle);
  BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle );
  BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle );
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors(void)
{

  BSP_ACCELERO_Sensor_Enable(LSM6DS0_X_0_handle);
  BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void disableAllSensors(void)
{

  BSP_ACCELERO_Sensor_Disable(LSM6DS0_X_0_handle);
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
}
/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* User button. */
  if (GPIO_Pin == KEY_BUTTON_PIN)
  {
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
    {
      if (acquire_data_enabled == 0)
      {
        acquire_data_enable_request = 1;
      }
      else
      {
        acquire_data_disable_request = 1;
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */

  // snprintf( dataOut, MAX_BUF_SIZE, "\r\nTime Interrupted!\r\n");
  // HAL_UART_Transmit( &huart2, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

  if(htim->Instance == TIM2){
    if(acquire_data_enabled == 1){
      Accelero_Sensor_Handler(LSM6DS0_X_0_handle);
      Humidity_Sensor_Handler( HTS221_H_0_handle );
      Temperature_Sensor_Handler( HTS221_T_0_handle );
      RTC_Handler(); //print time
      snprintf(dataOut, MAX_BUF_SIZE, "%d %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.2f %.2f\r\n",
        hdetect.step,
        data.acc_data.AccX, data.acc_data.AccY, data.acc_data.AccZ,
        data.grav_data.AccX, data.grav_data.AccY, data.grav_data.AccZ,
        coord_data.lp_dot_data.unfiltered[0], coord_data.lp_dot_data.filtered[0],
        temperature_temp[0], humidity_temp[0]);
      HAL_UART_Transmit(&huart2, (uint8_t *)dataOut, strlen(dataOut), 5000);
      if(step_temp[0] != step_temp[1]){
        Display_Step_Update(step_temp[0]);
        step_temp[1] = step_temp[0];
      }
      if(temperature_temp[0] != temperature_temp[1]){
        Display_Temperature_Update(temperature_temp[0]);
        temperature_temp[1] = temperature_temp[0];
      }
      if(humidity_temp[0] != humidity_temp[1]){
        Display_Humidity_Update(humidity_temp[0]);
        humidity_temp[1] = humidity_temp[0];
      }
    }
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
