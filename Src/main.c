/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/IKS01A1/DataLogTerminal/Src/main.c
 * @author  CL
 * @version V4.0.0
 * @date    1-May-2017
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* snprintf */
#include <math.h>   /* trunc */
#include "main.h"
#include "pedometer.h"
/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup DATALOG_TERMINAL
  * @{
  */

extern int use_LSI;
int RTC_SYNCH_PREDIV;

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t acquire_data_enable_request  = 1;
static volatile uint8_t acquire_data_disable_request = 0;
static volatile uint8_t no_X_LSM6DS3_DIL24 = 0;
static volatile uint8_t no_G_LSM6DS3_DIL24 = 0;
static volatile uint8_t no_P_LPS25HB_DIL24 = 0;
static volatile uint8_t no_T_LPS25HB_DIL24 = 0;
static volatile uint8_t no_P_LPS22HB_DIL24 = 0;
static volatile uint8_t no_T_LPS22HB_DIL24 = 0;

static uint8_t acquire_data_enabled = 0;
static uint8_t verbose              = 1;  /* Verbose output to UART terminal ON/OFF. */
static RTC_HandleTypeDef RtcHandle;
static char dataOut[MAX_BUF_SIZE];

static void *LSM6DS0_X_0_handle = NULL;
// static void *LSM6DS3_X_0_handle = NULL;
static void *LSM6DS0_G_0_handle = NULL;
// static void *LSM6DS3_G_0_handle = NULL;
// static void *LIS3MDL_0_handle   = NULL;
// static void *HTS221_H_0_handle  = NULL;
// static void *HTS221_T_0_handle  = NULL;
// static void *LPS25HB_P_0_handle  = NULL;
// static void *LPS25HB_T_0_handle  = NULL;
// static void *LPS25HB_P_1_handle  = NULL;  /* LPS25H/B pressure sensor via DIL24 */
// static void *LPS25HB_T_1_handle  = NULL;  /* LPS25H/B temperature sensor via DIL24 */
// static void *LPS22HB_P_0_handle  = NULL;
// static void *LPS22HB_T_0_handle  = NULL;

AccVector acc;
Acc data;
FilterAccBuffer coord_data;
float processed_data[BUFFER_SIZE];
int steps=0;
float max,min;

/* Private function prototypes -----------------------------------------------*/

static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void disableAllSensors( void );
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
static void RTC_Handler( void );

static void Accelero_Sensor_Handler( void *handle );
static void Gyro_Sensor_Handler( void *handle );
static void Magneto_Sensor_Handler( void *handle );
static void Humidity_Sensor_Handler( void *handle );
static void Temperature_Sensor_Handler( void *handle );
static void Pressure_Sensor_Handler( void *handle );



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure.
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope, and Magnetometer.
 * @param  None
 * @retval Integer
 */
int main( void )
{

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED */
  BSP_LED_Init( LED2 );

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize UART */
  USARTConfig();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  initializeAllSensors();

  while (1)
  {

    if ( acquire_data_enable_request == 1 )
    {
      enableAllSensors();
      acquire_data_enabled = 1;
      acquire_data_enable_request = 0;
    }

    if ( acquire_data_disable_request == 1 )
    {
      disableAllSensors();
      acquire_data_enabled = 0;
      acquire_data_disable_request = 0;
    }

    if ( acquire_data_enabled == 1 )
    {

      /* Perform all handlers */
      RTC_Handler();
      Accelero_Sensor_Handler( LSM6DS0_X_0_handle );
      // if(!no_X_LSM6DS3_DIL24)
      // {
      //   Accelero_Sensor_Handler( LSM6DS3_X_0_handle );
      // }
      // Gyro_Sensor_Handler( LSM6DS0_G_0_handle );
      // if(!no_G_LSM6DS3_DIL24)
      // {
      //   Gyro_Sensor_Handler( LSM6DS3_G_0_handle );
      // }
      // Magneto_Sensor_Handler( LIS3MDL_0_handle );
      // Humidity_Sensor_Handler( HTS221_H_0_handle );
      // Temperature_Sensor_Handler( HTS221_T_0_handle );
      // Temperature_Sensor_Handler( LPS25HB_T_0_handle );
      // if(!no_T_LPS22HB_DIL24)
      // {
      //   Temperature_Sensor_Handler( LPS22HB_T_0_handle );
      // }
      // if(!no_T_LPS25HB_DIL24)
      // {
      //   Temperature_Sensor_Handler( LPS25HB_T_1_handle );
      // }
      // Pressure_Sensor_Handler( LPS25HB_P_0_handle );
      // if(!no_P_LPS22HB_DIL24)
      // {
      //   Pressure_Sensor_Handler( LPS22HB_P_0_handle );
      // }
      // if(!no_P_LPS25HB_DIL24)
      // {
      //   Pressure_Sensor_Handler( LPS25HB_P_1_handle );
      // }
    }

    HAL_Delay( 40 );
  }
}



/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }else
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
static void RTC_Handler( void )
{

  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  HAL_RTC_GetTime( &RtcHandle, &stimestructure, FORMAT_BIN );
  HAL_RTC_GetDate( &RtcHandle, &sdatestructureget, FORMAT_BIN );
  subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) /
             ( RTC_SYNCH_PREDIV + 1 )) & 0xff );

  // snprintf( dataOut, MAX_BUF_SIZE, "\r\n\r\n\r\nTimeStamp: %02d:%02d:%02d.%02d\r\n", stimestructure.Hours, stimestructure.Minutes,
  //          stimestructure.Seconds, subSec );
  snprintf( dataOut, MAX_BUF_SIZE, "TimeStamp: %02d:%02d:%02d.%02d\t", stimestructure.Hours, stimestructure.Minutes,
           stimestructure.Seconds, subSec );
  HAL_UART_Transmit( &UartHandle, ( uint8_t *)dataOut, strlen( dataOut ), 5000 );
}


/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Accelero_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t acceleration;
  uint8_t status;
  displayFloatToInt_t out_value;

  // For pedometer
  AccVector data_in;
  
  
  BSP_ACCELERO_Get_Instance( handle, &id );

  BSP_ACCELERO_IsInitialized( handle, &status );


  if ( status == 1 )
  {
    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
      acceleration.AXIS_X = 0;
      acceleration.AXIS_Y = 0;
      acceleration.AXIS_Z = 0;
    }

  // For pedometer
    data_in.AccX = (float)acceleration.AXIS_X / 1000.0f;
    data_in.AccY = (float)acceleration.AXIS_Y / 1000.0f;
    data_in.AccZ = (float)acceleration.AXIS_Z / 1000.0f;
    processed_data[2] = processed_data[1]; 
    processed_data[1] = processed_data[0];   
    pedometer_update(data_in, &data, &coord_data, processed_data);
    measure_steps(&steps,processed_data,&max,&min);
    // snprintf( dataOut, MAX_BUF_SIZE, "%d\t%5f\t%5f\t%5f\t",(int)1, data_in.AccX,
    //          data_in.AccY, data_in.AccZ );

    // HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    // snprintf( dataOut, MAX_BUF_SIZE, "%d\t%5f\t%5f\t%5f\t", 2, data.grav_data.AccX,
    //         data.grav_data.AccY, data.grav_data.AccZ );

    // HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
		
    snprintf( dataOut, MAX_BUF_SIZE, "%d\t%5f\t%5f\t%5f\t%5f\t%5f\t%d\t%5f\t%5f\r\n", 4, coord_data.lp_dot_data.unfiltered[0], coord_data.lp_dot_data.filtered[0],processed_data[0],processed_data[1],processed_data[2],steps,max,min );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
		// snprintf( dataOut, MAX_BUF_SIZE, "%d\t%5f\t%5f\t%5f\r\n", 5, data.user_data.AccX,
    //          data.user_data.AccY, data.user_data.AccZ );

    // HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    // snprintf( dataOut, MAX_BUF_SIZE, "\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n", (int)id, (int)acceleration.AXIS_X, (int)id,
    //          (int)acceleration.AXIS_Y, (int)id, (int)acceleration.AXIS_Z );

    // HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    // if ( verbose == 1 )
    // {
    //   if ( BSP_ACCELERO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
    //   {
    //     snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
    //   }
    //   else
    //   {
    //     snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
    //   }

    //   HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    //   if ( BSP_ACCELERO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
    //   {
    //     snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
    //   }
    //   else
    //   {
    //     floatToInt( odr, &out_value, 3 );
    //     snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
    //   }

    //   HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    //   if ( BSP_ACCELERO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
    //   {
    //     snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
    //   }
    //   else
    //   {
    //     floatToInt( fullScale, &out_value, 3 );
    //     snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d g\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
    //   }

    //   HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    // }
  }
}



/**
 * @brief  Handles the gyroscope axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Gyro_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t angular_velocity;
  uint8_t status;
  displayFloatToInt_t out_value;

  BSP_GYRO_Get_Instance( handle, &id );

  BSP_GYRO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }

    snprintf( dataOut, MAX_BUF_SIZE, "\r\nGYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n", (int)id, (int)angular_velocity.AXIS_X, (int)id,
             (int)angular_velocity.AXIS_Y, (int)id, (int)angular_velocity.AXIS_Z );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_GYRO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_GYRO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_GYRO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( fullScale, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d dps\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the magneto axes data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Magneto_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  SensorAxes_t magnetic_field;
  uint8_t status;
  displayFloatToInt_t out_value;

  BSP_MAGNETO_Get_Instance( handle, &id );

  BSP_MAGNETO_IsInitialized( handle, &status );

  if ( status == 1 )
  {
    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }

    snprintf( dataOut, MAX_BUF_SIZE, "\r\nMAG_X[%d]: %d, MAG_Y[%d]: %d, MAG_Z[%d]: %d\r\n", (int)id, (int)magnetic_field.AXIS_X, (int)id,
             (int)magnetic_field.AXIS_Y, (int)id, (int)magnetic_field.AXIS_Z );

    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_MAGNETO_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_MAGNETO_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_MAGNETO_Get_FS( handle, &fullScale ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( fullScale, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "FS[%d]: %d.%03d Gauss\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
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

    floatToInt( humidity, &out_value, 2 );
    snprintf( dataOut, MAX_BUF_SIZE, "\r\nHUM[%d]: %d.%02d\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

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

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_HUMIDITY_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
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

    floatToInt( temperature, &out_value, 2 );
    snprintf( dataOut, MAX_BUF_SIZE, "\r\nTEMP[%d]: %c%d.%02d\r\n", (int)id, ((out_value.sign) ? '-' : '+'), (int)out_value.out_int, (int)out_value.out_dec );
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

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

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_TEMPERATURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Handles the pressure sensor data getting/sending
 * @param  handle the device handle
 * @retval None
 */
static void Pressure_Sensor_Handler( void *handle )
{

  uint8_t who_am_i;
  float odr;
  uint8_t id;
  float pressure;
  uint8_t status;
  displayFloatToInt_t out_value;

  BSP_PRESSURE_Get_Instance( handle, &id );

  BSP_PRESSURE_IsInitialized( handle, &status );

  if( status == 1 )
  {
    if ( BSP_PRESSURE_Get_Press( handle, &pressure ) == COMPONENT_ERROR )
    {
      pressure = 0.0f;
    }

    floatToInt( pressure, &out_value, 2 );
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPRESS[%d]: %d.%02d\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec);
    HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

    if ( verbose == 1 )
    {
      if ( BSP_PRESSURE_Get_WhoAmI( handle, &who_am_i ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: ERROR\r\n", id );
      }
      else
      {
        snprintf( dataOut, MAX_BUF_SIZE, "WHO AM I address[%d]: 0x%02X\r\n", id, who_am_i );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );

      if ( BSP_PRESSURE_Get_ODR( handle, &odr ) == COMPONENT_ERROR )
      {
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", id );
      }
      else
      {
        floatToInt( odr, &out_value, 3 );
        snprintf( dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)id, (int)out_value.out_int, (int)out_value.out_dec );
      }

      HAL_UART_Transmit( &UartHandle, ( uint8_t * )dataOut, strlen( dataOut ), 5000 );
    }
  }
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{

  BSP_ACCELERO_Init( LSM6DS0_X_0, &LSM6DS0_X_0_handle );
  // if(BSP_ACCELERO_Init( LSM6DS3_X_0, &LSM6DS3_X_0_handle ) == COMPONENT_ERROR)
  // {
  //   no_X_LSM6DS3_DIL24 = 1;
  // }
  BSP_GYRO_Init( LSM6DS0_G_0, &LSM6DS0_G_0_handle );
  // if(BSP_GYRO_Init( LSM6DS3_G_0, &LSM6DS3_G_0_handle ) == COMPONENT_ERROR)
  // {
  //   no_G_LSM6DS3_DIL24 = 1;
  // }
  // BSP_MAGNETO_Init( LIS3MDL_0, &LIS3MDL_0_handle );
  // BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle );
  // BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle );
  // BSP_TEMPERATURE_Init( LPS25HB_T_0, &LPS25HB_T_0_handle );
  // if(BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ) == COMPONENT_ERROR)
  // {
  //   no_T_LPS22HB_DIL24 = 1;
  // }
  // if(BSP_TEMPERATURE_Init( LPS25HB_T_1, &LPS25HB_T_1_handle ) == COMPONENT_ERROR)
  // {
  //   no_T_LPS25HB_DIL24 = 1;
  // }
  // BSP_PRESSURE_Init( LPS25HB_P_0, &LPS25HB_P_0_handle );
  // if(BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle ) == COMPONENT_ERROR)
  // {
  //   no_P_LPS22HB_DIL24 = 1;
  // }
  // if(BSP_PRESSURE_Init( LPS25HB_P_1, &LPS25HB_P_1_handle ) == COMPONENT_ERROR)
  // {
  //   no_P_LPS25HB_DIL24 = 1;
  // }
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Enable( LSM6DS0_X_0_handle );
  // if(!no_X_LSM6DS3_DIL24)
  // {
  //   BSP_ACCELERO_Sensor_Enable( LSM6DS3_X_0_handle );
  // }
  // BSP_GYRO_Sensor_Enable( LSM6DS0_G_0_handle );
  // if(!no_G_LSM6DS3_DIL24)
  // {
  //   BSP_GYRO_Sensor_Enable( LSM6DS3_G_0_handle );
  // }
  // BSP_MAGNETO_Sensor_Enable( LIS3MDL_0_handle );
  // BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  // BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
  // BSP_TEMPERATURE_Sensor_Enable( LPS25HB_T_0_handle );
  // if(!no_T_LPS22HB_DIL24)
  // {
  //   BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
  // }
  // if(!no_T_LPS25HB_DIL24)
  // {
  //   BSP_TEMPERATURE_Sensor_Enable( LPS25HB_T_1_handle );
  // }
  // BSP_PRESSURE_Sensor_Enable( LPS25HB_P_0_handle );
  // if(!no_P_LPS22HB_DIL24)
  // {
  //   BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
  // }
  // if(!no_P_LPS25HB_DIL24)
  // {
  //   BSP_PRESSURE_Sensor_Enable( LPS25HB_P_1_handle );
  // }
}



/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void disableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Disable( LSM6DS0_X_0_handle );
  // if(!no_X_LSM6DS3_DIL24)
  // {
  //   BSP_ACCELERO_Sensor_Disable( LSM6DS3_X_0_handle );
  // }
  // BSP_GYRO_Sensor_Disable( LSM6DS0_G_0_handle );
  // if(!no_G_LSM6DS3_DIL24)
  // {
  //   BSP_GYRO_Sensor_Disable( LSM6DS3_G_0_handle );
  // }
  // BSP_MAGNETO_Sensor_Disable( LIS3MDL_0_handle );
  // BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  // BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
  // BSP_TEMPERATURE_Sensor_Disable( LPS25HB_T_0_handle );
  // if(!no_T_LPS22HB_DIL24)
  // {
  //   BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
  // }
  // if(!no_T_LPS25HB_DIL24)
  // {
  //   BSP_TEMPERATURE_Sensor_Disable( LPS25HB_T_1_handle );
  // }
  // BSP_PRESSURE_Sensor_Disable( LPS25HB_P_0_handle );
  // if(!no_P_LPS22HB_DIL24)
  // {
  //   BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
  // }
  // if(!no_P_LPS25HB_DIL24)
  // {
  //   BSP_PRESSURE_Sensor_Disable( LPS25HB_P_1_handle );
  // }
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config( void )
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef        RCC_OscInitStruct;

  /*##-1- Configue LSE as RTC clock soucre ###################################*/
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    use_LSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    use_LSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RTC_SYNCH_PREDIV = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig( void )
{

  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-3- Configure the Date using BCD format ################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure the Time using BCD format#################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
  {

    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  /* User button. */
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_RESET )
    {
      if ( acquire_data_enabled == 0 )
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



/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler( void )
{

  while (1)
  {}
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\r\n", file, line) */

  while (1)
  {}
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
