/**
  ******************************************************************************
  * @file    motion_sm.h
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-November-2017
  * @brief   Header for motion_sm module
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
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_SM_H_
#define _MOTION_SM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_SM MOTION_SM
  * @{
  */

/** @defgroup MOTION_SM_Exported_Types MOTION_SM_Exported_Types
 * @{
 */

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MSM_NOSLEEP   = 0x00,
  MSM_SLEEP     = 0x01
} MSM_sleep_t;

typedef struct
{
  float AccX;           /* Acceleration in X axis in [g] */
  float AccY;           /* Acceleration in Y axis in [g] */
  float AccZ;           /* Acceleration in Z axis in [g] */
} MSM_input_t;

typedef struct
{
  MSM_sleep_t SleepFlag;
  uint32_t TotalSleepTime;
} MSM_output_t;


/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_SM_Exported_Functions MOTION_SM_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionSM engine
 * @param  none
 * @retval none
 */
void MotionSM_Initialize(void);

/**
 * @brief  Set the MotionSM accelerometer data orientation
 * @param  *acc_orientation: reference system of the accelerometer raw data (for instance: south west up became "swu", north east up became "ned")
 * @retval none
 */
void MotionSM_SetOrientation_Acc(const char *acc_orientation);

/**
 * @brief  Run sleep monitoring algorithm
 * @param  data_in  Pointer to accaleration in [g]
 * @param  data_out pointer to output data (sleep indication, total sleep time)
 * @retval none
 */
void MotionSM_Update(MSM_input_t *data_in, MSM_output_t *data_out);

/**
 * @brief  Reset sleep monitorin algorithm
 * @param  none
 * @retval none
 */
void MotionSM_Reset(void);

/**
 * @brief  Get the library version
 * @param  version  Pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionSM_GetLibVersion(char *version);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_SM_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
