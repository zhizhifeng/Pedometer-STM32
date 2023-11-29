/**
  ******************************************************************************
  * @file    motion_pw.h
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-November-2017
  * @brief   Header for motion_pw module
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
#ifndef _MOTION_PW_H_
#define _MOTION_PW_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
 * @{
 */

/** @defgroup MOTION_PW MOTION_PW
 * @{
 */

/** @defgroup MOTION_PW_Exported_Types MOTION_PW_Exported_Types
 * @{
 */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MPW_UNKNOWN_ACTIVITY = 0x00,
  MPW_WALKING          = 0x01,
  MPW_FASTWALKING      = 0x02,
  MPW_JOGGING          = 0x03
} MPW_activity_t;

typedef struct
{
  float AccX;                      /* Acceleration in X axis in [g] */
  float AccY;                      /* Acceleration in Y axis in [g] */
  float AccZ;                      /* Acceleration in Z axis in [g] */
  MPW_activity_t CurrentActivity;  /* Current user activity */
} MPW_input_t;

typedef struct
{
  uint8_t Cadence;      /* [steps/min] */
  uint32_t Nsteps;
  uint8_t Confidence;
} MPW_output_t;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_PW_Exported_Functions MOTION_PW_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionPW engine
 * @param  None
 * @retval None
 */
void MotionPW_Initialize(void);

/**
 * @brief  Run pedometer algorithm
 * @param  data_in  pointer to accaleration in [g]
 * @param  data_out  pointer to MPW_output_t structure
 * @retval None
 */
void MotionPW_Update(MPW_input_t *data_in, MPW_output_t *data_out);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionPW_GetLibVersion(char *version);

/**
 * @brief  Reset library
 * @param  None
 * @retval None
 */
void MotionPW_ResetPedometerLibrary(void);

/**
 * @brief  Reset step count
 * @param  None
 * @retval None
 */
void MotionPW_ResetStepCount(void);

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

#endif /* _MOTION_PW_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
