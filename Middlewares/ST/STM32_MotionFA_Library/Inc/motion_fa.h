/**
  ******************************************************************************
  * @file    motion_fa.h
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-May-2017
  * @brief   Header for motion_fa module
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
#ifndef _MOTION_FA_H_
#define _MOTION_FA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_FA MOTION_FA
  * @{
  */


/** @defgroup MOTION_FA_Exported_Types MOTION_FA_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/
#define NUM_FA_ACTIVITIES 3

/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  float AccX;           /* Acceleration in X axis in [g] */
  float AccY;           /* Acceleration in Y axis in [g] */
  float AccZ;           /* Acceleration in Z axis in [g] */
  float Press;          /* Atmospheric pressure in [hPa] */
} MFA_input_t;

typedef enum
{
  MFA_NOACTIVITY          = 0x00,
  MFA_BICEPCURL           = 0x01,
  MFA_SQUAT               = 0x02
} MFA_activity_t;

typedef struct {
  uint32_t Counter;
} MFA_output_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_FA_Exported_Functions MOTION_FA_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionFA engine
 * @param  none
 * @retval none
 */
void MotionFA_Initialize(void);

/**
 * @brief  Set the MotionFA accelerometer data orientation
 * @param  *acc_orientation: reference system of the accelerometer raw data (for instance: south west up became "swu", north east up became "ned")
 * @retval  none
 */
void MotionFA_SetOrientation_Acc(const char *acc_orientation);

/**
 * @brief  Run bicep curl counter
 * @param  data_in  pointer to accaleration in [g] and atmospheric pressure in [hPa]
 * @param  data_out  pointer to bicep curl counter
 * @retval none
 */
void MotionFA_BicepCurl_Update(MFA_input_t *data_in, MFA_output_t *data_out);

/**
 * @brief  Run squat counter
 * @param  data_in  pointer to accaleration in [g] and atmospheric pressure in [hPa]
 * @param  data_out  pointer to squat counter
 * @retval none
 */
void MotionFA_Squat_Update(MFA_input_t *data_in, MFA_output_t *data_out);

/**
 * @brief  Reset bicep curl counter
 * @param  none
 * @retval none
 */
void MotionFA_BicepCurl_Reset(void);

/**
 * @brief  Reset squat counter
 * @param  none
 * @retval none
 */
void MotionFA_Squat_Reset(void);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionFA_GetLibVersion(char *version);


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

#endif /* _MOTION_FA_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
