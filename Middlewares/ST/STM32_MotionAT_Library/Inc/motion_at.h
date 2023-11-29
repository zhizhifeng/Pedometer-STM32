/**
  ******************************************************************************
  * @file    motion_at.h
  * @author  MEMS Application Team
  * @version V1.0.0
  * @date    01-November-2017
  * @brief   Header for motion_at module
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
#ifndef _MOTION_AT_H_
#define _MOTION_AT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
 * @{
 */

/** @defgroup MOTION_AT MOTION_AT
 * @{
 */

/** @defgroup MOTION_AT_Exported_Types MOTION_AT_Exported_Types
 * @{
 */

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float acc_x; /**< @brief Acceleration in X axis in [g] */
  float acc_y; /**< @brief Acceleration in Y axis in [g] */
  float acc_z; /**< @brief Acceleration in Z axis in [g] */
} MAT_input_t;

typedef struct
{
  uint8_t active; /**< @brief 1 if currently active, 0 otherwise */
} MAT_output_t;

/**
 * @}
 */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_AT_Exported_Functions MOTION_AT_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize MotionAT engine (dynamically allocate memory)
 * @param  None
 * @retval None
 */
void MotionAT_Initialize(void);

/**
 * @brief  Deinitialize MotionAT engine (free dynamically allocated memory)
 * @param  None
 * @retval None
 */
void MotionAT_Deinitialize(void);

/**
 * @brief  Run Active Time algorithm
 * @param  data_in pointer to MAT_input_t structure
 * @param  data_out pointer to MAT_output_t structure
 * @retval None
 */
void MotionAT_Update(MAT_input_t *data_in, MAT_output_t *data_out);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionAT_GetLibVersion(char *version);

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

#endif /* _MOTION_AT_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
