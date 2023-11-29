/**
  ******************************************************************************
  * @file    motion_pe.h
  * @author  MEMS Application Team
  * @version V2.0.0
  * @date    01-May-2017
  * @brief   Header for motion_pe module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_PE_H_
#define _MOTION_PE_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_PE MOTION_PE
  * @{
  */

/** @defgroup MOTION_PE_Exported_Types MOTION_PE_Exported_Types
  * @{
  */
/* Exported types ------------------------------------------------------------*/
typedef struct
{
  float AccX; /* Acceleration in X axis in [g] */
  float AccY; /* Acceleration in Y axis in [g] */
  float AccZ; /* Acceleration in Z axis in [g] */
} MPE_input_t;

typedef enum
{
  MPE_UNKNOWN    = 0x00,
  MPE_SITTING    = 0x01,
  MPE_STANDING   = 0x02,
  MPE_LYING_DOWN = 0x03
} MPE_output_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_PE_Exported_Functions MOTION_PE_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialize the MotionPE engine
  * @param  None
  * @retval None
  */
void MotionPE_Initialize(void);

/**
  * @brief  Set the MotionPE Accelerometer data orientation
  * @param  *acc_orientation: reference system of the Accelerometer raw data
  *         (for instance: south west up became "swu", north east up became "ned")
  * @retval  None
  */
void MotionPE_SetOrientation_Acc(const char *acc_orientation);

/**
  * @brief  Run Pose Estimation algorithm
  * @param  data_in: Structure containing input data
  * @param  data_out: Structure containing output data
  * @retval  None
  */
void MotionPE_Update(MPE_input_t *data_in, MPE_output_t *data_out);

/**
  * @brief  Reset Libray
  * @param  None
  * @retval None
  */
void MotionPE_ResetLib(void);

/**
  * @brief  Get the library version
  * @param  version: pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionPE_GetLibVersion(char *version);

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

#endif /* _MOTION_PE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
