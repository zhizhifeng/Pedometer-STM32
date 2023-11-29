/**
  ******************************************************************************
  * @file    motion_aw.h
  * @author  MEMS Application Team
  * @version V2.0.0
  * @date    01-May-2017
  * @brief   Header for motion_aw module
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
#ifndef _MOTION_AW_H_
#define _MOTION_AW_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_AW MOTION_AW
  * @{
  */


/** @defgroup MOTION_AW_Exported_Types MOTION_AW_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/
#define NUM_ACTIVITIES 9

/* Exported types ------------------------------------------------------------*/
 typedef struct
{
  float AccX;           /* Acceleration in X axis in [g] */
  float AccY;           /* Acceleration in Y axis in [g] */
  float AccZ;           /* Acceleration in Z axis in [g] */
} MAW_input_t;

typedef enum
{
  MAW_NOACTIVITY     = 0x00,
  MAW_STATIONARY     = 0x01,
  MAW_STANDING       = 0x02,
  MAW_SITTING        = 0x03,
  MAW_LYING          = 0x04,
  MAW_WALKING        = 0x05,
  MAW_FASTWALKING    = 0x06,
  MAW_JOGGING        = 0x07,
  MAW_BIKING         = 0x08
} MAW_activity_t;

typedef struct {
  MAW_activity_t current_activity;
  uint16_t confidence;
  uint32_t current_activity_duration;
  uint32_t activity_total_duration[NUM_ACTIVITIES];
} MAW_output_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_AW_Exported_Functions MOTION_AW_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionAW engine
 * @param  none
 * @retval none
 */
void MotionAW_Initialize(void);

/**
 * @brief  Set the MotionAW accelerometer data orientation
 * @param  *acc_orientation: reference system of the accelerometer raw data (for instance: south west up became "swu", north east up became "ned")
 * @retval  none
 */
void MotionAW_SetOrientation_Acc(const char *acc_orientation);

/**
 * @brief  Run activity recognition algorithm
 * @param  data_in  pointer to accaleration in [g]
 * @param  data_out  pointer to output activity structure
 * @retval none
 */
void MotionAW_Update(MAW_input_t *data_in, MAW_output_t *data_out);

/**
 * @brief  Resets activity recognition algorithm
 * @param  none
 * @retval none
 */
void MotionAW_Reset(void);

/**
 * @brief  Resets total activity duration counters
 * @param  none
 * @retval none
 */
void MotionAW_Reset_Activity_Duration(void);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionAW_GetLibVersion(char *version);


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

#endif /* _MOTION_AW_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
