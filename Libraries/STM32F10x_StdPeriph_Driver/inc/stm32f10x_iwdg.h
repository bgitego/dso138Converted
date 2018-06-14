/**
  ******************************************************************************
  * @file    stm32f10x_iwdg.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the IWDG 
  *          firmware library.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IWDG_H
#define __STM32F10x_IWDG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "legacy.h"
#include "ll_includes.h"
#include "stm32f1xx_hal_conf.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @addtogroup IWDG
  * @{
  */

/** @defgroup IWDG_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup IWDG_Exported_Constants
  * @{
  */

/** @defgroup IWDG_WriteAccess
  * @{
  */

#define IWDG_WriteAccess_Enable     ((uint16_t)0x5555)
#define IWDG_WriteAccess_Disable    ((uint16_t)0x0000)
#define IS_IWDG_WRITE_ACCESS(ACCESS) (((ACCESS) == IWDG_WriteAccess_Enable) || \
                                      ((ACCESS) == IWDG_WriteAccess_Disable))
/**
  * @}
  */

/** @defgroup IWDG_prescaler 
  * @{
  */

#define LL_IWDG_PRESCALER_4            ((uint8_t)0x00)
#define LL_IWDG_PRESCALER_8            ((uint8_t)0x01)
#define LL_IWDG_PRESCALER_16           ((uint8_t)0x02)
#define LL_IWDG_PRESCALER_32           ((uint8_t)0x03)
#define LL_IWDG_PRESCALER_64           ((uint8_t)0x04)
#define LL_IWDG_PRESCALER_128          ((uint8_t)0x05)
#define LL_IWDG_PRESCALER_256          ((uint8_t)0x06)
#define IS_IWDG_PRESCALER(PRESCALER) (((PRESCALER) == LL_IWDG_PRESCALER_4)  || \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_8)  || \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_16) || \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_32) || \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_64) || \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_128)|| \
                                      ((PRESCALER) == LL_IWDG_PRESCALER_256))
/**
  * @}
  */

/** @defgroup IWDG_Flag 
  * @{
  */

#define IWDG_FLAG_PVU               ((uint16_t)0x0001)
#define IWDG_FLAG_RVU               ((uint16_t)0x0002)
#define IS_IWDG_FLAG(FLAG) (((FLAG) == IWDG_FLAG_PVU) || ((FLAG) == IWDG_FLAG_RVU))
#define IS_IWDG_RELOAD(RELOAD) ((RELOAD) <= 0xFFF)
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup IWDG_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup IWDG_Exported_Functions
  * @{
  */

void IWDG_WriteAccessCmd(uint16_t IWDG_WriteAccess);
void LL_IWDG_SetPrescaler(IWDG, uint8_t IWDG_Prescaler);
void LL_IWDG_SetReloadCounter(IWDG, uint16_t Reload);
void LL_IWDG_ReloadCounter(IWDG);
void LL_IWDG_Enable(IWDG);
FlagStatus IWDG_GetFlagStatus(uint16_t IWDG_FLAG);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_IWDG_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
