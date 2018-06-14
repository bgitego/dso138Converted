/**
  ******************************************************************************
  * @file    stm32f10x_wwdg.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the WWDG firmware
  *          library.
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
#ifndef __STM32F10x_WWDG_H
#define __STM32F10x_WWDG_H

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

/** @addtogroup WWDG
  * @{
  */ 

/** @defgroup WWDG_Exported_Types
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Constants
  * @{
  */ 
  
/** @defgroup WWDG_Prescaler 
  * @{
  */ 
  
#define LL_WWDG_PRESCALER_1    ((uint32_t)0x00000000)
#define LL_WWDG_PRESCALER_2    ((uint32_t)0x00000080)
#define LL_WWDG_PRESCALER_4    ((uint32_t)0x00000100)
#define LL_WWDG_PRESCALER_8    ((uint32_t)0x00000180)
#define IS_WWDG_PRESCALER(PRESCALER) (((PRESCALER) == LL_WWDG_PRESCALER_1) || \
                                      ((PRESCALER) == LL_WWDG_PRESCALER_2) || \
                                      ((PRESCALER) == LL_WWDG_PRESCALER_4) || \
                                      ((PRESCALER) == LL_WWDG_PRESCALER_8))
#define IS_WWDG_WINDOW_VALUE(VALUE) ((VALUE) <= 0x7F)
#define IS_WWDG_COUNTER(COUNTER) (((COUNTER) >= 0x40) && ((COUNTER) <= 0x7F))

/**
  * @}
  */ 

/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup WWDG_Exported_Functions
  * @{
  */ 
  
LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_WWDG);
void LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_WWDG);
void LL_WWDG_SetPrescaler(WWDG, uint32_t WWDG_Prescaler);
void LL_WWDG_SetWindow(WWDG, uint8_t WindowValue);
void LL_WWDG_EnableIT_EWKUP(WWDG);
void LL_WWDG_SetCounter(WWDG, uint8_t Counter);
LL_WWDG_SetCounter(WWDG, uint8_t Counter);
void LL_WWDG_Enable(WWDG);
FlagStatus LL_WWDG_IsActiveFlag_EWKUP(WWDG);
void LL_WWDG_ClearFlag_EWKUP(WWDG);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_WWDG_H */

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
