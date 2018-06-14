/**
  ******************************************************************************
  * @file    stm32f10x_pwr.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the PWR firmware 
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
#ifndef __STM32F10x_PWR_H
#define __STM32F10x_PWR_H

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

/** @addtogroup PWR
  * @{
  */ 

/** @defgroup PWR_Exported_Types
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup PWR_Exported_Constants
  * @{
  */ 

/** @defgroup PVD_detection_level 
  * @{
  */ 

#define LL_PWR_PVDLEVEL_0          ((uint32_t)0x00000000)
#define LL_PWR_PVDLEVEL_1          ((uint32_t)0x00000020)
#define LL_PWR_PVDLEVEL_2          ((uint32_t)0x00000040)
#define LL_PWR_PVDLEVEL_3          ((uint32_t)0x00000060)
#define LL_PWR_PVDLEVEL_4          ((uint32_t)0x00000080)
#define LL_PWR_PVDLEVEL_5          ((uint32_t)0x000000A0)
#define LL_PWR_PVDLEVEL_6          ((uint32_t)0x000000C0)
#define LL_PWR_PVDLEVEL_7          ((uint32_t)0x000000E0)
#define IS_PWR_PVD_LEVEL(LEVEL) (((LEVEL) == LL_PWR_PVDLEVEL_0) || ((LEVEL) == LL_PWR_PVDLEVEL_1)|| \
                                 ((LEVEL) == LL_PWR_PVDLEVEL_2) || ((LEVEL) == LL_PWR_PVDLEVEL_3)|| \
                                 ((LEVEL) == LL_PWR_PVDLEVEL_4) || ((LEVEL) == LL_PWR_PVDLEVEL_5)|| \
                                 ((LEVEL) == LL_PWR_PVDLEVEL_6) || ((LEVEL) == LL_PWR_PVDLEVEL_7))
/**
  * @}
  */

/** @defgroup Regulator_state_is_STOP_mode 
  * @{
  */

#define LL_PWR_MODE_STOP_MAINREGU          ((uint32_t)0x00000000)
#define LL_PWR_MODE_STOP_LPREGU    ((uint32_t)0x00000001)
#define IS_PWR_REGULATOR(REGULATOR) (((REGULATOR) == LL_PWR_MODE_STOP_MAINREGU) || \
                                     ((REGULATOR) == LL_PWR_MODE_STOP_LPREGU))
/**
  * @}
  */

/** @defgroup STOP_mode_entry 
  * @{
  */

#define PWR_STOPEntry_WFI         ((uint8_t)0x01)
#define PWR_STOPEntry_WFE         ((uint8_t)0x02)
#define IS_PWR_STOP_ENTRY(ENTRY) (((ENTRY) == PWR_STOPEntry_WFI) || ((ENTRY) == PWR_STOPEntry_WFE))
 
/**
  * @}
  */

/** @defgroup PWR_Flag 
  * @{
  */

#define PWR_FLAG_WU               ((uint32_t)0x00000001)
#define PWR_FLAG_SB               ((uint32_t)0x00000002)
#define PWR_FLAG_PVDO             ((uint32_t)0x00000004)
#define IS_PWR_GET_FLAG(FLAG) (((FLAG) == PWR_FLAG_WU) || ((FLAG) == PWR_FLAG_SB) || \
                               ((FLAG) == PWR_FLAG_PVDO))

#define IS_PWR_CLEAR_FLAG(FLAG) (((FLAG) == PWR_FLAG_WU) || ((FLAG) == PWR_FLAG_SB))
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup PWR_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup PWR_Exported_Functions
  * @{
  */

void LL_PWR_DeInit();
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void LL_PWR_SetPVDLevel(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
LL_PWR_ClearFlag_WU();
LL_PWR_SetPowerMode(LL_PWR_MODE_STANDBY);
LL_LPM_EnableDeepSleep();
void __WFI();
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_PWR_H */
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
