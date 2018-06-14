/**
  ******************************************************************************
  * @file    stm32f10x_bkp.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the BKP firmware 
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
#ifndef __STM32F10x_BKP_H
#define __STM32F10x_BKP_H

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

/** @addtogroup BKP
  * @{
  */

/** @defgroup BKP_Exported_Types
  * @{
  */

/**
  * @}
  */

/** @defgroup BKP_Exported_Constants
  * @{
  */

/** @defgroup Tamper_Pin_active_level 
  * @{
  */

#define LL_RTC_TAMPER_ACTIVELEVEL_HIGH           ((uint16_t)0x0000)
#define LL_RTC_TAMPER_ACTIVELEVEL_LOW            ((uint16_t)0x0001)
#define IS_BKP_TAMPER_PIN_LEVEL(LEVEL) (((LEVEL) == LL_RTC_TAMPER_ACTIVELEVEL_HIGH) || \
                                        ((LEVEL) == LL_RTC_TAMPER_ACTIVELEVEL_LOW))
/**
  * @}
  */

/** @defgroup RTC_output_source_to_output_on_the_Tamper_pin 
  * @{
  */

#define LL_RTC_CALIB_OUTPUT_NONE          ((uint16_t)0x0000)
#define LL_RTC_CALIB_OUTPUT_RTCCLOCK    ((uint16_t)0x0080)
#define LL_RTC_CALIB_OUTPUT_ALARM         ((uint16_t)0x0100)
#define LL_RTC_CALIB_OUTPUT_SECOND        ((uint16_t)0x0300)
#define IS_BKP_RTC_OUTPUT_SOURCE(SOURCE) (((SOURCE) == LL_RTC_CALIB_OUTPUT_NONE) || \
                                          ((SOURCE) == LL_RTC_CALIB_OUTPUT_RTCCLOCK) || \
                                          ((SOURCE) == LL_RTC_CALIB_OUTPUT_ALARM) || \
                                          ((SOURCE) == LL_RTC_CALIB_OUTPUT_SECOND))
/**
  * @}
  */

/** @defgroup Data_Backup_Register 
  * @{
  */

#define LL_RTC_BKP_DR1                           ((uint16_t)0x0004)
#define LL_RTC_BKP_DR2                           ((uint16_t)0x0008)
#define LL_RTC_BKP_DR3                           ((uint16_t)0x000C)
#define LL_RTC_BKP_DR4                           ((uint16_t)0x0010)
#define LL_RTC_BKP_DR5                            ((uint16_t)0x0014)
#define LL_RTC_BKP_DR6                            ((uint16_t)0x0018)
#define LL_RTC_BKP_DR7                            ((uint16_t)0x001C)
#define LL_RTC_BKP_DR8                            ((uint16_t)0x0020)
#define LL_RTC_BKP_DR9                            ((uint16_t)0x0024)
#define LL_RTC_BKP_DR10                          ((uint16_t)0x0028)
#define LL_RTC_BKP_DR11                          ((uint16_t)0x0040)
#define LL_RTC_BKP_DR12                          ((uint16_t)0x0044)
#define LL_RTC_BKP_DR13                          ((uint16_t)0x0048)
#define LL_RTC_BKP_DR14                          ((uint16_t)0x004C)
#define LL_RTC_BKP_DR15                          ((uint16_t)0x0050)
#define LL_RTC_BKP_DR16                          ((uint16_t)0x0054)
#define LL_RTC_BKP_DR17                          ((uint16_t)0x0058)
#define LL_RTC_BKP_DR18                          ((uint16_t)0x005C)
#define LL_RTC_BKP_DR19                          ((uint16_t)0x0060)
#define LL_RTC_BKP_DR20                          ((uint16_t)0x0064)
#define LL_RTC_BKP_DR21                          ((uint16_t)0x0068)
#define LL_RTC_BKP_DR22                          ((uint16_t)0x006C)
#define LL_RTC_BKP_DR23                          ((uint16_t)0x0070)
#define LL_RTC_BKP_DR24                          ((uint16_t)0x0074)
#define LL_RTC_BKP_DR25                          ((uint16_t)0x0078)
#define LL_RTC_BKP_DR26                          ((uint16_t)0x007C)
#define LL_RTC_BKP_DR27                          ((uint16_t)0x0080)
#define LL_RTC_BKP_DR28                          ((uint16_t)0x0084)
#define LL_RTC_BKP_DR29                          ((uint16_t)0x0088)
#define LL_RTC_BKP_DR30                          ((uint16_t)0x008C)
#define LL_RTC_BKP_DR31                          ((uint16_t)0x0090)
#define LL_RTC_BKP_DR32                          ((uint16_t)0x0094)
#define LL_RTC_BKP_DR33                          ((uint16_t)0x0098)
#define LL_RTC_BKP_DR34                          ((uint16_t)0x009C)
#define LL_RTC_BKP_DR35                          ((uint16_t)0x00A0)
#define LL_RTC_BKP_DR36                          ((uint16_t)0x00A4)
#define LL_RTC_BKP_DR37                          ((uint16_t)0x00A8)
#define LL_RTC_BKP_DR38                          ((uint16_t)0x00AC)
#define LL_RTC_BKP_DR39                          ((uint16_t)0x00B0)
#define LL_RTC_BKP_DR40                          ((uint16_t)0x00B4)
#define LL_RTC_BKP_DR41                          ((uint16_t)0x00B8)
#define LL_RTC_BKP_DR42                          ((uint16_t)0x00BC)

#define IS_BKP_DR(DR) (((DR) == LL_RTC_BKP_DR1)  || ((DR) == LL_RTC_BKP_DR2)  || ((DR) == LL_RTC_BKP_DR3)  || \
                       ((DR) == LL_RTC_BKP_DR4)  || ((DR) == LL_RTC_BKP_DR5 )  || ((DR) == LL_RTC_BKP_DR6 )  || \
                       ((DR) == LL_RTC_BKP_DR7 )  || ((DR) == LL_RTC_BKP_DR8 )  || ((DR) == LL_RTC_BKP_DR9 )  || \
                       ((DR) == LL_RTC_BKP_DR10) || ((DR) == LL_RTC_BKP_DR11) || ((DR) == LL_RTC_BKP_DR12) || \
                       ((DR) == LL_RTC_BKP_DR13) || ((DR) == LL_RTC_BKP_DR14) || ((DR) == LL_RTC_BKP_DR15) || \
                       ((DR) == LL_RTC_BKP_DR16) || ((DR) == LL_RTC_BKP_DR17) || ((DR) == LL_RTC_BKP_DR18) || \
                       ((DR) == LL_RTC_BKP_DR19) || ((DR) == LL_RTC_BKP_DR20) || ((DR) == LL_RTC_BKP_DR21) || \
                       ((DR) == LL_RTC_BKP_DR22) || ((DR) == LL_RTC_BKP_DR23) || ((DR) == LL_RTC_BKP_DR24) || \
                       ((DR) == LL_RTC_BKP_DR25) || ((DR) == LL_RTC_BKP_DR26) || ((DR) == LL_RTC_BKP_DR27) || \
                       ((DR) == LL_RTC_BKP_DR28) || ((DR) == LL_RTC_BKP_DR29) || ((DR) == LL_RTC_BKP_DR30) || \
                       ((DR) == LL_RTC_BKP_DR31) || ((DR) == LL_RTC_BKP_DR32) || ((DR) == LL_RTC_BKP_DR33) || \
                       ((DR) == LL_RTC_BKP_DR34) || ((DR) == LL_RTC_BKP_DR35) || ((DR) == LL_RTC_BKP_DR36) || \
                       ((DR) == LL_RTC_BKP_DR37) || ((DR) == LL_RTC_BKP_DR38) || ((DR) == LL_RTC_BKP_DR39) || \
                       ((DR) == LL_RTC_BKP_DR40) || ((DR) == LL_RTC_BKP_DR41) || ((DR) == LL_RTC_BKP_DR42))

#define IS_BKP_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x7F)
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup BKP_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup BKP_Exported_Functions
  * @{
  */

LL_RCC_ForceBackupDomainReset();
void LL_RCC_ReleaseBackupDomainReset();
void LL_RTC_TAMPER_SetActiveLevel(BKP, uint16_t BKP_TamperPinLevel);
void BKP_TamperPinCmd(FunctionalState NewState);
void BKP_ITConfig(FunctionalState NewState);
void LL_RTC_SetOutputSource(BKP, uint16_t BKP_RTCOutputSource);
void LL_RTC_CAL_SetCoarseDigital(BKP, uint8_t CalibrationValue);
void LL_RTC_BKP_SetRegister(BKP, uint16_t BKP_DR, uint16_t Data);
uint16_t LL_RTC_BKP_GetRegister(BKP, uint16_t BKP_DR);
FlagStatus LL_RTC_IsActiveFlag_TAMPE(BKP);
void LL_RTC_ClearFlag_TAMPE(BKP);
ITStatus LL_RTC_IsActiveFlag_TAMPI(BKP);
void LL_RTC_ClearFlag_TAMPI(BKP);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_BKP_H */
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
