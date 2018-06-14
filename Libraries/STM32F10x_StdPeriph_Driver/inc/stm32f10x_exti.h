/**
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the EXTI firmware
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
#ifndef __STM32F10x_EXTI_H
#define __STM32F10x_EXTI_H

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

/** @addtogroup EXTI
  * @{
  */

/** @defgroup EXTI_Exported_Types
  * @{
  */

/** 
  * @brief  EXTI mode enumeration  
  */

typedef enum
{
  LL_EXTI_MODE_IT = 0x00,
  LL_EXTI_MODE_EVENT = 0x04
}EXTIMode_TypeDef;

#define IS_EXTI_MODE(MODE) (((MODE) == LL_EXTI_MODE_IT) || ((MODE) == LL_EXTI_MODE_EVENT))

/** 
  * @brief  EXTI Trigger enumeration  
  */

typedef enum
{
  LL_EXTI_TRIGGER_RISING = 0x08,
  LL_EXTI_TRIGGER_FALLING = 0x0C,  
  LL_EXTI_TRIGGER_RISING_FALLING = 0x10
}EXTITrigger_TypeDef;

#define IS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == LL_EXTI_TRIGGER_RISING) || \
                                  ((TRIGGER) == LL_EXTI_TRIGGER_FALLING) || \
                                  ((TRIGGER) == LL_EXTI_TRIGGER_RISING_FALLING))
/** 
  * @brief  EXTI Init Structure definition  
  */

typedef struct
{
  uint32_t EXTI_Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination of @ref EXTI_Lines */
   
  EXTIMode_TypeDef EXTI_Mode;       /*!< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

  EXTITrigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

  FunctionalState EXTI_LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */ 
}LL_EXTI_InitTypeDef;

/**
  * @}
  */

/** @defgroup EXTI_Exported_Constants
  * @{
  */

/** @defgroup EXTI_Lines 
  * @{
  */

#define LL_EXTI_LINE_0       ((uint32_t)0x00001)  /*!< External interrupt line 0 */
#define LL_EXTI_LINE_1       ((uint32_t)0x00002)  /*!< External interrupt line 1 */
#define LL_EXTI_LINE_2       ((uint32_t)0x00004)  /*!< External interrupt line 2 */
#define LL_EXTI_LINE_3       ((uint32_t)0x00008)  /*!< External interrupt line 3 */
#define LL_EXTI_LINE_4       ((uint32_t)0x00010)  /*!< External interrupt line 4 */
#define LL_EXTI_LINE_5       ((uint32_t)0x00020)  /*!< External interrupt line 5 */
#define LL_EXTI_LINE_6       ((uint32_t)0x00040)  /*!< External interrupt line 6 */
#define LL_EXTI_LINE_7       ((uint32_t)0x00080)  /*!< External interrupt line 7 */
#define LL_EXTI_LINE_8       ((uint32_t)0x00100)  /*!< External interrupt line 8 */
#define LL_EXTI_LINE_9       ((uint32_t)0x00200)  /*!< External interrupt line 9 */
#define LL_EXTI_LINE_10      ((uint32_t)0x00400)  /*!< External interrupt line 10 */
#define LL_EXTI_LINE_11      ((uint32_t)0x00800)  /*!< External interrupt line 11 */
#define LL_EXTI_LINE_12      ((uint32_t)0x01000)  /*!< External interrupt line 12 */
#define LL_EXTI_LINE_13      ((uint32_t)0x02000)  /*!< External interrupt line 13 */
#define LL_EXTI_LINE_14      ((uint32_t)0x04000)  /*!< External interrupt line 14 */
#define LL_EXTI_LINE_15      ((uint32_t)0x08000)  /*!< External interrupt line 15 */
#define LL_EXTI_LINE_16      ((uint32_t)0x10000)  /*!< External interrupt line 16 Connected to the PVD Output */
#define LL_EXTI_LINE_17      ((uint32_t)0x20000)  /*!< External interrupt line 17 Connected to the RTC Alarm event */
#define LL_EXTI_LINE_18      ((uint32_t)0x40000)  /*!< External interrupt line 18 Connected to the USB Device/USB OTG FS
                                                   Wakeup from suspend event */                                    
#define LL_EXTI_LINE_19      ((uint32_t)0x80000)  /*!< External interrupt line 19 Connected to the Ethernet Wakeup event */
                                          
#define IS_EXTI_LINE(LINE) ((((LINE) & (uint32_t)0xFFF00000) == 0x00) && ((LINE) != (uint16_t)0x00))
#define IS_GET_EXTI_LINE(LINE) (((LINE) == LL_EXTI_LINE_0) || ((LINE) == LL_EXTI_LINE_1) || \
                            ((LINE) == LL_EXTI_LINE_2) || ((LINE) == LL_EXTI_LINE_3) || \
                            ((LINE) == LL_EXTI_LINE_4) || ((LINE) == LL_EXTI_LINE_5) || \
                            ((LINE) == LL_EXTI_LINE_6) || ((LINE) == LL_EXTI_LINE_7) || \
                            ((LINE) == LL_EXTI_LINE_8) || ((LINE) == LL_EXTI_LINE_9) || \
                            ((LINE) == LL_EXTI_LINE_10) || ((LINE) == LL_EXTI_LINE_11) || \
                            ((LINE) == LL_EXTI_LINE_12) || ((LINE) == LL_EXTI_LINE_13) || \
                            ((LINE) == LL_EXTI_LINE_14) || ((LINE) == LL_EXTI_LINE_15) || \
                            ((LINE) == LL_EXTI_LINE_16) || ((LINE) == LL_EXTI_LINE_17) || \
                            ((LINE) == LL_EXTI_LINE_18) || ((LINE) == LL_EXTI_LINE_19))

                    
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup EXTI_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Exported_Functions
  * @{
  */

void LL_EXTI_DeInit();
void LL_EXTI_Init(LL_EXTI_InitTypeDef* EXTI_InitStruct);
void LL_EXTI_StructInit(LL_EXTI_InitTypeDef* EXTI_InitStruct);
void EXTI_GenerateSWInterrupt(uint32_t EXTI_Line);
FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
void EXTI_ClearFlag(uint32_t EXTI_Line);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_EXTI_H */
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
