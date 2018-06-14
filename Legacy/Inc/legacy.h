/**
  ******************************************************************************
  * @file    legacy.h 
  * @author  MCD Application Team
  * @brief   Header for legacy.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEGACY_H
#define __LEGACY_H

/* Includes ------------------------------------------------------------------*/
#include "cmsis_ref.h"
#include "ll_includes.h"

#if defined(STM32F0)
#include "stm32f0xx_hal_conf.h"

#elif defined(STM32F1)
#include "stm32f1xx_hal_conf.h"

#elif defined(STM32F2)
#include "stm32f2xx_hal_conf.h"

#elif defined(STM32F3)
#include "stm32f3xx_hal_conf.h"

#elif defined(STM32F4)
#include "stm32f4xx_hal_conf.h"

#elif defined(STM32F7)
#include "stm32f7xx_hal_conf.h"

#elif defined(STM32L0)
#include "stm32l0xx_hal_conf.h"

#elif defined(STM32L1)
#include "stm32l1xx_hal_conf.h"

#elif defined(STM32L4)
#include "stm32l4xx_hal_conf.h"

#endif /* STM32F0 */

/* -------------------------------------------------------------------------- */
/*                                MISC APIs                                    */
/* -------------------------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  NVIC Init Structure definition  
  */

typedef struct
{
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be an enumerator of @ref IRQn_Type 
                                                   enumeration (For the complete STM32 Devices IRQ Channels
                                                   list, please refer to stm32f4xx.h file) */

#if defined(STM32F0)
  uint8_t NVIC_IRQChannelPriority;            /*!< Specifies the priority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 3.  */
#else
  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref MISC_NVIC_Priority_Table
                                                   A lower priority value indicates a higher priority */
#endif /* STM32F0 */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;

/* Exported constants --------------------------------------------------------*/

/** @defgroup MISC_System_Low_Power 
  * @{
  */
#define NVIC_LP_SEVONPEND            ((uint8_t)0x10)
#define NVIC_LP_SLEEPDEEP            ((uint8_t)0x04)
#define NVIC_LP_SLEEPONEXIT          ((uint8_t)0x02)
/**
  * @}
  */

#if !defined(STM32F0)
/** @defgroup MISC_Vector_Table_Base 
  * @{
  */
#define NVIC_VectTab_RAM             ((uint32_t)0x20000000)
#define NVIC_VectTab_FLASH           ((uint32_t)0x08000000)
/**
  * @}
  */

/** @defgroup MISC_Preemption_Priority_Group 
  * @{
  */
#define NVIC_PriorityGroup_0         ((uint32_t)0x700) /*!< 0 bits for pre-emption priority
                                                            4 bits for subpriority */
#define NVIC_PriorityGroup_1         ((uint32_t)0x600) /*!< 1 bits for pre-emption priority
                                                            3 bits for subpriority */
#define NVIC_PriorityGroup_2         ((uint32_t)0x500) /*!< 2 bits for pre-emption priority
                                                            2 bits for subpriority */
#define NVIC_PriorityGroup_3         ((uint32_t)0x400) /*!< 3 bits for pre-emption priority
                                                            1 bits for subpriority */
#define NVIC_PriorityGroup_4         ((uint32_t)0x300) /*!< 4 bits for pre-emption priority
                                                            0 bits for subpriority */
/**
  * @}
  */
#endif /* !STM32F0 */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
#if !defined(STM32F0)
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
#endif /* !STM32F0 */

/* -------------------------------------------------------------------------- */
/*                                RTC APIs                                     */
/* -------------------------------------------------------------------------- */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/** @defgroup RTC_Add_1_Second_Parameter_Definitions
  * @{
  */
#if !(defined(STM32F1) || defined(STM32F2))
#define RTC_ShiftAdd1S_Reset      ((uint32_t)0x00000000)
#define RTC_ShiftAdd1S_Set        ((uint32_t)0x80000000)
#define RECALPF_TIMEOUT           ((uint32_t)0x00020000)
#endif /* !STM32F1 || !STM32F2 */

#if !defined(STM32F1)
#define INITMODE_TIMEOUT          ((uint32_t)0x00010000)
#endif /* !STM32F1 */
/**
  * @}
  */

/** @defgroup RTC_Substract_Fraction_Of_Second_Value
  * @{
  */
#if !(defined(STM32F1) || defined(STM32F2))
#define SHPF_TIMEOUT              ((uint32_t)0x00001000)
#endif /* !STM32F1 || !STM32F2 */
/**
  * @}
  */

/** @defgroup RTC_Alarms_Definitions 
  * @{
  */ 
#if !defined(STM32F1)
#define RTC_Alarm_A         RTC_CR_ALRAE
#define RTC_Alarm_B         RTC_CR_ALRBE
#endif /* !STM32F1 */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#if !defined(STM32F1)
ErrorStatus RTC_RefClockCmd(FunctionalState NewState);
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState);
#if !(defined(STM32F0) && !defined(RTC_WAKEUP_SUPPORT))
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState);
#endif /* !(STM32F0 && !RTC_WAKEUP_SUPPORT) */
#endif /* !STM32F1 */
#if !(defined(STM32F0) || defined(STM32F1) || defined(STM32F30) || defined(STM32F37) || defined(STM32F7) || defined(STM32L0))
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value);
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState);
#endif /* !(STM32F0 || STM32F1 || STM32F30 || STM32F37 || STM32F7 || STM32L0) */
#if defined(RTC_ISR_RECALPF)
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod, 
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue);
#endif /* RTC_ISR_RECALPF */
#if defined(RTC_ISR_SHPF)
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS);
#endif /* RTC_ISR_SHPF */

/* -------------------------------------------------------------------------- */
/*                                ADC APIs                                     */
/* -------------------------------------------------------------------------- */

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  ADC Init structure definition  
  */

typedef struct
{
#if defined(STM32F1)	
  uint32_t ADC_Mode;                      /*!< Configures the ADC to operate in independent or
                                               dual mode. 
                                               This parameter can be a value of @ref ADC_mode */
#endif /* STM32F1 */

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F37)
  FunctionalState ADC_ScanConvMode;       /*!< Specifies whether the conversion is performed in
                                               Scan (multichannels) or Single (one channel) mode.
                                               This parameter can be set to ENABLE or DISABLE */
#endif /* STM32F1|STM32F2|STM32F4|STM32F7|STM32L1|STM32f37 */

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)||defined(STM32F30)||defined(STM32L4)||defined(STM32F37)
  FunctionalState ADC_ContinuousConvMode; /*!< Specifies whether the conversion is performed in
                                               Continuous or Single mode.
                                               This parameter can be set to ENABLE or DISABLE. */

  uint32_t ADC_DataAlign;                 /*!< Specifies whether the ADC data alignment is left or right.
                                               This parameter can be a value of @ref ADC_data_align */
#endif /* STM32F1|STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0|STM32F30|STM32L4|STM32F37*/

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)||defined(STM32F37)
  uint32_t ADC_ExternalTrigConv;          /*!< Defines the external trigger used to start the analog
                                               to digital conversion of regular channels. This parameter
                                               can be a value of @ref ADC_external_trigger_sources_for_regular_channels_conversion */
#endif /* STM32F1|STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0|STM32F37*/


#if defined(STM32F1)||defined(STM32F37)
  uint8_t ADC_NbrOfChannel;               /*!< Specifies the number of ADC channels that will be converted
                                               using the sequencer for regular channel group.
                                               This parameter must range from 1 to 16. */
#endif /* STM32F1|STM32F37*/

#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)||defined(STM32F30)||defined(STM32L4)
  uint32_t ADC_Resolution;                /*!< Configures the ADC resolution dual mode. 
                                               This parameter can be a value of @ref ADC_resolution */
#endif /* STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0|STM32F30|STM32L4 */

#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)
  uint32_t ADC_ExternalTrigConvEdge;      /*!< Select the external trigger edge and
                                               enable the trigger of a regular group. 
                                               This parameter can be a value of 
                                               @ref ADC_external_trigger_edge_for_regular_channels_conversion */
#endif /* STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0 */
		
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)
  uint8_t  ADC_NbrOfConversion;           /*!< Specifies the number of ADC conversions
                                               that will be done using the sequencer for
                                               regular channel group.
                                               This parameter must range from 1 to 16. */
#endif /* STM32F2|STM32F4|STM32F7|STM32L1 */
											   
#if defined(STM32F0)||defined(STM32L0)
  uint32_t  ADC_ScanDirection;              /*!< Specifies in which direction the channels will be scanned
                                                 in the sequence. 
                                                 This parameter can be a value of @ref ADC_Scan_Direction */

#endif /* STM32F0|STM32L0 */
											   
#if defined(STM32F30)||defined(STM32L4)
  uint32_t ADC_ExternalTrigConvEvent;      /*!< Defines the external trigger used to start the analog
                                               to digital conversion of regular channels. This parameter
                                               can be a value of @ref ADC_external_trigger_sources_for_regular_channels_conversion */

  uint32_t ADC_ExternalTrigEventEdge;      /*!< Select the external trigger edge and enable the trigger of a regular group.                                               
                                               This parameter can be a value of 
                                               @ref ADC_external_trigger_edge_for_regular_channels_conversion */

  uint32_t ADC_OverrunMode;                /*!< This parameter can be set to ENABLE or DISABLE. */

  uint32_t ADC_AutoInjMode;                /*!< This parameter can be set to ENABLE or DISABLE. */

  uint32_t ADC_NbrOfRegChannel;            /*!< Specifies the number of ADC channels that will be converted
                                               using the sequencer for regular channel group.
                                               This parameter must range from 1 to 16. */
#endif /* STM32F30|STM32L4 */
}LEGACY_ADC_InitTypeDef;


#if defined(STM32F30)||defined(STM32L4)

/** 
  * @brief   ADC Injected Init structure definition  
  */ 
typedef struct
{
   uint32_t ADC_ExternalTrigInjecConvEvent;     /*!< Defines the external trigger used to start the analog
                                                     to digital conversion of injected channels. This parameter
                                                     can be a value of @ref ADC_external_trigger_sources_for_Injected_channels_conversion */
  uint32_t ADC_ExternalTrigInjecEventEdge;     /*!< Select the external trigger edge and enable the trigger of an injected group. 
                                                    This parameter can be a value of 
                                                    @ref ADC_external_trigger_edge_for_Injected_channels_conversion */
  uint8_t ADC_NbrOfInjecChannel;               /*!< Specifies the number of ADC channels that will be converted
                                                    using the sequencer for injected channel group.
                                                    This parameter must range from 1 to 4. */ 
  uint32_t ADC_InjecSequence1; 
  uint32_t ADC_InjecSequence2;
  uint32_t ADC_InjecSequence3;
  uint32_t ADC_InjecSequence4;  
}LEGACY_ADC_InjectedInitTypeDef;

/** 
  * @brief   ADC Common Init structure definition  
  */ 
typedef struct
{
  uint32_t ADC_Mode;                      /*!< Configures the ADC to operate in 
                                               independent or multi mode. 
                                               This parameter can be a value of @ref ADC_mode */                                              
  uint32_t ADC_Clock;                    /*!< Select the clock of the ADC. The clock is common for both master 
                                              and slave ADCs.
                                              This parameter can be a value of @ref ADC_Clock */
  uint32_t ADC_DMAAccessMode;             /*!< Configures the Direct memory access mode for multi ADC mode.                                               
                                               This parameter can be a value of 
                                               @ref ADC_Direct_memory_access_mode_for_multi_mode */
  uint32_t ADC_DMAMode;                  /*!< Configures the DMA mode for ADC.                                             
                                              This parameter can be a value of @ref ADC_DMA_Mode_definition */
  uint8_t ADC_TwoSamplingDelay;          /*!< Configures the Delay between 2 sampling phases.
                                               This parameter can be a value between  0x0 and 0xF  */
  
}LEGACY_ADC_CommonInitTypeDef;

#endif /* STM32F30|STM32L4 */

/* Exported constants --------------------------------------------------------*/

#if defined(STM32F1)
#define ADC_Mode_Independent                       ((uint32_t)0x00000000)
#define ADC_Mode_RegInjecSimult                    ((uint32_t)0x00010000)
#define ADC_Mode_RegSimult_AlterTrig               ((uint32_t)0x00020000)
#define ADC_Mode_InjecSimult_FastInterl            ((uint32_t)0x00030000)
#define ADC_Mode_InjecSimult_SlowInterl            ((uint32_t)0x00040000)
#define ADC_Mode_InjecSimult                       ((uint32_t)0x00050000)
#define ADC_Mode_RegSimult                         ((uint32_t)0x00060000)
#define ADC_Mode_FastInterl                        ((uint32_t)0x00070000)
#define ADC_Mode_SlowInterl                        ((uint32_t)0x00080000)
#define ADC_Mode_AlterTrig                         ((uint32_t)0x00090000)
#endif /* STM32F1 */


/** @defgroup ADC_external_trigger_sources_for_regular_channels_conversion 
  * @{
  */
#if defined(STM32F1)
#define ADC_ExternalTrigConv_T1_CC1                ((uint32_t)0x00000000) /*!< For ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T1_CC2                ((uint32_t)0x00020000) /*!< For ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T2_CC2                ((uint32_t)0x00060000) /*!< For ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T3_TRGO               ((uint32_t)0x00080000) /*!< For ADC1 and ADC2 */
#define ADC_ExternalTrigConv_T4_CC4                ((uint32_t)0x000A0000) /*!< For ADC1 and ADC2 */
#define ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO    ((uint32_t)0x000C0000) /*!< For ADC1 and ADC2 */

#define ADC_ExternalTrigConv_T1_CC3                ((uint32_t)0x00040000) /*!< For ADC1, ADC2 and ADC3 */
#define ADC_ExternalTrigConv_None                  ((uint32_t)0x000E0000) /*!< For ADC1, ADC2 and ADC3 */

#define ADC_ExternalTrigConv_T3_CC1                ((uint32_t)0x00000000) /*!< For ADC3 only */
#define ADC_ExternalTrigConv_T2_CC3                ((uint32_t)0x00020000) /*!< For ADC3 only */
#define ADC_ExternalTrigConv_T8_CC1                ((uint32_t)0x00060000) /*!< For ADC3 only */
#define ADC_ExternalTrigConv_T8_TRGO               ((uint32_t)0x00080000) /*!< For ADC3 only */
#define ADC_ExternalTrigConv_T5_CC1                ((uint32_t)0x000A0000) /*!< For ADC3 only */
#define ADC_ExternalTrigConv_T5_CC3                ((uint32_t)0x000C0000) /*!< For ADC3 only */

#endif /* STM32F1*/
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)
#define ADC_ExternalTrigConv_T1_CC1                ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_T1_CC2                ((uint32_t)0x01000000)
#define ADC_ExternalTrigConv_T1_CC3                ((uint32_t)0x02000000)
#define ADC_ExternalTrigConv_T2_CC2                ((uint32_t)0x03000000)
#define ADC_ExternalTrigConv_T2_CC3                ((uint32_t)0x04000000)
#define ADC_ExternalTrigConv_T2_CC4                ((uint32_t)0x05000000)
#define ADC_ExternalTrigConv_T2_TRGO               ((uint32_t)0x06000000)
#define ADC_ExternalTrigConv_T3_CC1                ((uint32_t)0x07000000)
#define ADC_ExternalTrigConv_T3_TRGO               ((uint32_t)0x08000000)
#define ADC_ExternalTrigConv_T4_CC4                ((uint32_t)0x09000000)
#define ADC_ExternalTrigConv_T5_CC1                ((uint32_t)0x0A000000)
#define ADC_ExternalTrigConv_T5_CC2                ((uint32_t)0x0B000000)
#define ADC_ExternalTrigConv_T5_CC3                ((uint32_t)0x0C000000)
#define ADC_ExternalTrigConv_T8_CC1                ((uint32_t)0x0D000000)
#define ADC_ExternalTrigConv_T8_TRGO               ((uint32_t)0x0E000000)
#define ADC_ExternalTrigConv_Ext_IT11              ((uint32_t)0x0F000000)
#endif /* STM32F2|STM32F4|STM32F7*/

#if defined(STM32L1)
/* TIM2 */
#define ADC_ExternalTrigConv_T2_CC3                ((uint32_t)0x02000000)
#define ADC_ExternalTrigConv_T2_CC2                ((uint32_t)0x03000000)
#define ADC_ExternalTrigConv_T2_TRGO               ((uint32_t)0x06000000)

/* TIM3 */
#define ADC_ExternalTrigConv_T3_CC1                ((uint32_t)0x07000000)
#define ADC_ExternalTrigConv_T3_CC3                ((uint32_t)0x08000000)
#define ADC_ExternalTrigConv_T3_TRGO               ((uint32_t)0x04000000)

/* TIM4 */
#define ADC_ExternalTrigConv_T4_CC4                ((uint32_t)0x05000000)
#define ADC_ExternalTrigConv_T4_TRGO               ((uint32_t)0x09000000)

/* TIM6 */
#define ADC_ExternalTrigConv_T6_TRGO               ((uint32_t)0x0A000000)

/* TIM9 */
#define ADC_ExternalTrigConv_T9_CC2                ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_T9_TRGO               ((uint32_t)0x01000000)

/* EXTI */
#define ADC_ExternalTrigConv_Ext_IT11              ((uint32_t)0x0F000000)

#endif /* STM32L1*/

#if defined(STM32F0)||defined(STM32L0)
/* TIM1 */
#define ADC_ExternalTrigConv_T1_TRGO               ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_T1_CC4                ADC_CFGR1_EXTSEL_0

/* TIM2 */
#define ADC_ExternalTrigConv_T2_TRGO               ADC_CFGR1_EXTSEL_1

/* TIM3 */
#define ADC_ExternalTrigConv_T3_TRGO               ((uint32_t)(ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTSEL_1))

/* TIM15 */
#define ADC_ExternalTrigConv_T15_TRGO              ADC_CFGR1_EXTSEL_2

#define IS_ADC_EXTERNAL_TRIG_CONV(CONV) (((CONV) == ADC_ExternalTrigConv_T1_TRGO) || \
                                         ((CONV) == ADC_ExternalTrigConv_T1_CC4)   || \
                                         ((CONV) == ADC_ExternalTrigConv_T2_TRGO)  || \
                                         ((CONV) == ADC_ExternalTrigConv_T3_TRGO)  || \
                                         ((CONV) == ADC_ExternalTrigConv_T15_TRGO)) 
#endif /* STM32F0|STM32L0*/

#if defined(STM32F37)
#define ADC_ExternalTrigConv_T19_TRGO              ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_T19_CC3               ADC_CR2_EXTSEL_0
#define ADC_ExternalTrigConv_T19_CC4               ADC_CR2_EXTSEL_1
#define ADC_ExternalTrigConv_T2_CC2                ((uint32_t)0x00060000)
#define ADC_ExternalTrigConv_T3_TRGO               ADC_CR2_EXTSEL_2
#define ADC_ExternalTrigConv_T4_CC4                ((uint32_t)0x000A0000)
#define ADC_ExternalTrigConv_Ext_IT11              ((uint32_t)0x000C0000)
#define ADC_ExternalTrigConv_None                  ((uint32_t)0x000E0000)

#endif /* STM32F37*/
/**
  * @}
  */

/** @defgroup ADC_data_align 
  * @{
  */ 
#define ADC_DataAlign_Right                        ((uint32_t)0x00000000)
#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32F0)||defined(STM32F30)||defined(STM32L4)||defined(STM32F37)||defined(STM32L0)
#define ADC_DataAlign_Left                         ((uint32_t)0x00000800)
#elif defined(STM32L1)
#define ADC_DataAlign_Left                         ((uint32_t)0x00000020)
#endif /* STM32F1|STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0|STM32F30|STM32L4|STM32F37*/
/**
  * @}
  */ 

/** @defgroup ADC_resolution 
  * @{
  */
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32F30)||defined(STM32L4)||defined(STM32L0)
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1) 
#define ADC_Resolution_12b                         ((uint32_t)0x00000000)
#define ADC_Resolution_10b                         ((uint32_t)0x01000000)
#define ADC_Resolution_8b                          ((uint32_t)0x02000000)
#define ADC_Resolution_6b                          ((uint32_t)0x03000000)
#elif defined(STM32F0)||defined(STM32F30)||defined(STM32L4)||defined(STM32L0)
#define ADC_Resolution_12b                         ((uint32_t)0x00000000)  /*!<  ADC 12-bit resolution */
#define ADC_Resolution_10b                         ((uint32_t)0x00000008)  /*!<  ADC 10-bit resolution */
#define ADC_Resolution_8b                          ((uint32_t)0x00000010)  /*!<  ADC 8-bit resolution */
#define ADC_Resolution_6b                          ((uint32_t)0x00000018)  /*!<  ADC 6-bit resolution */

#endif
#endif
/**
  * @}
  */ 
  

/** @defgroup ADC_external_trigger_edge_for_regular_channels_conversion 
  * @{
  */ 
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32F30)||defined(STM32L4)||defined(STM32L0)
#define ADC_ExternalTrigConvEdge_None          ((uint32_t)0x00000000)
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)
#define ADC_ExternalTrigConvEdge_Rising        ((uint32_t)0x10000000)
#define ADC_ExternalTrigConvEdge_Falling       ((uint32_t)0x20000000)
#define ADC_ExternalTrigConvEdge_RisingFalling ((uint32_t)0x30000000)
#elif defined(STM32F0)||defined(STM32F30)||defined(STM32L4)||defined(STM32L0)
#define ADC_ExternalTrigConvEdge_Rising        ((uint32_t)0x00000400)
#define ADC_ExternalTrigConvEdge_Falling       ((uint32_t)0x00000800)
#define ADC_ExternalTrigConvEdge_RisingFalling ((uint32_t)0x00000C00)
#endif
#endif /* STM32F2|STM32F4|STM32F7|STM32L1|STM32F0|STM32L0|STM32F30|STM32L4 */

/**
  * @}
  */ 

/** @defgroup ADC_Scan_Direction 
  * @{
  */ 
#if defined(STM32F0)||defined(STM32L0)
#define ADC_ScanDirection_Upward                   ((uint32_t)0x00000000)
#define ADC_ScanDirection_Backward                 ADC_CFGR1_SCANDIR

#endif /* STM32F0|STM32L0 */
/**
  * @}
  */ 
  
#if defined(STM32F30)||defined(STM32L4)
/** @defgroup ADC_external_trigger_sources_for_regular_channels_conversion 
  * @{
  */
#define ADC_ExternalTrigConvEvent_0              ((uint16_t)0x0000)   /*!<  ADC external trigger event 0 */
#define ADC_ExternalTrigConvEvent_1              ((uint16_t)0x0040)   /*!<  ADC external trigger event 1 */
#define ADC_ExternalTrigConvEvent_2              ((uint16_t)0x0080)   /*!<  ADC external trigger event 2 */
#define ADC_ExternalTrigConvEvent_3              ((uint16_t)0x00C0)   /*!<  ADC external trigger event 3 */
#define ADC_ExternalTrigConvEvent_4              ((uint16_t)0x0100)   /*!<  ADC external trigger event 4 */
#define ADC_ExternalTrigConvEvent_5              ((uint16_t)0x0140)   /*!<  ADC external trigger event 5 */
#define ADC_ExternalTrigConvEvent_6              ((uint16_t)0x0180)   /*!<  ADC external trigger event 6 */
#define ADC_ExternalTrigConvEvent_7              ((uint16_t)0x01C0)   /*!<  ADC external trigger event 7 */
#define ADC_ExternalTrigConvEvent_8              ((uint16_t)0x0200)   /*!<  ADC external trigger event 8 */
#define ADC_ExternalTrigConvEvent_9              ((uint16_t)0x0240)   /*!<  ADC external trigger event 9 */
#define ADC_ExternalTrigConvEvent_10             ((uint16_t)0x0280)   /*!<  ADC external trigger event 10 */
#define ADC_ExternalTrigConvEvent_11             ((uint16_t)0x02C0)   /*!<  ADC external trigger event 11 */
#define ADC_ExternalTrigConvEvent_12             ((uint16_t)0x0300)   /*!<  ADC external trigger event 12 */
#define ADC_ExternalTrigConvEvent_13             ((uint16_t)0x0340)   /*!<  ADC external trigger event 13 */
#define ADC_ExternalTrigConvEvent_14             ((uint16_t)0x0380)   /*!<  ADC external trigger event 14 */
#define ADC_ExternalTrigConvEvent_15             ((uint16_t)0x03C0)   /*!<  ADC external trigger event 15 */

/**
  * @}
  */
/** @defgroup ADC_external_trigger_sources_for_Injected_channels_conversion 
  * @{
  */
        
#define ADC_ExternalTrigInjecConvEvent_0              ((uint16_t)0x0000)  /*!<  ADC external trigger for injected conversion event 0 */
#define ADC_ExternalTrigInjecConvEvent_1              ((uint16_t)0x0004)  /*!<  ADC external trigger for injected conversion event 1 */
#define ADC_ExternalTrigInjecConvEvent_2              ((uint16_t)0x0008)  /*!<  ADC external trigger for injected conversion event 2 */
#define ADC_ExternalTrigInjecConvEvent_3              ((uint16_t)0x000C)  /*!<  ADC external trigger for injected conversion event 3 */
#define ADC_ExternalTrigInjecConvEvent_4              ((uint16_t)0x0010)  /*!<  ADC external trigger for injected conversion event 4 */
#define ADC_ExternalTrigInjecConvEvent_5              ((uint16_t)0x0014)  /*!<  ADC external trigger for injected conversion event 5 */
#define ADC_ExternalTrigInjecConvEvent_6              ((uint16_t)0x0018)  /*!<  ADC external trigger for injected conversion event 6 */
#define ADC_ExternalTrigInjecConvEvent_7              ((uint16_t)0x001C)  /*!<  ADC external trigger for injected conversion event 7 */
#define ADC_ExternalTrigInjecConvEvent_8              ((uint16_t)0x0020)  /*!<  ADC external trigger for injected conversion event 8 */
#define ADC_ExternalTrigInjecConvEvent_9              ((uint16_t)0x0024)  /*!<  ADC external trigger for injected conversion event 9 */
#define ADC_ExternalTrigInjecConvEvent_10             ((uint16_t)0x0028)  /*!<  ADC external trigger for injected conversion event 10 */
#define ADC_ExternalTrigInjecConvEvent_11             ((uint16_t)0x002C)  /*!<  ADC external trigger for injected conversion event 11 */
#define ADC_ExternalTrigInjecConvEvent_12             ((uint16_t)0x0030)  /*!<  ADC external trigger for injected conversion event 12 */
#define ADC_ExternalTrigInjecConvEvent_13             ((uint16_t)0x0034)  /*!<  ADC external trigger for injected conversion event 13 */
#define ADC_ExternalTrigInjecConvEvent_14             ((uint16_t)0x0038)  /*!<  ADC external trigger for injected conversion event 14 */
#define ADC_ExternalTrigInjecConvEvent_15             ((uint16_t)0x003C)  /*!<  ADC external trigger for injected conversion event 15 */

/**
  * @}
  */
/** @defgroup ADC_external_trigger_edge_for_Injected_channels_conversion 
  * @{
  */     
#define ADC_ExternalTrigInjecEventEdge_None		     ((uint16_t)0x0000)    /*!<  ADC No external trigger for regular conversion */
#define ADC_ExternalTrigInjecEventEdge_RisingEdge	 ((uint16_t)0x0040)    /*!<  ADC external trigger rising edge for injected conversion */
#define ADC_ExternalTrigInjecEventEdge_FallingEdge	 ((uint16_t)0x0080)  /*!<  ADC external trigger falling edge for injected conversion */
#define ADC_ExternalTrigInjecEventEdge_BothEdge	     ((uint16_t)0x00C0)  /*!<  ADC external trigger both edges for injected conversion */

  /**
  * @}
  */
/** @defgroup ADC_mode 
  * @{
  */    
#define ADC_Mode_Independent                  ((uint32_t)0x00000000) /*!<  ADC independent mode */
#define ADC_Mode_CombRegSimulInjSimul         ((uint32_t)0x00000001) /*!<  ADC multi ADC mode: Combined Regular simultaneous injected simultaneous mode */
#define ADC_Mode_CombRegSimulAltTrig          ((uint32_t)0x00000002) /*!<  ADC multi ADC mode: Combined Regular simultaneous Alternate trigger mode */
#define ADC_Mode_InjSimul                     ((uint32_t)0x00000005) /*!<  ADC multi ADC mode: Injected simultaneous mode */
#define ADC_Mode_RegSimul                     ((uint32_t)0x00000006) /*!<  ADC multi ADC mode: Regular simultaneous mode */
#define ADC_Mode_Interleave                   ((uint32_t)0x00000007) /*!<  ADC multi ADC mode: Interleave mode */
#define ADC_Mode_AltTrig                      ((uint32_t)0x00000009) /*!<  ADC multi ADC mode: Alternate Trigger mode */

                                     
/**
  * @}
  */
/** @defgroup ADC_Clock 
  * @{
  */ 
#define ADC_Clock_AsynClkMode                  ((uint32_t)0x00000000)   /*!< ADC Asynchronous clock mode */
#define ADC_Clock_SynClkModeDiv1               ((uint32_t)0x00010000)   /*!< Synchronous clock mode divided by 1 */
#define ADC_Clock_SynClkModeDiv2               ((uint32_t)0x00020000)   /*!<  Synchronous clock mode divided by 2 */
#define ADC_Clock_SynClkModeDiv4               ((uint32_t)0x00030000)   /*!<  Synchronous clock mode divided by 4 */

/**
  * @}
  */
/** @defgroup ADC_Direct_memory_access_mode_for_multi_mode 
  * @{
  */ 
#define ADC_DMAAccessMode_Disabled      ((uint32_t)0x00000000)     /*!<  DMA mode disabled */
#define ADC_DMAAccessMode_1             ((uint32_t)0x00008000)     /*!<  DMA mode enabled for 12 and 10-bit resolution (6 bit) */
#define ADC_DMAAccessMode_2             ((uint32_t)0x0000C000)     /*!<  DMA mode enabled for 8 and 6-bit resolution (8bit) */
                                     
/**
  * @}
  */
/** @defgroup ADC_DMA_Mode_definition 
  * @{
  */
#define ADC_DMAMode_OneShot	   ((uint32_t)0x00000000)   /*!<  ADC DMA Oneshot mode */
#define ADC_DMAMode_Circular   ((uint32_t)0x00000002)   /*!<  ADC DMA circular mode */

/**
  * @}
  */
  
/** @defgroup ADC_external_trigger_edge_for_regular_channels_conversion 
  * @{
  */
#define ADC_ExternalTrigEventEdge_None            ((uint16_t)0x0000)     /*!<  ADC No external trigger for regular conversion */
#define ADC_ExternalTrigEventEdge_RisingEdge      ((uint16_t)0x0400)     /*!<  ADC external trigger rising edge for regular conversion */
#define ADC_ExternalTrigEventEdge_FallingEdge     ((uint16_t)0x0800)     /*!<  ADC ADC external trigger falling edge for regular conversion */
#define ADC_ExternalTrigEventEdge_BothEdge        ((uint16_t)0x0C00)     /*!<  ADC ADC external trigger both edges for regular conversion */

  
/**
  * @}
  */

/** @defgroup ADC_injected_Channel_selection 
  * @{
  */

#define ADC_InjectedChannel_1                       ((uint8_t)0x01)       /*!<  ADC Injected channel 1 */
#define ADC_InjectedChannel_2                       ((uint8_t)0x02)       /*!<  ADC Injected channel 2 */
#define ADC_InjectedChannel_3                       ((uint8_t)0x03)       /*!<  ADC Injected channel 3 */
#define ADC_InjectedChannel_4                       ((uint8_t)0x04)       /*!<  ADC Injected channel 4 */
#define ADC_InjectedChannel_5                       ((uint8_t)0x05)       /*!<  ADC Injected channel 5 */
#define ADC_InjectedChannel_6                       ((uint8_t)0x06)       /*!<  ADC Injected channel 6 */
#define ADC_InjectedChannel_7                       ((uint8_t)0x07)       /*!<  ADC Injected channel 7 */
#define ADC_InjectedChannel_8                       ((uint8_t)0x08)       /*!<  ADC Injected channel 8 */
#define ADC_InjectedChannel_9                       ((uint8_t)0x09)       /*!<  ADC Injected channel 9 */
#define ADC_InjectedChannel_10                      ((uint8_t)0x0A)       /*!<  ADC Injected channel 10 */
#define ADC_InjectedChannel_11                      ((uint8_t)0x0B)       /*!<  ADC Injected channel 11 */
#define ADC_InjectedChannel_12                      ((uint8_t)0x0C)       /*!<  ADC Injected channel 12 */
#define ADC_InjectedChannel_13                      ((uint8_t)0x0D)       /*!<  ADC Injected channel 13 */
#define ADC_InjectedChannel_14                      ((uint8_t)0x0E)       /*!<  ADC Injected channel 14 */
#define ADC_InjectedChannel_15                      ((uint8_t)0x0F)       /*!<  ADC Injected channel 15 */
#define ADC_InjectedChannel_16                      ((uint8_t)0x10)       /*!<  ADC Injected channel 16 */
#define ADC_InjectedChannel_17                      ((uint8_t)0x11)       /*!<  ADC Injected channel 17 */
#define ADC_InjectedChannel_18                      ((uint8_t)0x12)       /*!<  ADC Injected channel 18 */

/**
  * @}
  */
									   
#endif /* STM32F30|STM32L4 */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ADC_Init(ADC_TypeDef* ADCx, LEGACY_ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(LEGACY_ADC_InitTypeDef* ADC_InitStruct);
#if defined (STM32F30)
void ADC_InjectedInit(ADC_TypeDef* ADCx, LEGACY_ADC_InjectedInitTypeDef* ADC_InjectedInitStruct); 
void ADC_InjectedStructInit(LEGACY_ADC_InjectedInitTypeDef* ADC_InjectedInitStruct);
#if defined(ADC12_COMMON)
void ADC_CommonInit(ADC_TypeDef* ADCx, LEGACY_ADC_CommonInitTypeDef* ADC_CommonInitStruct);   
void ADC_CommonStructInit(LEGACY_ADC_CommonInitTypeDef* ADC_CommonInitStruct);
#endif /* ADC12_COMMON */
#endif /* STM32F30 */

/* -------------------------------------------------------------------------- */
/*                                RCC API                                     */
/* -------------------------------------------------------------------------- */
ErrorStatus RCC_WaitForHSEStartUp(void);


/* -------------------------------------------------------------------------- */
/*                               HRTIM APIs                                   */
/* -------------------------------------------------------------------------- */

#if defined(STM32F334x8)

/* Exported types ------------------------------------------------------------*/ 

/** 
  * @brief  HRTIM Configuration Structure definition - Time base related parameters
  */
typedef struct
{
  uint32_t Period;                 /*!< Specifies the timer period
                                        The period value must be above 3 periods of the fHRTIM clock.
                                        Maximum value is = 0xFFDF */
  uint32_t RepetitionCounter;      /*!< Specifies the timer repetition period
                                        This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF. */ 
  uint32_t PrescalerRatio;         /*!< Specifies the timer clock prescaler ratio. 
                                        This parameter can be any value of @ref HRTIM_PrescalerRatio   */           
  uint32_t Mode;                   /*!< Specifies the counter operating mode
                                        This parameter can be any value of @ref HRTIM_Mode   */           
} LEGACY_HRTIM_BaseInitTypeDef;
/** 
  * @brief  Waveform mode initialization parameters definition
  */
typedef struct {
  uint32_t HalfModeEnable;    /*!< Specifies whether or not half mode is enabled
                                   This parameter can be a combination of @ref HRTIM_HalfModeEnable  */
  uint32_t StartOnSync;       /*!< Specifies whether or not timer is reset by a rising edge on the synchronization input (when enabled)
                                   This parameter can be a combination of @ref HRTIM_StartOnSyncInputEvent  */
  uint32_t ResetOnSync;        /*!< Specifies whether or not timer is reset by a rising edge on the synchronization input (when enabled)
                                   This parameter can be a combination of @ref HRTIM_ResetOnSyncInputEvent  */
  uint32_t DACSynchro;        /*!< Indicates whether or not the a DAC synchronization event is generated 
                                   This parameter can be any value of @ref HRTIM_DACSynchronization   */
  uint32_t PreloadEnable;     /*!< Specifies whether or not register preload is enabled
                                   This parameter can be a combination of @ref HRTIM_RegisterPreloadEnable  */
  uint32_t UpdateGating;      /*!< Specifies how the update occurs with respect to a burst DMA transaction or
                                   update enable inputs (Slave timers only)  
                                   This parameter can be any value of @ref HRTIM_UpdateGating   */
  uint32_t BurstMode;         /*!< Specifies how the timer behaves during a burst mode operation
                                    This parameter can be a combination of @ref HRTIM_TimerBurstMode  */
  uint32_t RepetitionUpdate;  /*!< Specifies whether or not registers update is triggered by the repetition event 
                                   This parameter can be a combination of @ref HRTIM_TimerRepetitionUpdate */
} LEGACY_HRTIM_TimerInitTypeDef;

/** 
  * @brief  Basic output compare mode configuration definition
  */
typedef struct {
  uint32_t Mode;       /*!< Specifies the output compare mode (toggle, active, inactive)
                            This parameter can be a combination of @ref HRTIM_BasicOCMode */ 
  uint32_t Pulse;      /*!< Specifies the compare value to be loaded into the Compare Register. 
                            The compare value must be above or equal to 3 periods of the fHRTIM clock */
  uint32_t Polarity;   /*!< Specifies the output polarity 
                            This parameter can be any value of @ref HRTIM_Output_Polarity */
  uint32_t IdleState;  /*!< Specifies whether the output level is active or inactive when in IDLE state  
                            This parameter can be any value of @ref HRTIM_OutputIDLEState */
} LEGACY_HRTIM_BasicOCChannelCfgTypeDef;

/** 
  * @brief  Basic PWM output mode configuration definition
  */
typedef struct {
  uint32_t Pulse;            /*!< Specifies the compare value to be loaded into the Compare Register. 
                                  The compare value must be above or equal to 3 periods of the fHRTIM clock */
  uint32_t Polarity;        /*!< Specifies the output polarity 
                                 This parameter can be any value of @ref HRTIM_OutputPolarity */
  uint32_t IdleState;       /*!< Specifies whether the output level is active or inactive when in IDLE state  
                                 This parameter can be any value of @ref HRTIM_OutputIDLEState */
} LEGACY_HRTIM_BasicPWMChannelCfgTypeDef;

/** 
  * @brief  Basic capture mode configuration definition
  */
typedef struct {
  uint32_t CaptureUnit;      /*!< Specifies the external event Channel 
                                   This parameter can be any 'EEVx' value of @ref HRTIM_CaptureUnit */
  uint32_t Event;             /*!< Specifies the external event triggering the capture 
                                   This parameter can be any 'EEVx' value of @ref HRTIM_ExternalEventChannels */
  uint32_t EventPolarity;     /*!< Specifies the polarity of the external event (in case of level sensitivity) 
                                   This parameter can be a value of @ref HRTIM_ExternalEventPolarity */ 
  uint32_t EventSensitivity;  /*!< Specifies the sensitivity of the external event 
                                   This parameter can be a value of @ref HRTIM_ExternalEventSensitivity */ 
  uint32_t EventFilter;       /*!< Defines the frequency used to sample the External Event and the length of the digital filter 
                                   This parameter can be a value of @ref HRTIM_ExternalEventFilter */ 
} LEGACY_HRTIM_BasicCaptureChannelCfgTypeDef;

/** 
  * @brief  Basic One Pulse mode configuration definition
  */
typedef struct {
  uint32_t Pulse;             /*!< Specifies the compare value to be loaded into the Compare Register. 
                                   The compare value must be above or equal to 3 periods of the fHRTIM clock */
  uint32_t OutputPolarity;    /*!< Specifies the output polarity 
                                   This parameter can be any value of @ref HRTIM_Output_Polarity */
  uint32_t OutputIdleState;   /*!< Specifies whether the output level is active or inactive when in IDLE state  
                                   This parameter can be any value of @ref HRTIM_Output_IDLE_State */
  uint32_t Event;             /*!< Specifies the external event triggering the pulse generation 
                                   This parameter can be any 'EEVx' value of @ref HRTIM_Capture_Unit_Trigger */
  uint32_t EventPolarity;     /*!< Specifies the polarity of the external event (in case of level sensitivity) 
                                   This parameter can be a value of @ref HRTIM_ExternalEventPolarity */ 
  uint32_t EventSensitivity;  /*!< Specifies the sensitivity of the external event 
                                   This parameter can be a value of @ref HRTIM_ExternalEventSensitivity */ 
  uint32_t EventFilter;       /*!< Defines the frequency used to sample the External Event and the length of the digital filter 
                                   This parameter can be a value of @ref HRTIM_ExternalEventFilter */ 
} LEGACY_HRTIM_BasicOnePulseChannelCfgTypeDef;

/** 
  * @brief  Timer configuration definition
  */
typedef struct {
  uint32_t PushPull;                  /*!< Specifies whether or not the push-pull mode is enabled
                                           This parameter can be a value of @ref HRTIM_TimerPushPullMode */
  uint32_t FaultEnable;               /*!< Specifies which fault channels are enabled for the timer
                                           This parameter can be a combination of @ref HRTIM_TimerFaultEnabling  */
  uint32_t FaultLock;                 /*!< Specifies whether or not fault enabling status is write protected
                                           This parameter can be a value of @ref HRTIM_TimerFaultLock */
  uint32_t DeadTimeInsertion;         /*!< Specifies whether or not dead time insertion is enabled for the timer
                                           This parameter can be a value of @ref HRTIM_TimerDeadtimeInsertion */
  uint32_t DelayedProtectionMode;     /*!< Specifies the delayed protection mode 
                                          This parameter can be a value of @ref HRTIM_TimerDelayedProtectionMode */
  uint32_t UpdateTrigger;             /*!< Specifies source(s) triggering the timer registers update 
                                            This parameter can be a combination of @ref HRTIM_TimerUpdateTrigger */
  uint32_t ResetTrigger;              /*!< Specifies source(s) triggering the timer counter reset 
                                           This parameter can be a combination of @ref HRTIM_TimerResetTrigger */
  uint32_t ResetUpdate;              /*!< Specifies whether or not registers update is triggered when the timer counter is reset 
                                           This parameter can be a combination of @ref HRTIM_TimerResetUpdate */
} LEGACY_HRTIM_TimerCfgTypeDef;

/** 
  * @brief  Compare unit configuration definition
  */
typedef struct {
  uint32_t CompareValue;         /*!< Specifies the compare value of the timer compare unit 
                                      the minimum value must be greater than or equal to 3 periods of the fHRTIM clock
                                      the maximum value must be less than or equal to 0xFFFF - 1 periods of the fHRTIM clock */
  uint32_t AutoDelayedMode;      /*!< Specifies the auto delayed mode for compare unit 2 or 4 
                                      This parameter can be a value of @ref HRTIM_CompareUnitAutoDelayedMode */
  uint32_t AutoDelayedTimeout;   /*!< Specifies compare value for timing unit 1 or 3 when auto delayed mode with time out is selected 
                                      CompareValue +  AutoDelayedTimeout must be less than 0xFFFF */
} LEGACY_HRTIM_CompareCfgTypeDef;

/** 
  * @brief  Capture unit configuration definition
  */
typedef struct {
  uint32_t Trigger;   /*!< Specifies source(s) triggering the capture 
                           This parameter can be a combination of @ref HRTIM_CaptureUnitTrigger */
} LEGACY_HRTIM_CaptureCfgTypeDef;

/** 
  * @brief  Output configuration definition
  */
typedef struct {
  uint32_t Polarity;              /*!< Specifies the output polarity 
                                       This parameter can be any value of @ref HRTIM_Output_Polarity */
  uint32_t SetSource;             /*!< Specifies the event(s) transitioning the output from its inactive level to its active level  
                                       This parameter can be any value of @ref HRTIM_OutputSetSource */
  uint32_t ResetSource;           /*!< Specifies the event(s) transitioning the output from its active level to its inactive level  
                                       This parameter can be any value of @ref HRTIM_OutputResetSource */
  uint32_t IdleMode;              /*!< Specifies whether or not the output is affected by a burst mode operation  
                                       This parameter can be any value of @ref HRTIM_OutputIdleMode */
  uint32_t IdleState;             /*!< Specifies whether the output level is active or inactive when in IDLE state  
                                       This parameter can be any value of @ref HRTIM_OutputIDLEState */
  uint32_t FaultState;            /*!< Specifies whether the output level is active or inactive when in FAULT state  
                                       This parameter can be any value of @ref HRTIM_OutputFAULTState */
  uint32_t ChopperModeEnable;     /*!< Indicates whether or not the chopper mode is enabled 
                                       This parameter can be any value of @ref HRTIM_OutputChopperModeEnable */
  uint32_t BurstModeEntryDelayed;  /* !<Indicates whether or not deadtime is inserted when entering the IDLE state
                                        during a burst mode operation
                                        This parameters can be any value of @ref HRTIM_OutputBurstModeEntryDelayed */
} LEGACY_HRTIM_OutputCfgTypeDef;

/** 
  * @brief  External event filtering in timing units configuration definition
  */ 
typedef struct {
  uint32_t Filter;       /*!< Specifies the type of event filtering within the timing unit 
                             This parameter can be a value of @ref HRTIM_TimerExternalEventFilter */ 
  uint32_t Latch;       /*!< Specifies whether or not the signal is latched
                             This parameter can be a value of @ref HRTIM_TimerExternalEventLatch */
} LEGACY_HRTIM_TimerEventFilteringCfgTypeDef;

/** 
  * @brief  Dead time feature configuration definition
  */
typedef struct {
  uint32_t Prescaler;       /*!< Specifies the Deadtime Prescaler 
                                 This parameter can be a number between 0x0 and = 0x7 */ 
  uint32_t RisingValue;     /*!< Specifies the Deadtime following a rising edge 
                                 This parameter can be a number between 0x0 and 0xFF */ 
  uint32_t RisingSign;      /*!< Specifies whether the deadtime is positive or negative on rising edge
                                 This parameter can be a value of @ref HRTIM_DeadtimeRisingSign */ 
  uint32_t RisingLock;      /*!< Specifies whether or not deadtime rising settings (value and sign) are write protected 
                                 This parameter can be a value of @ref HRTIM_DeadtimeRisingLock */ 
  uint32_t RisingSignLock;  /*!< Specifies whether or not deadtime rising sign is write protected 
                                 This parameter can be a value of @ref HRTIM_DeadtimeRisingSignLock */ 
  uint32_t FallingValue;    /*!< Specifies the Deadtime following a falling edge 
                                This parameter can be a number between 0x0 and 0xFF */ 
  uint32_t FallingSign;     /*!< Specifies whether the deadtime is positive or negative on falling edge 
                                This parameter can be a value of @ref HRTIM_DeadtimeFallingSign */ 
  uint32_t FallingLock;     /*!< Specifies whether or not deadtime falling settings (value and sign) are write protected 
                                This parameter can be a value of @ref HRTIM_DeadtimeFallingLock */ 
  uint32_t FallingSignLock; /*!< Specifies whether or not deadtime falling sign is write protected 
                                This parameter can be a value of @ref HRTIM_DeadtimeFallingSignLock */ 
} LEGACY_HRTIM_DeadTimeCfgTypeDef;

/** 
  * @brief  Chopper mode configuration definition
  */
typedef struct {
  uint32_t CarrierFreq;  /*!< Specifies the Timer carrier frequency value.
                              This parameter can be a value between 0 and 0xF */
  uint32_t DutyCycle;   /*!< Specifies the Timer chopper duty cycle value.
                             This parameter can be a value between 0 and 0x7 */
  uint32_t StartPulse;  /*!< Specifies the Timer pulse width value.
                             This parameter can be a value between 0 and 0xF */   
} LEGACY_HRTIM_ChopperModeCfgTypeDef;

/** 
  * @brief  Master synchronization configuration definition
  */
typedef struct {
  uint32_t SyncInputSource;     /*!< Specifies the external synchronization input source 
                                     This parameter can be a value of @ref HRTIM_SynchronizationInputSource */
  uint32_t SyncOutputSource;    /*!< Specifies the source and event to be sent on the external synchronization outputs 
                                     This parameter can be a value of @ref HRTIM_SynchronizationOutputSource */
  uint32_t SyncOutputPolarity;  /*!< Specifies the conditioning of the event to be sent on the external synchronization outputs 
                                     This parameter can be a value of @ref HRTIM_SynchronizationOutputPolarity */
} LEGACY_HRTIM_SynchroCfgTypeDef;

/** 
  * @brief  External event channel configuration definition
  */ 
typedef struct {
  uint32_t Source;        /*!< Identifies the source of the external event 
                                This parameter can be a value of @ref HRTIM_ExternalEventSources */ 
  uint32_t Polarity;      /*!< Specifies the polarity of the external event (in case of level sensitivity) 
                               This parameter can be a value of @ref HRTIM_ExternalEventPolarity */ 
  uint32_t Sensitivity;   /*!< Specifies the sensitivity of the external event 
                               This parameter can be a value of @ref HRTIM_ExternalEventSensitivity */ 
  uint32_t Filter;        /*!< Defines the frequency used to sample the External Event and the length of the digital filter 
                               This parameter can be a value of @ref HRTIM_ExternalEventFilter */ 
  uint32_t FastMode;     /*!< Indicates whether or not low latency mode is enabled for the external event 
                              This parameter can be a value of @ref HRTIM_ExternalEventFastMode */
} LEGACY_HRTIM_EventCfgTypeDef;

/** 
  * @brief  Fault channel configuration definition
  */ 
typedef struct {
  uint32_t Source;        /*!< Identifies the source of the fault 
                                This parameter can be a value of @ref HRTIM_FaultSources */ 
  uint32_t Polarity;      /*!< Specifies the polarity of the fault event 
                               This parameter can be a value of @ref HRTIM_FaultPolarity */ 
  uint32_t Filter;        /*!< Defines the frequency used to sample the Fault input and the length of the digital filter 
                               This parameter can be a value of @ref HRTIM_FaultFilter */ 
  uint32_t Lock;          /*!< Indicates whether or not fault programming bits are write protected 
                              This parameter can be a value of @ref HRTIM_FaultLock */
} LEGACY_HRTIM_FaultCfgTypeDef;

/** 
  * @brief  Burst mode configuration definition
  */
typedef struct {
  uint32_t Mode;           /*!< Specifies the burst mode operating mode
                                This parameter can be a value of @ref HRTIM_BurstModeOperatingMode */
  uint32_t ClockSource;    /*!< Specifies the burst mode clock source
                                This parameter can be a value of @ref HRTIM_BurstModeClockSource */
  uint32_t Prescaler;      /*!< Specifies the burst mode prescaler
                                This parameter can be a value of @ref HRTIM_BurstModePrescaler */
  uint32_t PreloadEnable;  /*!< Specifies whether or not preload is enabled for burst mode related registers (HRTIM_BMCMPR and HRTIM_BMPER)
                                This parameter can be a combination of @ref HRTIM_BurstModeRegisterPreloadEnable  */
  uint32_t Trigger;        /*!< Specifies the event(s) triggering the burst operation 
                                This parameter can be a combination of @ref HRTIM_BurstModeTrigger  */
  uint32_t IdleDuration;   /*!< Specifies number of periods during which the selected timers are in idle state 
                                This parameter can be a number between 0x0 and 0xFFFF  */
  uint32_t Period;        /*!< Specifies burst mode repetition period 
                                This parameter can be a number between 0x1 and 0xFFFF  */
} LEGACY_HRTIM_BurstModeCfgTypeDef;

/** 
  * @brief  ADC trigger configuration definition
  */
typedef struct {
  uint32_t UpdateSource;  /*!< Specifies the ADC trigger update source  
                               This parameter can be a combination of @ref HRTIM_ADCTriggerUpdateSource  */
  uint32_t Trigger;      /*!< Specifies the event(s) triggering the ADC conversion  
                              This parameter can be a combination of @ref HRTIM_ADCTriggerEvent  */
} LEGACY_HRTIM_ADCTriggerCfgTypeDef;


/* Exported constants --------------------------------------------------------*/
/** @defgroup HRTIM_Exported_Constants
  * @{
  */

/** @defgroup HRTIM_BasicOCMode
  * @{
  * @brief Constants defining the behaviour of the output signal when the timer
           operates in basic output compare mode
  */              
#define HRTIM_BASICOCMODE_TOGGLE    (0x00000001U)  /*!< Output toggles when the timer counter reaches the compare value */
#define HRTIM_BASICOCMODE_INACTIVE  (0x00000002U)  /*!< Output forced to active level when the timer counter reaches the compare value */
#define HRTIM_BASICOCMODE_ACTIVE    (0x00000003U)  /*!< Output forced to inactive level when the timer counter reaches the compare value */

#define IS_HRTIM_BASICOCMODE(BASICOCMODE)\
              (((BASICOCMODE) == HRTIM_BASICOCMODE_TOGGLE)   || \
               ((BASICOCMODE) == HRTIM_BASICOCMODE_INACTIVE) || \
               ((BASICOCMODE) == HRTIM_BASICOCMODE_ACTIVE))
/**
  * @}
  */

/** @defgroup HRTIM_CompareUnit 
  * @{
  * @brief Constants defining compare unit identifiers
  */  
#define HRTIM_COMPAREUNIT_1 0x00000001U  /*!< Compare unit 1 identifier */
#define HRTIM_COMPAREUNIT_2 0x00000002U  /*!< Compare unit 2 identifier */
#define HRTIM_COMPAREUNIT_3 0x00000004U  /*!< Compare unit 3 identifier */
#define HRTIM_COMPAREUNIT_4 0x00000008U  /*!< Compare unit 4 identifier */

#define IS_HRTIM_COMPAREUNIT(COMPAREUNIT)\
    (((COMPAREUNIT) == HRTIM_COMPAREUNIT_1)  || \
     ((COMPAREUNIT) == HRTIM_COMPAREUNIT_2)  || \
     ((COMPAREUNIT) == HRTIM_COMPAREUNIT_3)  || \
     ((COMPAREUNIT) == HRTIM_COMPAREUNIT_4))
 /**
  * @}
  */

/** @defgroup HRTIM_FaultChannel
  * @{
  * @brief Constants defining fault channel identifiers
  */ 
#define HRTIM_FAULT_1      (0x01U)     /*!< Fault channel 1 identifier */
#define HRTIM_FAULT_2      (0x02U)     /*!< Fault channel 2 identifier */
#define HRTIM_FAULT_3      (0x04U)     /*!< Fault channel 3 identifier */
#define HRTIM_FAULT_4      (0x08U)     /*!< Fault channel 4 identifier */
#define HRTIM_FAULT_5      (0x10U)     /*!< Fault channel 5 identifier */

#define IS_HRTIM_FAULT(FAULT)\
      (((FAULT) == HRTIM_FAULT_1)   || \
       ((FAULT) == HRTIM_FAULT_2)   || \
       ((FAULT) == HRTIM_FAULT_3)   || \
       ((FAULT) == HRTIM_FAULT_4)   || \
       ((FAULT) == HRTIM_FAULT_5))
/**
  * @}
  */
  
/** @defgroup HRTIM_ADCTrigger
  * @{
  * @brief Constants defining ADC triggers identifiers
  */
#define HRTIM_ADCTRIGGER_1  0x00000001U  /*!< ADC trigger 1 identifier */
#define HRTIM_ADCTRIGGER_2  0x00000002U  /*!< ADC trigger 2 identifier */
#define HRTIM_ADCTRIGGER_3  0x00000004U  /*!< ADC trigger 3 identifier */
#define HRTIM_ADCTRIGGER_4  0x00000008U  /*!< ADC trigger 4 identifier */

#define IS_HRTIM_ADCTRIGGER(ADCTRIGGER)\
    (((ADCTRIGGER) == HRTIM_ADCTRIGGER_1)   || \
     ((ADCTRIGGER) == HRTIM_ADCTRIGGER_2)   || \
     ((ADCTRIGGER) == HRTIM_ADCTRIGGER_3)   || \
     ((ADCTRIGGER) == HRTIM_ADCTRIGGER_4))
/**
  * @}
  */
  
/** @defgroup HRTIM_TimerIndex 
  * @{
  * @brief Constants defining the timer indexes
  */
#define HRTIM_TIMERINDEX_TIMER_A 0x0U   /*!< Index used to access timer A registers */
#define HRTIM_TIMERINDEX_TIMER_B 0x1U   /*!< Index used to access timer B registers */
#define HRTIM_TIMERINDEX_TIMER_C 0x2U   /*!< Index used to access timer C registers */
#define HRTIM_TIMERINDEX_TIMER_D 0x3U   /*!< Index used to access timer D registers */
#define HRTIM_TIMERINDEX_TIMER_E 0x4U   /*!< Index used to access timer E registers */
#define HRTIM_TIMERINDEX_MASTER  0x5U   /*!< Index used to access master registers */
#define HRTIM_COMMONINDEX        0x6U   /*!< Index associated to Common space */

#define IS_HRTIM_TIMERINDEX(TIMERINDEX)\
    (((TIMERINDEX) == HRTIM_TIMERINDEX_MASTER)   || \
     ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_A)  || \
     ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_B)  || \
     ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_C)  || \
     ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_D)  || \
     ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_E))

#define IS_HRTIM_TIMING_UNIT(TIMERINDEX)\
     (((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_A)  || \
      ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_B)  || \
      ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_C)  || \
      ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_D)  || \
      ((TIMERINDEX) == HRTIM_TIMERINDEX_TIMER_E))
/**
  * @}
  */
  
 
/** @defgroup HRTIM_TimerOutput 
  * @{
  * @brief Constants defining timer output identifiers
  */  
#define HRTIM_OUTPUT_TA1  0x00000001U  /*!< Timer A - Output 1 identifier */
#define HRTIM_OUTPUT_TA2  0x00000002U  /*!< Timer A - Output 2 identifier */
#define HRTIM_OUTPUT_TB1  0x00000004U  /*!< Timer B - Output 1 identifier */
#define HRTIM_OUTPUT_TB2  0x00000008U  /*!< Timer B - Output 2 identifier */
#define HRTIM_OUTPUT_TC1  0x00000010U  /*!< Timer C - Output 1 identifier */
#define HRTIM_OUTPUT_TC2  0x00000020U  /*!< Timer C - Output 2 identifier */
#define HRTIM_OUTPUT_TD1  0x00000040U  /*!< Timer D - Output 1 identifier */
#define HRTIM_OUTPUT_TD2  0x00000080U  /*!< Timer D - Output 2 identifier */
#define HRTIM_OUTPUT_TE1  0x00000100U  /*!< Timer E - Output 1 identifier */
#define HRTIM_OUTPUT_TE2  0x00000200U  /*!< Timer E - Output 2 identifier */
      
#define IS_HRTIM_OUTPUT(OUTPUT) (((OUTPUT) & 0xFFFFFC00U) == 0x00000000U)
      
#define IS_HRTIM_TIMER_OUTPUT(TIMER, OUTPUT)\
    ((((TIMER) == HRTIM_TIMERINDEX_TIMER_A) &&   \
     (((OUTPUT) == HRTIM_OUTPUT_TA1) ||          \
      ((OUTPUT) == HRTIM_OUTPUT_TA2)))           \
    ||                                           \
    (((TIMER) == HRTIM_TIMERINDEX_TIMER_B) &&    \
     (((OUTPUT) == HRTIM_OUTPUT_TB1) ||          \
      ((OUTPUT) == HRTIM_OUTPUT_TB2)))           \
    ||                                           \
    (((TIMER) == HRTIM_TIMERINDEX_TIMER_C) &&    \
     (((OUTPUT) == HRTIM_OUTPUT_TC1) ||          \
      ((OUTPUT) == HRTIM_OUTPUT_TC2)))           \
    ||                                           \
    (((TIMER) == HRTIM_TIMERINDEX_TIMER_D) &&    \
     (((OUTPUT) == HRTIM_OUTPUT_TD1) ||          \
      ((OUTPUT) == HRTIM_OUTPUT_TD2)))           \
    ||                                           \
    (((TIMER) == HRTIM_TIMERINDEX_TIMER_E) &&    \
     (((OUTPUT) == HRTIM_OUTPUT_TE1) ||          \
      ((OUTPUT) == HRTIM_OUTPUT_TE2))))
/**
  * @}
  */
  
/** @defgroup HRTIM_CompareUnitAutoDelayedMode
  * @{
  * @brief Constants defining whether the compare register is behaving in 
  *        regular mode (compare match issued as soon as counter equal compare),
  *        or in auto-delayed mode
  */
#define HRTIM_AUTODELAYEDMODE_REGULAR                 (0x00000000U)                                   /*!< standard compare mode */    
#define HRTIM_AUTODELAYEDMODE_AUTODELAYED_NOTIMEOUT   (HRTIM_TIMCR_DELCMP2_0)                         /*!< Compare event generated only if a capture has occurred */    
#define HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1 (HRTIM_TIMCR_DELCMP2_1)                         /*!< Compare event generated if a capture has occurred or after a Compare 1 match (timeout if capture event is missing) */    
#define HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3 (HRTIM_TIMCR_DELCMP2_1 | HRTIM_TIMCR_DELCMP2_0) /*!< Compare event generated if a capture has occurred or after a Compare 3 match (timeout if capture event is missing) */    
         
#define IS_HRTIM_AUTODELAYEDMODE(AUTODELAYEDMODE)\
              (((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_REGULAR)                  || \
               ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_NOTIMEOUT)    || \
               ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1)  || \
               ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3))

/* Auto delayed mode is only available for compare units 2 and 4 */
#define IS_HRTIM_COMPAREUNIT_AUTODELAYEDMODE(COMPAREUNIT, AUTODELAYEDMODE)     \
    ((((COMPAREUNIT) == HRTIM_COMPAREUNIT_2) &&                                 \
     (((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_REGULAR)                 ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_NOTIMEOUT)   ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1) ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3)))   \
    ||                                                                         \
    (((COMPAREUNIT) == HRTIM_COMPAREUNIT_4) &&                                 \
     (((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_REGULAR)                 ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_NOTIMEOUT)   ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1) ||  \
      ((AUTODELAYEDMODE) == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3))))
/**
  * @}
  */
  
/** @defgroup HRTIM_OutputFAULTState
  * @{
  * @brief Constants defining the FAULT state of a timer output
  */  
#define HRTIM_OUTPUTFAULTSTATE_NONE      (uint32_t)0x00000000                         /*!< The output is not affected by the fault input */
#define HRTIM_OUTPUTFAULTSTATE_ACTIVE    (HRTIM_OUTR_FAULT1_0)                        /*!< Output at active level when in FAULT state */
#define HRTIM_OUTPUTFAULTSTATE_INACTIVE  (HRTIM_OUTR_FAULT1_1)                        /*!< Output at inactive level when in FAULT state */
#define HRTIM_OUTPUTFAULTSTATE_HIGHZ     (HRTIM_OUTR_FAULT1_1 | HRTIM_OUTR_FAULT1_0)  /*!< Output is tri-stated when in FAULT state */
              
#define IS_HRTIM_OUTPUTFAULTSTATE(OUTPUTFAULTSTATE)\
              (((OUTPUTFAULTSTATE) == HRTIM_OUTPUTFAULTSTATE_NONE)     || \
               ((OUTPUTFAULTSTATE) == HRTIM_OUTPUTFAULTSTATE_ACTIVE)   || \
               ((OUTPUTFAULTSTATE) == HRTIM_OUTPUTFAULTSTATE_INACTIVE) || \
               ((OUTPUTFAULTSTATE) == HRTIM_OUTPUTFAULTSTATE_HIGHZ))
/**
  * @}
  */  

/** @defgroup HRTIM_OutputSetSource
  * @{
  * @brief Constants defining the events that can be selected to configure the
  *        set crossbar of a timer output
  */
#define HRTIM_OUTPUTSET_NONE       0x00000000U             /*!< Reset the output set crossbar */
#define HRTIM_OUTPUTSET_TIMCMP1    (HRTIM_SET1R_CMP1)      /*!< Timer compare 1 event forces the output to its active state */
#define HRTIM_OUTPUTSET_TIMCMP2    (HRTIM_SET1R_CMP2)      /*!< Timer compare 2 event forces the output to its active state */
#define HRTIM_OUTPUTSET_TIMPER     (HRTIM_SET1R_PER)       /*!< Timer period event forces the output to its active state */

/**
  * @}
  */   

/** @defgroup HRTIM_OutputResetSource
  * @{
  * @brief Constants defining the events that can be selected to configure the
  *        set crossbar of a timer output
  */ 
#define HRTIM_OUTPUTRESET_NONE       0x00000000U             /*!< Reset the output reset crossbar */
#define HRTIM_OUTPUTRESET_TIMCMP1    (HRTIM_RST1R_CMP1)      /*!< Timer compare 1 event forces the output to its inactive state */
#define HRTIM_OUTPUTRESET_TIMCMP2    (HRTIM_RST1R_CMP2)      /*!< Timer compare 2 event forces the output to its inactive state */

/**
  * @}
  */  
  
/** @defgroup HRTIM_TimerUpdateTrigger
  * @{
  * @brief Constants defining whether the registers update is done synchronously 
  *        with any other timer or master update
  */
#define HRTIM_TIMUPDATETRIGGER_NONE     0x00000000U          /*!< Register update is disabled */    
#define HRTIM_TIMUPDATETRIGGER_MASTER   (HRTIM_TIMCR_MSTU)   /*!< Register update is triggered by the master timer update */    
#define HRTIM_TIMUPDATETRIGGER_TIMER_A  (HRTIM_TIMCR_TAU)    /*!< Register update is triggered by the timer A update */    
#define HRTIM_TIMUPDATETRIGGER_TIMER_B  (HRTIM_TIMCR_TBU)    /*!< Register update is triggered by the timer B update */    
#define HRTIM_TIMUPDATETRIGGER_TIMER_C  (HRTIM_TIMCR_TCU)    /*!< Register update is triggered by the timer C update*/    
#define HRTIM_TIMUPDATETRIGGER_TIMER_D  (HRTIM_TIMCR_TDU)    /*!< Register update is triggered by the timer D update */    
#define HRTIM_TIMUPDATETRIGGER_TIMER_E  (HRTIM_TIMCR_TEU)    /*!< Register update is triggered by the timer E update */    

#define IS_HRTIM_TIMUPDATETRIGGER(TIMUPDATETRIGGER) (((TIMUPDATETRIGGER) & 0xFE07FFFFU) == 0x00000000U)
/**
  * @}
  */
  
/** @defgroup HRTIM_ExternalEventChannels
  * @{
  * @brief Constants defining external event channel identifiers
  */
#define HRTIM_EVENT_NONE     (0x00000000U)     /*!< Undefined event channel */
#define HRTIM_EVENT_1        (0x00000001U)     /*!< External event channel 1 identifier */
#define HRTIM_EVENT_2        (0x00000002U)     /*!< External event channel 2 identifier */
#define HRTIM_EVENT_3        (0x00000004U)     /*!< External event channel 3 identifier */
#define HRTIM_EVENT_4        (0x00000008U)     /*!< External event channel 4 identifier */
#define HRTIM_EVENT_5        (0x00000010U)     /*!< External event channel 5 identifier */
#define HRTIM_EVENT_6        (0x00000020U)     /*!< External event channel 6 identifier */
#define HRTIM_EVENT_7        (0x00000040U)     /*!< External event channel 7 identifier */
#define HRTIM_EVENT_8        (0x00000080U)     /*!< External event channel 8 identifier */
#define HRTIM_EVENT_9        (0x00000100U)     /*!< External event channel 9 identifier */
#define HRTIM_EVENT_10       (0x00000200U)     /*!< External event channel 10 identifier */

#define IS_HRTIM_EVENT(EVENT)\
      (((EVENT) == HRTIM_EVENT_1)   || \
       ((EVENT) == HRTIM_EVENT_2)   || \
       ((EVENT) == HRTIM_EVENT_3)   || \
       ((EVENT) == HRTIM_EVENT_4)   || \
       ((EVENT) == HRTIM_EVENT_5)   || \
       ((EVENT) == HRTIM_EVENT_6)   || \
       ((EVENT) == HRTIM_EVENT_7)   || \
       ((EVENT) == HRTIM_EVENT_8)   || \
       ((EVENT) == HRTIM_EVENT_9)   || \
       ((EVENT) == HRTIM_EVENT_10))
/**
  * @}
  */

/** @defgroup HRTIM_OutputIdleMode
  * @{
  * @brief Constants defining whether or not the timer output transition to its 
           IDLE state when burst mode is entered
  */  
#define HRTIM_OUTPUTIDLEMODE_NONE     0x00000000U           /*!< The output is not affected by the burst mode operation */
#define HRTIM_OUTPUTIDLEMODE_IDLE     (HRTIM_OUTR_IDLM1)    /*!< The output is in idle state when requested by the burst mode controller */
              
#define IS_HRTIM_OUTPUTIDLEMODE(OUTPUTIDLEMODE)\
              (((OUTPUTIDLEMODE) == HRTIM_OUTPUTIDLEMODE_NONE) || \
               ((OUTPUTIDLEMODE) == HRTIM_OUTPUTIDLEMODE_IDLE))
 /**
  * @}
  */

/** @defgroup HRTIM_OutputChopperModeEnable
  * @{
  * @brief Constants defining whether or not chopper mode is enabled for a timer
           output
  */  
#define HRTIM_OUTPUTCHOPPERMODE_DISABLED   0x00000000U           /*!< The output is not affected by the fault input */
#define HRTIM_OUTPUTCHOPPERMODE_ENABLED    (HRTIM_OUTR_CHP1)     /*!< Output at active level when in FAULT state */

#define IS_HRTIM_OUTPUTCHOPPERMODE(OUTPUTCHOPPERMODE)\
              (((OUTPUTCHOPPERMODE) == HRTIM_OUTPUTCHOPPERMODE_DISABLED)  || \
               ((OUTPUTCHOPPERMODE) == HRTIM_OUTPUTCHOPPERMODE_ENABLED))
/**
  * @}
  */

/** @defgroup HRTIM_OutputBurstModeEntryDelayed
  * @{
  * @brief Constants defining the idle mode entry is delayed by forcing a 
           deadtime insertion before switching the outputs to their idle state
  */ 
#define HRTIM_OUTPUTBURSTMODEENTRY_REGULAR   0x00000000U           /*!< The programmed Idle state is applied immediately to the Output */
#define HRTIM_OUTPUTBURSTMODEENTRY_DELAYED   (HRTIM_OUTR_DIDL1)    /*!< Deadtime is inserted on output before entering the idle mode */

#define IS_HRTIM_OUTPUTBURSTMODEENTRY(OUTPUTBURSTMODEENTRY)\
              (((OUTPUTBURSTMODEENTRY) == HRTIM_OUTPUTBURSTMODEENTRY_REGULAR)  || \
               ((OUTPUTBURSTMODEENTRY) == HRTIM_OUTPUTBURSTMODEENTRY_DELAYED))
/**
  * @}
  */  

/** @defgroup HRTIM_ExternalEventFastMode
  * @{
  * @brief Constants defining whether or not an external event is programmed in
           fast mode
  */
#define HRTIM_EVENTFASTMODE_DISABLE         (0x00000000U)            /*!< External Event is acting asynchronously on outputs (low latency mode) */
#define HRTIM_EVENTFASTMODE_ENABLE          (HRTIM_EECR1_EE1FAST)    /*!< External Event is re-synchronized by the HRTIM logic before acting on outputs */

#if !defined(HAL_HRTIM_MODULE_ENABLED)
#define IS_HRTIM_EVENTFASTMODE(EVENTFASTMODE)\
                      (((EVENTFASTMODE) == HRTIM_EVENTFASTMODE_ENABLE)    || \
                       ((EVENTFASTMODE) == HRTIM_EVENTFASTMODE_DISABLE))
#endif /* HAL_HRTIM_MODULE_ENABLED */

#define IS_HRTIM_FASTMODE_AVAILABLE(EVENT)\
              (((EVENT) == HRTIM_EVENT_1)    || \
               ((EVENT) == HRTIM_EVENT_2)    || \
               ((EVENT) == HRTIM_EVENT_3)    || \
               ((EVENT) == HRTIM_EVENT_4)    || \
               ((EVENT) == HRTIM_EVENT_5))
/**
  * @}
  */

/** @defgroup HRTIM_ExternalEventSources
  * @{
  * @brief Constants defining available sources associated to external events
  */
#define HRTIM_EVENTSRC_1         (0x00000000U)                                  /*!< External event source 1 */
#define HRTIM_EVENTSRC_2         (HRTIM_EECR1_EE1SRC_0)                         /*!< External event source 2 */
#define HRTIM_EVENTSRC_3         (HRTIM_EECR1_EE1SRC_1)                         /*!< External event source 3 */
#define HRTIM_EVENTSRC_4         (HRTIM_EECR1_EE1SRC_1 | HRTIM_EECR1_EE1SRC_0)  /*!< External event source 4 */

#define IS_HRTIM_EVENTSRC(EVENTSRC)\
                (((EVENTSRC) == HRTIM_EVENTSRC_1)   || \
                 ((EVENTSRC) == HRTIM_EVENTSRC_2)   || \
                 ((EVENTSRC) == HRTIM_EVENTSRC_3)   || \
                 ((EVENTSRC) == HRTIM_EVENTSRC_4))
/**
  * @}
  */

/** @defgroup HRTIM_CaptureUnit 
  * @{
  * @brief Constants defining capture unit identifiers
  */  
#define HRTIM_CAPTUREUNIT_1 0x00000001U  /*!< Capture unit 1 identifier */
#define HRTIM_CAPTUREUNIT_2 0x00000002U  /*!< Capture unit 2 identifier */

#define IS_HRTIM_CAPTUREUNIT(CAPTUREUNIT)\
    (((CAPTUREUNIT) == HRTIM_CAPTUREUNIT_1)   || \
     ((CAPTUREUNIT) == HRTIM_CAPTUREUNIT_2))
/**
  * @}
  */

/** @defgroup HRTIM_TimerExternalEventFilter
  * @{
  * @brief Constants defining the event filtering applied to external events
  *        by a timer
  */
#define HRTIM_TIMEVENTFILTER_NONE             (0x00000000U)
/**
  * @}
  */

/** @defgroup HRTIM_CaptureUnitTrigger
  * @{
  * @brief Constants defining the events that can be selected to trigger the 
  *        capture of the timing unit counter
  */
#define HRTIM_CAPTURETRIGGER_EEV_1        (HRTIM_CPT1CR_EXEV1CPT)  /*!< The External event 1 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_2        (HRTIM_CPT1CR_EXEV2CPT)  /*!< The External event 2 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_3        (HRTIM_CPT1CR_EXEV3CPT)  /*!< The External event 3 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_4        (HRTIM_CPT1CR_EXEV4CPT)  /*!< The External event 4 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_5        (HRTIM_CPT1CR_EXEV5CPT)  /*!< The External event 5 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_6        (HRTIM_CPT1CR_EXEV6CPT)  /*!< The External event 6 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_7        (HRTIM_CPT1CR_EXEV7CPT)  /*!< The External event 7 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_8        (HRTIM_CPT1CR_EXEV8CPT)  /*!< The External event 8 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_9        (HRTIM_CPT1CR_EXEV9CPT)  /*!< The External event 9 triggers the Capture */    
#define HRTIM_CAPTURETRIGGER_EEV_10       (HRTIM_CPT1CR_EXEV10CPT) /*!< The External event 10 triggers the Capture */    
/**
  * @}
  */ 

/** @defgroup HRTIM_TimerRepetitionUpdate
  * @{
  * @brief Constants defining whether registers are updated when the timer
  *        repetition period is completed (either due to roll-over or
  *        reset events)
  */
#define HRTIM_UPDATEONREPETITION_DISABLED 0x00000000U          /*!< Update on repetition disabled */
#define HRTIM_UPDATEONREPETITION_ENABLED  (HRTIM_MCR_MREPU)    /*!< Update on repetition enabled */

#define IS_HRTIM_UPDATEONREPETITION(UPDATEONREPETITION)                               \
                (((UPDATEONREPETITION) == HRTIM_UPDATEONREPETITION_DISABLED)  || \
                 ((UPDATEONREPETITION) == HRTIM_UPDATEONREPETITION_ENABLED))
/**
  * @}
  */
  
  
  
/* Exported functions --------------------------------------------------------*/

/* Simple time base related functions  *****************************************/
void HRTIM_SimpleBase_Init(HRTIM_TypeDef* HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);

/* Simple output compare related functions  ************************************/
void HRTIM_SimpleOC_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);

void HRTIM_SimpleOCChannelConfig(HRTIM_TypeDef *HRTIMx,
                                                 uint32_t TimerIdx,
                                                 uint32_t OCChannel,
                                                 LEGACY_HRTIM_BasicOCChannelCfgTypeDef* pBasicOCChannelCfg);
												 
/* Simple PWM output related functions  ****************************************/
void HRTIM_SimplePWM_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);

void HRTIM_SimplePWMChannelConfig(HRTIM_TypeDef *HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t PWMChannel,
                                                  LEGACY_HRTIM_BasicPWMChannelCfgTypeDef* pBasicPWMChannelCfg);
												  
/* Simple capture related functions  *******************************************/
void HRTIM_SimpleCapture_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);

void HRTIM_SimpleCaptureChannelConfig(HRTIM_TypeDef *HRTIMx,
                                                      uint32_t TimerIdx,
                                                      uint32_t CaptureChannel,
                                                      LEGACY_HRTIM_BasicCaptureChannelCfgTypeDef* pBasicCaptureChannelCfg);

/* SImple one pulse related functions  *****************************************/
void HRTIM_SimpleOnePulse_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);

void HRTIM_SimpleOnePulseChannelConfig(HRTIM_TypeDef *HRTIMx,
                                                       uint32_t TimerIdx,
                                                       uint32_t OnePulseChannel,
                                                       LEGACY_HRTIM_BasicOnePulseChannelCfgTypeDef* pBasicOnePulseChannelCfg);

/* Waveform related functions *************************************************/
void HRTIM_Waveform_Init(HRTIM_TypeDef * HRTIMx,
                                         uint32_t TimerIdx,
                                         LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct,
                                         LEGACY_HRTIM_TimerInitTypeDef* HRTIM_TimerInitStruct);

void HRTIM_WaveformTimerConfig(HRTIM_TypeDef *HRTIMx,
                                                uint32_t TimerIdx,
                                                LEGACY_HRTIM_TimerCfgTypeDef * HRTIM_TimerCfgStruct);

void HRTIM_WaveformCompareConfig(HRTIM_TypeDef *HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CompareUnit,
                                                  LEGACY_HRTIM_CompareCfgTypeDef* pCompareCfg);
void HRTIM_SlaveSetCompare(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CompareUnit,
                                                  uint32_t Compare);

void HRTIM_MasterSetCompare(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t CompareUnit,
                                                  uint32_t Compare);

void HRTIM_WaveformCaptureConfig(HRTIM_TypeDef *HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CaptureUnit,
                                                  LEGACY_HRTIM_CaptureCfgTypeDef* pCaptureCfg);

void HRTIM_TimerEventFilteringConfig(HRTIM_TypeDef *HRTIMx,
                                                      uint32_t TimerIdx,
                                                      uint32_t Event,
                                                      LEGACY_HRTIM_TimerEventFilteringCfgTypeDef * pTimerEventFilteringCfg);

void HRTIM_DeadTimeConfig(HRTIM_TypeDef *HRTIMx,
                                           uint32_t TimerIdx,
                                           LEGACY_HRTIM_DeadTimeCfgTypeDef* pDeadTimeCfg);

void HRTIM_ChopperModeConfig(HRTIM_TypeDef *HRTIMx,
                                              uint32_t TimerIdx,
                                              LEGACY_HRTIM_ChopperModeCfgTypeDef* pChopperModeCfg);			

void HRTIM_SynchronizationConfig(HRTIM_TypeDef *HRTIMx,
                                                  LEGACY_HRTIM_SynchroCfgTypeDef * pSynchroCfg);

void HRTIM_BurstModeConfig(HRTIM_TypeDef *HRTIMx,
                                            LEGACY_HRTIM_BurstModeCfgTypeDef* pBurstModeCfg);

void HRTIM_EventConfig(HRTIM_TypeDef *HRTIMx,
                                        uint32_t Event,
                                        LEGACY_HRTIM_EventCfgTypeDef* pEventCfg);											  
										 
void HRTIM_FaultConfig(HRTIM_TypeDef *hrtim,
                                        LEGACY_HRTIM_FaultCfgTypeDef* pFaultCfg,
                                        uint32_t Fault);

void HRTIM_ADCTriggerConfig(HRTIM_TypeDef *HRTIMx,
                                             uint32_t ADCTrigger,
                                             LEGACY_HRTIM_ADCTriggerCfgTypeDef* pADCTriggerCfg);										

void HRTIM_WaveformOutputConfig(HRTIM_TypeDef * HRTIMx,
                                                uint32_t TimerIdx,
                                                uint32_t Output,
                                                LEGACY_HRTIM_OutputCfgTypeDef * pOutputCfg);

#endif /* STM32F334x8 */

#endif /* __LEGACY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
