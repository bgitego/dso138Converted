/**
  ******************************************************************************
  * @file    stm32f10x_dac.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the DAC firmware 
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
#ifndef __STM32F10x_DAC_H
#define __STM32F10x_DAC_H

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

/** @addtogroup DAC
  * @{
  */

/** @defgroup DAC_Exported_Types
  * @{
  */

/** 
  * @brief  DAC Init structure definition
  */

typedef struct
{
  uint32_t DAC_Trigger;                      /*!< Specifies the external trigger for the selected DAC channel.
                                                  This parameter can be a value of @ref DAC_trigger_selection */

  uint32_t DAC_WaveGeneration;               /*!< Specifies whether DAC channel noise waves or triangle waves
                                                  are generated, or whether no wave is generated.
                                                  This parameter can be a value of @ref DAC_wave_generation */

  uint32_t DAC_LFSRUnmask_TriangleAmplitude; /*!< Specifies the LFSR mask for noise wave generation or
                                                  the maximum amplitude triangle generation for the DAC channel. 
                                                  This parameter can be a value of @ref DAC_lfsrunmask_triangleamplitude */

  uint32_t DAC_OutputBuffer;                 /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                                  This parameter can be a value of @ref DAC_output_buffer */
}LL_DAC_InitTypeDef;

/**
  * @}
  */

/** @defgroup DAC_Exported_Constants
  * @{
  */

/** @defgroup DAC_trigger_selection 
  * @{
  */

#define 0x00000000U                   ((uint32_t)0x00000000) /*!< Conversion is automatic once the DAC1_DHRxxxx register 
                                                                       has been loaded, and not by external trigger */
#define LL_DAC_TRIG_EXT_TIM6_TRGO                ((uint32_t)0x00000004) /*!< TIM6 TRGO selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_EXT_TIM8_TRGO                ((uint32_t)0x0000000C) /*!< TIM8 TRGO selected as external conversion trigger for DAC channel
                                                                       only in High-density devices*/
#define LL_DAC_TRIG_EXT_TIM3_TRGO                ((uint32_t)0x0000000C) /*!< TIM8 TRGO selected as external conversion trigger for DAC channel
                                                                       only in Connectivity line, Medium-density and Low-density Value Line devices */
#define LL_DAC_TRIG_EXT_TIM7_TRGO                ((uint32_t)0x00000014) /*!< TIM7 TRGO selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_EXT_TIM5_TRGO                ((uint32_t)0x0000001C) /*!< TIM5 TRGO selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_EXT_TIM15_TRGO               ((uint32_t)0x0000001C) /*!< TIM15 TRGO selected as external conversion trigger for DAC channel 
                                                                       only in Medium-density and Low-density Value Line devices*/
#define LL_DAC_TRIG_EXT_TIM2_TRGO                ((uint32_t)0x00000024) /*!< TIM2 TRGO selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_EXT_TIM4_TRGO                ((uint32_t)0x0000002C) /*!< TIM4 TRGO selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_EXT_EXTI_LINE9                ((uint32_t)0x00000034) /*!< EXTI Line9 event selected as external conversion trigger for DAC channel */
#define LL_DAC_TRIG_SOFTWARE               ((uint32_t)0x0000003C) /*!< Conversion started by software trigger for DAC channel */

#define IS_DAC_TRIGGER(TRIGGER) (((TRIGGER) == 0x00000000U) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM6_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM8_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM7_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM5_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM2_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_TIM4_TRGO) || \
                                 ((TRIGGER) == LL_DAC_TRIG_EXT_EXTI_LINE9) || \
                                 ((TRIGGER) == LL_DAC_TRIG_SOFTWARE))

/**
  * @}
  */

/** @defgroup DAC_wave_generation 
  * @{
  */

#define LL_DAC_WAVE_AUTO_GENERATION_NONE            ((uint32_t)0x00000000)
#define LL_DAC_WAVE_AUTO_GENERATION_NOISE           ((uint32_t)0x00000040)
#define LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE        ((uint32_t)0x00000080)
#define IS_DAC_GENERATE_WAVE(WAVE) (((WAVE) == LL_DAC_WAVE_AUTO_GENERATION_NONE) || \
                                    ((WAVE) == LL_DAC_WAVE_AUTO_GENERATION_NOISE) || \
                                    ((WAVE) == LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE))
/**
  * @}
  */

/** @defgroup DAC_lfsrunmask_triangleamplitude
  * @{
  */

#define LL_DAC_WAVE_AUTO_GENERATION_NONE                ((uint32_t)0x00000000) /*!< Unmask DAC channel LFSR bit0 for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS1_0             ((uint32_t)0x00000100) /*!< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS2_0             ((uint32_t)0x00000200) /*!< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS3_0             ((uint32_t)0x00000300) /*!< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS4_0             ((uint32_t)0x00000400) /*!< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS5_0             ((uint32_t)0x00000500) /*!< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS6_0             ((uint32_t)0x00000600) /*!< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS7_0             ((uint32_t)0x00000700) /*!< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS8_0             ((uint32_t)0x00000800) /*!< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS9_0             ((uint32_t)0x00000900) /*!< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS10_0            ((uint32_t)0x00000A00) /*!< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
#define LL_DAC_NOISE_LFSR_UNMASK_BITS11_0            ((uint32_t)0x00000B00) /*!< Unmask DAC channel LFSR bit[11:0] for noise wave generation */
#define LL_DAC_TRIANGLE_AMPLITUDE_1            ((uint32_t)0x00000000) /*!< Select max triangle amplitude of 1 */
#define LL_DAC_TRIANGLE_AMPLITUDE_3            ((uint32_t)0x00000100) /*!< Select max triangle amplitude of 3 */
#define LL_DAC_TRIANGLE_AMPLITUDE_7            ((uint32_t)0x00000200) /*!< Select max triangle amplitude of 7 */
#define LL_DAC_TRIANGLE_AMPLITUDE_15           ((uint32_t)0x00000300) /*!< Select max triangle amplitude of 15 */
#define LL_DAC_TRIANGLE_AMPLITUDE_31           ((uint32_t)0x00000400) /*!< Select max triangle amplitude of 31 */
#define LL_DAC_TRIANGLE_AMPLITUDE_63           ((uint32_t)0x00000500) /*!< Select max triangle amplitude of 63 */
#define LL_DAC_TRIANGLE_AMPLITUDE_127          ((uint32_t)0x00000600) /*!< Select max triangle amplitude of 127 */
#define LL_DAC_TRIANGLE_AMPLITUDE_255          ((uint32_t)0x00000700) /*!< Select max triangle amplitude of 255 */
#define LL_DAC_TRIANGLE_AMPLITUDE_511          ((uint32_t)0x00000800) /*!< Select max triangle amplitude of 511 */
#define LL_DAC_TRIANGLE_AMPLITUDE_1023         ((uint32_t)0x00000900) /*!< Select max triangle amplitude of 1023 */
#define LL_DAC_TRIANGLE_AMPLITUDE_2047         ((uint32_t)0x00000A00) /*!< Select max triangle amplitude of 2047 */
#define LL_DAC_TRIANGLE_AMPLITUDE_4095         ((uint32_t)0x00000B00) /*!< Select max triangle amplitude of 4095 */

#define IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(VALUE) (((VALUE) == LL_DAC_WAVE_AUTO_GENERATION_NONE) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS1_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS2_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS3_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS4_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS5_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS6_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS7_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS8_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS9_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS10_0) || \
                                                      ((VALUE) == LL_DAC_NOISE_LFSR_UNMASK_BITS11_0) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_1) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_3) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_7) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_15) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_31) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_63) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_127) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_255) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_511) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_1023) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_2047) || \
                                                      ((VALUE) == LL_DAC_TRIANGLE_AMPLITUDE_4095))
/**
  * @}
  */

/** @defgroup DAC_output_buffer 
  * @{
  */

#define LL_DAC_OUTPUT_BUFFER_ENABLE            ((uint32_t)0x00000000)
#define LL_DAC_OUTPUT_BUFFER_DISABLE           ((uint32_t)0x00000002)
#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == LL_DAC_OUTPUT_BUFFER_ENABLE) || \
                                           ((STATE) == LL_DAC_OUTPUT_BUFFER_DISABLE))
/**
  * @}
  */

/** @defgroup DAC_Channel_selection 
  * @{
  */

#define LL_DAC_CHANNEL_1                      ((uint32_t)0x00000000)
#define LL_DAC_CHANNEL_2                      ((uint32_t)0x00000010)
#define IS_DAC_CHANNEL(CHANNEL) (((CHANNEL) == LL_DAC_CHANNEL_1) || \
                                 ((CHANNEL) == LL_DAC_CHANNEL_2))
/**
  * @}
  */

/** @defgroup DAC_data_alignement 
  * @{
  */

#define DAC_Align_12b_R                    ((uint32_t)0x00000000)
#define DAC_Align_12b_L                    ((uint32_t)0x00000004)
#define DAC_Align_8b_R                     ((uint32_t)0x00000008)
#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_Align_12b_R) || \
                             ((ALIGN) == DAC_Align_12b_L) || \
                             ((ALIGN) == DAC_Align_8b_R))
/**
  * @}
  */

/** @defgroup DAC_wave_generation 
  * @{
  */

#define LL_DAC_WAVE_AUTO_GENERATION_NOISE                     ((uint32_t)0x00000040)
#define LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE                  ((uint32_t)0x00000080)
#define IS_DAC_WAVE(WAVE) (((WAVE) == LL_DAC_WAVE_AUTO_GENERATION_NOISE) || \
                           ((WAVE) == LL_DAC_WAVE_AUTO_GENERATION_TRIANGLE))
/**
  * @}
  */

/** @defgroup DAC_data 
  * @{
  */

#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0) 
/**
  * @}
  */
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)  
/** @defgroup DAC_interrupts_definition 
  * @{
  */ 
  
#define DAC_IT_DMAUDR                      ((uint32_t)0x00002000)  
#define IS_DAC_IT(IT) (((IT) == DAC_IT_DMAUDR)) 

/**
  * @}
  */ 

/** @defgroup DAC_flags_definition 
  * @{
  */ 
  
#define DAC_FLAG_DMAUDR                    ((uint32_t)0x00002000)  
#define IS_DAC_FLAG(FLAG) (((FLAG) == DAC_FLAG_DMAUDR))  

/**
  * @}
  */
#endif

/**
  * @}
  */

/** @defgroup DAC_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup DAC_Exported_Functions
  * @{
  */

void LL_DAC_DeInit(DAC1);
if(DAC_InitTypeDef* DAC_InitStruct.TriggerSource != 0)
{
LL_DAC_EnableTrigger(DAC1,);
}
void LL_DAC_Init(DAC1, uint32_t DAC_Channel, LL_DAC_InitTypeDef* DAC_InitStruct);
void LL_DAC_StructInit(LL_DAC_InitTypeDef* DAC_InitStruct);
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState);
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)  
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState);
#endif
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState);
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState);
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState);
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data);
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1);
uint16_t LL_DAC_RetrieveOutputData(DAC1, uint32_t DAC_Channel);
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)  
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG);
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG);
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT);
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT);
#endif

#ifdef __cplusplus
}
#endif

#endif /*__STM32F10x_DAC_H */
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
