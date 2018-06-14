/**
  ******************************************************************************
  * @file    stm32f10x_tim.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the TIM firmware 
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
#ifndef __STM32F10x_TIM_H
#define __STM32F10x_TIM_H

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

/** @addtogroup TIM
  * @{
  */ 

/** @defgroup TIM_Exported_Types
  * @{
  */ 

/** 
  * @brief  TIM Time Base Init structure definition
  * @note   This sturcture is used with all TIMx except for TIM6 and TIM7.    
  */

typedef struct
{
  uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */

  uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */

  uint16_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */ 

  uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */

  uint8_t TIM_RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                       reaches zero, an update event is generated and counting restarts
                                       from the RCR value (N).
                                       This means in PWM mode that (N+1) corresponds to:
                                          - the number of PWM periods in edge-aligned mode
                                          - the number of half PWM period in center-aligned mode
                                       This parameter must be a number between 0x00 and 0xFF. 
                                       @note This parameter is valid only for TIM1 and TIM8. */
} LL_TIM_InitTypeDef;       

/** 
  * @brief  TIM Output Compare Init structure definition  
  */

typedef struct
{
  uint16_t TIM_OCMode;        /*!< Specifies the TIM mode.
                                   This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

  uint16_t TIM_OutputState;   /*!< Specifies the TIM Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_state */

  uint16_t TIM_OutputNState;  /*!< Specifies the TIM complementary Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_state
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint16_t TIM_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register. 
                                   This parameter can be a number between 0x0000 and 0xFFFF */

  uint16_t TIM_OCPolarity;    /*!< Specifies the output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_Polarity */

  uint16_t TIM_OCNPolarity;   /*!< Specifies the complementary output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint16_t TIM_OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint16_t TIM_OCNIdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */
} LL_TIM_OC_InitTypeDef;

/** 
  * @brief  TIM Input Capture Init structure definition  
  */

typedef struct
{

  uint16_t TIM_Channel;      /*!< Specifies the TIM channel.
                                  This parameter can be a value of @ref TIM_Channel */

  uint16_t TIM_ICPolarity;   /*!< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */

  uint16_t TIM_ICSelection;  /*!< Specifies the input.
                                  This parameter can be a value of @ref TIM_Input_Capture_Selection */

  uint16_t TIM_ICPrescaler;  /*!< Specifies the Input Capture Prescaler.
                                  This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

  uint16_t TIM_ICFilter;     /*!< Specifies the input capture filter.
                                  This parameter can be a number between 0x0 and 0xF */
} LL_TIM_IC_InitTypeDef;

/** 
  * @brief  BDTR structure definition 
  * @note   This sturcture is used only with TIM1 and TIM8.    
  */

typedef struct
{

  uint16_t TIM_OSSRState;        /*!< Specifies the Off-State selection used in Run mode.
                                      This parameter can be a value of @ref OSSR_Off_State_Selection_for_Run_mode_state */

  uint16_t TIM_OSSIState;        /*!< Specifies the Off-State used in Idle state.
                                      This parameter can be a value of @ref OSSI_Off_State_Selection_for_Idle_mode_state */

  uint16_t TIM_LOCKLevel;        /*!< Specifies the LOCK level parameters.
                                      This parameter can be a value of @ref Lock_level */ 

  uint16_t TIM_DeadTime;         /*!< Specifies the delay time between the switching-off and the
                                      switching-on of the outputs.
                                      This parameter can be a number between 0x00 and 0xFF  */

  uint16_t TIM_Break;            /*!< Specifies whether the TIM Break input is enabled or not. 
                                      This parameter can be a value of @ref Break_Input_enable_disable */

  uint16_t TIM_BreakPolarity;    /*!< Specifies the TIM Break Input pin polarity.
                                      This parameter can be a value of @ref Break_Polarity */

  uint16_t TIM_AutomaticOutput;  /*!< Specifies whether the TIM Automatic Output feature is enabled or not. 
                                      This parameter can be a value of @ref TIM_AOE_Bit_Set_Reset */
} LL_TIM_BDTR_InitTypeDef;

/** @defgroup TIM_Exported_constants 
  * @{
  */

#define IS_TIM_ALL_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                   ((PERIPH) == TIM2) || \
                                   ((PERIPH) == TIM3) || \
                                   ((PERIPH) == TIM4) || \
                                   ((PERIPH) == TIM5) || \
                                   ((PERIPH) == TIM6) || \
                                   ((PERIPH) == TIM7) || \
                                   ((PERIPH) == TIM8) || \
                                   ((PERIPH) == TIM9) || \
                                   ((PERIPH) == TIM10)|| \
                                   ((PERIPH) == TIM11)|| \
                                   ((PERIPH) == TIM12)|| \
                                   ((PERIPH) == TIM13)|| \
                                   ((PERIPH) == TIM14)|| \
                                   ((PERIPH) == TIM15)|| \
                                   ((PERIPH) == TIM16)|| \
                                   ((PERIPH) == TIM17))

/* LIST1: TIM 1 and 8 */
#define IS_TIM_LIST1_PERIPH(PERIPH)  (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM8))

/* LIST2: TIM 1, 8, 15 16 and 17 */
#define IS_TIM_LIST2_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                     ((PERIPH) == TIM8) || \
                                     ((PERIPH) == TIM15)|| \
                                     ((PERIPH) == TIM16)|| \
                                     ((PERIPH) == TIM17)) 

/* LIST3: TIM 1, 2, 3, 4, 5 and 8 */
#define IS_TIM_LIST3_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                     ((PERIPH) == TIM2) || \
                                     ((PERIPH) == TIM3) || \
                                     ((PERIPH) == TIM4) || \
                                     ((PERIPH) == TIM5) || \
                                     ((PERIPH) == TIM8)) 
									                                 
/* LIST4: TIM 1, 2, 3, 4, 5, 8, 15, 16 and 17 */
#define IS_TIM_LIST4_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                     ((PERIPH) == TIM2) || \
                                     ((PERIPH) == TIM3) || \
                                     ((PERIPH) == TIM4) || \
                                     ((PERIPH) == TIM5) || \
                                     ((PERIPH) == TIM8) || \
                                     ((PERIPH) == TIM15)|| \
                                     ((PERIPH) == TIM16)|| \
                                     ((PERIPH) == TIM17))

/* LIST5: TIM 1, 2, 3, 4, 5, 8 and 15 */                                            
#define IS_TIM_LIST5_PERIPH(PERIPH) (((PERIPH) == TIM1) || \
                                     ((PERIPH) == TIM2) || \
                                     ((PERIPH) == TIM3) || \
                                     ((PERIPH) == TIM4) || \
                                     ((PERIPH) == TIM5) || \
                                     ((PERIPH) == TIM8) || \
                                     ((PERIPH) == TIM15)) 

/* LIST6: TIM 1, 2, 3, 4, 5, 8, 9, 12 and 15 */
#define IS_TIM_LIST6_PERIPH(PERIPH)  (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM5) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM9) || \
									  ((PERIPH) == TIM12)|| \
                                      ((PERIPH) == TIM15))

/* LIST7: TIM 1, 2, 3, 4, 5, 6, 7, 8, 9, 12 and 15 */
#define IS_TIM_LIST7_PERIPH(PERIPH)  (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM5) || \
                                      ((PERIPH) == TIM6) || \
                                      ((PERIPH) == TIM7) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM9) || \
                                      ((PERIPH) == TIM12)|| \
                                      ((PERIPH) == TIM15))                                    

/* LIST8: TIM 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, 16 and 17 */                                        
#define IS_TIM_LIST8_PERIPH(PERIPH)  (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM5) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM9) || \
                                      ((PERIPH) == TIM10)|| \
                                      ((PERIPH) == TIM11)|| \
                                      ((PERIPH) == TIM12)|| \
                                      ((PERIPH) == TIM13)|| \
                                      ((PERIPH) == TIM14)|| \
                                      ((PERIPH) == TIM15)|| \
                                      ((PERIPH) == TIM16)|| \
                                      ((PERIPH) == TIM17))

/* LIST9: TIM 1, 2, 3, 4, 5, 6, 7, 8, 15, 16, and 17 */
#define IS_TIM_LIST9_PERIPH(PERIPH)  (((PERIPH) == TIM1) || \
                                      ((PERIPH) == TIM2) || \
                                      ((PERIPH) == TIM3) || \
                                      ((PERIPH) == TIM4) || \
                                      ((PERIPH) == TIM5) || \
                                      ((PERIPH) == TIM6) || \
                                      ((PERIPH) == TIM7) || \
                                      ((PERIPH) == TIM8) || \
                                      ((PERIPH) == TIM15)|| \
                                      ((PERIPH) == TIM16)|| \
                                      ((PERIPH) == TIM17))  
                                                                                                                                                                                                                          
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_and_PWM_modes 
  * @{
  */

#define LL_TIM_OCMODE_FROZEN                  ((uint16_t)0x0000)
#define LL_TIM_OCMODE_ACTIVE                  ((uint16_t)0x0010)
#define LL_TIM_OCMODE_INACTIVE                ((uint16_t)0x0020)
#define LL_TIM_OCMODE_TOGGLE                  ((uint16_t)0x0030)
#define LL_TIM_OCMODE_PWM1                    ((uint16_t)0x0060)
#define LL_TIM_OCMODE_PWM2                    ((uint16_t)0x0070)
#define IS_TIM_OC_MODE(MODE) (((MODE) == LL_TIM_OCMODE_FROZEN) || \
                              ((MODE) == LL_TIM_OCMODE_ACTIVE) || \
                              ((MODE) == LL_TIM_OCMODE_INACTIVE) || \
                              ((MODE) == LL_TIM_OCMODE_TOGGLE)|| \
                              ((MODE) == LL_TIM_OCMODE_PWM1) || \
                              ((MODE) == LL_TIM_OCMODE_PWM2))
#define IS_TIM_OCM(MODE) (((MODE) == LL_TIM_OCMODE_FROZEN) || \
                          ((MODE) == LL_TIM_OCMODE_ACTIVE) || \
                          ((MODE) == LL_TIM_OCMODE_INACTIVE) || \
                          ((MODE) == LL_TIM_OCMODE_TOGGLE)|| \
                          ((MODE) == LL_TIM_OCMODE_PWM1) || \
                          ((MODE) == LL_TIM_OCMODE_PWM2) ||	\
                          ((MODE) == LL_TIM_OCMODE_FORCED_ACTIVE) || \
                          ((MODE) == LL_TIM_OCMODE_FORCED_INACTIVE))
/**
  * @}
  */

/** @defgroup TIM_One_Pulse_Mode 
  * @{
  */

#define LL_TIM_ONEPULSEMODE_SINGLE                  ((uint16_t)0x0008)
#define LL_TIM_ONEPULSEMODE_REPETITIVE              ((uint16_t)0x0000)
#define IS_TIM_OPM_MODE(MODE) (((MODE) == LL_TIM_ONEPULSEMODE_SINGLE) || \
                               ((MODE) == LL_TIM_ONEPULSEMODE_REPETITIVE))
/**
  * @}
  */ 

/** @defgroup TIM_Channel 
  * @{
  */

#define LL_TIM_CHANNEL_CH1                      ((uint16_t)0x0000)
#define LL_TIM_CHANNEL_CH2                      ((uint16_t)0x0004)
#define LL_TIM_CHANNEL_CH3                      ((uint16_t)0x0008)
#define LL_TIM_CHANNEL_CH4                      ((uint16_t)0x000C)
#define IS_TIM_CHANNEL(CHANNEL) (((CHANNEL) == LL_TIM_CHANNEL_CH1) || \
                                 ((CHANNEL) == LL_TIM_CHANNEL_CH2) || \
                                 ((CHANNEL) == LL_TIM_CHANNEL_CH3) || \
                                 ((CHANNEL) == LL_TIM_CHANNEL_CH4))
#define IS_TIM_PWMI_CHANNEL(CHANNEL) (((CHANNEL) == LL_TIM_CHANNEL_CH1) || \
                                      ((CHANNEL) == LL_TIM_CHANNEL_CH2))
#define IS_TIM_COMPLEMENTARY_CHANNEL(CHANNEL) (((CHANNEL) == LL_TIM_CHANNEL_CH1) || \
                                               ((CHANNEL) == LL_TIM_CHANNEL_CH2) || \
                                               ((CHANNEL) == LL_TIM_CHANNEL_CH3))
/**
  * @}
  */ 

/** @defgroup TIM_Clock_Division_CKD 
  * @{
  */

#define LL_TIM_CLOCKDIVISION_DIV1                       ((uint16_t)0x0000)
#define LL_TIM_CLOCKDIVISION_DIV2                       ((uint16_t)0x0100)
#define LL_TIM_CLOCKDIVISION_DIV4                       ((uint16_t)0x0200)
#define IS_TIM_CKD_DIV(DIV) (((DIV) == LL_TIM_CLOCKDIVISION_DIV1) || \
                             ((DIV) == LL_TIM_CLOCKDIVISION_DIV2) || \
                             ((DIV) == LL_TIM_CLOCKDIVISION_DIV4))
/**
  * @}
  */

/** @defgroup TIM_Counter_Mode 
  * @{
  */

#define LL_TIM_COUNTERMODE_UP                 ((uint16_t)0x0000)
#define LL_TIM_COUNTERMODE_DOWN               ((uint16_t)0x0010)
#define LL_TIM_COUNTERMODE_CENTER_UP     ((uint16_t)0x0020)
#define LL_TIM_COUNTERMODE_CENTER_DOWN     ((uint16_t)0x0040)
#define LL_TIM_COUNTERMODE_CENTER_UP_DOWN     ((uint16_t)0x0060)
#define IS_TIM_COUNTER_MODE(MODE) (((MODE) == LL_TIM_COUNTERMODE_UP) ||  \
                                   ((MODE) == LL_TIM_COUNTERMODE_DOWN) || \
                                   ((MODE) == LL_TIM_COUNTERMODE_CENTER_UP) || \
                                   ((MODE) == LL_TIM_COUNTERMODE_CENTER_DOWN) || \
                                   ((MODE) == LL_TIM_COUNTERMODE_CENTER_UP_DOWN))
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_Polarity 
  * @{
  */

#define LL_TIM_OCPOLARITY_HIGH                ((uint16_t)0x0000)
#define LL_TIM_OCPOLARITY_LOW                 ((uint16_t)0x0002)
#define IS_TIM_OC_POLARITY(POLARITY) (((POLARITY) == LL_TIM_OCPOLARITY_HIGH) || \
                                      ((POLARITY) == LL_TIM_OCPOLARITY_LOW))
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_N_Polarity 
  * @{
  */
  
#define LL_TIM_OCPOLARITY_HIGH               ((uint16_t)0x0000)
#define LL_TIM_OCPOLARITY_LOW                ((uint16_t)0x0008)
#define IS_TIM_OCN_POLARITY(POLARITY) (((POLARITY) == LL_TIM_OCPOLARITY_HIGH) || \
                                       ((POLARITY) == LL_TIM_OCPOLARITY_LOW))
/**
  * @}
  */

/** @defgroup TIM_Output_Compare_state 
  * @{
  */

#define LL_TIM_OCSTATE_DISABLE            ((uint16_t)0x0000)
#define LL_TIM_OCSTATE_ENABLE             ((uint16_t)0x0001)
#define IS_TIM_OUTPUT_STATE(STATE) (((STATE) == LL_TIM_OCSTATE_DISABLE) || \
                                    ((STATE) == LL_TIM_OCSTATE_ENABLE))
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_N_state 
  * @{
  */

#define LL_TIM_OCSTATE_DISABLE           ((uint16_t)0x0000)
#define LL_TIM_OCSTATE_ENABLE            ((uint16_t)0x0004)
#define IS_TIM_OUTPUTN_STATE(STATE) (((STATE) == LL_TIM_OCSTATE_DISABLE) || \
                                     ((STATE) == LL_TIM_OCSTATE_ENABLE))
/**
  * @}
  */ 

/** @defgroup TIM_Capture_Compare_state 
  * @{
  */

#define TIM_CCx_Enable                      ((uint16_t)0x0001)
#define TIM_CCx_Disable                     ((uint16_t)0x0000)
#define IS_TIM_CCX(CCX) (((CCX) == TIM_CCx_Enable) || \
                         ((CCX) == TIM_CCx_Disable))
/**
  * @}
  */ 

/** @defgroup TIM_Capture_Compare_N_state 
  * @{
  */

#define TIM_CCxN_Enable                     ((uint16_t)0x0004)
#define TIM_CCxN_Disable                    ((uint16_t)0x0000)
#define IS_TIM_CCXN(CCXN) (((CCXN) == TIM_CCxN_Enable) || \
                           ((CCXN) == TIM_CCxN_Disable))
/**
  * @}
  */ 

/** @defgroup Break_Input_enable_disable 
  * @{
  */

#define LL_TIM_BREAK_ENABLE                   ((uint16_t)0x1000)
#define LL_TIM_BREAK_DISABLE                  ((uint16_t)0x0000)
#define IS_TIM_BREAK_STATE(STATE) (((STATE) == LL_TIM_BREAK_ENABLE) || \
                                   ((STATE) == LL_TIM_BREAK_DISABLE))
/**
  * @}
  */ 

/** @defgroup Break_Polarity 
  * @{
  */

#define LL_TIM_BREAK_POLARITY_LOW              ((uint16_t)0x0000)
#define LL_TIM_BREAK_POLARITY_HIGH             ((uint16_t)0x2000)
#define IS_TIM_BREAK_POLARITY(POLARITY) (((POLARITY) == LL_TIM_BREAK_POLARITY_LOW) || \
                                         ((POLARITY) == LL_TIM_BREAK_POLARITY_HIGH))
/**
  * @}
  */ 

/** @defgroup TIM_AOE_Bit_Set_Reset 
  * @{
  */

#define LL_TIM_AUTOMATICOUTPUT_ENABLE         ((uint16_t)0x4000)
#define LL_TIM_AUTOMATICOUTPUT_DISABLE        ((uint16_t)0x0000)
#define IS_TIM_AUTOMATIC_OUTPUT_STATE(STATE) (((STATE) == LL_TIM_AUTOMATICOUTPUT_ENABLE) || \
                                              ((STATE) == LL_TIM_AUTOMATICOUTPUT_DISABLE))
/**
  * @}
  */ 

/** @defgroup Lock_level 
  * @{
  */

#define LL_TIM_LOCKLEVEL_OFF                  ((uint16_t)0x0000)
#define LL_TIM_LOCKLEVEL_1                    ((uint16_t)0x0100)
#define LL_TIM_LOCKLEVEL_2                    ((uint16_t)0x0200)
#define LL_TIM_LOCKLEVEL_3                    ((uint16_t)0x0300)
#define IS_TIM_LOCK_LEVEL(LEVEL) (((LEVEL) == LL_TIM_LOCKLEVEL_OFF) || \
                                  ((LEVEL) == LL_TIM_LOCKLEVEL_1) || \
                                  ((LEVEL) == LL_TIM_LOCKLEVEL_2) || \
                                  ((LEVEL) == LL_TIM_LOCKLEVEL_3))
/**
  * @}
  */ 

/** @defgroup OSSI_Off_State_Selection_for_Idle_mode_state 
  * @{
  */

#define LL_TIM_OSSI_ENABLE               ((uint16_t)0x0400)
#define LL_TIM_OSSI_DISABLE              ((uint16_t)0x0000)
#define IS_TIM_OSSI_STATE(STATE) (((STATE) == LL_TIM_OSSI_ENABLE) || \
                                  ((STATE) == LL_TIM_OSSI_DISABLE))
/**
  * @}
  */

/** @defgroup OSSR_Off_State_Selection_for_Run_mode_state 
  * @{
  */

#define LL_TIM_OSSR_ENABLE               ((uint16_t)0x0800)
#define LL_TIM_OSSR_DISABLE              ((uint16_t)0x0000)
#define IS_TIM_OSSR_STATE(STATE) (((STATE) == LL_TIM_OSSR_ENABLE) || \
                                  ((STATE) == LL_TIM_OSSR_DISABLE))
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_Idle_State 
  * @{
  */

#define LL_TIM_OCIDLESTATE_HIGH                ((uint16_t)0x0100)
#define LL_TIM_OCIDLESTATE_LOW              ((uint16_t)0x0000)
#define IS_TIM_OCIDLE_STATE(STATE) (((STATE) == LL_TIM_OCIDLESTATE_HIGH) || \
                                    ((STATE) == LL_TIM_OCIDLESTATE_LOW))
/**
  * @}
  */ 

/** @defgroup TIM_Output_Compare_N_Idle_State 
  * @{
  */

#define LL_TIM_OCIDLESTATE_HIGH               ((uint16_t)0x0200)
#define LL_TIM_OCIDLESTATE_LOW             ((uint16_t)0x0000)
#define IS_TIM_OCNIDLE_STATE(STATE) (((STATE) == LL_TIM_OCIDLESTATE_HIGH) || \
                                     ((STATE) == LL_TIM_OCIDLESTATE_LOW))
/**
  * @}
  */ 

/** @defgroup TIM_Input_Capture_Polarity 
  * @{
  */

#define  LL_TIM_IC_POLARITY_RISING             ((uint16_t)0x0000)
#define  LL_TIM_IC_POLARITY_FALLING            ((uint16_t)0x0002)
#define IS_TIM_IC_POLARITY(POLARITY) (((POLARITY) == LL_TIM_IC_POLARITY_RISING) || \
                                      ((POLARITY) == LL_TIM_IC_POLARITY_FALLING))
/**
  * @}
  */ 

/** @defgroup TIM_Input_Capture_Selection 
  * @{
  */

#define LL_TIM_ACTIVEINPUT_DIRECTTI           ((uint16_t)0x0001) /*!< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                   connected to IC1, IC2, IC3 or IC4, respectively */
#define LL_TIM_ACTIVEINPUT_INDIRECTTI         ((uint16_t)0x0002) /*!< TIM Input 1, 2, 3 or 4 is selected to be
                                                                   connected to IC2, IC1, IC4 or IC3, respectively. */
#define LL_TIM_ACTIVEINPUT_TRC                ((uint16_t)0x0003) /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. */
#define IS_TIM_IC_SELECTION(SELECTION) (((SELECTION) == LL_TIM_ACTIVEINPUT_DIRECTTI) || \
                                        ((SELECTION) == LL_TIM_ACTIVEINPUT_INDIRECTTI) || \
                                        ((SELECTION) == LL_TIM_ACTIVEINPUT_TRC))
/**
  * @}
  */ 

/** @defgroup TIM_Input_Capture_Prescaler 
  * @{
  */

#define LL_TIM_ICPSC_DIV1                     ((uint16_t)0x0000) /*!< Capture performed each time an edge is detected on the capture input. */
#define LL_TIM_ICPSC_DIV2                     ((uint16_t)0x0004) /*!< Capture performed once every 2 events. */
#define LL_TIM_ICPSC_DIV4                     ((uint16_t)0x0008) /*!< Capture performed once every 4 events. */
#define LL_TIM_ICPSC_DIV8                     ((uint16_t)0x000C) /*!< Capture performed once every 8 events. */
#define IS_TIM_IC_PRESCALER(PRESCALER) (((PRESCALER) == LL_TIM_ICPSC_DIV1) || \
                                        ((PRESCALER) == LL_TIM_ICPSC_DIV2) || \
                                        ((PRESCALER) == LL_TIM_ICPSC_DIV4) || \
                                        ((PRESCALER) == LL_TIM_ICPSC_DIV8))
/**
  * @}
  */ 

/** @defgroup TIM_interrupt_sources 
  * @{
  */

#define LL_TIM_DIER_UIE                      ((uint16_t)0x0001)
#define LL_TIM_DIER_CC1IE                         ((uint16_t)0x0002)
#define LL_TIM_DIER_CC2IE                         ((uint16_t)0x0004)
#define LL_TIM_DIER_CC3IE                         ((uint16_t)0x0008)
#define LL_TIM_DIER_CC4IE                         ((uint16_t)0x0010)
#define LL_TIM_DIER_COMIE                         ((uint16_t)0x0020)
#define LL_TIM_DIER_TIE                     ((uint16_t)0x0040)
#define LL_TIM_DIER_BIE                       ((uint16_t)0x0080)
#define IS_TIM_IT(IT) ((((IT) & (uint16_t)0xFF00) == 0x0000) && ((IT) != 0x0000))

#define IS_TIM_GET_IT(IT) (((IT) == LL_TIM_DIER_UIE) || \
                           ((IT) == LL_TIM_DIER_CC1IE) || \
                           ((IT) == LL_TIM_DIER_CC2IE) || \
                           ((IT) == LL_TIM_DIER_CC3IE) || \
                           ((IT) == LL_TIM_DIER_CC4IE) || \
                           ((IT) == LL_TIM_DIER_COMIE) || \
                           ((IT) == LL_TIM_DIER_TIE) || \
                           ((IT) == LL_TIM_DIER_BIE))
/**
  * @}
  */ 

/** @defgroup TIM_DMA_Base_address 
  * @{
  */

#define LL_TIM_DMABURST_BASEADDR_CR1                    ((uint16_t)0x0000)
#define LL_TIM_DMABURST_BASEADDR_CR2                    ((uint16_t)0x0001)
#define LL_TIM_DMABURST_BASEADDR_SMCR                   ((uint16_t)0x0002)
#define LL_TIM_DMABURST_BASEADDR_DIER                   ((uint16_t)0x0003)
#define LL_TIM_DMABURST_BASEADDR_SR                     ((uint16_t)0x0004)
#define LL_TIM_DMABURST_BASEADDR_EGR                    ((uint16_t)0x0005)
#define LL_TIM_DMABURST_BASEADDR_CCMR1                  ((uint16_t)0x0006)
#define LL_TIM_DMABURST_BASEADDR_CCMR2                  ((uint16_t)0x0007)
#define LL_TIM_DMABURST_BASEADDR_CCER                   ((uint16_t)0x0008)
#define LL_TIM_DMABURST_BASEADDR_CNT                    ((uint16_t)0x0009)
#define LL_TIM_DMABURST_BASEADDR_PSC                    ((uint16_t)0x000A)
#define LL_TIM_DMABURST_BASEADDR_ARR                    ((uint16_t)0x000B)
#define LL_TIM_DMABURST_BASEADDR_RCR                    ((uint16_t)0x000C)
#define LL_TIM_DMABURST_BASEADDR_CCR1                   ((uint16_t)0x000D)
#define LL_TIM_DMABURST_BASEADDR_CCR2                   ((uint16_t)0x000E)
#define LL_TIM_DMABURST_BASEADDR_CCR3                   ((uint16_t)0x000F)
#define LL_TIM_DMABURST_BASEADDR_CCR4                   ((uint16_t)0x0010)
#define LL_TIM_DMABURST_BASEADDR_BDTR                   ((uint16_t)0x0011)
#define TIM_DMABase_DCR                    ((uint16_t)0x0012)
#define IS_TIM_DMA_BASE(BASE) (((BASE) == LL_TIM_DMABURST_BASEADDR_CR1) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CR2) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_SMCR) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_DIER) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_SR) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_EGR) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCMR1) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCMR2) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCER) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CNT) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_PSC) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_ARR) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_RCR) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCR1) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCR2) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCR3) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_CCR4) || \
                               ((BASE) == LL_TIM_DMABURST_BASEADDR_BDTR) || \
                               ((BASE) == TIM_DMABase_DCR))
/**
  * @}
  */ 

/** @defgroup TIM_DMA_Burst_Length 
  * @{
  */

#define LL_TIM_DMABURST_LENGTH_1TRANSFER           ((uint16_t)0x0000)
#define LL_TIM_DMABURST_LENGTH_2TRANSFERS          ((uint16_t)0x0100)
#define LL_TIM_DMABURST_LENGTH_3TRANSFERS          ((uint16_t)0x0200)
#define LL_TIM_DMABURST_LENGTH_4TRANSFERS          ((uint16_t)0x0300)
#define LL_TIM_DMABURST_LENGTH_5TRANSFERS          ((uint16_t)0x0400)
#define LL_TIM_DMABURST_LENGTH_6TRANSFERS          ((uint16_t)0x0500)
#define LL_TIM_DMABURST_LENGTH_7TRANSFERS          ((uint16_t)0x0600)
#define LL_TIM_DMABURST_LENGTH_8TRANSFERS          ((uint16_t)0x0700)
#define LL_TIM_DMABURST_LENGTH_9TRANSFERS          ((uint16_t)0x0800)
#define LL_TIM_DMABURST_LENGTH_10TRANSFERS         ((uint16_t)0x0900)
#define LL_TIM_DMABURST_LENGTH_11TRANSFERS         ((uint16_t)0x0A00)
#define LL_TIM_DMABURST_LENGTH_12TRANSFERS         ((uint16_t)0x0B00)
#define LL_TIM_DMABURST_LENGTH_13TRANSFERS         ((uint16_t)0x0C00)
#define LL_TIM_DMABURST_LENGTH_14TRANSFERS         ((uint16_t)0x0D00)
#define LL_TIM_DMABURST_LENGTH_15TRANSFERS         ((uint16_t)0x0E00)
#define LL_TIM_DMABURST_LENGTH_16TRANSFERS         ((uint16_t)0x0F00)
#define LL_TIM_DMABURST_LENGTH_17TRANSFERS         ((uint16_t)0x1000)
#define LL_TIM_DMABURST_LENGTH_18TRANSFERS         ((uint16_t)0x1100)
#define IS_TIM_DMA_LENGTH(LENGTH) (((LENGTH) == LL_TIM_DMABURST_LENGTH_1TRANSFER) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_2TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_3TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_4TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_5TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_6TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_7TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_8TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_9TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_10TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_11TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_12TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_13TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_14TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_15TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_16TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_17TRANSFERS) || \
                                   ((LENGTH) == LL_TIM_DMABURST_LENGTH_18TRANSFERS))
/**
  * @}
  */ 

/** @defgroup TIM_DMA_sources 
  * @{
  */

#define TIM_DMA_Update                     ((uint16_t)0x0100)
#define TIM_DMA_CC1                        ((uint16_t)0x0200)
#define TIM_DMA_CC2                        ((uint16_t)0x0400)
#define TIM_DMA_CC3                        ((uint16_t)0x0800)
#define TIM_DMA_CC4                        ((uint16_t)0x1000)
#define TIM_DMA_COM                        ((uint16_t)0x2000)
#define TIM_DMA_Trigger                    ((uint16_t)0x4000)
#define IS_TIM_DMA_SOURCE(SOURCE) ((((SOURCE) & (uint16_t)0x80FF) == 0x0000) && ((SOURCE) != 0x0000))

/**
  * @}
  */ 

/** @defgroup TIM_External_Trigger_Prescaler 
  * @{
  */

#define LL_TIM_ETR_PRESCALER_DIV1                  ((uint16_t)0x0000)
#define LL_TIM_ETR_PRESCALER_DIV2                 ((uint16_t)0x1000)
#define LL_TIM_ETR_PRESCALER_DIV4                 ((uint16_t)0x2000)
#define LL_TIM_ETR_PRESCALER_DIV8                 ((uint16_t)0x3000)
#define IS_TIM_EXT_PRESCALER(PRESCALER) (((PRESCALER) == LL_TIM_ETR_PRESCALER_DIV1) || \
                                         ((PRESCALER) == LL_TIM_ETR_PRESCALER_DIV2) || \
                                         ((PRESCALER) == LL_TIM_ETR_PRESCALER_DIV4) || \
                                         ((PRESCALER) == LL_TIM_ETR_PRESCALER_DIV8))
/**
  * @}
  */ 

/** @defgroup TIM_Internal_Trigger_Selection 
  * @{
  */

#define LL_TIM_TS_ITR0                        ((uint16_t)0x0000)
#define LL_TIM_TS_ITR1                        ((uint16_t)0x0010)
#define LL_TIM_TS_ITR2                        ((uint16_t)0x0020)
#define LL_TIM_TS_ITR3                        ((uint16_t)0x0030)
#define LL_TIM_TS_TI1F_ED                     ((uint16_t)0x0040)
#define LL_TIM_TS_TI1FP1                      ((uint16_t)0x0050)
#define LL_TIM_TS_TI2FP2                      ((uint16_t)0x0060)
#define LL_TIM_TS_ETRF                        ((uint16_t)0x0070)
#define IS_TIM_TRIGGER_SELECTION(SELECTION) (((SELECTION) == LL_TIM_TS_ITR0) || \
                                             ((SELECTION) == LL_TIM_TS_ITR1) || \
                                             ((SELECTION) == LL_TIM_TS_ITR2) || \
                                             ((SELECTION) == LL_TIM_TS_ITR3) || \
                                             ((SELECTION) == LL_TIM_TS_TI1F_ED) || \
                                             ((SELECTION) == LL_TIM_TS_TI1FP1) || \
                                             ((SELECTION) == LL_TIM_TS_TI2FP2) || \
                                             ((SELECTION) == LL_TIM_TS_ETRF))
#define IS_TIM_INTERNAL_TRIGGER_SELECTION(SELECTION) (((SELECTION) == LL_TIM_TS_ITR0) || \
                                                      ((SELECTION) == LL_TIM_TS_ITR1) || \
                                                      ((SELECTION) == LL_TIM_TS_ITR2) || \
                                                      ((SELECTION) == LL_TIM_TS_ITR3))
/**
  * @}
  */ 

/** @defgroup TIM_TIx_External_Clock_Source 
  * @{
  */

#define LL_TIM_TS_TI1FP1      ((uint16_t)0x0050)
#define LL_TIM_TS_TI2FP2      ((uint16_t)0x0060)
#define LL_TIM_TS_TI1F_ED    ((uint16_t)0x0040)
#define IS_TIM_TIXCLK_SOURCE(SOURCE) (((SOURCE) == LL_TIM_TS_TI1FP1) || \
                                      ((SOURCE) == LL_TIM_TS_TI2FP2) || \
                                      ((SOURCE) == LL_TIM_TS_TI1F_ED))
/**
  * @}
  */ 

/** @defgroup TIM_External_Trigger_Polarity 
  * @{
  */ 
#define LL_TIM_ETR_POLARITY_INVERTED        ((uint16_t)0x8000)
#define LL_TIM_ETR_POLARITY_NONINVERTED     ((uint16_t)0x0000)
#define IS_TIM_EXT_POLARITY(POLARITY) (((POLARITY) == LL_TIM_ETR_POLARITY_INVERTED) || \
                                       ((POLARITY) == LL_TIM_ETR_POLARITY_NONINVERTED))
/**
  * @}
  */

/** @defgroup TIM_Prescaler_Reload_Mode 
  * @{
  */

#define TIM_PSCReloadMode_Update           ((uint16_t)0x0000)
#define TIM_PSCReloadMode_Immediate        ((uint16_t)0x0001)
#define IS_TIM_PRESCALER_RELOAD(RELOAD) (((RELOAD) == TIM_PSCReloadMode_Update) || \
                                         ((RELOAD) == TIM_PSCReloadMode_Immediate))
/**
  * @}
  */ 

/** @defgroup TIM_Forced_Action 
  * @{
  */

#define LL_TIM_OCMODE_FORCED_ACTIVE            ((uint16_t)0x0050)
#define LL_TIM_OCMODE_FORCED_INACTIVE          ((uint16_t)0x0040)
#define IS_TIM_FORCED_ACTION(ACTION) (((ACTION) == LL_TIM_OCMODE_FORCED_ACTIVE) || \
                                      ((ACTION) == LL_TIM_OCMODE_FORCED_INACTIVE))
/**
  * @}
  */ 

/** @defgroup TIM_Encoder_Mode 
  * @{
  */

#define LL_TIM_ENCODERMODE_X2_TI1                ((uint16_t)0x0001)
#define LL_TIM_ENCODERMODE_X2_TI2                ((uint16_t)0x0002)
#define LL_TIM_ENCODERMODE_X4_TI12               ((uint16_t)0x0003)
#define IS_TIM_ENCODER_MODE(MODE) (((MODE) == LL_TIM_ENCODERMODE_X2_TI1) || \
                                   ((MODE) == LL_TIM_ENCODERMODE_X2_TI2) || \
                                   ((MODE) == LL_TIM_ENCODERMODE_X4_TI12))
/**
  * @}
  */ 


/** @defgroup TIM_Event_Source 
  * @{
  */

#define TIM_EventSource_Update             ((uint16_t)0x0001)
#define TIM_EventSource_CC1                ((uint16_t)0x0002)
#define TIM_EventSource_CC2                ((uint16_t)0x0004)
#define TIM_EventSource_CC3                ((uint16_t)0x0008)
#define TIM_EventSource_CC4                ((uint16_t)0x0010)
#define TIM_EventSource_COM                ((uint16_t)0x0020)
#define TIM_EventSource_Trigger            ((uint16_t)0x0040)
#define TIM_EventSource_Break              ((uint16_t)0x0080)
#define IS_TIM_EVENT_SOURCE(SOURCE) ((((SOURCE) & (uint16_t)0xFF00) == 0x0000) && ((SOURCE) != 0x0000))

/**
  * @}
  */ 

/** @defgroup TIM_Update_Source 
  * @{
  */

#define LL_TIM_UPDATESOURCE_REGULAR            ((uint16_t)0x0000) /*!< Source of update is the counter overflow/underflow
                                                                   or the setting of UG bit, or an update generation
                                                                   through the slave mode controller. */
#define LL_TIM_UPDATESOURCE_COUNTER           ((uint16_t)0x0001) /*!< Source of update is counter overflow/underflow. */
#define IS_TIM_UPDATE_SOURCE(SOURCE) (((SOURCE) == LL_TIM_UPDATESOURCE_REGULAR) || \
                                      ((SOURCE) == LL_TIM_UPDATESOURCE_COUNTER))
/**
  * @}
  */ 

/** @defgroup TIM_Ouput_Compare_Preload_State 
  * @{
  */

#define TIM_OCPreload_Enable               ((uint16_t)0x0008)
#define TIM_OCPreload_Disable              ((uint16_t)0x0000)
#define IS_TIM_OCPRELOAD_STATE(STATE) (((STATE) == TIM_OCPreload_Enable) || \
                                       ((STATE) == TIM_OCPreload_Disable))
/**
  * @}
  */ 

/** @defgroup TIM_Ouput_Compare_Fast_State 
  * @{
  */

#define TIM_OCFast_Enable                  ((uint16_t)0x0004)
#define TIM_OCFast_Disable                 ((uint16_t)0x0000)
#define IS_TIM_OCFAST_STATE(STATE) (((STATE) == TIM_OCFast_Enable) || \
                                    ((STATE) == TIM_OCFast_Disable))
                                     
/**
  * @}
  */ 

/** @defgroup TIM_Ouput_Compare_Clear_State 
  * @{
  */

#define TIM_OCClear_Enable                 ((uint16_t)0x0080)
#define TIM_OCClear_Disable                ((uint16_t)0x0000)
#define IS_TIM_OCCLEAR_STATE(STATE) (((STATE) == TIM_OCClear_Enable) || \
                                     ((STATE) == TIM_OCClear_Disable))
/**
  * @}
  */ 

/** @defgroup TIM_Trigger_Output_Source 
  * @{
  */

#define LL_TIM_TRGO_RESET               ((uint16_t)0x0000)
#define LL_TIM_TRGO_ENABLE              ((uint16_t)0x0010)
#define LL_TIM_TRGO_UPDATE              ((uint16_t)0x0020)
#define LL_TIM_TRGO_CC1IF                 ((uint16_t)0x0030)
#define LL_TIM_TRGO_OC1REF              ((uint16_t)0x0040)
#define LL_TIM_TRGO_OC2REF              ((uint16_t)0x0050)
#define LL_TIM_TRGO_OC3REF              ((uint16_t)0x0060)
#define LL_TIM_TRGO_OC4REF              ((uint16_t)0x0070)
#define IS_TIM_TRGO_SOURCE(SOURCE) (((SOURCE) == LL_TIM_TRGO_RESET) || \
                                    ((SOURCE) == LL_TIM_TRGO_ENABLE) || \
                                    ((SOURCE) == LL_TIM_TRGO_UPDATE) || \
                                    ((SOURCE) == LL_TIM_TRGO_CC1IF) || \
                                    ((SOURCE) == LL_TIM_TRGO_OC1REF) || \
                                    ((SOURCE) == LL_TIM_TRGO_OC2REF) || \
                                    ((SOURCE) == LL_TIM_TRGO_OC3REF) || \
                                    ((SOURCE) == LL_TIM_TRGO_OC4REF))
/**
  * @}
  */ 

/** @defgroup TIM_Slave_Mode 
  * @{
  */

#define LL_TIM_SLAVEMODE_RESET                ((uint16_t)0x0004)
#define LL_TIM_SLAVEMODE_GATED                ((uint16_t)0x0005)
#define LL_TIM_SLAVEMODE_TRIGGER              ((uint16_t)0x0006)
#define TIM_SlaveMode_External1            ((uint16_t)0x0007)
#define IS_TIM_SLAVE_MODE(MODE) (((MODE) == LL_TIM_SLAVEMODE_RESET) || \
                                 ((MODE) == LL_TIM_SLAVEMODE_GATED) || \
                                 ((MODE) == LL_TIM_SLAVEMODE_TRIGGER) || \
                                 ((MODE) == TIM_SlaveMode_External1))
/**
  * @}
  */ 

/** @defgroup TIM_Master_Slave_Mode 
  * @{
  */

#define TIM_MasterSlaveMode_Enable         ((uint16_t)0x0080)
#define TIM_MasterSlaveMode_Disable        ((uint16_t)0x0000)
#define IS_TIM_MSM_STATE(STATE) (((STATE) == TIM_MasterSlaveMode_Enable) || \
                                 ((STATE) == TIM_MasterSlaveMode_Disable))
/**
  * @}
  */ 

/** @defgroup TIM_Flags 
  * @{
  */

#define TIM_FLAG_Update                    ((uint16_t)0x0001)
#define TIM_FLAG_CC1                       ((uint16_t)0x0002)
#define TIM_FLAG_CC2                       ((uint16_t)0x0004)
#define TIM_FLAG_CC3                       ((uint16_t)0x0008)
#define TIM_FLAG_CC4                       ((uint16_t)0x0010)
#define TIM_FLAG_COM                       ((uint16_t)0x0020)
#define TIM_FLAG_Trigger                   ((uint16_t)0x0040)
#define TIM_FLAG_Break                     ((uint16_t)0x0080)
#define TIM_FLAG_CC1OF                     ((uint16_t)0x0200)
#define TIM_FLAG_CC2OF                     ((uint16_t)0x0400)
#define TIM_FLAG_CC3OF                     ((uint16_t)0x0800)
#define TIM_FLAG_CC4OF                     ((uint16_t)0x1000)
#define IS_TIM_GET_FLAG(FLAG) (((FLAG) == TIM_FLAG_Update) || \
                               ((FLAG) == TIM_FLAG_CC1) || \
                               ((FLAG) == TIM_FLAG_CC2) || \
                               ((FLAG) == TIM_FLAG_CC3) || \
                               ((FLAG) == TIM_FLAG_CC4) || \
                               ((FLAG) == TIM_FLAG_COM) || \
                               ((FLAG) == TIM_FLAG_Trigger) || \
                               ((FLAG) == TIM_FLAG_Break) || \
                               ((FLAG) == TIM_FLAG_CC1OF) || \
                               ((FLAG) == TIM_FLAG_CC2OF) || \
                               ((FLAG) == TIM_FLAG_CC3OF) || \
                               ((FLAG) == TIM_FLAG_CC4OF))
                               
                               
#define IS_TIM_CLEAR_FLAG(TIM_FLAG) ((((TIM_FLAG) & (uint16_t)0xE100) == 0x0000) && ((TIM_FLAG) != 0x0000))
/**
  * @}
  */ 

/** @defgroup TIM_Input_Capture_Filer_Value 
  * @{
  */

#define IS_TIM_IC_FILTER(ICFILTER) ((ICFILTER) <= 0xF) 
/**
  * @}
  */ 

/** @defgroup TIM_External_Trigger_Filter 
  * @{
  */

#define IS_TIM_EXT_FILTER(EXTFILTER) ((EXTFILTER) <= 0xF)
/**
  * @}
  */ 

/**
  * @}
  */

/** @defgroup TIM_Exported_Macros
  * @{
  */

/**
  * @}
  */ 

/** @defgroup TIM_Exported_Functions
  * @{
  */

void LL_TIM_DeInit(TIM_TypeDef* TIMx);
void LL_TIM_Init(TIM_TypeDef* TIMx, LL_TIM_InitTypeDef* TIM_TimeBaseInitStruct);
void LL_TIM_OC_Init(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OC_InitTypeDef* TIM_OCInitStruct);
void LL_TIM_OC_Init(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OC_InitTypeDef* TIM_OCInitStruct);
void LL_TIM_OC_Init(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OC_InitTypeDef* TIM_OCInitStruct);
void LL_TIM_OC_Init(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OC_InitTypeDef* TIM_OCInitStruct);
void LL_TIM_IC_Init(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, &LL_TIM_IC_InitTypeDef* TIM_ICInitStruct);
uint16_t tmp_tim_icoppositepolarity_17 = (uint16_t)LL_TIM_IC_POLARITY_RISING;
uint16_t tmp_tim_icoppositeselection_17 = (uint16_t)LL_TIM_ACTIVEINPUT_DIRECTTI;
/* Select the Opposite Input Polarity */
if (TIM_ICInitTypeDef* TIM_ICInitStruct.ICPolarity == LL_TIM_IC_POLARITY_RISING)
{
tmp_tim_icoppositepolarity_17 = (uint16_t)LL_TIM_IC_POLARITY_FALLING;
}
else
{
tmp_tim_icoppositepolarity_17 = (uint16_t)LL_TIM_IC_POLARITY_RISING;
}
/* Select the Opposite Input */
if (TIM_ICInitTypeDef* TIM_ICInitStruct.ICActiveInput == LL_TIM_ACTIVEINPUT_DIRECTTI)
{
tmp_tim_icoppositeselection_17 = (uint16_t)LL_TIM_ACTIVEINPUT_INDIRECTTI;
}
else
{
tmp_tim_icoppositeselection_17 = (uint16_t)LL_TIM_ACTIVEINPUT_INDIRECTTI;
}
/* TI1 Configuration */
LL_TIM_CC_DisableChannel(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1);
LL_TIM_IC_SetActiveInput(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, TIM_ICInitTypeDef* TIM_ICInitStruct.ICActiveInput);
LL_TIM_IC_SetFilter(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, TIM_ICInitTypeDef* TIM_ICInitStruct.ICFilter);
LL_TIM_IC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, TIM_ICInitTypeDef* TIM_ICInitStruct.ICPolarity);
LL_TIM_CC_EnableChannel(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1);
/* Set the Input Capture Prescaler value */
LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, TIM_ICInitTypeDef* TIM_ICInitStruct.ICPrescaler);
/* TI2 Configuration */
LL_TIM_CC_DisableChannel(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2);
LL_TIM_IC_SetActiveInput(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, tmp_tim_icoppositeselection_17);
LL_TIM_IC_SetFilter(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, TIM_ICInitTypeDef* TIM_ICInitStruct.ICFilter);
LL_TIM_IC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, tmp_tim_icoppositepolarity_17);
LL_TIM_CC_EnableChannel(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2);
/* Set the Input Capture Prescaler value */
void LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_IC_InitTypeDef* TIM_ICInitStruct.ICPrescaler);
void LL_TIM_BDTR_Init(TIM_TypeDef* TIMx, LL_TIM_BDTR_InitTypeDef *TIM_BDTRInitStruct);
void LL_TIM_StructInit(LL_TIM_InitTypeDef* TIM_TimeBaseInitStruct);
void LL_TIM_OC_StructInit(LL_TIM_OC_InitTypeDef* TIM_OCInitStruct);
void LL_TIM_IC_StructInit(LL_TIM_IC_InitTypeDef* TIM_ICInitStruct);
void LL_TIM_BDTR_StructInit(LL_TIM_BDTR_InitTypeDef* TIM_BDTRInitStruct);
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
void TIM_GenerateEvent(TIM_TypeDef* TIMx, uint16_t TIM_EventSource);
void LL_TIM_ConfigDMABurst(TIM_TypeDef* TIMx, uint16_t TIM_DMABase, uint16_t TIM_DMABurstLength);
void TIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState);
void LL_TIM_SetClockSource(TIM_TypeDef* TIMx, LL_TIM_CLOCKSOURCE_INTERNAL);
LL_TIM_SetTriggerInput(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
void LL_TIM_SetSlaveMode(TIM_TypeDef* TIMx, (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0));
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, uint16_t TIM_TIxExternalCLKSource,
                                uint16_t TIM_ICPolarity, uint16_t ICFilter);
LL_TIM_ConfigETR(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPolarity, uint16_t TIM_ExtTRGPrescaler, uint16_t ExtTRGFilter);
LL_TIM_SetSlaveMode(TIM_TypeDef* TIMx, (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0));
void LL_TIM_SetTriggerInput(TIM_TypeDef* TIMx, LL_TIM_TS_ETRF);
LL_TIM_ConfigETR(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPolarity, uint16_t TIM_ExtTRGPrescaler, uint16_t ExtTRGFilter);
void LL_TIM_SetClockSource(TIM_TypeDef* TIMx, LL_TIM_CLOCKSOURCE_EXT_MODE2);
void LL_TIM_ConfigETR(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPolarity, uint16_t TIM_ExtTRGPrescaler, uint16_t ExtTRGFilter);
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, uint16_t Prescaler, uint16_t TIM_PSCReloadMode);
void LL_TIM_SetCounterMode(TIM_TypeDef* TIMx, uint16_t TIM_CounterMode);
void LL_TIM_SetTriggerInput(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
LL_TIM_SetEncoderMode(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode);
LL_TIM_IC_SetActiveInput(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
LL_TIM_IC_SetActiveInput(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
LL_TIM_IC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, uint16_t TIM_IC1Polarity);
void LL_TIM_IC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, uint16_t TIM_IC2Polarity);
void LL_TIM_OC_SetMode(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, uint16_t TIM_ForcedAction);
void LL_TIM_OC_SetMode(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, uint16_t TIM_ForcedAction);
void LL_TIM_OC_SetMode(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH3, uint16_t TIM_ForcedAction);
void LL_TIM_OC_SetMode(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH4, uint16_t TIM_ForcedAction);
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState);
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, uint16_t TIM_OCFast);
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, uint16_t TIM_OCClear);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, uint16_t TIM_OCPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1N, uint16_t TIM_OCNPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, uint16_t TIM_OCPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2N, uint16_t TIM_OCNPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH3, uint16_t TIM_OCPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH3N, uint16_t TIM_OCNPolarity);
void LL_TIM_OC_SetPolarity(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH4, uint16_t TIM_OCPolarity);
void TIM_CCxCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);
void TIM_CCxNCmd(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_CCxN);
void LL_TIM_OC_SetMode(TIM_TypeDef* TIMx, uint16_t TIM_Channel, uint16_t TIM_OCMode);
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState);
void LL_TIM_SetUpdateSource(TIM_TypeDef* TIMx, uint16_t TIM_UpdateSource);
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState);
void LL_TIM_SetOnePulseMode(TIM_TypeDef* TIMx, uint16_t TIM_OPMode);
void LL_TIM_SetTriggerOutput(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
void LL_TIM_SetSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);
void LL_TIM_SetCounter(TIM_TypeDef* TIMx, uint16_t Counter);
void LL_TIM_SetAutoReload(TIM_TypeDef* TIMx, uint16_t Autoreload);
void LL_TIM_OC_SetCompareCH1(TIM_TypeDef* TIMx, uint16_t Compare1);
void LL_TIM_OC_SetCompareCH2(TIM_TypeDef* TIMx, uint16_t Compare2);
void LL_TIM_OC_SetCompareCH3(TIM_TypeDef* TIMx, uint16_t Compare3);
void LL_TIM_OC_SetCompareCH4(TIM_TypeDef* TIMx, uint16_t Compare4);
void LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH1, uint16_t TIM_ICPSC);
void LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH2, uint16_t TIM_ICPSC);
void LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH3, uint16_t TIM_ICPSC);
void LL_TIM_IC_SetPrescaler(TIM_TypeDef* TIMx, LL_TIM_CHANNEL_CH4, uint16_t TIM_ICPSC);
void LL_TIM_SetClockDivision(TIM_TypeDef* TIMx, uint16_t TIM_CKD);
uint16_t LL_TIM_IC_GetCaptureCH1(TIM_TypeDef* TIMx);
uint16_t LL_TIM_IC_GetCaptureCH2(TIM_TypeDef* TIMx);
uint16_t LL_TIM_IC_GetCaptureCH3(TIM_TypeDef* TIMx);
uint16_t LL_TIM_IC_GetCaptureCH4(TIM_TypeDef* TIMx);
uint16_t LL_TIM_GetCounter(TIM_TypeDef* TIMx);
uint16_t LL_TIM_GetPrescaler(TIM_TypeDef* TIMx);
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F10x_TIM_H */
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
