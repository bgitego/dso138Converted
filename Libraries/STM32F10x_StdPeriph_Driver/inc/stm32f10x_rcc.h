/**
  ******************************************************************************
  * @file    stm32f10x_rcc.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the RCC firmware 
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
#ifndef __STM32F10x_RCC_H
#define __STM32F10x_RCC_H

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

/** @addtogroup RCC
  * @{
  */

/** @defgroup RCC_Exported_Types
  * @{
  */

typedef struct
{
  uint32_t SYSCLK_Frequency;  /*!< returns SYSCLK clock frequency expressed in Hz */
  uint32_t HCLK_Frequency;    /*!< returns HCLK clock frequency expressed in Hz */
  uint32_t PCLK1_Frequency;   /*!< returns PCLK1 clock frequency expressed in Hz */
  uint32_t PCLK2_Frequency;   /*!< returns PCLK2 clock frequency expressed in Hz */
  uint32_t ADCCLK_Frequency;  /*!< returns ADCCLK clock frequency expressed in Hz */
}LL_RCC_ClocksTypeDef;

/**
  * @}
  */

/** @defgroup RCC_Exported_Constants
  * @{
  */

/** @defgroup HSE_configuration 
  * @{
  */

#define RCC_HSE_OFF                      ((uint32_t)0x00000000)
#define RCC_HSE_ON                       ((uint32_t)0x00010000)
#define RCC_HSE_Bypass                   ((uint32_t)0x00040000)
#define IS_RCC_HSE(HSE) (((HSE) == RCC_HSE_OFF) || ((HSE) == RCC_HSE_ON) || \
                         ((HSE) == RCC_HSE_Bypass))

/**
  * @}
  */ 

/** @defgroup PLL_entry_clock_source 
  * @{
  */

#define LL_RCC_PLLSOURCE_HSI_DIV_2           ((uint32_t)0x00000000)

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_CL)
 #define LL_RCC_PLLSOURCE_HSE_DIV_1           ((uint32_t)0x00010000)
 #define LL_RCC_PLLSOURCE_HSE_DIV_2           ((uint32_t)0x00030000)
 #define IS_RCC_PLL_SOURCE(SOURCE) (((SOURCE) == LL_RCC_PLLSOURCE_HSI_DIV_2) || \
                                   ((SOURCE) == LL_RCC_PLLSOURCE_HSE_DIV_1) || \
                                   ((SOURCE) == LL_RCC_PLLSOURCE_HSE_DIV_2))
#else
 #define RCC_PLLSource_PREDIV1            ((uint32_t)0x00010000)
 #define IS_RCC_PLL_SOURCE(SOURCE) (((SOURCE) == LL_RCC_PLLSOURCE_HSI_DIV_2) || \
                                   ((SOURCE) == RCC_PLLSource_PREDIV1))
#endif /* STM32F10X_CL */ 

/**
  * @}
  */ 

/** @defgroup PLL_multiplication_factor 
  * @{
  */
#ifndef STM32F10X_CL
 #define LL_RCC_PLL_MUL_2                    ((uint32_t)0x00000000)
 #define LL_RCC_PLL_MUL_3                    ((uint32_t)0x00040000)
 #define LL_RCC_PLL_MUL_4                    ((uint32_t)0x00080000)
 #define LL_RCC_PLL_MUL_5                    ((uint32_t)0x000C0000)
 #define LL_RCC_PLL_MUL_6                    ((uint32_t)0x00100000)
 #define LL_RCC_PLL_MUL_7                    ((uint32_t)0x00140000)
 #define LL_RCC_PLL_MUL_8                    ((uint32_t)0x00180000)
 #define LL_RCC_PLL_MUL_9                    ((uint32_t)0x001C0000)
 #define LL_RCC_PLL_MUL_10                   ((uint32_t)0x00200000)
 #define LL_RCC_PLL_MUL_11                   ((uint32_t)0x00240000)
 #define LL_RCC_PLL_MUL_12                   ((uint32_t)0x00280000)
 #define LL_RCC_PLL_MUL_13                   ((uint32_t)0x002C0000)
 #define LL_RCC_PLL_MUL_14                   ((uint32_t)0x00300000)
 #define LL_RCC_PLL_MUL_15                   ((uint32_t)0x00340000)
 #define LL_RCC_PLL_MUL_16                   ((uint32_t)0x00380000)
 #define IS_RCC_PLL_MUL(MUL) (((MUL) == LL_RCC_PLL_MUL_2) || ((MUL) == LL_RCC_PLL_MUL_3)   || \
                              ((MUL) == LL_RCC_PLL_MUL_4) || ((MUL) == LL_RCC_PLL_MUL_5)   || \
                              ((MUL) == LL_RCC_PLL_MUL_6) || ((MUL) == LL_RCC_PLL_MUL_7)   || \
                              ((MUL) == LL_RCC_PLL_MUL_8) || ((MUL) == LL_RCC_PLL_MUL_9)   || \
                              ((MUL) == LL_RCC_PLL_MUL_10) || ((MUL) == LL_RCC_PLL_MUL_11) || \
                              ((MUL) == LL_RCC_PLL_MUL_12) || ((MUL) == LL_RCC_PLL_MUL_13) || \
                              ((MUL) == LL_RCC_PLL_MUL_14) || ((MUL) == LL_RCC_PLL_MUL_15) || \
                              ((MUL) == LL_RCC_PLL_MUL_16))

#else
 #define LL_RCC_PLL_MUL_4                    ((uint32_t)0x00080000)
 #define LL_RCC_PLL_MUL_5                    ((uint32_t)0x000C0000)
 #define LL_RCC_PLL_MUL_6                    ((uint32_t)0x00100000)
 #define LL_RCC_PLL_MUL_7                    ((uint32_t)0x00140000)
 #define LL_RCC_PLL_MUL_8                    ((uint32_t)0x00180000)
 #define LL_RCC_PLL_MUL_9                    ((uint32_t)0x001C0000)
 #define LL_RCC_PLL_MUL_6_5                  ((uint32_t)0x00340000)

 #define IS_RCC_PLL_MUL(MUL) (((MUL) == LL_RCC_PLL_MUL_4) || ((MUL) == LL_RCC_PLL_MUL_5) || \
                              ((MUL) == LL_RCC_PLL_MUL_6) || ((MUL) == LL_RCC_PLL_MUL_7) || \
                              ((MUL) == LL_RCC_PLL_MUL_8) || ((MUL) == LL_RCC_PLL_MUL_9) || \
                              ((MUL) == LL_RCC_PLL_MUL_6_5))
#endif /* STM32F10X_CL */                              
/**
  * @}
  */

/** @defgroup PREDIV1_division_factor
  * @{
  */
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_CL)
 #define  LL_RCC_PREDIV_DIV_1               ((uint32_t)0x00000000)
 #define  LL_RCC_PREDIV_DIV_2               ((uint32_t)0x00000001)
 #define  LL_RCC_PREDIV_DIV_3               ((uint32_t)0x00000002)
 #define  LL_RCC_PREDIV_DIV_4               ((uint32_t)0x00000003)
 #define  LL_RCC_PREDIV_DIV_5               ((uint32_t)0x00000004)
 #define  LL_RCC_PREDIV_DIV_6               ((uint32_t)0x00000005)
 #define  LL_RCC_PREDIV_DIV_7               ((uint32_t)0x00000006)
 #define  LL_RCC_PREDIV_DIV_8               ((uint32_t)0x00000007)
 #define  LL_RCC_PREDIV_DIV_9               ((uint32_t)0x00000008)
 #define  LL_RCC_PREDIV_DIV_10              ((uint32_t)0x00000009)
 #define  LL_RCC_PREDIV_DIV_11              ((uint32_t)0x0000000A)
 #define  LL_RCC_PREDIV_DIV_12              ((uint32_t)0x0000000B)
 #define  LL_RCC_PREDIV_DIV_13              ((uint32_t)0x0000000C)
 #define  LL_RCC_PREDIV_DIV_14              ((uint32_t)0x0000000D)
 #define  LL_RCC_PREDIV_DIV_15              ((uint32_t)0x0000000E)
 #define  LL_RCC_PREDIV_DIV_16              ((uint32_t)0x0000000F)

 #define IS_RCC_PREDIV1(PREDIV1) (((PREDIV1) == LL_RCC_PREDIV_DIV_1) || ((PREDIV1) == LL_RCC_PREDIV_DIV_2) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_3) || ((PREDIV1) == LL_RCC_PREDIV_DIV_4) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_5) || ((PREDIV1) == LL_RCC_PREDIV_DIV_6) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_7) || ((PREDIV1) == LL_RCC_PREDIV_DIV_8) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_9) || ((PREDIV1) == LL_RCC_PREDIV_DIV_10) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_11) || ((PREDIV1) == LL_RCC_PREDIV_DIV_12) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_13) || ((PREDIV1) == LL_RCC_PREDIV_DIV_14) || \
                                  ((PREDIV1) == LL_RCC_PREDIV_DIV_15) || ((PREDIV1) == LL_RCC_PREDIV_DIV_16))
#endif
/**
  * @}
  */


/** @defgroup PREDIV1_clock_source
  * @{
  */
#ifdef STM32F10X_CL
/* PREDIV1 clock source (for STM32 connectivity line devices) */
 #define  LL_RCC_PLLSOURCE_HSE         ((uint32_t)0x00000000) 
 #define  LL_RCC_PLLSOURCE_PLL2        ((uint32_t)0x00010000) 

 #define IS_RCC_PREDIV1_SOURCE(SOURCE) (((SOURCE) == LL_RCC_PLLSOURCE_HSE) || \
                                        ((SOURCE) == LL_RCC_PLLSOURCE_PLL2)) 
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
/* PREDIV1 clock source (for STM32 Value line devices) */
 #define  LL_RCC_PLLSOURCE_HSE         ((uint32_t)0x00000000) 

 #define IS_RCC_PREDIV1_SOURCE(SOURCE) (((SOURCE) == LL_RCC_PLLSOURCE_HSE)) 
#endif
/**
  * @}
  */

#ifdef STM32F10X_CL
/** @defgroup PREDIV2_division_factor
  * @{
  */
  
 #define  LL_RCC_HSE_PREDIV2_DIV_1               ((uint32_t)0x00000000)
 #define  LL_RCC_HSE_PREDIV2_DIV_2               ((uint32_t)0x00000010)
 #define  LL_RCC_HSE_PREDIV2_DIV_3               ((uint32_t)0x00000020)
 #define  LL_RCC_HSE_PREDIV2_DIV_4               ((uint32_t)0x00000030)
 #define  LL_RCC_HSE_PREDIV2_DIV_5               ((uint32_t)0x00000040)
 #define  LL_RCC_HSE_PREDIV2_DIV_6               ((uint32_t)0x00000050)
 #define  LL_RCC_HSE_PREDIV2_DIV_7               ((uint32_t)0x00000060)
 #define  LL_RCC_HSE_PREDIV2_DIV_8               ((uint32_t)0x00000070)
 #define  LL_RCC_HSE_PREDIV2_DIV_9               ((uint32_t)0x00000080)
 #define  LL_RCC_HSE_PREDIV2_DIV_10              ((uint32_t)0x00000090)
 #define  LL_RCC_HSE_PREDIV2_DIV_11              ((uint32_t)0x000000A0)
 #define  LL_RCC_HSE_PREDIV2_DIV_12              ((uint32_t)0x000000B0)
 #define  LL_RCC_HSE_PREDIV2_DIV_13              ((uint32_t)0x000000C0)
 #define  LL_RCC_HSE_PREDIV2_DIV_14              ((uint32_t)0x000000D0)
 #define  LL_RCC_HSE_PREDIV2_DIV_15              ((uint32_t)0x000000E0)
 #define  LL_RCC_HSE_PREDIV2_DIV_16              ((uint32_t)0x000000F0)

 #define IS_RCC_PREDIV2(PREDIV2) (((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_1) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_2) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_3) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_4) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_5) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_6) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_7) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_8) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_9) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_10) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_11) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_12) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_13) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_14) || \
                                  ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_15) || ((PREDIV2) == LL_RCC_HSE_PREDIV2_DIV_16))
/**
  * @}
  */


/** @defgroup PLL2_multiplication_factor
  * @{
  */
  
 #define  LL_RCC_PLL2_MUL_8                  ((uint32_t)0x00000600)
 #define  LL_RCC_PLL2_MUL_9                  ((uint32_t)0x00000700)
 #define  LL_RCC_PLL2_MUL_10                 ((uint32_t)0x00000800)
 #define  LL_RCC_PLL2_MUL_11                 ((uint32_t)0x00000900)
 #define  LL_RCC_PLL2_MUL_12                 ((uint32_t)0x00000A00)
 #define  LL_RCC_PLL2_MUL_13                 ((uint32_t)0x00000B00)
 #define  LL_RCC_PLL2_MUL_14                 ((uint32_t)0x00000C00)
 #define  LL_RCC_PLL2_MUL_16                 ((uint32_t)0x00000E00)
 #define  LL_RCC_PLL2_MUL_20                 ((uint32_t)0x00000F00)

 #define IS_RCC_PLL2_MUL(MUL) (((MUL) == LL_RCC_PLL2_MUL_8) || ((MUL) == LL_RCC_PLL2_MUL_9)  || \
                               ((MUL) == LL_RCC_PLL2_MUL_10) || ((MUL) == LL_RCC_PLL2_MUL_11) || \
                               ((MUL) == LL_RCC_PLL2_MUL_12) || ((MUL) == LL_RCC_PLL2_MUL_13) || \
                               ((MUL) == LL_RCC_PLL2_MUL_14) || ((MUL) == LL_RCC_PLL2_MUL_16) || \
                               ((MUL) == LL_RCC_PLL2_MUL_20))
/**
  * @}
  */


/** @defgroup PLL3_multiplication_factor
  * @{
  */

 #define  LL_RCC_PLLI2S_MUL_8                  ((uint32_t)0x00006000)
 #define  LL_RCC_PLLI2S_MUL_9                  ((uint32_t)0x00007000)
 #define  LL_RCC_PLLI2S_MUL_10                 ((uint32_t)0x00008000)
 #define  LL_RCC_PLLI2S_MUL_11                 ((uint32_t)0x00009000)
 #define  LL_RCC_PLLI2S_MUL_12                 ((uint32_t)0x0000A000)
 #define  LL_RCC_PLLI2S_MUL_13                 ((uint32_t)0x0000B000)
 #define  LL_RCC_PLLI2S_MUL_14                 ((uint32_t)0x0000C000)
 #define  LL_RCC_PLLI2S_MUL_16                 ((uint32_t)0x0000E000)
 #define  LL_RCC_PLLI2S_MUL_20                 ((uint32_t)0x0000F000)

 #define IS_RCC_PLL3_MUL(MUL) (((MUL) == LL_RCC_PLLI2S_MUL_8) || ((MUL) == LL_RCC_PLLI2S_MUL_9)  || \
                               ((MUL) == LL_RCC_PLLI2S_MUL_10) || ((MUL) == LL_RCC_PLLI2S_MUL_11) || \
                               ((MUL) == LL_RCC_PLLI2S_MUL_12) || ((MUL) == LL_RCC_PLLI2S_MUL_13) || \
                               ((MUL) == LL_RCC_PLLI2S_MUL_14) || ((MUL) == LL_RCC_PLLI2S_MUL_16) || \
                               ((MUL) == LL_RCC_PLLI2S_MUL_20))
/**
  * @}
  */

#endif /* STM32F10X_CL */


/** @defgroup System_clock_source 
  * @{
  */

#define LL_RCC_SYS_CLKSOURCE_HSI             ((uint32_t)0x00000000)
#define LL_RCC_SYS_CLKSOURCE_HSE             ((uint32_t)0x00000001)
#define LL_RCC_SYS_CLKSOURCE_PLL          ((uint32_t)0x00000002)
#define IS_RCC_SYSCLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_SYS_CLKSOURCE_HSI) || \
                                      ((SOURCE) == LL_RCC_SYS_CLKSOURCE_HSE) || \
                                      ((SOURCE) == LL_RCC_SYS_CLKSOURCE_PLL))
/**
  * @}
  */

/** @defgroup AHB_clock_source 
  * @{
  */

#define LL_RCC_SYSCLK_DIV_1                  ((uint32_t)0x00000000)
#define LL_RCC_SYSCLK_DIV_2                  ((uint32_t)0x00000080)
#define LL_RCC_SYSCLK_DIV_4                  ((uint32_t)0x00000090)
#define LL_RCC_SYSCLK_DIV_8                  ((uint32_t)0x000000A0)
#define LL_RCC_SYSCLK_DIV_16                 ((uint32_t)0x000000B0)
#define LL_RCC_SYSCLK_DIV_64                 ((uint32_t)0x000000C0)
#define LL_RCC_SYSCLK_DIV_128                ((uint32_t)0x000000D0)
#define LL_RCC_SYSCLK_DIV_256                ((uint32_t)0x000000E0)
#define LL_RCC_SYSCLK_DIV_512                ((uint32_t)0x000000F0)
#define IS_RCC_HCLK(HCLK) (((HCLK) == LL_RCC_SYSCLK_DIV_1) || ((HCLK) == LL_RCC_SYSCLK_DIV_2) || \
                           ((HCLK) == LL_RCC_SYSCLK_DIV_4) || ((HCLK) == LL_RCC_SYSCLK_DIV_8) || \
                           ((HCLK) == LL_RCC_SYSCLK_DIV_16) || ((HCLK) == LL_RCC_SYSCLK_DIV_64) || \
                           ((HCLK) == LL_RCC_SYSCLK_DIV_128) || ((HCLK) == LL_RCC_SYSCLK_DIV_256) || \
                           ((HCLK) == LL_RCC_SYSCLK_DIV_512))
/**
  * @}
  */ 

/** @defgroup APB1_APB2_clock_source 
  * @{
  */

#define LL_RCC_APB1_DIV_1                    ((uint32_t)0x00000000)
#define LL_RCC_APB1_DIV_2                    ((uint32_t)0x00000400)
#define LL_RCC_APB1_DIV_4                    ((uint32_t)0x00000500)
#define LL_RCC_APB1_DIV_8                    ((uint32_t)0x00000600)
#define LL_RCC_APB1_DIV_16                   ((uint32_t)0x00000700)
#define IS_RCC_PCLK(PCLK) (((PCLK) == LL_RCC_APB1_DIV_1) || ((PCLK) == LL_RCC_APB1_DIV_2) || \
                           ((PCLK) == LL_RCC_APB1_DIV_4) || ((PCLK) == LL_RCC_APB1_DIV_8) || \
                           ((PCLK) == LL_RCC_APB1_DIV_16))
/**
  * @}
  */

/** @defgroup RCC_Interrupt_source 
  * @{
  */

#define LL_RCC_CIR_LSIRDYIE                    ((uint8_t)0x01)
#define LL_RCC_CIR_LSERDYIE                    ((uint8_t)0x02)
#define LL_RCC_CIR_HSIRDYIE                    ((uint8_t)0x04)
#define LL_RCC_CIR_HSERDYIE                    ((uint8_t)0x08)
#define LL_RCC_CIR_PLLRDYIE                    ((uint8_t)0x10)
#define LL_RCC_CIR_CSSC                       ((uint8_t)0x80)

#ifndef STM32F10X_CL
 #define IS_RCC_IT(IT) ((((IT) & (uint8_t)0xE0) == 0x00) && ((IT) != 0x00))
 #define IS_RCC_GET_IT(IT) (((IT) == LL_RCC_CIR_LSIRDYIE) || ((IT) == LL_RCC_CIR_LSERDYIE) || \
                            ((IT) == LL_RCC_CIR_HSIRDYIE) || ((IT) == LL_RCC_CIR_HSERDYIE) || \
                            ((IT) == LL_RCC_CIR_PLLRDYIE) || ((IT) == LL_RCC_CIR_CSSC))
 #define IS_RCC_CLEAR_IT(IT) ((((IT) & (uint8_t)0x60) == 0x00) && ((IT) != 0x00))
#else
 #define LL_RCC_CIR_PLL2RDYIE                  ((uint8_t)0x20)
 #define LL_RCC_CIR_PLL3RDYIE                  ((uint8_t)0x40)
 #define IS_RCC_IT(IT) ((((IT) & (uint8_t)0x80) == 0x00) && ((IT) != 0x00))
 #define IS_RCC_GET_IT(IT) (((IT) == LL_RCC_CIR_LSIRDYIE) || ((IT) == LL_RCC_CIR_LSERDYIE) || \
                            ((IT) == LL_RCC_CIR_HSIRDYIE) || ((IT) == LL_RCC_CIR_HSERDYIE) || \
                            ((IT) == LL_RCC_CIR_PLLRDYIE) || ((IT) == LL_RCC_CIR_CSSC) || \
                            ((IT) == LL_RCC_CIR_PLL2RDYIE) || ((IT) == LL_RCC_CIR_PLL3RDYIE))
 #define IS_RCC_CLEAR_IT(IT) ((IT) != 0x00)
#endif /* STM32F10X_CL */ 


/**
  * @}
  */

#ifndef STM32F10X_CL
/** @defgroup USB_Device_clock_source 
  * @{
  */

 #define LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5   ((uint8_t)0x00)
 #define LL_RCC_USB_CLKSOURCE_PLL    ((uint8_t)0x01)

 #define IS_RCC_USBCLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5) || \
                                      ((SOURCE) == LL_RCC_USB_CLKSOURCE_PLL))
/**
  * @}
  */
#else
/** @defgroup USB_OTG_FS_clock_source 
  * @{
  */
 #define LL_RCC_USB_CLKSOURCE_PLL_DIV_3    ((uint8_t)0x00)
 #define LL_RCC_USB_CLKSOURCE_PLL_DIV_2    ((uint8_t)0x01)

 #define IS_RCC_OTGFSCLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_USB_CLKSOURCE_PLL_DIV_3) || \
                                         ((SOURCE) == LL_RCC_USB_CLKSOURCE_PLL_DIV_2))
/**
  * @}
  */
#endif /* STM32F10X_CL */ 


#ifdef STM32F10X_CL
/** @defgroup I2S2_clock_source 
  * @{
  */
 #define LL_RCC_I2S2_CLKSOURCE_SYSCLK        ((uint8_t)0x00)
 #define LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO      ((uint8_t)0x01)

 #define IS_RCC_I2S2CLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_I2S2_CLKSOURCE_SYSCLK) || \
                                        ((SOURCE) == LL_RCC_I2S2_CLKSOURCE_PLLI2S_VCO))
/**
  * @}
  */

/** @defgroup I2S3_clock_source 
  * @{
  */
 #define LL_RCC_I2S3_CLKSOURCE_SYSCLK        ((uint8_t)0x00)
 #define LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO      ((uint8_t)0x01)

 #define IS_RCC_I2S3CLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_I2S3_CLKSOURCE_SYSCLK) || \
                                        ((SOURCE) == LL_RCC_I2S3_CLKSOURCE_PLLI2S_VCO))    
/**
  * @}
  */
#endif /* STM32F10X_CL */  
  

/** @defgroup ADC_clock_source 
  * @{
  */

#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_2                   ((uint32_t)0x00000000)
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_4                   ((uint32_t)0x00004000)
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_6                   ((uint32_t)0x00008000)
#define LL_RCC_ADC_CLKSRC_PCLK2_DIV_8                   ((uint32_t)0x0000C000)
#define IS_RCC_ADCCLK(ADCCLK) (((ADCCLK) == LL_RCC_ADC_CLKSRC_PCLK2_DIV_2) || ((ADCCLK) == LL_RCC_ADC_CLKSRC_PCLK2_DIV_4) || \
                               ((ADCCLK) == LL_RCC_ADC_CLKSRC_PCLK2_DIV_6) || ((ADCCLK) == LL_RCC_ADC_CLKSRC_PCLK2_DIV_8))
/**
  * @}
  */

/** @defgroup LSE_configuration 
  * @{
  */

#define RCC_LSE_OFF                      ((uint8_t)0x00)
#define RCC_LSE_ON                       ((uint8_t)0x01)
#define RCC_LSE_Bypass                   ((uint8_t)0x04)
#define IS_RCC_LSE(LSE) (((LSE) == RCC_LSE_OFF) || ((LSE) == RCC_LSE_ON) || \
                         ((LSE) == RCC_LSE_Bypass))
/**
  * @}
  */

/** @defgroup RTC_clock_source 
  * @{
  */

#define LL_RCC_RTC_CLKSOURCE_LSE             ((uint32_t)0x00000100)
#define RCC_RTCCLKSource_LSI             ((uint32_t)0x00000200)
#define LL_RCC_RTC_HSE_DIV_128      ((uint32_t)0x00000300)
#define IS_RCC_RTCCLK_SOURCE(SOURCE) (((SOURCE) == LL_RCC_RTC_CLKSOURCE_LSE) || \
                                      ((SOURCE) == RCC_RTCCLKSource_LSI) || \
                                      ((SOURCE) == LL_RCC_RTC_HSE_DIV_128))
/**
  * @}
  */

/** @defgroup AHB_peripheral 
  * @{
  */

#define LL_AHB1_GRP1_PERIPH_DMA1               ((uint32_t)0x00000001)
#define LL_AHB1_GRP1_PERIPH_DMA2               ((uint32_t)0x00000002)
#define LL_AHB1_GRP1_PERIPH_SRAM               ((uint32_t)0x00000004)
#define LL_AHB1_GRP1_PERIPH_FLASH              ((uint32_t)0x00000010)
#define LL_AHB1_GRP1_PERIPH_CRC                ((uint32_t)0x00000040)

#ifndef STM32F10X_CL
 #define LL_AHB1_GRP1_PERIPH_FSMC              ((uint32_t)0x00000100)
 #define LL_AHB1_GRP1_PERIPH_SDIO              ((uint32_t)0x00000400)
 #define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFAA8) == 0x00) && ((PERIPH) != 0x00))
#else
 #define LL_AHB1_GRP1_PERIPH_OTGFS            ((uint32_t)0x00001000)
 #define LL_AHB1_GRP1_PERIPH_ETHMAC           ((uint32_t)0x00004000)
 #define LL_AHB1_GRP1_PERIPH_ETHMACRX        ((uint32_t)0x00008000)
 #define LL_AHB1_GRP1_PERIPH_ETHMACTX        ((uint32_t)0x00010000)

 #define IS_RCC_AHB_PERIPH(PERIPH) ((((PERIPH) & 0xFFFE2FA8) == 0x00) && ((PERIPH) != 0x00))
 #define IS_RCC_AHB_PERIPH_RESET(PERIPH) ((((PERIPH) & 0xFFFFAFFF) == 0x00) && ((PERIPH) != 0x00))
#endif /* STM32F10X_CL */
/**
  * @}
  */

/** @defgroup APB2_peripheral 
  * @{
  */

#define LL_APB2_GRP1_PERIPH_AFIO              ((uint32_t)0x00000001)
#define LL_APB2_GRP1_PERIPH_GPIOA             ((uint32_t)0x00000004)
#define LL_APB2_GRP1_PERIPH_GPIOB             ((uint32_t)0x00000008)
#define LL_APB2_GRP1_PERIPH_GPIOC             ((uint32_t)0x00000010)
#define LL_APB2_GRP1_PERIPH_GPIOD             ((uint32_t)0x00000020)
#define LL_APB2_GRP1_PERIPH_GPIOE             ((uint32_t)0x00000040)
#define LL_APB2_GRP1_PERIPH_GPIOF             ((uint32_t)0x00000080)
#define LL_APB2_GRP1_PERIPH_GPIOG             ((uint32_t)0x00000100)
#define LL_APB2_GRP1_PERIPH_ADC1              ((uint32_t)0x00000200)
#define LL_APB2_GRP1_PERIPH_ADC2              ((uint32_t)0x00000400)
#define LL_APB2_GRP1_PERIPH_TIM1              ((uint32_t)0x00000800)
#define LL_APB2_GRP1_PERIPH_SPI1              ((uint32_t)0x00001000)
#define LL_APB2_GRP1_PERIPH_TIM8              ((uint32_t)0x00002000)
#define LL_APB2_GRP1_PERIPH_USART1            ((uint32_t)0x00004000)
#define LL_APB2_GRP1_PERIPH_ADC3              ((uint32_t)0x00008000)
#define LL_APB2_GRP1_PERIPH_TIM15             ((uint32_t)0x00010000)
#define LL_APB2_GRP1_PERIPH_TIM16             ((uint32_t)0x00020000)
#define LL_APB2_GRP1_PERIPH_TIM17             ((uint32_t)0x00040000)
#define LL_APB2_GRP1_PERIPH_TIM9              ((uint32_t)0x00080000)
#define LL_APB2_GRP1_PERIPH_TIM10             ((uint32_t)0x00100000)
#define LL_APB2_GRP1_PERIPH_TIM11             ((uint32_t)0x00200000)

#define IS_RCC_APB2_PERIPH(PERIPH) ((((PERIPH) & 0xFFC00002) == 0x00) && ((PERIPH) != 0x00))
/**
  * @}
  */ 

/** @defgroup APB1_peripheral 
  * @{
  */

#define LL_APB1_GRP1_PERIPH_TIM2              ((uint32_t)0x00000001)
#define LL_APB1_GRP1_PERIPH_TIM3              ((uint32_t)0x00000002)
#define LL_APB1_GRP1_PERIPH_TIM4              ((uint32_t)0x00000004)
#define LL_APB1_GRP1_PERIPH_TIM5              ((uint32_t)0x00000008)
#define LL_APB1_GRP1_PERIPH_TIM6              ((uint32_t)0x00000010)
#define LL_APB1_GRP1_PERIPH_TIM7              ((uint32_t)0x00000020)
#define LL_APB1_GRP1_PERIPH_TIM12             ((uint32_t)0x00000040)
#define LL_APB1_GRP1_PERIPH_TIM13             ((uint32_t)0x00000080)
#define LL_APB1_GRP1_PERIPH_TIM14             ((uint32_t)0x00000100)
#define LL_APB1_GRP1_PERIPH_WWDG              ((uint32_t)0x00000800)
#define LL_APB1_GRP1_PERIPH_SPI2              ((uint32_t)0x00004000)
#define LL_APB1_GRP1_PERIPH_SPI3              ((uint32_t)0x00008000)
#define LL_APB1_GRP1_PERIPH_USART2            ((uint32_t)0x00020000)
#define LL_APB1_GRP1_PERIPH_USART3            ((uint32_t)0x00040000)
#define LL_APB1_GRP1_PERIPH_UART4             ((uint32_t)0x00080000)
#define LL_APB1_GRP1_PERIPH_UART5             ((uint32_t)0x00100000)
#define LL_APB1_GRP1_PERIPH_I2C1              ((uint32_t)0x00200000)
#define LL_APB1_GRP1_PERIPH_I2C2              ((uint32_t)0x00400000)
#define LL_APB1_GRP1_PERIPH_USB               ((uint32_t)0x00800000)
#define LL_APB1_GRP1_PERIPH_CAN1              ((uint32_t)0x02000000)
#define LL_APB1_GRP1_PERIPH_CAN2              ((uint32_t)0x04000000)
#define LL_APB1_GRP1_PERIPH_BKP               ((uint32_t)0x08000000)
#define LL_APB1_GRP1_PERIPH_PWR               ((uint32_t)0x10000000)
#define LL_APB1_GRP1_PERIPH_DAC1               ((uint32_t)0x20000000)
#define LL_APB1_GRP1_PERIPH_CEC               ((uint32_t)0x40000000)
 
#define IS_RCC_APB1_PERIPH(PERIPH) ((((PERIPH) & 0x81013600) == 0x00) && ((PERIPH) != 0x00))

/**
  * @}
  */

/** @defgroup Clock_source_to_output_on_MCO_pin 
  * @{
  */

#define LL_RCC_MCO1SOURCE_NOCLOCK                  ((uint8_t)0x00)
#define LL_RCC_MCO1SOURCE_SYSCLK                   ((uint8_t)0x04)
#define LL_RCC_MCO1SOURCE_HSI                      ((uint8_t)0x05)
#define LL_RCC_MCO1SOURCE_HSE                      ((uint8_t)0x06)
#define LL_RCC_MCO1SOURCE_PLLCLK_DIV_2              ((uint8_t)0x07)

#ifndef STM32F10X_CL
 #define IS_RCC_MCO(MCO) (((MCO) == LL_RCC_MCO1SOURCE_NOCLOCK) || ((MCO) == LL_RCC_MCO1SOURCE_HSI) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_SYSCLK)  || ((MCO) == LL_RCC_MCO1SOURCE_HSE) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_PLLCLK_DIV_2))
#else
 #define LL_RCC_MCO1SOURCE_PLL2CLK                 ((uint8_t)0x08)
 #define LL_RCC_MCO1SOURCE_PLLI2SCLK_DIV2            ((uint8_t)0x09)
 #define LL_RCC_MCO1SOURCE_EXT_HSE                     ((uint8_t)0x0A)
 #define LL_RCC_MCO1SOURCE_PLLI2SCLK                 ((uint8_t)0x0B)

 #define IS_RCC_MCO(MCO) (((MCO) == LL_RCC_MCO1SOURCE_NOCLOCK) || ((MCO) == LL_RCC_MCO1SOURCE_HSI) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_SYSCLK)  || ((MCO) == LL_RCC_MCO1SOURCE_HSE) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_PLLCLK_DIV_2) || ((MCO) == LL_RCC_MCO1SOURCE_PLL2CLK) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_PLLI2SCLK_DIV2) || ((MCO) == LL_RCC_MCO1SOURCE_EXT_HSE) || \
                          ((MCO) == LL_RCC_MCO1SOURCE_PLLI2SCLK))
#endif /* STM32F10X_CL */ 

/**
  * @}
  */

/** @defgroup RCC_Flag 
  * @{
  */

#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

#ifndef STM32F10X_CL
 #define IS_RCC_FLAG(FLAG) (((FLAG) == RCC_FLAG_HSIRDY) || ((FLAG) == RCC_FLAG_HSERDY) || \
                            ((FLAG) == RCC_FLAG_PLLRDY) || ((FLAG) == RCC_FLAG_LSERDY) || \
                            ((FLAG) == RCC_FLAG_LSIRDY) || ((FLAG) == RCC_FLAG_PINRST) || \
                            ((FLAG) == RCC_FLAG_PORRST) || ((FLAG) == RCC_FLAG_SFTRST) || \
                            ((FLAG) == RCC_FLAG_IWDGRST)|| ((FLAG) == RCC_FLAG_WWDGRST)|| \
                            ((FLAG) == RCC_FLAG_LPWRRST))
#else
 #define RCC_FLAG_PLL2RDY                ((uint8_t)0x3B) 
 #define RCC_FLAG_PLL3RDY                ((uint8_t)0x3D) 
 #define IS_RCC_FLAG(FLAG) (((FLAG) == RCC_FLAG_HSIRDY) || ((FLAG) == RCC_FLAG_HSERDY) || \
                            ((FLAG) == RCC_FLAG_PLLRDY) || ((FLAG) == RCC_FLAG_LSERDY) || \
                            ((FLAG) == RCC_FLAG_PLL2RDY) || ((FLAG) == RCC_FLAG_PLL3RDY) || \
                            ((FLAG) == RCC_FLAG_LSIRDY) || ((FLAG) == RCC_FLAG_PINRST) || \
                            ((FLAG) == RCC_FLAG_PORRST) || ((FLAG) == RCC_FLAG_SFTRST) || \
                            ((FLAG) == RCC_FLAG_IWDGRST)|| ((FLAG) == RCC_FLAG_WWDGRST)|| \
                            ((FLAG) == RCC_FLAG_LPWRRST))
#endif /* STM32F10X_CL */ 

#define IS_RCC_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x1F)
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup RCC_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup RCC_Exported_Functions
  * @{
  */

void LL_RCC_DeInit();
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp();
void LL_RCC_HSI_SetCalibTrimming(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void LL_RCC_PLL_ConfigDomain_SYS(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_CL)
 void MODIFY_REG(RCC->CFGR2, (RCC_CFGR_PLLSRC | RCC_CFGR2_PREDIV1), (uint32_t RCC_PREDIV1_Source|uint32_t RCC_PREDIV1_Div));
#endif

#ifdef  STM32F10X_CL
 void MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PREDIV2, uint32_t RCC_PREDIV2_Div);
 void MODIFY_REG(RCC->CFGR2,  RCC_CFGR2_PLL2MUL,  uint32_t RCC_PLL2Mul);
 void RCC_PLL2Cmd(FunctionalState NewState);
 void MODIFY_REG(RCC->CFGR2, RCC_CFGR2_PLL3MUL, uint32_t RCC_PLL3Mul);
 void RCC_PLL3Cmd(FunctionalState NewState);
#endif /* STM32F10X_CL */ 

void LL_RCC_SetSysClkSource(uint32_t RCC_SYSCLKSource);
uint8_t LL_RCC_GetSysClkSource();
void LL_RCC_SetAHBPrescaler(uint32_t RCC_SYSCLK);
void LL_RCC_SetAPB1Prescaler(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);

#ifndef STM32F10X_CL
 void LL_RCC_SetUSBClockSource(uint32_t RCC_USBCLKSource);
#else
 void LL_RCC_SetUSBClockSource(uint32_t RCC_OTGFSCLKSource);
#endif /* STM32F10X_CL */ 

void LL_RCC_SetADCClockSource(uint32_t RCC_PCLK2);

#ifdef STM32F10X_CL
 void LL_RCC_SetI2SClockSource(uint32_t RCC_I2S2CLKSource);                                  
 void LL_RCC_SetI2SClockSource(uint32_t RCC_I2S3CLKSource);
#endif /* STM32F10X_CL */ 

void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);

#ifdef STM32F10X_CL
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
#endif /* STM32F10X_CL */ 

void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void LL_RCC_ConfigMCO(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void LL_RCC_ClearResetFlags();
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_RCC_H */
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
