/**
  ******************************************************************************
  * @file    stm32f10x_usart.h
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file contains all the functions prototypes for the USART 
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
#ifndef __STM32F10x_USART_H
#define __STM32F10x_USART_H

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

/** @addtogroup USART
  * @{
  */ 

/** @defgroup USART_Exported_Types
  * @{
  */ 

/** 
  * @brief  USART Init Structure definition  
  */ 
  
typedef struct
{
  uint32_t USART_BaudRate;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

  uint16_t USART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint16_t USART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */

  uint16_t USART_Parity;              /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
 
  uint16_t USART_Mode;                /*!< Specifies wether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */

  uint16_t USART_HardwareFlowControl; /*!< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} LL_USART_InitTypeDef;

/** 
  * @brief  USART Clock Init Structure definition  
  */ 
  
typedef struct
{

  uint16_t USART_Clock;   /*!< Specifies whether the USART clock is enabled or disabled.
                               This parameter can be a value of @ref USART_Clock */

  uint16_t USART_CPOL;    /*!< Specifies the steady state value of the serial clock.
                               This parameter can be a value of @ref USART_Clock_Polarity */

  uint16_t USART_CPHA;    /*!< Specifies the clock transition on which the bit capture is made.
                               This parameter can be a value of @ref USART_Clock_Phase */

  uint16_t USART_LastBit; /*!< Specifies whether the clock pulse corresponding to the last transmitted
                               data bit (MSB) has to be output on the SCLK pin in synchronous mode.
                               This parameter can be a value of @ref USART_Last_Bit */
} LL_USART_ClockInitTypeDef;

/**
  * @}
  */ 

/** @defgroup USART_Exported_Constants
  * @{
  */ 
  
#define IS_USART_ALL_PERIPH(PERIPH) (((PERIPH) == USART1) || \
                                     ((PERIPH) == USART2) || \
                                     ((PERIPH) == USART3) || \
                                     ((PERIPH) == UART4) || \
                                     ((PERIPH) == UART5))

#define IS_USART_123_PERIPH(PERIPH) (((PERIPH) == USART1) || \
                                     ((PERIPH) == USART2) || \
                                     ((PERIPH) == USART3))

#define IS_USART_1234_PERIPH(PERIPH) (((PERIPH) == USART1) || \
                                      ((PERIPH) == USART2) || \
                                      ((PERIPH) == USART3) || \
                                      ((PERIPH) == UART4))
/** @defgroup USART_Word_Length 
  * @{
  */ 
  
#define LL_USART_DATAWIDTH_8B                  ((uint16_t)0x0000)
#define LL_USART_DATAWIDTH_9B                  ((uint16_t)0x1000)
                                    
#define IS_USART_WORD_LENGTH(LENGTH) (((LENGTH) == LL_USART_DATAWIDTH_8B) || \
                                      ((LENGTH) == LL_USART_DATAWIDTH_9B))
/**
  * @}
  */ 

/** @defgroup USART_Stop_Bits 
  * @{
  */ 
  
#define LL_USART_STOPBITS_1                     ((uint16_t)0x0000)
#define LL_USART_STOPBITS_0_5                   ((uint16_t)0x1000)
#define LL_USART_STOPBITS_2                     ((uint16_t)0x2000)
#define LL_USART_STOPBITS_1_5                   ((uint16_t)0x3000)
#define IS_USART_STOPBITS(STOPBITS) (((STOPBITS) == LL_USART_STOPBITS_1) || \
                                     ((STOPBITS) == LL_USART_STOPBITS_0_5) || \
                                     ((STOPBITS) == LL_USART_STOPBITS_2) || \
                                     ((STOPBITS) == LL_USART_STOPBITS_1_5))
/**
  * @}
  */ 

/** @defgroup USART_Parity 
  * @{
  */ 
  
#define LL_USART_PARITY_NONE                      ((uint16_t)0x0000)
#define LL_USART_PARITY_EVEN                    ((uint16_t)0x0400)
#define LL_USART_PARITY_ODD                     ((uint16_t)0x0600) 
#define IS_USART_PARITY(PARITY) (((PARITY) == LL_USART_PARITY_NONE) || \
                                 ((PARITY) == LL_USART_PARITY_EVEN) || \
                                 ((PARITY) == LL_USART_PARITY_ODD))
/**
  * @}
  */ 

/** @defgroup USART_Mode 
  * @{
  */ 
  
#define LL_USART_DIRECTION_RX                        ((uint16_t)0x0004)
#define LL_USART_DIRECTION_TX                        ((uint16_t)0x0008)
#define IS_USART_MODE(MODE) ((((MODE) & (uint16_t)0xFFF3) == 0x00) && ((MODE) != (uint16_t)0x00))
/**
  * @}
  */ 

/** @defgroup USART_Hardware_Flow_Control 
  * @{
  */ 
#define LL_USART_HWCONTROL_NONE       ((uint16_t)0x0000)
#define LL_USART_HWCONTROL_RTS        ((uint16_t)0x0100)
#define LL_USART_HWCONTROL_CTS        ((uint16_t)0x0200)
#define LL_USART_HWCONTROL_RTS_CTS    ((uint16_t)0x0300)
#define IS_USART_HARDWARE_FLOW_CONTROL(CONTROL)\
                              (((CONTROL) == LL_USART_HWCONTROL_NONE) || \
                               ((CONTROL) == LL_USART_HWCONTROL_RTS) || \
                               ((CONTROL) == LL_USART_HWCONTROL_CTS) || \
                               ((CONTROL) == LL_USART_HWCONTROL_RTS_CTS))
/**
  * @}
  */ 

/** @defgroup USART_Clock 
  * @{
  */ 
#define LL_USART_CLOCK_DISABLE                  ((uint16_t)0x0000)
#define LL_USART_CLOCK_ENABLE                   ((uint16_t)0x0800)
#define IS_USART_CLOCK(CLOCK) (((CLOCK) == LL_USART_CLOCK_DISABLE) || \
                               ((CLOCK) == LL_USART_CLOCK_ENABLE))
/**
  * @}
  */ 

/** @defgroup USART_Clock_Polarity 
  * @{
  */
  
#define LL_USART_POLARITY_LOW                       ((uint16_t)0x0000)
#define LL_USART_POLARITY_HIGH                      ((uint16_t)0x0400)
#define IS_USART_CPOL(CPOL) (((CPOL) == LL_USART_POLARITY_LOW) || ((CPOL) == LL_USART_POLARITY_HIGH))

/**
  * @}
  */ 

/** @defgroup USART_Clock_Phase
  * @{
  */

#define LL_USART_PHASE_1EDGE                     ((uint16_t)0x0000)
#define LL_USART_PHASE_2EDGE                     ((uint16_t)0x0200)
#define IS_USART_CPHA(CPHA) (((CPHA) == LL_USART_PHASE_1EDGE) || ((CPHA) == LL_USART_PHASE_2EDGE))

/**
  * @}
  */

/** @defgroup USART_Last_Bit
  * @{
  */

#define LL_USART_LASTCLKPULSE_NO_OUTPUT                ((uint16_t)0x0000)
#define LL_USART_LASTCLKPULSE_OUTPUT                 ((uint16_t)0x0100)
#define IS_USART_LASTBIT(LASTBIT) (((LASTBIT) == LL_USART_LASTCLKPULSE_NO_OUTPUT) || \
                                   ((LASTBIT) == LL_USART_LASTCLKPULSE_OUTPUT))
/**
  * @}
  */ 

/** @defgroup USART_Interrupt_definition 
  * @{
  */
  
#define USART_IT_PE                          ((uint16_t)0x0028)
#define USART_IT_TXE                         ((uint16_t)0x0727)
#define USART_IT_TC                          ((uint16_t)0x0626)
#define USART_IT_RXNE                        ((uint16_t)0x0525)
#define USART_IT_IDLE                        ((uint16_t)0x0424)
#define USART_IT_LBD                         ((uint16_t)0x0846)
#define USART_IT_CTS                         ((uint16_t)0x096A)
#define USART_IT_ERR                         ((uint16_t)0x0060)
#define USART_IT_ORE                         ((uint16_t)0x0360)
#define USART_IT_NE                          ((uint16_t)0x0260)
#define USART_IT_FE                          ((uint16_t)0x0160)
#define IS_USART_CONFIG_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                               ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                               ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                               ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ERR))
#define IS_USART_GET_IT(IT) (((IT) == USART_IT_PE) || ((IT) == USART_IT_TXE) || \
                            ((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                            ((IT) == USART_IT_IDLE) || ((IT) == USART_IT_LBD) || \
                            ((IT) == USART_IT_CTS) || ((IT) == USART_IT_ORE) || \
                            ((IT) == USART_IT_NE) || ((IT) == USART_IT_FE))
#define IS_USART_CLEAR_IT(IT) (((IT) == USART_IT_TC) || ((IT) == USART_IT_RXNE) || \
                               ((IT) == USART_IT_LBD) || ((IT) == USART_IT_CTS))
/**
  * @}
  */

/** @defgroup USART_DMA_Requests 
  * @{
  */

#define USART_DMAReq_Tx                      ((uint16_t)0x0080)
#define USART_DMAReq_Rx                      ((uint16_t)0x0040)
#define IS_USART_DMAREQ(DMAREQ) ((((DMAREQ) & (uint16_t)0xFF3F) == 0x00) && ((DMAREQ) != (uint16_t)0x00))

/**
  * @}
  */ 

/** @defgroup USART_WakeUp_methods
  * @{
  */

#define LL_USART_WAKEUP_IDLELINE                ((uint16_t)0x0000)
#define LL_USART_WAKEUP_ADDRESSMARK             ((uint16_t)0x0800)
#define IS_USART_WAKEUP(WAKEUP) (((WAKEUP) == LL_USART_WAKEUP_IDLELINE) || \
                                 ((WAKEUP) == LL_USART_WAKEUP_ADDRESSMARK))
/**
  * @}
  */

/** @defgroup USART_LIN_Break_Detection_Length 
  * @{
  */
  
#define LL_USART_LINBREAK_DETECT_10B      ((uint16_t)0x0000)
#define LL_USART_LINBREAK_DETECT_11B      ((uint16_t)0x0020)
#define IS_USART_LIN_BREAK_DETECT_LENGTH(LENGTH) \
                               (((LENGTH) == LL_USART_LINBREAK_DETECT_10B) || \
                                ((LENGTH) == LL_USART_LINBREAK_DETECT_11B))
/**
  * @}
  */

/** @defgroup USART_IrDA_Low_Power 
  * @{
  */

#define LL_USART_IRDA_POWER_LOW              ((uint16_t)0x0004)
#define LL_USART_IRDA_POWER_NORMAL                ((uint16_t)0x0000)
#define IS_USART_IRDA_MODE(MODE) (((MODE) == LL_USART_IRDA_POWER_LOW) || \
                                  ((MODE) == LL_USART_IRDA_POWER_NORMAL))
/**
  * @}
  */ 

/** @defgroup USART_Flags 
  * @{
  */

#define LL_USART_ISR_CTS                       ((uint16_t)0x0200)
#define LL_USART_ISR_LBDF                       ((uint16_t)0x0100)
#define LL_USART_ISR_TXE                       ((uint16_t)0x0080)
#define LL_USART_ISR_TC                        ((uint16_t)0x0040)
#define LL_USART_ISR_RXNE                      ((uint16_t)0x0020)
#define LL_USART_ISR_IDLE                      ((uint16_t)0x0010)
#define LL_USART_ISR_ORE                       ((uint16_t)0x0008)
#define LL_USART_ISR_NE                        ((uint16_t)0x0004)
#define LL_USART_ISR_FE                        ((uint16_t)0x0002)
#define LL_USART_ISR_PE                        ((uint16_t)0x0001)
#define IS_USART_FLAG(FLAG) (((FLAG) == LL_USART_ISR_PE) || ((FLAG) == LL_USART_ISR_TXE) || \
                             ((FLAG) == LL_USART_ISR_TC) || ((FLAG) == LL_USART_ISR_RXNE) || \
                             ((FLAG) == LL_USART_ISR_IDLE) || ((FLAG) == LL_USART_ISR_LBDF) || \
                             ((FLAG) == LL_USART_ISR_CTS) || ((FLAG) == LL_USART_ISR_ORE) || \
                             ((FLAG) == LL_USART_ISR_NE) || ((FLAG) == LL_USART_ISR_FE))
                              
#define IS_USART_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0xFC9F) == 0x00) && ((FLAG) != (uint16_t)0x00))
#define IS_USART_PERIPH_FLAG(PERIPH, USART_FLAG) ((((*(uint32_t*)&(PERIPH)) != UART4_BASE) &&\
                                                  ((*(uint32_t*)&(PERIPH)) != UART5_BASE)) \
                                                  || ((USART_FLAG) != LL_USART_ISR_CTS)) 
#define IS_USART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 0x0044AA21))
#define IS_USART_ADDRESS(ADDRESS) ((ADDRESS) <= 0xF)
#define IS_USART_DATA(DATA) ((DATA) <= 0x1FF)

/**
  * @}
  */ 

/**
  * @}
  */ 

/** @defgroup USART_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USART_Exported_Functions
  * @{
  */

void LL_USART_DeInit(USART_TypeDef* USARTx);
USART_InitTypeDef* USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
void LL_USART_Init(USART_TypeDef* USARTx, LL_USART_InitTypeDef* USART_InitStruct);
void LL_USART_StructInit(LL_USART_InitTypeDef* USART_InitStruct);
void LL_USART_ClockInit(USART_TypeDef* USARTx, LL_USART_ClockInitTypeDef* USART_ClockInitStruct);
void LL_USART_ClockStructInit(LL_USART_ClockInitTypeDef* USART_ClockInitStruct);
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState);
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);
void LL_USART_SetNodeAddress(USART_TypeDef* USARTx, uint8_t USART_Address);
void LL_USART_SetWakeUpMethod(USART_TypeDef* USARTx, uint16_t USART_WakeUp);
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void LL_USART_SetLINBrkDetectionLen(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength);
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void LL_USART_TransmitData9(USART_TypeDef* USARTx, uint16_t Data);
uint16_t LL_USART_ReceiveData9(USART_TypeDef* USARTx);
void LL_USART_RequestBreakSending(USART_TypeDef* USARTx);
void LL_USART_SetSmartcardGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime);
void LL_USART_SetSmartcardPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler);
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState);
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState);
void LL_USART_SetIrdaPowerMode(USART_TypeDef* USARTx, uint16_t USART_IrDAMode);
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState);
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG);
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG);
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT);
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_USART_H */
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
