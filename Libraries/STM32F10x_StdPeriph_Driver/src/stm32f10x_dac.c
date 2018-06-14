/**
  ******************************************************************************
  * @file    stm32f10x_dac.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   This file provides all the DAC firmware functions.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_dac.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup DAC 
  * @brief DAC driver modules
  * @{
  */ 

/** @defgroup DAC_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup DAC_Private_Defines
  * @{
  */

/* DAC EN mask */
#define CR_EN_Set                  ((uint32_t)0x00000001)

/* DAC DMAEN mask */
#define CR_DMAEN_Set               ((uint32_t)0x00001000)

/* CR register Mask */
#define CR_CLEAR_Mask              ((uint32_t)0x00000FFE)

/* DAC SWTRIG mask */
#define SWTRIGR_SWTRIG_Set         ((uint32_t)0x00000001)

/* DAC Dual Channels SWTRIG masks */
#define DUAL_SWTRIG_Set            ((uint32_t)0x00000003)
#define DUAL_SWTRIG_Reset          ((uint32_t)0xFFFFFFFC)

/* DHR registers offsets */
#define DHR12R1_Offset             ((uint32_t)0x00000008)
#define DHR12R2_Offset             ((uint32_t)0x00000014)
#define DHR12RD_Offset             ((uint32_t)0x00000020)

/* DOR register offset */
#define DOR_Offset                 ((uint32_t)0x0000002C)
/**
  * @}
  */

/** @defgroup DAC_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup DAC_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup DAC_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup DAC_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the DAC peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void LL_DAC_DeInit(DAC1)
{
  /* Enable DAC reset state */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_DAC1);
  /* Release DAC from reset state */
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_DAC1);
}

/**
  * @brief  Initializes the DAC peripheral according to the specified 
  *   parameters in the DAC_InitStruct.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure that
  *   contains the configuration information for the specified DAC channel.
  * @retval None
  */
if(DAC_InitTypeDef* DAC_InitStruct.TriggerSource != 0)
{
LL_DAC_EnableTrigger(DAC1,);
}
void LL_DAC_Init(DAC1, uint32_t DAC_Channel, LL_DAC_InitTypeDef* DAC_InitStruct)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;
  /* Check the DAC parameters */
  assert_param(IS_DAC_TRIGGER(DAC_InitStruct->TriggerSource));
  assert_param(IS_DAC_GENERATE_WAVE(DAC_InitStruct->WaveAutoGeneration));
  assert_param(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(DAC_InitStruct->WaveAutoGenerationConfig));
  assert_param(IS_DAC_OUTPUT_BUFFER_STATE(DAC_InitStruct->OutputBuffer));
/*---------------------------- DAC CR Configuration --------------------------*/
  /* Get the DAC CR value */
  tmpreg1 = DAC->CR;
  /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
  tmpreg1 &= ~(CR_CLEAR_Mask << DAC_Channel);
  /* Configure for the selected DAC channel: buffer output, trigger, wave genration,
     mask/amplitude for wave genration */
  /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set WAVEx bits according to DAC_WaveGeneration value */
  /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
  /* Set BOFFx bit according to DAC_OutputBuffer value */   
  tmpreg2 = (DAC_InitStruct->TriggerSource | DAC_InitStruct->WaveAutoGeneration |
             DAC_InitStruct->WaveAutoGenerationConfig | DAC_InitStruct->OutputBuffer);
  /* Calculate CR register value depending on DAC_Channel */
  tmpreg1 |= tmpreg2 << DAC_Channel;
  /* Write to DAC CR */
  DAC->CR = tmpreg1;
}

/**
  * @brief  Fills each DAC_InitStruct member with its default value.
  * @param  DAC_InitStruct : pointer to a DAC_InitTypeDef structure which will
  *   be initialized.
  * @retval None
  */
void LL_DAC_StructInit(LL_DAC_InitTypeDef* DAC_InitStruct)
{
/*--------------- Reset DAC init structure parameters values -----------------*/
  /* Initialize the DAC_Trigger member */
  DAC_InitStruct->TriggerSource = 0x00000000U;
  /* Initialize the DAC_WaveGeneration member */
  DAC_InitStruct->WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  /* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
  DAC_InitStruct->WaveAutoGenerationConfig = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  /* Initialize the DAC_OutputBuffer member */
  DAC_InitStruct->OutputBuffer = LL_DAC_OUTPUT_BUFFER_ENABLE;
}

/**
  * @brief  Enables or disables the specified DAC channel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the DAC channel. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel */
    DAC->CR |= CR_EN_Set << DAC_Channel;
  }
  else
  {
    /* Disable the selected DAC channel */
    DAC->CR &= ~(CR_EN_Set << DAC_Channel);
  }
}
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
/**
  * @brief  Enables or disables the specified DAC interrupts.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt sources to be enabled or disabled. 
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                      
  * @param  NewState: new state of the specified DAC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */ 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState)  
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_DAC_IT(DAC_IT)); 

  if (NewState != DISABLE)
  {
    /* Enable the selected DAC interrupts */
    DAC->CR |=  (DAC_IT << DAC_Channel);
  }
  else
  {
    /* Disable the selected DAC interrupts */
    DAC->CR &= (~(uint32_t)(DAC_IT << DAC_Channel));
  }
}
#endif

/**
  * @brief  Enables or disables the specified DAC channel DMA request.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel DMA request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel DMA request */
    DAC->CR |= CR_DMAEN_Set << DAC_Channel;
  }
  else
  {
    /* Disable the selected DAC channel DMA request */
    DAC->CR &= ~(CR_DMAEN_Set << DAC_Channel);
  }
}

/**
  * @brief  Enables or disables the selected DAC channel software trigger.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel software trigger.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable software trigger for the selected DAC channel */
    DAC->SWTRIGR |= SWTRIGR_SWTRIG_Set << (DAC_Channel >> 4);
  }
  else
  {
    /* Disable software trigger for the selected DAC channel */
    DAC->SWTRIGR &= ~(SWTRIGR_SWTRIG_Set << (DAC_Channel >> 4));
  }
}

/**
  * @brief  Enables or disables simultaneously the two DAC channels software
  *   triggers.
  * @param  NewState: new state of the DAC channels software triggers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable software trigger for both DAC channels */
    DAC->SWTRIGR |= DUAL_SWTRIG_Set ;
  }
  else
  {
    /* Disable software trigger for both DAC channels */
    DAC->SWTRIGR &= DUAL_SWTRIG_Reset;
  }
}

/**
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_Wave: Specifies the wave type to enable or disable.
  *   This parameter can be one of the following values:
  *     @arg DAC_Wave_Noise: noise wave generation
  *     @arg DAC_Wave_Triangle: triangle wave generation
  * @param  NewState: new state of the selected DAC channel wave generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_WAVE(DAC_Wave)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected wave generation for the selected DAC channel */
    DAC->CR |= DAC_Wave << DAC_Channel;
  }
  else
  {
    /* Disable the selected wave generation for the selected DAC channel */
    DAC->CR &= ~(DAC_Wave << DAC_Channel);
  }
}

/**
  * @brief  Set the specified data holding register value for DAC channel1.
  * @param  DAC_Align: Specifies the data alignement for DAC channel1.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignement selected
  *     @arg DAC_Align_12b_L: 12bit left data alignement selected
  *     @arg DAC_Align_12b_R: 12bit right data alignement selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
{  
  __IO uint32_t tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));
  
  tmp = (uint32_t)DAC_BASE; 
  tmp += DHR12R1_Offset + DAC_Align;

  /* Set the DAC channel1 selected data holding register */
  *(__IO uint32_t *) tmp = Data;
}

/**
  * @brief  Set the specified data holding register value for DAC channel2.
  * @param  DAC_Align: Specifies the data alignement for DAC channel2.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignement selected
  *     @arg DAC_Align_12b_L: 12bit left data alignement selected
  *     @arg DAC_Align_12b_R: 12bit right data alignement selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data)
{
  __IO uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));
  
  tmp = (uint32_t)DAC_BASE;
  tmp += DHR12R2_Offset + DAC_Align;

  /* Set the DAC channel2 selected data holding register */
  *(__IO uint32_t *)tmp = Data;
}

/**
  * @brief  Set the specified data holding register value for dual channel
  *   DAC.
  * @param  DAC_Align: Specifies the data alignement for dual channel DAC.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignement selected
  *     @arg DAC_Align_12b_L: 12bit left data alignement selected
  *     @arg DAC_Align_12b_R: 12bit right data alignement selected
  * @param  Data2: Data for DAC Channel2 to be loaded in the selected data 
  *   holding register.
  * @param  Data1: Data for DAC Channel1 to be loaded in the selected data 
  *   holding register.
  * @retval None
  */
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1)
{
  uint32_t data = 0, tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data1));
  assert_param(IS_DAC_DATA(Data2));
  
  /* Calculate and set dual DAC data holding register value */
  if (DAC_Align == DAC_Align_8b_R)
  {
    data = ((uint32_t)Data2 << 8) | Data1; 
  }
  else
  {
    data = ((uint32_t)Data2 << 16) | Data1;
  }
  
  tmp = (uint32_t)DAC_BASE;
  tmp += DHR12RD_Offset + DAC_Align;

  /* Set the dual DAC selected data holding register */
  *(__IO uint32_t *)tmp = data;
}

/**
  * @brief  Returns the last data output value of the selected DAC cahnnel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
uint16_t LL_DAC_RetrieveOutputData(DAC1, uint32_t DAC_Channel)
{
  __IO uint32_t tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  
  tmp = (uint32_t) DAC_BASE ;
  tmp += DOR_Offset + ((uint32_t)DAC_Channel >> 2);
  
  /* Returns the DAC channel data output register value */
  return (uint16_t) (*(__IO uint32_t*) tmp);
}

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL)
/**
  * @brief  Checks whether the specified DAC flag is set or not.
  * @param  DAC_Channel: thee selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to check. 
  *   This parameter can be only of the following value:
  *     @arg DAC_FLAG_DMAUDR: DMA underrun flag                                                 
  * @retval The new state of DAC_FLAG (SET or RESET).
  */
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_FLAG(DAC_FLAG));

  /* Check the status of the specified DAC flag */
  if ((DAC->SR & (DAC_FLAG << DAC_Channel)) != (uint8_t)RESET)
  {
    /* DAC_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* DAC_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the DAC_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the DAC channelx's pending flags.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to clear. 
  *   This parameter can be of the following value:
  *     @arg DAC_FLAG_DMAUDR: DMA underrun flag                           
  * @retval None
  */
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_FLAG(DAC_FLAG));

  /* Clear the selected DAC flags */
  DAC->SR = (DAC_FLAG << DAC_Channel);
}

/**
  * @brief  Checks whether the specified DAC interrupt has occurred or not.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt source to check. 
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                       
  * @retval The new state of DAC_IT (SET or RESET).
  */
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_IT(DAC_IT));

  /* Get the DAC_IT enable bit status */
  enablestatus = (DAC->CR & (DAC_IT << DAC_Channel)) ;
  
  /* Check the status of the specified DAC interrupt */
  if (((DAC->SR & (DAC_IT << DAC_Channel)) != (uint32_t)RESET) && enablestatus)
  {
    /* DAC_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* DAC_IT is reset */
    bitstatus = RESET;
  }
  /* Return the DAC_IT status */
  return  bitstatus;
}

/**
  * @brief  Clears the DAC channelx�s interrupt pending bits.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt pending bit to clear.
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                         
  * @retval None
  */
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_IT(DAC_IT)); 

  /* Clear the selected DAC interrupt pending bits */
  DAC->SR = (DAC_IT << DAC_Channel);
}
#endif

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