/**
  ******************************************************************************
  * @file    legacy.c
  * @author  MCD Application Team
  * @brief   StdPeriph legacy body
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

/* Includes ------------------------------------------------------------------*/
#include "legacy.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* -------------------------------------------------------------------------- */
/*                                MISC APIs                                    */
/* -------------------------------------------------------------------------- */

#if defined(STM32F0)
/**
  * @brief  Initializes the NVIC peripheral according to the specified
  *         parameters in the NVIC_InitStruct.
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
  uint32_t tmppriority = 0x00;

  if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
  {
    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02];
    tmppriority &= (uint32_t)(~(((uint32_t)0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8)));
    tmppriority |= (uint32_t)((((uint32_t)NVIC_InitStruct->NVIC_IRQChannelPriority << 6) & 0xFF) << ((NVIC_InitStruct->NVIC_IRQChannel & 0x03) * 8));    

    NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel >> 0x02] = tmppriority;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
  else
  {
    /* Disable the Selected IRQ Channels -------------------------------------*/
    NVIC->ICER[0] = (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
}
#else
/**
  * @brief  Initializes the NVIC peripheral according to the specified
  *         parameters in the NVIC_InitStruct.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before. 
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
  uint8_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;

  if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
  {
    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  (uint8_t)(NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub);

    tmppriority = tmppriority << 0x04;

    NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel] = tmppriority;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
    (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
  else
  {
    /* Disable the Selected IRQ Channels -------------------------------------*/
    NVIC->ICER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
    (uint32_t)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (uint8_t)0x1F);
  }
}
#endif /* STM32F0 */

/**
  * @brief  Selects the condition for the system to enter low power mode.
  * @param  LowPowerMode: Specifies the new mode for the system to enter low power mode.
  *   This parameter can be one of the following values:
  *     @arg NVIC_LP_SEVONPEND: Low Power SEV on Pend.
  *     @arg NVIC_LP_SLEEPDEEP: Low Power DEEPSLEEP request.
  *     @arg NVIC_LP_SLEEPONEXIT: Low Power Sleep on Exit.
  * @param  NewState: new state of LP condition. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    SCB->SCR |= LowPowerMode;
  }
  else
  {
    SCB->SCR &= (uint32_t)(~(uint32_t)LowPowerMode);
  }
}

#if !defined(STM32F0)
/**
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *   This parameter can be one of the following values:
  *     @arg NVIC_VectTab_RAM: Vector Table in internal SRAM.
  *     @arg NVIC_VectTab_FLASH: Vector Table in internal FLASH.
  * @param  Offset: Vector Table base offset field. This value must be a multiple of 0x200.
  * @retval None
  */
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{
  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}
#endif /* !STM32F0 */

/* -------------------------------------------------------------------------- */
/*                                RTC APIs                                     */
/* -------------------------------------------------------------------------- */

#if defined(RTC_ISR_SHPF)
/**
  * @brief  Configures the Synchronization Shift Control Settings.
  * @note   When REFCKON is set, firmware must not write to Shift control register
  * @param  RTC_ShiftAdd1S : Select to add or not 1 second to the time Calendar.
  *   This parameter can be one of the following values :
  *     @arg RTC_ShiftAdd1S_Set  : Add one second to the clock calendar.
  *     @arg RTC_ShiftAdd1S_Reset: No effect.
  * @param  RTC_ShiftSubFS: Select the number of Second Fractions to Substitute.
  *         This parameter can be one any value from 0 to 0x7FFF.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Shift registers are configured
  *          - ERROR: RTC Shift registers are not configured
  */
ErrorStatus RTC_SynchroShiftConfig(uint32_t RTC_ShiftAdd1S, uint32_t RTC_ShiftSubFS)
{
  ErrorStatus status = ERROR;
  uint32_t shpfcount = 0;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* Check if a Shift is pending */
  if ((RTC->ISR & RTC_ISR_SHPF) != RESET)
  {
    /* Wait until the shift is completed */
    while ((LL_RTC_IsActiveFlag_SHP(RTC) != RESET) && (shpfcount != SHPF_TIMEOUT))
    {
      shpfcount++;
    }
  }

  /* Check if the Shift pending is completed or if there is no Shift operation at all */
  if ((RTC->ISR & RTC_ISR_SHPF) == RESET)
  {
    /* check if the reference clock detection is disabled */
    if((RTC->CR & RTC_CR_REFCKON) == RESET)
    {
      /* Configure the Shift settings */
      LL_RTC_TIME_Synchronize(RTC, RTC_ShiftAdd1S, RTC_ShiftSubFS);

      if(LL_RTC_WaitForSynchro(RTC) == ERROR)
      {
        status = ERROR;
      }
      else
      {
        status = SUCCESS;
      }
    }
    else
    {
      status = ERROR;
    }
  }
  else
  {
    status = ERROR;
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return (ErrorStatus)(status);
}
#endif /* RTC_ISR_SHPF */

#if defined(RTC_ISR_RECALPF)
/**
  * @brief  Configures the Smooth Calibration Settings.
  * @param  RTC_SmoothCalibPeriod : Select the Smooth Calibration Period.
  *   This parameter can be can be one of the following values:
  *     @arg RTC_SmoothCalibPeriod_32sec : The smooth calibration period is 32s.
  *     @arg RTC_SmoothCalibPeriod_16sec : The smooth calibration period is 16s.
  *     @arg RTC_SmoothCalibPeriod_8sec  : The smooth calibration period is 8s.
  * @param  RTC_SmoothCalibPlusPulses : Select to Set or reset the CALP bit.
  *   This parameter can be one of the following values:
  *     @arg RTC_SmoothCalibPlusPulses_Set  : Add one RTCCLK pulse every 2**11 pulses.
  *     @arg RTC_SmoothCalibPlusPulses_Reset: No RTCCLK pulses are added.
  * @param  RTC_SmouthCalibMinusPulsesValue: Select the value of CALM[8:0] bits.
  *   This parameter can be one any value from 0 to 0x000001FF.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Calib registers are configured
  *          - ERROR: RTC Calib registers are not configured
*/
ErrorStatus RTC_SmoothCalibConfig(uint32_t RTC_SmoothCalibPeriod,
                                  uint32_t RTC_SmoothCalibPlusPulses,
                                  uint32_t RTC_SmouthCalibMinusPulsesValue)
{
  ErrorStatus status = ERROR;
  uint32_t recalpfcount = 0;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* check if a calibration is pending */
  if (LL_RTC_IsActiveFlag_RECALP(RTC) != RESET)
  {
    /* wait until the Calibration is completed */
    while ((LL_RTC_IsActiveFlag_RECALP(RTC) != RESET) && (recalpfcount != RECALPF_TIMEOUT))
    {
      recalpfcount++;
    }
  }

  /* check if the calibration pending is completed or if there is no calibration operation at all */
  if (LL_RTC_IsActiveFlag_RECALP(RTC) == RESET)
  {
    /* Configure the Smooth calibration settings */
    LL_RTC_CAL_SetPeriod(RTC, RTC_SmoothCalibPeriod);
    LL_RTC_CAL_SetPulse(RTC, RTC_SmoothCalibPlusPulses);
    LL_RTC_CAL_SetMinus(RTC, RTC_SmouthCalibMinusPulsesValue);

    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return (ErrorStatus)(status);
}
#endif /* RTC_ISR_RECALPF */

#if !defined(STM32F1)
/**
  * @brief  Enables or disables the RTC reference clock detection.
  * @param  NewState: new state of the RTC reference clock.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC reference clock detection is enabled
  *          - ERROR: RTC reference clock detection is disabled  
  */
ErrorStatus RTC_RefClockCmd(FunctionalState NewState)
{
  ErrorStatus status = ERROR;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* Set Initialization mode */
  if (LL_RTC_EnterInitMode(RTC) == ERROR)
  {
    status = ERROR;
  }
  else
  {
    if (NewState != DISABLE)
    {
      /* Enable the RTC reference clock detection */
      LL_RTC_EnableRefClock(RTC);
    }
    else
    {
      /* Disable the RTC reference clock detection */
      LL_RTC_DisableRefClock(RTC);
    }
    /* Exit Initialization mode */
    LL_RTC_ExitInitMode(RTC);

    status = SUCCESS;
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return status;
}

/**
  * @brief  Enables or disables the specified RTC Alarm.
  * @param  RTC_Alarm: specifies the alarm to be configured.
  *          This parameter can be any combination of the following values:
  *            @arg RTC_Alarm_A: to select Alarm A
  *            @arg RTC_Alarm_B: to select Alarm B
  * @param  NewState: new state of the specified alarm.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Alarm is enabled/disabled
  *          - ERROR: RTC Alarm is not enabled/disabled
  */
ErrorStatus RTC_AlarmCmd(uint32_t RTC_Alarm, FunctionalState NewState)
{
  __IO uint32_t alarmcounter = 0x00;
  uint32_t alarmstatus = 0x00;
  ErrorStatus status = ERROR;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* Configure the Alarm state */
  if (NewState != DISABLE)
  {
    RTC->CR |= (uint32_t)RTC_Alarm;

    status = SUCCESS;    
  }
  else
  {
    /* Disable the Alarm in RTC_CR register */
    RTC->CR &= (uint32_t)~RTC_Alarm;

    /* Wait till RTC ALRxWF flag is set and if Time out is reached exit */
    do
    {
      alarmstatus = RTC->ISR & (RTC_Alarm >> 8);
      alarmcounter++;
    } while((alarmcounter != INITMODE_TIMEOUT) && (alarmstatus == 0x00));
    
    if ((RTC->ISR & (RTC_Alarm >> 8)) == RESET)
    {
      status = ERROR;
    }
    else
    {
      status = SUCCESS;
    }
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return status;
}

#if !(defined(STM32F0) && !defined(RTC_WAKEUP_SUPPORT))
/**
  * @brief  Enables or Disables the RTC WakeUp timer.
  * @param  NewState: new state of the WakeUp timer.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
ErrorStatus RTC_WakeUpCmd(FunctionalState NewState)
{
  __IO uint32_t wutcounter = 0x00;
  uint32_t wutwfstatus = 0x00;
  ErrorStatus status = ERROR;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  if (NewState != DISABLE)
  {
    /* Enable the Wakeup Timer */
    LL_RTC_WAKEUP_Enable(RTC);
    status = SUCCESS;    
  }
  else
  {
    /* Disable the Wakeup Timer */
    LL_RTC_DisableWriteProtection(RTC);
    /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
    do
    {
      wutwfstatus = LL_RTC_IsActiveFlag_WUTW(RTC);
      wutcounter++;
    } while((wutcounter != INITMODE_TIMEOUT) && (wutwfstatus == 0x00));

    if (LL_RTC_IsActiveFlag_WUTW(RTC) == RESET)
    {
      status = ERROR;
    }
    else
    {
      status = SUCCESS;
    }
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return status;
}
#endif /* !(STM32F0 && !RTC_WAKEUP_SUPPORT) */
#endif /* !STM32F1 */

#if !(defined(STM32F0) || defined(STM32F1) || defined(STM32F30) || defined(STM32F37) || defined(STM32F7) || defined(STM32L0) || defined(STM32L4))
/**
  * @brief  Configures the Coarse calibration parameters.
  * @param  RTC_CalibSign: specifies the sign of the coarse calibration value.
  *          This parameter can be  one of the following values:
  *            @arg RTC_CalibSign_Positive: The value sign is positive 
  *            @arg RTC_CalibSign_Negative: The value sign is negative
  * @param  Value: value of coarse calibration expressed in ppm (coded on 5 bits).
  *
  * @note   This Calibration value should be between 0 and 63 when using negative
  *         sign with a 2-ppm step.
  *
  * @note   This Calibration value should be between 0 and 126 when using positive
  *         sign with a 4-ppm step.
  *
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Coarse calibration are initialized
  *          - ERROR: RTC Coarse calibration are not initialized
  */
ErrorStatus RTC_CoarseCalibConfig(uint32_t RTC_CalibSign, uint32_t Value)
{
  ErrorStatus status = ERROR;

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* Set Initialization mode */
  if (LL_RTC_EnterInitMode(RTC) == ERROR)
  {
    status = ERROR;
  }
  else
  {
    /* Set the coarse calibration value */
    LL_RTC_CAL_ConfigCoarseDigital(RTC, RTC_CalibSign, Value);
    /* Exit Initialization mode */
    LL_RTC_ExitInitMode(RTC);

    status = SUCCESS;
  } 

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);

  return status;
}
/**
  * @brief  Enables or disables the Coarse calibration process.
  * @param  NewState: new state of the Coarse calibration.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Coarse calibration are enabled/disabled
  *          - ERROR: RTC Coarse calibration are not enabled/disabled
  */
ErrorStatus RTC_CoarseCalibCmd(FunctionalState NewState)
{
  ErrorStatus status = ERROR;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);

  /* Set Initialization mode */
  if (LL_RTC_EnterInitMode(RTC) == ERROR)
  {
    status =  ERROR;
  }
  else
  {
    if (NewState != DISABLE)
    {
      /* Enable the Coarse Calibration */
      LL_RTC_CAL_EnableCoarseDigital(RTC);
    }
    else
    {
      /* Disable the Coarse Calibration */
      LL_RTC_DisableWriteProtection(RTC);
    }
    /* Exit Initialization mode */
    LL_RTC_ExitInitMode(RTC);

    status = SUCCESS;
  }

  /* Enable the write protection for RTC registers */
  LL_RTC_EnableWriteProtection(RTC);
  
  return status;
}
#endif /* !(STM32F0 || STM32F1 || STM32F30 || STM32F37 || STM32F7 || STM32L0 || STM32L4) */

/* -------------------------------------------------------------------------- */
/*                                ADC APIs                                    */
/* -------------------------------------------------------------------------- */

/* Private define ------------------------------------------------------------*/
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)
/* CR1 register Mask */
#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)

/* CR2 register Mask */
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)

/* ADC L Mask */
#define SQR1_L_RESET              ((uint32_t)0xFF0FFFFF)
#endif

#if defined(STM32F0)||defined(STM32L0)
/* ADC CFGR mask */
#define CFGR1_CLEAR_MASK           ((uint32_t)0xFFFFD203)
#endif

#if defined(STM32F1)
/* CR1 register Mask */
#define CR1_CLEAR_Mask            ((uint32_t)0xFFF0FEFF)

/* ADC L Mask */
#define SQR1_L_RESET              ((uint32_t)0xFF0FFFFF) 

/* SQR1 register Mask */
#define SQR1_CLEAR_Mask           ((uint32_t)0xFF0FFFFF)
#endif

/* CR2 register Mask */
#define CR2_CLEAR_Mask            ((uint32_t)0xFFF1F7FD)

#if defined(STM32F37)

/* CR2 register Mask */
#define ADC_CR2_CLEAR_MASK         ((uint32_t)0xFFF1F7FD)
#endif

#if defined(STM32F30)||defined(STM32L4)
/* CFGR register Mask */
#define CFGR_CLEAR_Mask             ((uint32_t)0xFDFFC007)

/* JSQR register Mask */
#define JSQR_CLEAR_Mask             ((uint32_t)0x00000000)

/* ADC ADON mask */
#define CCR_CLEAR_MASK              ((uint32_t)0xFFFC10E0)
#endif

/**
  * @brief  Fills each ADC_InitStruct member with its default value.
  * @note   This function is used to initialize the global features of the ADC (
  *         Resolution and Data Alignment), however, the rest of the configuration
  *         parameters are specific to the regular channels group (scan mode
  *         activation, continuous mode activation, External trigger source and
  *         edge, number of conversion in the regular channels group sequencer).
  * @param  ADC_InitStruct: pointer to an LEGACY_ADC_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void ADC_StructInit(LEGACY_ADC_InitTypeDef* ADC_InitStruct)
{
#if defined(STM32F1)
  /* Initialize the ADC_Mode member */
  ADC_InitStruct->ADC_Mode = ADC_Mode_Independent;
#endif /* STM32F1 */

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F37)
  /* initialize the ADC_ScanConvMode member */
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;
#endif /* STM32F1|STM32F2|STM32F4|STM32L1|STM32F37 */

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)||defined(STM32F30)||defined(STM32L4)||defined(STM32F37)
  /* Initialize the ADC_ContinuousConvMode member */
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;

  /* Initialize the ADC_DataAlign member */
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;
#endif /* STM32F1|STM32F2|STM32F4|STM32L1|STM32F0|STM32F30||STM32L4|STM32F37*/

#if defined(STM32F1)||defined(STM32F2)||defined(STM32F4)||defined(STM32F7)
  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
#endif /* STM32F1|STM32F2|STM32F4*/
#if defined(STM32L1)
  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
#endif /* STM32L1*/
#if defined(STM32F0)||defined(STM32L0)
  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
#endif /* STM32F0*/
#if defined(STM32F37)
  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T19_TRGO;
#endif /* STM32F37*/

#if defined(STM32F1)||defined(STM32F37)
  /* Initialize the ADC_NbrOfChannel member */
  ADC_InitStruct->ADC_NbrOfChannel = 1;
#endif /* STM32F1|STM32F37*/

#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)||defined(STM32F30)||defined(STM32L4)
  /* Initialize the ADC_Mode member */
  ADC_InitStruct->ADC_Resolution = ADC_Resolution_12b;
#endif /* STM32F2|STM32F4|STM32L1|STM32F0|STM32F30||STM32L4 */

#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)||defined(STM32F0)||defined(STM32L0)
  /* Initialize the ADC_ExternalTrigConvEdge member */
  ADC_InitStruct->ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
#endif /* STM32F2|STM32F4|STM32L1|STM32F0 */

#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)||defined(STM32L1)
  /* Initialize the ADC_NbrOfConversion member */
  ADC_InitStruct->ADC_NbrOfConversion = 1;
#endif /* STM32F2|STM32F4|STM32L1 */

#if defined(STM32F0)||defined(STM32L0)
  /* Initialize the ADC_ScanDirection member */
  ADC_InitStruct->ADC_ScanDirection = ADC_ScanDirection_Upward;

#endif /* STM32F0 */

#if defined(STM32F30)||defined(STM32L4)
  ADC_InitStruct->ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStruct->ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStruct->ADC_OverrunMode = DISABLE;
  ADC_InitStruct->ADC_AutoInjMode = DISABLE;
  ADC_InitStruct->ADC_NbrOfRegChannel = 1;
#endif /* STM32F30||STM32L4 */
}

#if defined(STM32F30)||defined(STM32L4)
/**
  * @brief  Fills each ADC_InjectedInitStruct member with its default value.
  * @param  ADC_InjectedInitStruct : pointer to an LEGACY_ADC_InjectedInitTypeDef structure which will be initialized.
  * @retval None
  */
void ADC_InjectedStructInit(LEGACY_ADC_InjectedInitTypeDef* ADC_InjectedInitStruct)
{
  ADC_InjectedInitStruct->ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
  ADC_InjectedInitStruct->ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_None;
  ADC_InjectedInitStruct->ADC_NbrOfInjecChannel = 1;
  ADC_InjectedInitStruct->ADC_InjecSequence1 = ADC_InjectedChannel_1;
  ADC_InjectedInitStruct->ADC_InjecSequence2 = ADC_InjectedChannel_1;
  ADC_InjectedInitStruct->ADC_InjecSequence3 = ADC_InjectedChannel_1;
  ADC_InjectedInitStruct->ADC_InjecSequence4 = ADC_InjectedChannel_1;
}

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InjectInitStruct: pointer to an ADC_InjecInitTypeDef structure that contains
  *         the configuration information for the specified ADC injected channel.
  * @retval None
  */
void ADC_InjectedInit(ADC_TypeDef* ADCx, LEGACY_ADC_InjectedInitTypeDef* ADC_InjectedInitStruct)
{
  uint32_t tmpreg1 = 0;
  
  /*---------------------------- ADCx JSQR Configuration -----------------*/
  /* Get the ADCx JSQR value */
  tmpreg1 = ADCx->JSQR;
  /* Clear L bits */
  tmpreg1 &= JSQR_CLEAR_Mask;
  /* Configure ADCx: Injected channel sequence length, external trigger, 
     external trigger edge and sequences
  */
  tmpreg1 = (uint32_t) ((ADC_InjectedInitStruct->ADC_NbrOfInjecChannel - (uint8_t)1) |
                         ADC_InjectedInitStruct->ADC_ExternalTrigInjecConvEvent |         
                         ADC_InjectedInitStruct->ADC_ExternalTrigInjecEventEdge |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence1) << 8) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence2) << 14) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence3) << 20) |
                         (uint32_t)((ADC_InjectedInitStruct->ADC_InjecSequence4) << 26));
  /* Write to ADCx SQR1 */
  ADCx->JSQR = tmpreg1;  
}
#endif /* STM32F30||STM32L4 */

/**
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @note   This function is used to configure the global features of the ADC (
  *         Resolution and Data Alignment), however, the rest of the configuration
  *         parameters are specific to the regular channels group (scan mode
  *         activation, continuous mode activation, External trigger source and
  *         edge, number of conversion in the regular channels group sequencer).
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InitStruct: pointer to an LEGACY_ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
void ADC_Init(ADC_TypeDef* ADCx, LEGACY_ADC_InitTypeDef* ADC_InitStruct)
{
#if defined(STM32F2)||defined(STM32F4)||defined(STM32F7)
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  
  /* Clear RES and SCAN bits */
  tmpreg1 &= CR1_CLEAR_MASK;
  
  /* Configure ADCx: scan conversion mode and resolution */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  /* Set RES bit according to ADC_Resolution value */ 
  tmpreg1 |= (uint32_t)(((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8) | \
                                   ADC_InitStruct->ADC_Resolution);
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;
  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  
  /* Clear CONT, ALIGN, EXTEN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_MASK;
  
  /* Configure ADCx: external trigger event and edge, data alignment and 
     continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTEN bits according to ADC_ExternalTrigConvEdge value */ 
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | \
                        ADC_InitStruct->ADC_ExternalTrigConv | 
                        ADC_InitStruct->ADC_ExternalTrigConvEdge | \
                        ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
                        
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  
  /* Clear L bits */
  tmpreg1 &= SQR1_L_RESET;
  
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfConversion value */
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);
  
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;

#endif /* STM32F2|STM32F4 */

#if defined(STM32F1)
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  /* Clear DUALMOD and SCAN bits */
  tmpreg1 &= CR1_CLEAR_Mask;
  /* Configure ADCx: Dual mode and scan conversion mode */
  /* Set DUALMOD bits according to ADC_Mode value */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_Mode | ((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8));
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;

  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  /* Clear CONT, ALIGN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_Mask;
  /* Configure ADCx: external trigger event and continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv |
            ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;

  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  /* Clear L bits */
  tmpreg1 &= SQR1_CLEAR_Mask;
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfChannel value */
  tmpreg2 |= (uint8_t) (ADC_InitStruct->ADC_NbrOfChannel - (uint8_t)1);
  tmpreg1 |= (uint32_t)tmpreg2 << 20;
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;

#endif /* STM32F1*/
  
#if defined(STM32F37) 
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  /* Clear SCAN bit */
  tmpreg1 &= (uint32_t)(~ADC_CR1_SCAN);
  /* Configure ADCx: scan conversion mode */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  tmpreg1 |= (uint32_t)((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8);
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;

  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  /* Clear CONT, ALIGN and EXTSEL bits */
  tmpreg1 &= ADC_CR2_CLEAR_MASK;
  /* Configure ADCx: external trigger event and continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv |
            ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;

  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  /* Clear L bits */
  tmpreg1 &= (uint32_t) (~ADC_SQR1_L);
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfChannel value */
  tmpreg2 |= (uint8_t) (ADC_InitStruct->ADC_NbrOfChannel - (uint8_t)1);
  tmpreg1 |= (uint32_t)tmpreg2 << 20;
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;
#endif
  

#if defined(STM32L1)
  uint32_t tmpreg1 = 0;
  uint8_t tmpreg2 = 0;
  
  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  /* Clear RES and SCAN bits */ 
  tmpreg1 &= CR1_CLEAR_MASK;
  /* Configure ADCx: scan conversion mode and resolution */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  /* Set RES bit according to ADC_Resolution value */ 
  tmpreg1 |= (uint32_t)(((uint32_t)ADC_InitStruct->ADC_ScanConvMode << 8) | ADC_InitStruct->ADC_Resolution);
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;
  
  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  /* Clear CONT, ALIGN, EXTEN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_MASK;
  /* Configure ADCx: external trigger event and edge, data alignment and continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTEN bits according to ADC_ExternalTrigConvEdge value */ 
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (uint32_t)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv | 
              ADC_InitStruct->ADC_ExternalTrigConvEdge | ((uint32_t)ADC_InitStruct->ADC_ContinuousConvMode << 1));
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;
  
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  /* Clear L bits */
  tmpreg1 &= SQR1_L_RESET;
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfConversion value */ 
  tmpreg2 |= (uint8_t)(ADC_InitStruct->ADC_NbrOfConversion - (uint8_t)1);
  tmpreg1 |= ((uint32_t)tmpreg2 << 20);
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;

#endif /* STM32L1 */

#if defined(STM32F0)||defined(STM32L0)
  uint32_t tmpreg = 0;

  /* Get the ADCx CFGR value */
  tmpreg = ADCx->CFGR1;

  /* Clear SCANDIR, RES[1:0], ALIGN, EXTSEL[2:0], EXTEN[1:0] and CONT bits */
  tmpreg &= CFGR1_CLEAR_MASK;

  /*---------------------------- ADCx CFGR Configuration ---------------------*/

  /* Set RES[1:0] bits according to ADC_Resolution value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  /* Set EXTEN[1:0] bits according to ADC_ExternalTrigConvEdge value */
  /* Set EXTSEL[2:0] bits according to ADC_ExternalTrigConv value */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set SCANDIR bit according to ADC_ScanDirection value */
 
  tmpreg  |= (uint32_t)(ADC_InitStruct->ADC_Resolution | ((uint32_t)(ADC_InitStruct->ADC_ContinuousConvMode) << 13) |
             ADC_InitStruct->ADC_ExternalTrigConvEdge | ADC_InitStruct->ADC_ExternalTrigConv |
             ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ScanDirection);

  /* Write to ADCx CFGR */
  ADCx->CFGR1 = tmpreg;
#endif /* STM32F0 */

#if defined(STM32F30)||defined(STM32L4)
  uint32_t tmpreg1 = 0;

  /*---------------------------- ADCx CFGR Configuration -----------------*/
  /* Get the ADCx CFGR value */
  tmpreg1 = ADCx->CFGR;
  /* Clear SCAN bit */
  tmpreg1 &= CFGR_CLEAR_Mask; 
  /* Configure ADCx: scan conversion mode */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  tmpreg1 |= (uint32_t)ADC_InitStruct->ADC_ContinuousConvMode | 
  ADC_InitStruct->ADC_Resolution|                 
  ADC_InitStruct->ADC_ExternalTrigConvEvent|         
  ADC_InitStruct->ADC_ExternalTrigEventEdge|     
  ADC_InitStruct->ADC_DataAlign|                 
  ADC_InitStruct->ADC_OverrunMode|        
  ADC_InitStruct->ADC_AutoInjMode;
  
  /* Write to ADCx CFGR */
  ADCx->CFGR = tmpreg1;
  
  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  /* Clear L bits */
  tmpreg1 &= ~(uint32_t)(ADC_SQR1_L);
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfRegChannel value */
  tmpreg1 |= (uint32_t) (ADC_InitStruct->ADC_NbrOfRegChannel - 1);
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1; 

#endif /*STM32F30||STM32L4*/
}

#if (defined(STM32F30)||defined(STM32L4)) && defined(ADC12_COMMON)
/**
  * @brief  Initializes the ADCs peripherals according to the specified parameters 
  *         in the ADC_CommonInitStruct.
  * @param  ADCx: where x can be 1 or 4 to select the ADC peripheral.
  * @param  ADC_CommonInitStruct: pointer to an LEGACY_ADC_CommonInitTypeDef structure 
  *         that contains the configuration information for  All ADCs peripherals.
  * @retval None
  */
void ADC_CommonInit(ADC_TypeDef* ADCx, LEGACY_ADC_CommonInitTypeDef* ADC_CommonInitStruct)
{
  uint32_t tmpreg1 = 0;

  if((ADCx == ADC1) || (ADCx == ADC2))
  {
    /* Get the ADC CCR value */
    tmpreg1 = ADC12_COMMON->CCR;
  
    /* Clear MULTI, DELAY, DMA and ADCPRE bits */
    tmpreg1 &= CCR_CLEAR_MASK;
  }
#if defined (ADC34_COMMON)
  else
  {
    /* Get the ADC CCR value */
    tmpreg1 = ADC34_COMMON->CCR;
  
    /* Clear MULTI, DELAY, DMA and ADCPRE bits */
    tmpreg1 &= CCR_CLEAR_MASK;
  }
#endif
  /*---------------------------- ADC CCR Configuration -----------------*/  
  /* Configure ADCx: Multi mode, Delay between two sampling time, ADC clock, DMA mode
     and DMA access mode for dual mode */
  /* Set MULTI bits according to ADC_Mode value */
  /* Set CKMODE bits according to ADC_Clock value */
  /* Set MDMA bits according to ADC_DMAAccessMode value */
  /* Set DMACFG bits according to ADC_DMAMode value */
  /* Set DELAY bits according to ADC_TwoSamplingDelay value */    
  tmpreg1 |= (uint32_t)(ADC_CommonInitStruct->ADC_Mode | 
                        ADC_CommonInitStruct->ADC_Clock | 
                        ADC_CommonInitStruct->ADC_DMAAccessMode | 
                        (uint32_t)(ADC_CommonInitStruct->ADC_DMAMode << 12) |
                        (uint32_t)((uint32_t)ADC_CommonInitStruct->ADC_TwoSamplingDelay << 8));

  if((ADCx == ADC1) || (ADCx == ADC2))
  {                        
    /* Write to ADC CCR */
    ADC12_COMMON->CCR = tmpreg1;
  }
#if defined (ADC34_COMMON)
  else
  {
    /* Write to ADC CCR */
    ADC34_COMMON->CCR = tmpreg1;
  }
#endif
}

/**
  * @brief  Fills each ADC_CommonInitStruct member with its default value.
  * @param  ADC_CommonInitStruct: pointer to an LEGACY_ADC_CommonInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
void ADC_CommonStructInit(LEGACY_ADC_CommonInitTypeDef* ADC_CommonInitStruct)
{
  /* Initialize the ADC_Mode member */
  ADC_CommonInitStruct->ADC_Mode = ADC_Mode_Independent;

  /* initialize the ADC_Clock member */
  ADC_CommonInitStruct->ADC_Clock = ADC_Clock_AsynClkMode;

  /* Initialize the ADC_DMAAccessMode member */
  ADC_CommonInitStruct->ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

  /* Initialize the ADC_DMAMode member */
  ADC_CommonInitStruct->ADC_DMAMode = ADC_DMAMode_OneShot;

  /* Initialize the ADC_TwoSamplingDelay member */
  ADC_CommonInitStruct->ADC_TwoSamplingDelay = 0;
}
#endif /* STM32F30||STM32L4 && ADC12_COMMON */

/* -------------------------------------------------------------------------- */
/*                                RCC API                                     */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Waits for HSE start-up.
  * @note   This functions waits on HSERDY flag to be set and return SUCCESS if 
  *         this flag is set, otherwise returns ERROR if the timeout is reached 
  *         and this flag is not set. The timeout value is defined by the constant
  *         HSE_STARTUP_TIMEOUT in stm32f4xx.h file. You can tailor it depending
  *         on the HSE crystal used in your application. 
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: HSE oscillator is stable and ready to use
  *          - ERROR: HSE oscillator not yet ready
  */
ErrorStatus RCC_WaitForHSEStartUp(void)
{
  __IO uint32_t startupcounter = 0;
  ErrorStatus status = ERROR;
  FlagStatus hsestatus = RESET;
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    hsestatus = (FlagStatus) LL_RCC_HSE_IsReady();
    startupcounter++;
  } while((startupcounter != 0x05000) && (hsestatus == RESET));

  if (LL_RCC_HSE_IsReady() != RESET)
  {
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }
  return (status);
}

/* -------------------------------------------------------------------------- */
/*                               HRTIM APIs                                   */
/* -------------------------------------------------------------------------- */
#if defined(STM32F334x8)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HRTIM_FLTR_FLTxEN (HRTIM_FLTR_FLT1EN |\
                           HRTIM_FLTR_FLT2EN |\
                           HRTIM_FLTR_FLT3EN |\
                           HRTIM_FLTR_FLT4EN | \
                           HRTIM_FLTR_FLT5EN)

#define HRTIM_TIMCR_TIMUPDATETRIGGER (HRTIM_TIMUPDATETRIGGER_MASTER  |\
                                      HRTIM_TIMUPDATETRIGGER_TIMER_A |\
                                      HRTIM_TIMUPDATETRIGGER_TIMER_B |\
                                      HRTIM_TIMUPDATETRIGGER_TIMER_C |\
                                      HRTIM_TIMUPDATETRIGGER_TIMER_D |\
                                      HRTIM_TIMUPDATETRIGGER_TIMER_E)

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void HRTIM_MasterBase_Config(HRTIM_TypeDef* HRTIMx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruc);
static void HRTIM_TimingUnitBase_Config(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct);
static void HRTIM_MasterWaveform_Config(HRTIM_TypeDef * HRTIMx, LEGACY_HRTIM_TimerInitTypeDef * TimerInit);
static void HRTIM_CompareUnitConfig(HRTIM_TypeDef * HRTIMx,
                                    uint32_t TimerIdx,
                                    uint32_t CompareUnit,
                                    LEGACY_HRTIM_CompareCfgTypeDef * CompareCfg);
static void HRTIM_OutputConfig(HRTIM_TypeDef * HRTIMx,
                                uint32_t TimerIdx,
                                uint32_t Output,
                                LEGACY_HRTIM_OutputCfgTypeDef * OutputCfg);
static void HRTIM_ExternalEventConfig(HRTIM_TypeDef * HRTIMx,
                                      uint32_t Event,
                                      LEGACY_HRTIM_EventCfgTypeDef * EventCfg);
static void HRTIM_CaptureUnitConfig(HRTIM_TypeDef * HRTIMx,
                                    uint32_t TimerIdx,
                                    uint32_t CaptureUnit,
                                    uint32_t Event);
static void HRTIM_TimingUnitWaveform_Config(HRTIM_TypeDef * HRTIMx, 
                                            uint32_t TimerIdx, 
                                            LEGACY_HRTIM_TimerInitTypeDef * TimerInit);

/**
  * @brief  Initializes the HRTIMx timer in basic time base mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 for master timer
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   The time-base unit initialization parameters specify:
  *           The timer counter operating mode (continuous, one shot)
  *           The timer clock prescaler
  *           The timer period 
  *           The timer repetition counter.
  * @retval None
  */
void HRTIM_SimpleBase_Init(HRTIM_TypeDef* HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{   
  if (TimerIdx == HRTIM_TIMERINDEX_MASTER)
  {
    /* Configure master timer */
    HRTIM_MasterBase_Config(HRTIMx, HRTIM_BaseInitStruct);
  }
  else
  {
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
  }
}

/**
  * @brief  Initializes the HRTIMx timer in basic output compare mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in output compare mode
  * @retval None
  */
void HRTIM_SimpleOC_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{
  /* Configure timing unit */
  HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}

/**
  * @brief  Configures an output in basic output compare mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OCChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pBasicOCChannelCfg: pointer to the basic output compare output configuration structure
  * @note When the timer operates in basic output compare mode:
  *         Output 1 is  implicitly controlled by the compare unit 1
  *         Output 2 is  implicitly controlled by the compare unit 2
  *       Output Set/Reset crossbar is set according to the selected output compare mode:
  *         Toggle: SETxyR = RSTxyR = CMPy
  *         Active: SETxyR = CMPy, RSTxyR = 0
  *         Inactive: SETxy =0, RSTxy = CMPy
  * @retval None
  */
void HRTIM_SimpleOCChannelConfig(HRTIM_TypeDef * HRTIMx,
                                                 uint32_t TimerIdx,
                                                 uint32_t OCChannel,
                                                 LEGACY_HRTIM_BasicOCChannelCfgTypeDef* pBasicOCChannelCfg)
{
  uint32_t CompareUnit = HRTIM_COMPAREUNIT_1;
  LEGACY_HRTIM_CompareCfgTypeDef CompareCfg;
  LEGACY_HRTIM_OutputCfgTypeDef OutputCfg;

  /* Configure timer compare unit */  
  switch (OCChannel)
  {
    case HRTIM_OUTPUT_TA1:
    case HRTIM_OUTPUT_TB1:
    case HRTIM_OUTPUT_TC1:
    case HRTIM_OUTPUT_TD1:
    case HRTIM_OUTPUT_TE1:
    {
      CompareUnit = HRTIM_COMPAREUNIT_1;
    }
    break;
    case HRTIM_OUTPUT_TA2:
    case HRTIM_OUTPUT_TB2:
    case HRTIM_OUTPUT_TC2:
    case HRTIM_OUTPUT_TD2:
    case HRTIM_OUTPUT_TE2:
    {
      CompareUnit = HRTIM_COMPAREUNIT_2;
    }
    break;
    default:
    break;
  }
  
  CompareCfg.CompareValue = pBasicOCChannelCfg->Pulse;
  CompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  CompareCfg.AutoDelayedTimeout = 0;
  
  HRTIM_CompareUnitConfig(HRTIMx,
                          TimerIdx,
                          CompareUnit,
                          &CompareCfg);
  
  /* Configure timer output */
  OutputCfg.Polarity = pBasicOCChannelCfg->Polarity;
  OutputCfg.IdleState = pBasicOCChannelCfg->IdleState;
  OutputCfg.FaultState = HRTIM_OUTPUTFAULTSTATE_NONE;
  OutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  OutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  OutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  
  switch (pBasicOCChannelCfg->Mode)
  {
    case HRTIM_BASICOCMODE_TOGGLE:
    {
      if (CompareUnit == HRTIM_COMPAREUNIT_1)
      {
        OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
      }
      else
      {
        OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
      }
      OutputCfg.ResetSource = OutputCfg.SetSource;
    }
    break;
    case HRTIM_BASICOCMODE_ACTIVE:
    {
      if (CompareUnit == HRTIM_COMPAREUNIT_1)
      {
        OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
      }
      else
      {
        OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
      }
      OutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    }
    break;
    case HRTIM_BASICOCMODE_INACTIVE:
    {
      if (CompareUnit == HRTIM_COMPAREUNIT_1)
      {
        OutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
      }
      else
      {
        OutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
      }
      OutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
    }
    break;
    default:
    break;  
  }
  
  HRTIM_OutputConfig(HRTIMx, TimerIdx, OCChannel, &OutputCfg);   
}

/**
  * @brief  Initializes the HRTIMx timer in basic PWM mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in capture mode
  * @retval None
  */
void HRTIM_SimplePWM_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{
  /* Configure timing unit */
  HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}

/**
  * @brief  Configures an output in basic PWM mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  PWMChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pBasicPWMChannelCfg: pointer to the basic PWM output configuration structure
  * @note When the timer operates in basic PWM output mode:
  *         Output 1 is implicitly controlled by the compare unit 1
  *         Output 2 is implicitly controlled by the compare unit 2
  *         Output Set/Reset crossbar is set as follows:
  *         Output 1: SETx1R = CMP1, RSTx1R = PER
  *         Output 2: SETx2R = CMP2, RST2R = PER
  * @retval None
  */
void HRTIM_SimplePWMChannelConfig(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t PWMChannel,
                                                  LEGACY_HRTIM_BasicPWMChannelCfgTypeDef* pBasicPWMChannelCfg)
{
  uint32_t CompareUnit = HRTIM_COMPAREUNIT_1;
  LEGACY_HRTIM_CompareCfgTypeDef CompareCfg;
  LEGACY_HRTIM_OutputCfgTypeDef OutputCfg;

  /* Configure timer compare unit */  
  switch (PWMChannel)
  {
    case HRTIM_OUTPUT_TA1:
    case HRTIM_OUTPUT_TB1:
    case HRTIM_OUTPUT_TC1:
    case HRTIM_OUTPUT_TD1:
    case HRTIM_OUTPUT_TE1:
    {
      CompareUnit = HRTIM_COMPAREUNIT_1;
    }
    break;
    case HRTIM_OUTPUT_TA2:
    case HRTIM_OUTPUT_TB2:
    case HRTIM_OUTPUT_TC2:
    case HRTIM_OUTPUT_TD2:
    case HRTIM_OUTPUT_TE2:
    {
      CompareUnit = HRTIM_COMPAREUNIT_2;
    }
    break;
    default:
    break;  
  }
  
  CompareCfg.CompareValue = pBasicPWMChannelCfg->Pulse;
  CompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  CompareCfg.AutoDelayedTimeout = 0;
  
  HRTIM_CompareUnitConfig(HRTIMx,
                          TimerIdx,
                          CompareUnit,
                          &CompareCfg);
  
  /* Configure timer output */
  OutputCfg.Polarity = pBasicPWMChannelCfg->Polarity;
  OutputCfg.IdleState = pBasicPWMChannelCfg->IdleState;
  OutputCfg.FaultState = HRTIM_OUTPUTFAULTSTATE_NONE;
  OutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  OutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  OutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  
  if (CompareUnit == HRTIM_COMPAREUNIT_1)
  {
    OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  }
  else
  {
    OutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
  }
  OutputCfg.ResetSource = HRTIM_OUTPUTSET_TIMPER;
  
  HRTIM_OutputConfig(HRTIMx, TimerIdx, PWMChannel, &OutputCfg);  
}

/**
  * @brief  Initializes a timer operating in basic capture mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E 
  * @retval None
  */
void HRTIM_SimpleCapture_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{
  /* Configure timing unit */
  HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}

/**
  * @brief  Configures a basic capture 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Capture unit
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @param  pBasicCaptureChannelCfg: pointer to the basic capture configuration structure
  * @note When the timer operates in basic capture mode the capture is triggered
  *       by the designated external event and GPIO input is implicitly used as event source.
  *       The capture can be triggered by a rising edge, a falling edge or both
  *       edges on event channel.
  * @retval None
  */
void HRTIM_SimpleCaptureChannelConfig(HRTIM_TypeDef * HRTIMx,
                                                      uint32_t TimerIdx,
                                                      uint32_t CaptureChannel,
                                                      LEGACY_HRTIM_BasicCaptureChannelCfgTypeDef* pBasicCaptureChannelCfg)
{
  LEGACY_HRTIM_EventCfgTypeDef EventCfg;

  /* Configure external event channel */
  EventCfg.FastMode = HRTIM_EVENTFASTMODE_DISABLE;
  EventCfg.Filter = pBasicCaptureChannelCfg->EventFilter;
  EventCfg.Polarity = pBasicCaptureChannelCfg->EventPolarity;
  EventCfg.Sensitivity = pBasicCaptureChannelCfg->EventSensitivity;
  EventCfg.Source = HRTIM_EVENTSRC_1;
    
  HRTIM_ExternalEventConfig(HRTIMx,
                    pBasicCaptureChannelCfg->Event,
                    &EventCfg);

  /* Memorize capture trigger (will be configured when the capture is started */  
  HRTIM_CaptureUnitConfig(HRTIMx,
                          TimerIdx,
                          CaptureChannel,
                          pBasicCaptureChannelCfg->Event); 
}

/**
  * @brief  Initializes the HRTIMx timer in basic one pulse mode 
  * @param  HRTIMx: pointer to  HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in one pulse mode. In this mode the counter operates
  *         in single shot mode (retriggerable or not)
  * @retval None
  */
void HRTIM_SimpleOnePulse_Init(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{
  /* Configure timing unit */
  HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}

/**
  * @brief  Initializes a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 for master timer
  *                   @arg 0x1 to 0x5 for timers A to E 
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
void HRTIM_Waveform_Init(HRTIM_TypeDef * HRTIMx,
                                         uint32_t TimerIdx,
                                         LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct,
                                         LEGACY_HRTIM_TimerInitTypeDef* HRTIM_TimerInitStruct)
{
  if (TimerIdx == HRTIM_TIMERINDEX_MASTER)
  {
    /* Configure master timer */
    HRTIM_MasterBase_Config(HRTIMx, HRTIM_BaseInitStruct);
    HRTIM_MasterWaveform_Config(HRTIMx, HRTIM_TimerInitStruct);
  }
  else
  {
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
    HRTIM_TimingUnitWaveform_Config(HRTIMx, TimerIdx, HRTIM_TimerInitStruct);
  }
}

/**
  * @brief  Configures the general behavior of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pTimerCfg: pointer to the timer configuration structure
  * @note When the timer operates in waveform mode, all the features supported by
  *       the HRTIMx are available without any limitation.
  * @retval None
  */
void HRTIM_WaveformTimerConfig(HRTIM_TypeDef * HRTIMx,
                                                uint32_t TimerIdx,
                                                LEGACY_HRTIM_TimerCfgTypeDef * pTimerCfg)
{
  uint32_t HRTIM_timcr;
  uint32_t HRTIM_timfltr;
  uint32_t HRTIM_timoutr;
  uint32_t HRTIM_timrstr;

  /* Configure timing unit (Timer A to Timer E) */
  HRTIM_timcr = HRTIMx->sTimerxRegs[TimerIdx].TIMxCR;
  HRTIM_timfltr  = HRTIMx->sTimerxRegs[TimerIdx].FLTxR;
  HRTIM_timoutr  = HRTIMx->sTimerxRegs[TimerIdx].OUTxR;
  HRTIM_timrstr  = HRTIMx->sTimerxRegs[TimerIdx].RSTxR;
  
  /* Set the push-pull mode */
  HRTIM_timcr &= ~(HRTIM_TIMCR_PSHPLL);
  HRTIM_timcr |= pTimerCfg->PushPull;
  
  /* Enable/Disable registers update on timer counter reset */
  HRTIM_timcr &= ~(HRTIM_TIMCR_TRSTU);
  HRTIM_timcr |= pTimerCfg->ResetUpdate;
  
  /* Set the timer update trigger */
  HRTIM_timcr &= ~(HRTIM_TIMCR_TIMUPDATETRIGGER);
  HRTIM_timcr |= pTimerCfg->UpdateTrigger;
  
  /* Enable/Disable the fault channel at timer level */
  HRTIM_timfltr &= ~(HRTIM_FLTR_FLTxEN);
  HRTIM_timfltr |= (pTimerCfg->FaultEnable & HRTIM_FLTR_FLTxEN);
  
  /* Lock/Unlock fault sources at timer level */
  HRTIM_timfltr &= ~(HRTIM_FLTR_FLTLCK);
  HRTIM_timfltr |= pTimerCfg->FaultLock;
  
  /* Enable/Disable dead time insertion at timer level */
  HRTIM_timoutr &= ~(HRTIM_OUTR_DTEN);
  HRTIM_timoutr |= pTimerCfg->DeadTimeInsertion;

  /* Enable/Disable delayed protection at timer level */
  HRTIM_timoutr &= ~(HRTIM_OUTR_DLYPRT| HRTIM_OUTR_DLYPRTEN);
  HRTIM_timoutr |= pTimerCfg->DelayedProtectionMode;
  
  /* Set the timer counter reset trigger */
  HRTIM_timrstr = pTimerCfg->ResetTrigger;

  /* Update the HRTIMx registers */
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR  = HRTIM_timcr;
  HRTIMx->sTimerxRegs[TimerIdx].FLTxR = HRTIM_timfltr;
  HRTIMx->sTimerxRegs[TimerIdx].OUTxR = HRTIM_timoutr;
  HRTIMx->sTimerxRegs[TimerIdx].RSTxR = HRTIM_timrstr;
 }
 
/**
  * @brief  Configures the compare unit of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   0xFF for master timer
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  pCompareCfg: pointer to the compare unit configuration structure
  * @note When auto delayed mode is required for compare unit 2 or compare unit 4, 
  *       application has to configure separately the capture unit. Capture unit 
  *       to configure in that case depends on the compare unit auto delayed mode
  *       is applied to (see below):
  *         Auto delayed on output compare 2: capture unit 1 must be configured
  *         Auto delayed on output compare 4: capture unit 2 must be configured
  * @retval None
  */
 void HRTIM_WaveformCompareConfig(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CompareUnit,
                                                  LEGACY_HRTIM_CompareCfgTypeDef* pCompareCfg)
{
    uint32_t HRTIM_timcr;

  /* Configure the compare unit */
  switch (CompareUnit)
  {
    case HRTIM_COMPAREUNIT_1:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP1xR = pCompareCfg->CompareValue;
    }
    break;
    case HRTIM_COMPAREUNIT_2:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP2xR = pCompareCfg->CompareValue;
      
      if (pCompareCfg->AutoDelayedMode != HRTIM_AUTODELAYEDMODE_REGULAR)
      {
        /* Configure auto-delayed mode */
        HRTIM_timcr = HRTIMx->sTimerxRegs[TimerIdx].TIMxCR;
        HRTIM_timcr &= ~HRTIM_TIMCR_DELCMP2;
        HRTIM_timcr |= pCompareCfg->AutoDelayedMode;
        HRTIMx->sTimerxRegs[TimerIdx].TIMxCR = HRTIM_timcr;
        
        /* Set the compare value for timeout compare unit (if any) */
        if (pCompareCfg->AutoDelayedMode == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1)
        {
          HRTIMx->sTimerxRegs[TimerIdx].CMP1xR = pCompareCfg->AutoDelayedTimeout;
        }
        else if (pCompareCfg->AutoDelayedMode == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3)
        {
          HRTIMx->sTimerxRegs[TimerIdx].CMP3xR = pCompareCfg->AutoDelayedTimeout;
        }
      }
    }
    break;
    case HRTIM_COMPAREUNIT_3:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP3xR = pCompareCfg->CompareValue;
    }
    break;
    case HRTIM_COMPAREUNIT_4:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP4xR = pCompareCfg->CompareValue;
      
      if (pCompareCfg->AutoDelayedMode != HRTIM_AUTODELAYEDMODE_REGULAR)
      {
        /* Configure auto-delayed mode */
        HRTIM_timcr = HRTIMx->sTimerxRegs[TimerIdx].TIMxCR;
        HRTIM_timcr &= ~HRTIM_TIMCR_DELCMP4;
        HRTIM_timcr |= (pCompareCfg->AutoDelayedMode << 2);
        HRTIMx->sTimerxRegs[TimerIdx].TIMxCR = HRTIM_timcr;
        
        /* Set the compare value for timeout compare unit (if any) */
        if (pCompareCfg->AutoDelayedMode == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP1)
        {
          HRTIMx->sTimerxRegs[TimerIdx].CMP1xR = pCompareCfg->AutoDelayedTimeout;
        }
        else if (pCompareCfg->AutoDelayedMode == HRTIM_AUTODELAYEDMODE_AUTODELAYED_TIMEOUTCMP3)
        {
          HRTIMx->sTimerxRegs[TimerIdx].CMP3xR = pCompareCfg->AutoDelayedTimeout;
        }
      }
    }
    break;
    default:
    break;  
  }
}

/**
  * @brief  Sets the HRTIMx Slave Comparex Register value 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  Compare: specifies the Comparex register new value
  * @retval None
  */
void HRTIM_SlaveSetCompare(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CompareUnit,
                                                  uint32_t Compare)
{
  /* Configure the compare unit */
  switch (CompareUnit)
  {
    case HRTIM_COMPAREUNIT_1:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP1xR = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_2:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP2xR = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_3:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP3xR = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_4:
    {
      /* Set the compare value */
      HRTIMx->sTimerxRegs[TimerIdx].CMP4xR = Compare;
    }
    break;
    default:
    break;
  }  
}

/**
  * @brief  Sets the HRTIMx Master Comparex Register value 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  Compare: specifies the Comparex register new value
  * @retval None
  */
void HRTIM_MasterSetCompare(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t CompareUnit,
                                                  uint32_t Compare)
{
  /* Configure the compare unit */
  switch (CompareUnit)
  {
    case HRTIM_COMPAREUNIT_1:
    {
      /* Set the compare value */
      HRTIMx->sMasterRegs.MCMP1R = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_2:
    {
      /* Set the compare value */
      HRTIMx->sMasterRegs.MCMP2R = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_3:
    {
      /* Set the compare value */
      HRTIMx->sMasterRegs.MCMP3R = Compare;
    }
    break;
    case HRTIM_COMPAREUNIT_4:
    {
      /* Set the compare value */
      HRTIMx->sMasterRegs.MCMP4R = Compare;
    }
    break;
    default:
    break;
  }  
}

/**
  * @brief  Configures the capture unit of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Capture unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @param  pCaptureCfg: pointer to the compare unit configuration structure
  * @retval None
  */
void HRTIM_WaveformCaptureConfig(HRTIM_TypeDef * HRTIMx,
                                                  uint32_t TimerIdx,
                                                  uint32_t CaptureUnit,
                                                  LEGACY_HRTIM_CaptureCfgTypeDef* pCaptureCfg)
{
  /* Configure the capture unit */
  switch (CaptureUnit)
  {
    case HRTIM_CAPTUREUNIT_1:
    {
      HRTIMx->sTimerxRegs[TimerIdx].CPT1xCR = pCaptureCfg->Trigger;
    }
    break;
    case HRTIM_CAPTUREUNIT_2:
    {
      HRTIMx->sTimerxRegs[TimerIdx].CPT2xCR = pCaptureCfg->Trigger;
    }
    break;
    default:
    break;
  }
}

/**
  * @brief  Configures the event filtering capabilities of a timer (blanking, windowing) 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Event: external event for which timer event filtering must be configured
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_EVENT_1: External event 1
  *                    @arg HRTIM_EVENT_2: External event 2
  *                    @arg HRTIM_EVENT_3: External event 3
  *                    @arg HRTIM_EVENT_4: External event 4
  *                    @arg HRTIM_EVENT_5: External event 5
  *                    @arg HRTIM_EVENT_6: External event 6
  *                    @arg HRTIM_EVENT_7: External event 7
  *                    @arg HRTIM_EVENT_8: External event 8
  *                    @arg HRTIM_EVENT_9: External event 9
  *                    @arg HRTIM_EVENT_10: External event 10
  * @param  pTimerEventFilteringCfg: pointer to the timer event filtering configuration structure
  * @retval None
  */
void HRTIM_TimerEventFilteringConfig(HRTIM_TypeDef * HRTIMx,
                                                      uint32_t TimerIdx,
                                                      uint32_t Event,
                                                      LEGACY_HRTIM_TimerEventFilteringCfgTypeDef* pTimerEventFilteringCfg)
{
  uint32_t HRTIM_eefr;
  
  /* Configure timer event filtering capabilities */
  switch (Event)
  {
    case HRTIM_TIMEVENTFILTER_NONE:
    {
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = 0;
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = 0;
    }
    break;
    case HRTIM_EVENT_1:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR1;
      HRTIM_eefr &= ~(HRTIM_EEFR1_EE1FLTR | HRTIM_EEFR1_EE1LTCH);
      HRTIM_eefr |= (pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_2:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR1;
      HRTIM_eefr &= ~(HRTIM_EEFR1_EE2FLTR | HRTIM_EEFR1_EE2LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 6);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_3:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR1;
      HRTIM_eefr &= ~(HRTIM_EEFR1_EE3FLTR | HRTIM_EEFR1_EE3LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 12);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_4:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR1;
      HRTIM_eefr &= ~(HRTIM_EEFR1_EE4FLTR | HRTIM_EEFR1_EE4LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 18);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_5:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR1;
      HRTIM_eefr &= ~(HRTIM_EEFR1_EE5FLTR | HRTIM_EEFR1_EE5LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 24);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR1 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_6:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR2;
      HRTIM_eefr &= ~(HRTIM_EEFR2_EE6FLTR | HRTIM_EEFR2_EE6LTCH);
      HRTIM_eefr |= (pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_7:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR2;
      HRTIM_eefr &= ~(HRTIM_EEFR2_EE7FLTR | HRTIM_EEFR2_EE7LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 6);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_8:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR2;
      HRTIM_eefr &= ~(HRTIM_EEFR2_EE8FLTR | HRTIM_EEFR2_EE8LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 12);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_9:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR2;
      HRTIM_eefr &= ~(HRTIM_EEFR2_EE9FLTR | HRTIM_EEFR2_EE9LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 18);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = HRTIM_eefr;
    }
    break;
    case HRTIM_EVENT_10:
    {
      HRTIM_eefr = HRTIMx->sTimerxRegs[TimerIdx].EEFxR2;
      HRTIM_eefr &= ~(HRTIM_EEFR2_EE10FLTR | HRTIM_EEFR2_EE10LTCH);
      HRTIM_eefr |= ((pTimerEventFilteringCfg->Filter | pTimerEventFilteringCfg->Latch) << 24);
      HRTIMx->sTimerxRegs[TimerIdx].EEFxR2 = HRTIM_eefr;
    }
    break;
    default:
    break;
  }
}

/**
  * @brief  Configures the dead time insertion feature for a timer 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pDeadTimeCfg: pointer to the dead time insertion configuration structure
  * @retval None
  */
void HRTIM_DeadTimeConfig(HRTIM_TypeDef * HRTIMx,
                                           uint32_t TimerIdx,
                                           LEGACY_HRTIM_DeadTimeCfgTypeDef* pDeadTimeCfg)
{
  uint32_t HRTIM_dtr;
  
  HRTIM_dtr = HRTIMx->sTimerxRegs[TimerIdx].DTxR;
     
  /* Clear timer dead times configuration */
  HRTIM_dtr &= ~(HRTIM_DTR_DTR | HRTIM_DTR_SDTR | HRTIM_DTR_DTPRSC |
                 HRTIM_DTR_DTRSLK | HRTIM_DTR_DTRLK | HRTIM_DTR_SDTF |
                 HRTIM_DTR_SDTR | HRTIM_DTR_DTFSLK | HRTIM_DTR_DTFLK);
  
  /* Set timer dead times configuration */
  HRTIM_dtr |= (pDeadTimeCfg->Prescaler << 10);
  HRTIM_dtr |= pDeadTimeCfg->RisingValue;
  HRTIM_dtr |= pDeadTimeCfg->RisingSign;
  HRTIM_dtr |= pDeadTimeCfg->RisingSignLock;
  HRTIM_dtr |= pDeadTimeCfg->RisingLock;
  HRTIM_dtr |= (pDeadTimeCfg->FallingValue << 16);
  HRTIM_dtr |= pDeadTimeCfg->FallingSign;
  HRTIM_dtr |= pDeadTimeCfg->FallingSignLock;
  HRTIM_dtr |= pDeadTimeCfg->FallingLock;
    
  /* Update the HRTIMx registers */  
  HRTIMx->sTimerxRegs[TimerIdx].DTxR = HRTIM_dtr;
}

/**
  * @brief  Configures the chopper mode feature for a timer 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pChopperModeCfg: pointer to the chopper mode configuration structure
  * @retval None
  */
void HRTIM_ChopperModeConfig(HRTIM_TypeDef * HRTIMx,
                                              uint32_t TimerIdx,
                                              LEGACY_HRTIM_ChopperModeCfgTypeDef* pChopperModeCfg)
{
  uint32_t HRTIM_chpr;
  
  HRTIM_chpr = HRTIMx->sTimerxRegs[TimerIdx].CHPxR;
     
  /* Clear timer chopper mode configuration */
  HRTIM_chpr &= ~(HRTIM_CHPR_CARFRQ | HRTIM_CHPR_CARDTY | HRTIM_CHPR_STRPW);
  
  /* Set timer chopper mode configuration */
  HRTIM_chpr |= pChopperModeCfg->CarrierFreq;
  HRTIM_chpr |= (pChopperModeCfg->DutyCycle << 4);
  HRTIM_chpr |= (pChopperModeCfg->StartPulse << 7);
    
  /* Update the HRTIMx registers */  
  HRTIMx->sTimerxRegs[TimerIdx].CHPxR = HRTIM_chpr;
}

/**
  * @brief  Configures the external input/output synchronization of the HRTIMx 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  pSynchroCfg: pointer to the input/output synchronization configuration structure
  * @retval None
  */
void HRTIM_SynchronizationConfig(HRTIM_TypeDef *HRTIMx, LEGACY_HRTIM_SynchroCfgTypeDef * pSynchroCfg)
{
  uint32_t HRTIM_mcr;
  
  HRTIM_mcr = HRTIMx->sMasterRegs.MCR;

  /* Set the synchronization input source */
  HRTIM_mcr &= ~(HRTIM_MCR_SYNC_IN);
  HRTIM_mcr |= pSynchroCfg->SyncInputSource;
  
  /* Set the event to be sent on the synchronization output */
  HRTIM_mcr &= ~(HRTIM_MCR_SYNC_SRC);
  HRTIM_mcr |= pSynchroCfg->SyncOutputSource;
  
  /* Set the polarity of the synchronization output */
  HRTIM_mcr &= ~(HRTIM_MCR_SYNC_OUT);
  HRTIM_mcr |= pSynchroCfg->SyncOutputPolarity;
  
  /* Update the HRTIMx registers */  
  HRTIMx->sMasterRegs.MCR = HRTIM_mcr;
}

/**
  * @brief  Configures the burst mode feature of the HRTIMx 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  pBurstModeCfg: pointer to the burst mode configuration structure
  * @retval None
  */
void HRTIM_BurstModeConfig(HRTIM_TypeDef * HRTIMx,
                                            LEGACY_HRTIM_BurstModeCfgTypeDef* pBurstModeCfg)
{
  uint32_t HRTIM_bmcr;

  HRTIM_bmcr = HRTIMx->sCommonRegs.BMCR;

  /* Set the burst mode operating mode */
  HRTIM_bmcr &= ~(HRTIM_BMCR_BMOM);
  HRTIM_bmcr |= pBurstModeCfg->Mode;
  
  /* Set the burst mode clock source */
  HRTIM_bmcr &= ~(HRTIM_BMCR_BMCLK);
  HRTIM_bmcr |= pBurstModeCfg->ClockSource;
  
  /* Set the burst mode prescaler */
  HRTIM_bmcr &= ~(HRTIM_BMCR_BMPRSC);
  HRTIM_bmcr |= pBurstModeCfg->Prescaler;
 
  /* Enable/disable burst mode registers preload */
  HRTIM_bmcr &= ~(HRTIM_BMCR_BMPREN);
  HRTIM_bmcr |= pBurstModeCfg->PreloadEnable;
 
  /* Set the burst mode trigger */
  HRTIMx->sCommonRegs.BMTRGR = pBurstModeCfg->Trigger;
  
  /* Set the burst mode compare value */
  HRTIMx->sCommonRegs.BMCMPR = pBurstModeCfg->IdleDuration;
  
  /* Set the burst mode period */
  HRTIMx->sCommonRegs.BMPER = pBurstModeCfg->Period;
  
  /* Update the HRTIMx registers */  
  HRTIMx->sCommonRegs.BMCR = HRTIM_bmcr;
}

/**
  * @brief  Configures the conditioning of an external event
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Event: external event to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_EVENT_1: External event 1
  *                    @arg HRTIM_EVENT_2: External event 2
  *                    @arg HRTIM_EVENT_3: External event 3
  *                    @arg HRTIM_EVENT_4: External event 4
  *                    @arg HRTIM_EVENT_5: External event 5
  *                    @arg HRTIM_EVENT_6: External event 6
  *                    @arg HRTIM_EVENT_7: External event 7
  *                    @arg HRTIM_EVENT_8: External event 8
  *                    @arg HRTIM_EVENT_9: External event 9
  *                    @arg HRTIM_EVENT_10: External event 10
  * @param  pEventCfg: pointer to the event conditioning configuration structure
  * @retval None
  */
void HRTIM_EventConfig(HRTIM_TypeDef * HRTIMx,
                                        uint32_t Event,
                                        LEGACY_HRTIM_EventCfgTypeDef* pEventCfg)
{
  /* Configure the event channel */
  HRTIM_ExternalEventConfig(HRTIMx, Event, pEventCfg);
 
}

/**
  * @brief  Configures the conditioning of fault input
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Fault: fault input to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_FAULT_1: Fault input 1
  *                    @arg HRTIM_FAULT_2: Fault input 2
  *                    @arg HRTIM_FAULT_3: Fault input 3
  *                    @arg HRTIM_FAULT_4: Fault input 4
  *                    @arg HRTIM_FAULT_5: Fault input 5
  * @param  pFaultCfg: pointer to the fault conditioning configuration structure
  * @retval None
  */
void HRTIM_FaultConfig(HRTIM_TypeDef * HRTIMx,
                                        LEGACY_HRTIM_FaultCfgTypeDef* pFaultCfg,
                                        uint32_t Fault)
{
  uint32_t HRTIM_fltinr1;
  uint32_t HRTIM_fltinr2;

  /* Configure fault channel */
  HRTIM_fltinr1 = HRTIMx->sCommonRegs.FLTINR1;
  HRTIM_fltinr2 = HRTIMx->sCommonRegs.FLTINR2;
  
  switch (Fault)
  {
    case HRTIM_FAULT_1:
    {
      HRTIM_fltinr1 &= ~(HRTIM_FLTINR1_FLT1P | HRTIM_FLTINR1_FLT1SRC | HRTIM_FLTINR1_FLT1F | HRTIM_FLTINR1_FLT1LCK);
      HRTIM_fltinr1 |= pFaultCfg->Polarity;
      HRTIM_fltinr1 |= pFaultCfg->Source;
      HRTIM_fltinr1 |= pFaultCfg->Filter;
      HRTIM_fltinr1 |= pFaultCfg->Lock;
    }
    break;
    case HRTIM_FAULT_2:
    {
      HRTIM_fltinr1 &= ~(HRTIM_FLTINR1_FLT2P | HRTIM_FLTINR1_FLT2SRC | HRTIM_FLTINR1_FLT2F | HRTIM_FLTINR1_FLT2LCK);
      HRTIM_fltinr1 |= (pFaultCfg->Polarity << 8);
      HRTIM_fltinr1 |= (pFaultCfg->Source << 8);
      HRTIM_fltinr1 |= (pFaultCfg->Filter << 8);
      HRTIM_fltinr1 |= (pFaultCfg->Lock << 8);
    }
    break;
    case HRTIM_FAULT_3:
    {
      HRTIM_fltinr1 &= ~(HRTIM_FLTINR1_FLT3P | HRTIM_FLTINR1_FLT3SRC | HRTIM_FLTINR1_FLT3F | HRTIM_FLTINR1_FLT3LCK);
      HRTIM_fltinr1 |= (pFaultCfg->Polarity << 16);
      HRTIM_fltinr1 |= (pFaultCfg->Source << 16);
      HRTIM_fltinr1 |= (pFaultCfg->Filter << 16);
      HRTIM_fltinr1 |= (pFaultCfg->Lock << 16);
    }
    break;
    case HRTIM_FAULT_4:
    {
      HRTIM_fltinr1 &= ~(HRTIM_FLTINR1_FLT4P | HRTIM_FLTINR1_FLT4SRC | HRTIM_FLTINR1_FLT4F | HRTIM_FLTINR1_FLT4LCK);
      HRTIM_fltinr1 |= (pFaultCfg->Polarity << 24);
      HRTIM_fltinr1 |= (pFaultCfg->Source << 24);
      HRTIM_fltinr1 |= (pFaultCfg->Filter << 24);
      HRTIM_fltinr1 |= (pFaultCfg->Lock << 24);
    }
    break;
    case HRTIM_FAULT_5:
    {
      HRTIM_fltinr2 &= ~(HRTIM_FLTINR2_FLT5P | HRTIM_FLTINR2_FLT5SRC | HRTIM_FLTINR2_FLT5F | HRTIM_FLTINR2_FLT5LCK);
      HRTIM_fltinr2 |= pFaultCfg->Polarity;
      HRTIM_fltinr2 |= pFaultCfg->Source;
      HRTIM_fltinr2 |= pFaultCfg->Filter;
      HRTIM_fltinr2 |= pFaultCfg->Lock;
    }
    break;
    default:
    break;
  }

  /* Update the HRTIMx registers */
  HRTIMx->sCommonRegs.FLTINR1 = HRTIM_fltinr1;
  HRTIMx->sCommonRegs.FLTINR2 = HRTIM_fltinr2;
}

/**
  * @brief  Configures both the ADC trigger register update source and the ADC
  *         trigger source.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  ADC trigger: ADC trigger to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_ADCTRIGGER_1: ADC trigger 1
  *                    @arg HRTIM_ADCTRIGGER_2: ADC trigger 2
  *                    @arg HRTIM_ADCTRIGGER_3: ADC trigger 3
  *                    @arg HRTIM_ADCTRIGGER_4: ADC trigger 4
  * @param  pADCTriggerCfg: pointer to the ADC trigger configuration structure
  * @retval None
  */
void HRTIM_ADCTriggerConfig(HRTIM_TypeDef * HRTIMx,
                                             uint32_t ADCTrigger,
                                             LEGACY_HRTIM_ADCTriggerCfgTypeDef* pADCTriggerCfg)
{
  uint32_t HRTIM_cr1;
  
  /* Set the ADC trigger update source */
  HRTIM_cr1 = HRTIMx->sCommonRegs.CR1;
  
  switch (ADCTrigger)
  {
    case HRTIM_ADCTRIGGER_1:
    {
      HRTIM_cr1 &= ~(HRTIM_CR1_ADC1USRC);
      HRTIM_cr1 |= pADCTriggerCfg->UpdateSource;
      
      /* Set the ADC trigger 1 source */
      HRTIMx->sCommonRegs.ADC1R = pADCTriggerCfg->Trigger;
    }
    break;
    case HRTIM_ADCTRIGGER_2:
    {
      HRTIM_cr1 &= ~(HRTIM_CR1_ADC2USRC);
      HRTIM_cr1 |= (pADCTriggerCfg->UpdateSource << 3); 

      /* Set the ADC trigger 2 source */
      HRTIMx->sCommonRegs.ADC2R = pADCTriggerCfg->Trigger;
    }
    break;
    case HRTIM_ADCTRIGGER_3:
    {
      HRTIM_cr1 &= ~(HRTIM_CR1_ADC3USRC);
      HRTIM_cr1 |= (pADCTriggerCfg->UpdateSource << 6); 
      
      /* Set the ADC trigger 3 source */
      HRTIMx->sCommonRegs.ADC3R = pADCTriggerCfg->Trigger;
    }
    case HRTIM_ADCTRIGGER_4:
    {
      HRTIM_cr1 &= ~(HRTIM_CR1_ADC4USRC);
      HRTIM_cr1 |= (pADCTriggerCfg->UpdateSource << 9); 
      
      /* Set the ADC trigger 4 source */
      HRTIMx->sCommonRegs.ADC4R = pADCTriggerCfg->Trigger;
    }
    break;
    default:
    break;
  }
  
  /* Update the HRTIMx registers */
  HRTIMx->sCommonRegs.CR1 = HRTIM_cr1;
}

/**
  * @brief  Configures the output of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pOutputCfg: pointer to the timer output configuration structure
  * @retval None
  */
void HRTIM_WaveformOutputConfig(HRTIM_TypeDef * HRTIMx,
                                                uint32_t TimerIdx,
                                                uint32_t Output,
                                                LEGACY_HRTIM_OutputCfgTypeDef * pOutputCfg)
{
  /* Configure the timer output */
  HRTIM_OutputConfig(HRTIMx, TimerIdx, Output, pOutputCfg);  
}

///////////////////////////////////////////////

/**
  * @brief  Configures the master timer time base
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @retval None
  */
void  HRTIM_MasterBase_Config(HRTIM_TypeDef * HRTIMx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{  
  /* Set the prescaler ratio */
  HRTIMx->sMasterRegs.MCR &= (uint32_t) ~(HRTIM_MCR_CK_PSC);
  HRTIMx->sMasterRegs.MCR  |= (uint32_t)HRTIM_BaseInitStruct->PrescalerRatio;
  
  /* Set the operating mode */
  HRTIMx->sMasterRegs.MCR  &= (uint32_t) ~(HRTIM_MCR_CONT | HRTIM_MCR_RETRIG);
  HRTIMx->sMasterRegs.MCR  |= (uint32_t)HRTIM_BaseInitStruct->Mode;
  
  /* Update the HRTIMx registers */
  HRTIMx->sMasterRegs.MPER = HRTIM_BaseInitStruct->Period;
  HRTIMx->sMasterRegs.MREP = HRTIM_BaseInitStruct->RepetitionCounter;
}

/**
  * @brief  Configures timing unit (timer A to timer E) time base
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @retval None
  */
void HRTIM_TimingUnitBase_Config(HRTIM_TypeDef * HRTIMx, uint32_t TimerIdx, LEGACY_HRTIM_BaseInitTypeDef* HRTIM_BaseInitStruct)
{   
  /* Set the prescaler ratio */
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR &= (uint32_t) ~(HRTIM_TIMCR_CK_PSC);
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR |= (uint32_t)HRTIM_BaseInitStruct->PrescalerRatio;

  /* Set the operating mode */
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR &= (uint32_t) ~(HRTIM_TIMCR_CONT | HRTIM_TIMCR_RETRIG);
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR |= (uint32_t)HRTIM_BaseInitStruct->Mode;
  
  /* Update the HRTIMx registers */
  HRTIMx->sTimerxRegs[TimerIdx].PERxR = HRTIM_BaseInitStruct->Period;
  HRTIMx->sTimerxRegs[TimerIdx].REPxR = HRTIM_BaseInitStruct->RepetitionCounter;
}

/**
  * @brief  Configures a compare unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  CompareUnit: Compare unit identifier
  * @param  pCompareCfg: pointer to the compare unit configuration data structure
  * @retval None
  */
void  HRTIM_CompareUnitConfig(HRTIM_TypeDef * HRTIMx,
                              uint32_t TimerIdx,
                              uint32_t CompareUnit,
                              LEGACY_HRTIM_CompareCfgTypeDef * pCompareCfg)
{
  if (TimerIdx == HRTIM_TIMERINDEX_MASTER)
  {
    /* Configure the compare unit of the master timer */
    switch (CompareUnit)
    {
      case HRTIM_COMPAREUNIT_1:
      {
        HRTIMx->sMasterRegs.MCMP1R = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_2:
      {
        HRTIMx->sMasterRegs.MCMP2R = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_3:
      {
        HRTIMx->sMasterRegs.MCMP3R = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_4:
      {
        HRTIMx->sMasterRegs.MCMP4R = pCompareCfg->CompareValue;
      }
      break;
      default:
      break;
    }
  }
  else
  {
    /* Configure the compare unit of the timing unit */
    switch (CompareUnit)
    {
      case HRTIM_COMPAREUNIT_1:
      {
        HRTIMx->sTimerxRegs[TimerIdx].CMP1xR = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_2:
      {
        HRTIMx->sTimerxRegs[TimerIdx].CMP2xR = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_3:
      {
        HRTIMx->sTimerxRegs[TimerIdx].CMP3xR = pCompareCfg->CompareValue;
      }
      break;
      case HRTIM_COMPAREUNIT_4:
      {
        HRTIMx->sTimerxRegs[TimerIdx].CMP4xR = pCompareCfg->CompareValue;
      }
      break;
      default:
      break;
    }
  }
}

/**
  * @brief  Configures the output of a timing unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  Output: timing unit output identifier
  * @param  pOutputCfg: pointer to the output configuration data structure
  * @retval None
  */
void  HRTIM_OutputConfig(HRTIM_TypeDef * HRTIMx,
                         uint32_t TimerIdx,
                         uint32_t Output,
                         LEGACY_HRTIM_OutputCfgTypeDef * pOutputCfg)
{
  uint32_t HRTIM_outr;
  uint32_t shift = 0;
  
  HRTIM_outr = HRTIMx->sTimerxRegs[TimerIdx].OUTxR;
  
  switch (Output)
  {
    case HRTIM_OUTPUT_TA1:
    case HRTIM_OUTPUT_TB1:
    case HRTIM_OUTPUT_TC1:
    case HRTIM_OUTPUT_TD1:
    case HRTIM_OUTPUT_TE1:
    {
      /* Set the output set/reset crossbar */
      HRTIMx->sTimerxRegs[TimerIdx].SETx1R = pOutputCfg->SetSource;
      HRTIMx->sTimerxRegs[TimerIdx].RSTx1R = pOutputCfg->ResetSource;
      
      shift = 0;
    }
    break;
    case HRTIM_OUTPUT_TA2:
    case HRTIM_OUTPUT_TB2:
    case HRTIM_OUTPUT_TC2:
    case HRTIM_OUTPUT_TD2:
    case HRTIM_OUTPUT_TE2:
    {
      /* Set the output set/reset crossbar */
      HRTIMx->sTimerxRegs[TimerIdx].SETx2R = pOutputCfg->SetSource;
      HRTIMx->sTimerxRegs[TimerIdx].RSTx2R = pOutputCfg->ResetSource;

      shift = 16;
    }
    break;
    default:
    break;
  }
  
  /* Clear output config */
  HRTIM_outr &= ~((HRTIM_OUTR_POL1 |
                   HRTIM_OUTR_IDLM1 |
                   HRTIM_OUTR_IDLES1|
                   HRTIM_OUTR_FAULT1|
                   HRTIM_OUTR_CHP1 |
                   HRTIM_OUTR_DIDL1)  << shift);
  
  /* Set the polarity */
  HRTIM_outr |= (pOutputCfg->Polarity << shift);
  
  /* Set the IDLE mode */
  HRTIM_outr |= (pOutputCfg->IdleMode << shift);
  
  /* Set the IDLE state */
  HRTIM_outr |= (pOutputCfg->IdleState << shift);
  
  /* Set the FAULT state */
  HRTIM_outr |= (pOutputCfg->FaultState << shift);
  
  /* Set the chopper mode */
  HRTIM_outr |= (pOutputCfg->ChopperModeEnable << shift);

  /* Set the burst mode entry mode */
  HRTIM_outr |= (pOutputCfg->BurstModeEntryDelayed << shift);
  
  /* Update HRTIMx register */
  HRTIMx->sTimerxRegs[TimerIdx].OUTxR = HRTIM_outr;
}

/**
  * @brief  Configures an external event channel 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Event: Event channel identifier
  * @param  pEventCfg: pointer to the event channel configuration data structure
  * @retval None
  */
static void HRTIM_ExternalEventConfig(HRTIM_TypeDef * HRTIMx,
                              uint32_t Event,
                              LEGACY_HRTIM_EventCfgTypeDef *pEventCfg)
{
  uint32_t hrtim_eecr1;
  uint32_t hrtim_eecr2;
  uint32_t hrtim_eecr3;

  /* Configure external event channel */
  hrtim_eecr1 = HRTIMx->sCommonRegs.EECR1;
  hrtim_eecr2 = HRTIMx->sCommonRegs.EECR2;
  hrtim_eecr3 = HRTIMx->sCommonRegs.EECR3;
  
  switch (Event)
  {
    case HRTIM_EVENT_1:
    {
      hrtim_eecr1 &= ~(HRTIM_EECR1_EE1SRC | HRTIM_EECR1_EE1POL | HRTIM_EECR1_EE1SNS | HRTIM_EECR1_EE1FAST);
      hrtim_eecr1 |= pEventCfg->Source;
      hrtim_eecr1 |= pEventCfg->Polarity;
      hrtim_eecr1 |= pEventCfg->Sensitivity;
      /* Update the HRTIM registers (all bit fields but EE1FAST bit) */
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
      /* Update the HRTIM registers (EE1FAST bit) */
      hrtim_eecr1 |= pEventCfg->FastMode;
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
    }
    break;
    case HRTIM_EVENT_2:
    {
      hrtim_eecr1 &= ~(HRTIM_EECR1_EE2SRC | HRTIM_EECR1_EE2POL | HRTIM_EECR1_EE2SNS | HRTIM_EECR1_EE2FAST);
      hrtim_eecr1 |= (pEventCfg->Source << 6);
      hrtim_eecr1 |= (pEventCfg->Polarity << 6);
      hrtim_eecr1 |= (pEventCfg->Sensitivity << 6);
      /* Update the HRTIM registers (all bit fields but EE2FAST bit) */
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
      /* Update the HRTIM registers (EE2FAST bit) */
      hrtim_eecr1 |= (pEventCfg->FastMode << 6);
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
    }
    break;
    case HRTIM_EVENT_3:
    {
      hrtim_eecr1 &= ~(HRTIM_EECR1_EE3SRC | HRTIM_EECR1_EE3POL | HRTIM_EECR1_EE3SNS | HRTIM_EECR1_EE3FAST);
      hrtim_eecr1 |= (pEventCfg->Source << 12);
      hrtim_eecr1 |= (pEventCfg->Polarity << 12);
      hrtim_eecr1 |= (pEventCfg->Sensitivity << 12);
      /* Update the HRTIM registers (all bit fields but EE3FAST bit) */
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
      /* Update the HRTIM registers (EE3FAST bit) */
      hrtim_eecr1 |= (pEventCfg->FastMode << 12);
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
    }
    break;
    case HRTIM_EVENT_4:
    {
      hrtim_eecr1 &= ~(HRTIM_EECR1_EE4SRC | HRTIM_EECR1_EE4POL | HRTIM_EECR1_EE4SNS | HRTIM_EECR1_EE4FAST);
      hrtim_eecr1 |= (pEventCfg->Source << 18);
      hrtim_eecr1 |= (pEventCfg->Polarity << 18);
      hrtim_eecr1 |= (pEventCfg->Sensitivity << 18);
      /* Update the HRTIM registers (all bit fields but EE4FAST bit) */
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
      /* Update the HRTIM registers (EE4FAST bit) */
      hrtim_eecr1 |= (pEventCfg->FastMode << 18);
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
    }
    break;
    case HRTIM_EVENT_5:
    {
      hrtim_eecr1 &= ~(HRTIM_EECR1_EE5SRC | HRTIM_EECR1_EE5POL | HRTIM_EECR1_EE5SNS | HRTIM_EECR1_EE5FAST);
      hrtim_eecr1 |= (pEventCfg->Source << 24);
      hrtim_eecr1 |= (pEventCfg->Polarity << 24);
      hrtim_eecr1 |= (pEventCfg->Sensitivity << 24);
      /* Update the HRTIM registers (all bit fields but EE5FAST bit) */
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
      /* Update the HRTIM registers (EE5FAST bit) */
      hrtim_eecr1 |= (pEventCfg->FastMode << 24);
      HRTIMx->sCommonRegs.EECR1 = hrtim_eecr1;
    }
    break;
    case HRTIM_EVENT_6:
    {
      hrtim_eecr2 &= ~(HRTIM_EECR2_EE6SRC | HRTIM_EECR2_EE6POL | HRTIM_EECR2_EE6SNS);
      hrtim_eecr2 |= pEventCfg->Source;
      hrtim_eecr2 |= pEventCfg->Polarity;
      hrtim_eecr2 |= pEventCfg->Sensitivity;
      hrtim_eecr3 &= ~(HRTIM_EECR3_EE6F);
      hrtim_eecr3 |= pEventCfg->Filter;
      /* Update the HRTIM registers */
      HRTIMx->sCommonRegs.EECR2 = hrtim_eecr2;
      HRTIMx->sCommonRegs.EECR3 = hrtim_eecr3;
    }
    break;
    case HRTIM_EVENT_7:
    {
      hrtim_eecr2 &= ~(HRTIM_EECR2_EE7SRC | HRTIM_EECR2_EE7POL | HRTIM_EECR2_EE7SNS);
      hrtim_eecr2 |= (pEventCfg->Source << 6);
      hrtim_eecr2 |= (pEventCfg->Polarity << 6);
      hrtim_eecr2 |= (pEventCfg->Sensitivity << 6);
      hrtim_eecr3 &= ~(HRTIM_EECR3_EE7F);
      hrtim_eecr3 |= (pEventCfg->Filter << 6);
      /* Update the HRTIM registers */
      HRTIMx->sCommonRegs.EECR2 = hrtim_eecr2;
      HRTIMx->sCommonRegs.EECR3 = hrtim_eecr3;
    }
    break;
    case HRTIM_EVENT_8:
    {
      hrtim_eecr2 &= ~(HRTIM_EECR2_EE8SRC | HRTIM_EECR2_EE8POL | HRTIM_EECR2_EE8SNS);
      hrtim_eecr2 |= (pEventCfg->Source << 12);
      hrtim_eecr2 |= (pEventCfg->Polarity << 12);
      hrtim_eecr2 |= (pEventCfg->Sensitivity << 12);
      hrtim_eecr3 &= ~(HRTIM_EECR3_EE8F);
      hrtim_eecr3 |= (pEventCfg->Filter << 12);
      /* Update the HRTIM registers */
      HRTIMx->sCommonRegs.EECR2 = hrtim_eecr2;
      HRTIMx->sCommonRegs.EECR3 = hrtim_eecr3;
    }
    break;
    case HRTIM_EVENT_9:
    {
      hrtim_eecr2 &= ~(HRTIM_EECR2_EE9SRC | HRTIM_EECR2_EE9POL | HRTIM_EECR2_EE9SNS);
      hrtim_eecr2 |= (pEventCfg->Source << 18);
      hrtim_eecr2 |= (pEventCfg->Polarity << 18);
      hrtim_eecr2 |= (pEventCfg->Sensitivity << 18);
      hrtim_eecr3 &= ~(HRTIM_EECR3_EE9F);
      hrtim_eecr3 |= (pEventCfg->Filter << 18);
      /* Update the HRTIM registers */
      HRTIMx->sCommonRegs.EECR2 = hrtim_eecr2;
      HRTIMx->sCommonRegs.EECR3 = hrtim_eecr3;
    }
    break;
    case HRTIM_EVENT_10:
    {
      hrtim_eecr2 &= ~(HRTIM_EECR2_EE10SRC | HRTIM_EECR2_EE10POL | HRTIM_EECR2_EE10SNS);
      hrtim_eecr2 |= (pEventCfg->Source << 24);
      hrtim_eecr2 |= (pEventCfg->Polarity << 24);
      hrtim_eecr2 |= (pEventCfg->Sensitivity << 24);
      hrtim_eecr3 &= ~(HRTIM_EECR3_EE10F);
      hrtim_eecr3 |= (pEventCfg->Filter << 24);
      /* Update the HRTIM registers */
      HRTIMx->sCommonRegs.EECR2 = hrtim_eecr2;
      HRTIMx->sCommonRegs.EECR3 = hrtim_eecr3;
    }
    break;
    default:
    break;
  }
}

/**
  * @brief  Configures a capture unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  CaptureUnit: Capture unit identifier
  * @param  pCaptureCfg: pointer to the compare unit configuration data structure
  * @retval None
  */
void HRTIM_CaptureUnitConfig(HRTIM_TypeDef * HRTIMx,
                             uint32_t TimerIdx,
                             uint32_t CaptureUnit,
                             uint32_t Event)
{
  uint32_t CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_1;
  
  switch (Event)
  {
    case HRTIM_EVENT_1:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_1;
    }
    break;
    case HRTIM_EVENT_2:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_2;
    }
    break;
    case HRTIM_EVENT_3:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_3;
    }
    break;
    case HRTIM_EVENT_4:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_4;
    }
    break;
    case HRTIM_EVENT_5:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_5;
    }
    break;
    case HRTIM_EVENT_6:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_6;
    }
    break;
    case HRTIM_EVENT_7:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_7;
    }
    break;
    case HRTIM_EVENT_8:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_8;
    }
    break;
    case HRTIM_EVENT_9:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_9;
    }
    break;
    case HRTIM_EVENT_10:
    {
      CaptureTrigger = HRTIM_CAPTURETRIGGER_EEV_10;
    }
    break;
    default:
    break;  
    
  }  
  switch (CaptureUnit)
  {
    case HRTIM_CAPTUREUNIT_1:
    {
      HRTIMx->sTimerxRegs[TimerIdx].CPT1xCR = CaptureTrigger;
    }
    break;
    case HRTIM_CAPTUREUNIT_2:
    {
      HRTIMx->sTimerxRegs[TimerIdx].CPT2xCR = CaptureTrigger;
    }
    break;
    default:
    break;  
  }
}

/**
  * @brief  Configures the master timer in waveform mode
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
void  HRTIM_MasterWaveform_Config(HRTIM_TypeDef * HRTIMx, 
                                LEGACY_HRTIM_TimerInitTypeDef * pTimerInit)
{
  uint32_t HRTIM_mcr;
  uint32_t HRTIM_bmcr;
  
  /* Configure master timer */
  HRTIM_mcr = HRTIMx->sMasterRegs.MCR;
  HRTIM_bmcr = HRTIMx->sCommonRegs.BMCR;
  
  /* Enable/Disable the half mode */
  HRTIM_mcr &= ~(HRTIM_MCR_HALF);
  HRTIM_mcr |= pTimerInit->HalfModeEnable;
  
  /* Enable/Disable the timer start upon synchronization event reception */
  HRTIM_mcr &= ~(HRTIM_MCR_SYNCSTRTM);
  HRTIM_mcr |= pTimerInit->StartOnSync;
 
  /* Enable/Disable the timer reset upon synchronization event reception */
  HRTIM_mcr &= ~(HRTIM_MCR_SYNCRSTM);
  HRTIM_mcr |= pTimerInit->ResetOnSync;
  
  /* Enable/Disable the DAC synchronization event generation */
  HRTIM_mcr &= ~(HRTIM_MCR_DACSYNC);
  HRTIM_mcr |= pTimerInit->DACSynchro;
  
  /* Enable/Disable preload mechanism for timer registers */
  HRTIM_mcr &= ~(HRTIM_MCR_PREEN);
  HRTIM_mcr |= pTimerInit->PreloadEnable;
  
  /* Master timer registers update handling */
  HRTIM_mcr &= ~(HRTIM_MCR_BRSTDMA);
  HRTIM_mcr |= (pTimerInit->UpdateGating << 2);
  
  /* Enable/Disable registers update on repetition */
  HRTIM_mcr &= ~(HRTIM_MCR_MREPU);
  HRTIM_mcr |= pTimerInit->RepetitionUpdate;
  
  /* Set the timer burst mode */
  HRTIM_bmcr &= ~(HRTIM_BMCR_MTBM);
  HRTIM_bmcr |= pTimerInit->BurstMode;

  /* Update the HRTIMx registers */
  HRTIMx->sMasterRegs.MCR  = HRTIM_mcr;
  HRTIMx->sCommonRegs.BMCR = HRTIM_bmcr;
  
}

/**
  * @brief  Configures timing unit (timer A to timer E) in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
void HRTIM_TimingUnitWaveform_Config(HRTIM_TypeDef * HRTIMx, 
                                    uint32_t TimerIdx, 
                                    LEGACY_HRTIM_TimerInitTypeDef * pTimerInit)
{
  uint32_t HRTIM_timcr;
  uint32_t HRTIM_bmcr;
  
  /* Configure timing unit */
  HRTIM_timcr = HRTIMx->sTimerxRegs[TimerIdx].TIMxCR;
  HRTIM_bmcr = HRTIMx->sCommonRegs.BMCR;
  
  /* Enable/Disable the half mode */
  HRTIM_timcr &= ~(HRTIM_TIMCR_HALF);
  HRTIM_timcr |= pTimerInit->HalfModeEnable;
  
  /* Enable/Disable the timer start upon synchronization event reception */
  HRTIM_timcr &= ~(HRTIM_TIMCR_SYNCSTRT);
  HRTIM_timcr |= pTimerInit->StartOnSync;
 
  /* Enable/Disable the timer reset upon synchronization event reception */
  HRTIM_timcr &= ~(HRTIM_TIMCR_SYNCRST);
  HRTIM_timcr |= pTimerInit->ResetOnSync;
  
  /* Enable/Disable the DAC synchronization event generation */
  HRTIM_timcr &= ~(HRTIM_TIMCR_DACSYNC);
  HRTIM_timcr |= pTimerInit->DACSynchro;
  
  /* Enable/Disable preload mechanism for timer registers */
  HRTIM_timcr &= ~(HRTIM_TIMCR_PREEN);
  HRTIM_timcr |= pTimerInit->PreloadEnable;
  
  /* Timing unit registers update handling */
  HRTIM_timcr &= ~(HRTIM_TIMCR_UPDGAT);
  HRTIM_timcr |= pTimerInit->UpdateGating;
  
  /* Enable/Disable registers update on repetition */
  HRTIM_timcr &= ~(HRTIM_TIMCR_TREPU);
  if (pTimerInit->RepetitionUpdate == HRTIM_UPDATEONREPETITION_ENABLED)
  {
    HRTIM_timcr |= HRTIM_TIMCR_TREPU;
  }

  /* Set the timer burst mode */
  switch (TimerIdx)
  {
    case HRTIM_TIMERINDEX_TIMER_A:
    {
      HRTIM_bmcr &= ~(HRTIM_BMCR_TABM);
      HRTIM_bmcr |= ( pTimerInit->BurstMode << 1);
    }
    break;
    case HRTIM_TIMERINDEX_TIMER_B:
    {
      HRTIM_bmcr &= ~(HRTIM_BMCR_TBBM);
      HRTIM_bmcr |= ( pTimerInit->BurstMode << 2);
    }
    break;
    case HRTIM_TIMERINDEX_TIMER_C:
    {
      HRTIM_bmcr &= ~(HRTIM_BMCR_TCBM);
      HRTIM_bmcr |= ( pTimerInit->BurstMode << 3);
    }
    break;
    case HRTIM_TIMERINDEX_TIMER_D:
    {
      HRTIM_bmcr &= ~(HRTIM_BMCR_TDBM);
      HRTIM_bmcr |= ( pTimerInit->BurstMode << 4);
    }
    break;
    case HRTIM_TIMERINDEX_TIMER_E:
    {
      HRTIM_bmcr &= ~(HRTIM_BMCR_TEBM);
      HRTIM_bmcr |= ( pTimerInit->BurstMode << 5);
    }
    break;
    default:
    break;
  }
  
  /* Update the HRTIMx registers */
  HRTIMx->sTimerxRegs[TimerIdx].TIMxCR = HRTIM_timcr;
  HRTIMx->sCommonRegs.BMCR = HRTIM_bmcr;
}

#endif /* STM32F334x8 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
