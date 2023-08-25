/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};
  LL_RTC_AlarmTypeDef RTC_AlarmStruct = {0};

  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSE)
  {
    FlagStatus pwrclkchanged = RESET;
    /* Update LSE configuration in Backup Domain control register */
    /* Requires to enable write access to Backup Domain if necessary */
    if (LL_APB1_GRP1_IsEnabledClock (LL_APB1_GRP1_PERIPH_PWR) != 1U)
    {
      /* Enables the PWR Clock and Enables access to the backup domain */
      LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
      pwrclkchanged = SET;
    }
    if (LL_PWR_IsEnabledBkUpAccess () != 1U)
    {
      /* Enable write access to Backup domain */
      LL_PWR_EnableBkUpAccess();
      while (LL_PWR_IsEnabledBkUpAccess () == 0U)
      {
      }
    }
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
    while(LL_RCC_LSE_IsReady() != 1)
    {
    }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    /* Restore clock configuration if changed */
    if (pwrclkchanged == SET)
    {
      LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_PWR);
    }
  }

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* RTC interrupt Init */
  NVIC_SetPriority(RTC_Alarm_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
  NVIC_EnableIRQ(RTC_Alarm_IRQn);

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC and set the Time and Date
  */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  LL_RTC_SetAsynchPrescaler(RTC, 127);
  LL_RTC_SetSynchPrescaler(RTC, 255);

  /** Initialize RTC and set the Time and Date
  */
  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2){

  RTC_TimeStruct.Hours = 0;
  RTC_TimeStruct.Minutes = 0;
  RTC_TimeStruct.Seconds = 0;
  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);
  RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
  RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
  RTC_DateStruct.Day = 1;
  RTC_DateStruct.Year = 0;
  LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_DateStruct);
    LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }

  /** Initialize RTC and set the Time and Date
  */
  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2){

    LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }

  /** Enable the Alarm A
  */
  RTC_AlarmStruct.AlarmTime.Hours = 0;
  RTC_AlarmStruct.AlarmTime.Minutes = 0;
  RTC_AlarmStruct.AlarmTime.Seconds = 0;
  RTC_AlarmStruct.AlarmMask = LL_RTC_ALMA_MASK_NONE;
  RTC_AlarmStruct.AlarmDateWeekDaySel = LL_RTC_ALMA_DATEWEEKDAYSEL_DATE;
  RTC_AlarmStruct.AlarmDateWeekDay = 1;
  LL_RTC_ALMA_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_AlarmStruct);
  LL_RTC_EnableIT_ALRA(RTC);

  /** Initialize RTC and set the Time and Date
  */
  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2){

    LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }

  /** Enable the Alarm B
  */
  RTC_AlarmStruct.AlarmMask = LL_RTC_ALMB_MASK_NONE;
  RTC_AlarmStruct.AlarmDateWeekDaySel = LL_RTC_ALMB_DATEWEEKDAYSEL_DATE;

  LL_RTC_ALMB_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_AlarmStruct);
  LL_RTC_EnableIT_ALRB(RTC);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
