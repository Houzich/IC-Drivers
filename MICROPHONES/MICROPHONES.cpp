/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "Signaling.h"
#include "MICROPHONES.h"
#include "Board_LCD.h"
#include "DEFAULT_SETTINGS.h"
#define DEBUG(...)		do {LCD_Debug(__VA_ARGS__); printf(__VA_ARGS__);} while (0)

/* Private macro -------------------------------------------------------------*/
ADC_HandleTypeDef    AdcHandle;
bool adc_start = false;
/* Private variables ---------------------------------------------------------*/
namespace MICROPHONESDriver 
{ 
/* Extern variables ----------------------------------------------------------*/
/* ADC handler declaration */

/* Private functions ---------------------------------------------------------*/

MICROPHONESClass Microphones;


MICROPHONESClass::MICROPHONESClass()
{
    Data.flag_receive = false;
}

/**
  * @brief  Configure the ADC.
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /* Configure the ADC peripheral */
  AdcHandle.Instance          = ADC1;

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV8;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = ENABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled to have continuous conversion */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 2;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;        /* Conversion start not trigged by an external event */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 2;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization Error */
    printf("ADC ERROR Init!!!!\n");
    for(;;);
  }

  /* Configure ADC Temperature Channel */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    printf("ADC ERROR Init!!!!\n");
    for(;;);
  }
  
  /* Configure ADC Temperature Channel */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    printf("ADC ERROR Init!!!!\n");
    for(;;);
  }
//  ADC_MultiModeTypeDef MultiMode;
//  MultiMode.Mode=ADC_MODE_INDEPENDENT;
//  MultiMode.TwoSamplingDelay=ADC_TWOSAMPLINGDELAY_20CYCLES;
//  HAL_ADCEx_MultiModeConfigChannel(&AdcHandle, &MultiMode);
  
}

void MICROPHONESClass::Init()
{
  
  ADC_Config();
  /*##-3- Start the conversion process #######################################*/
  if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&Data.uhADCxConvertedValue[0], 2) != HAL_OK)
  {
    /* Start Conversation Error */
    printf("ADC ERROR Init!!!!\n");
    for(;;);
  }
  
//    GPIO_InitTypeDef          GPIO_InitStruct;

//  /* Enable GPIO clock ****************************************/
//  __HAL_RCC_GPIOA_CLK_ENABLE();

//  /*##-2- Configure peripheral GPIO ##########################################*/
//  /* ADC Channel GPIO pin configuration */
//  GPIO_InitStruct.Pin = GPIO_PIN_6;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  
}
void MICROPHONESClass::Reset()
{

}

void MICROPHONESClass::Start()
{
  Init();
}
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define VREF                  3300
void MICROPHONESClass::convertToVoltage()
{
  static int count_ave = 0;
  static int ave1[MOVING_AVERAGE_BUFFER_SIZE] = {0};
  static int ave2[MOVING_AVERAGE_BUFFER_SIZE] = {0};
  
  if((Data.uhADCxConvertedValue[1] == 0)&&(!adc_start))return;
  adc_start = true;
  Data.Sensor1_Voltage = ((Data.uhADCxConvertedValue[0] * VREF)/MAX_CONVERTED_VALUE);
  Data.Sensor2_Voltage = ((Data.uhADCxConvertedValue[1] * VREF)/MAX_CONVERTED_VALUE);

  ave1[count_ave] = Data.Sensor1_Voltage;
  ave2[count_ave] = Data.Sensor2_Voltage;
  count_ave++;
  int temp1 = 0;
  int temp2 = 0;
  for(int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++)
  {
    temp1 += ave1[i];     
    temp2 += ave2[i];     
  }
  Data.Sensor1_Voltage_ave = temp1/MOVING_AVERAGE_BUFFER_SIZE;
  Data.Sensor2_Voltage_ave = temp2/MOVING_AVERAGE_BUFFER_SIZE;
  
  
  if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) Data.flag_receive = true;
  if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) count_ave = 0; 


 
#if defined(USE_LCD)
  static LCDMsgInQueueTypeDef msg_lcd;
  static char str1[20];
  static char str2[20];
  msg_lcd.id = LCDMSGID_MICROPHONE1_DATA;
  snprintf(str1, sizeof(str1), "%d", Data.Sensor1_Voltage);                    
  msg_lcd.string1 = str1;
  osMessageQueuePut(lcd.queue_in, &msg_lcd, NULL, 0); 
  msg_lcd.id = LCDMSGID_MICROPHONE2_DATA;
  snprintf(str2, sizeof(str2), "%d", Data.Sensor2_Voltage);                    
  msg_lcd.string1 = str2;
  osMessageQueuePut(lcd.queue_in, &msg_lcd, NULL, 0); 
#endif
  
}



}//namespace MICROPHONESClass
