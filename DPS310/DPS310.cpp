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
#include  "Signaling.h"
#include "DPS310.h"
#include "DPS310Sensor.h"
#include "DEFAULT_SETTINGS.h"
#if defined(DEBUG_DPS310_SDO)
#define DEBUG_SDO(...)		printf(__VA_ARGS__)
#else
#define DEBUG_SDO(...)
#endif
#if defined(DEBUG_DPS310_LCD) && defined(USE_LCD)
#define DEBUG_LCD(...)		LCD_Debug(__VA_ARGS__)
#else
#define DEBUG_LCD(...)
#endif
#if defined(DEBUG_DPS310_SDO) && defined(DEBUG_DPS310_LCD) && defined(USE_LCD)
#define DEBUG_ALL(...)		do {LCD_Debug(__VA_ARGS__); printf(__VA_ARGS__);} while (0)
#else
#define DEBUG_ALL(...)
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
namespace DPS310Driver 
{ 
/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

DPS310Class DPS310MSP;


DPS310Class::DPS310Class()
{
    Data.flag_receive = false;
}

void DPS310Class::Init()
{
  START:  
  //osDelay(1000);
  DPS310MSP.Sensor.begin();
  int16_t temp_mr = 2;
  int16_t temp_osr = 2;
  int16_t prs_mr = 1; //2-times
  int16_t prs_osr = 7; //128 times (High Precision).
  int16_t ret = DPS310MSP.Sensor.startMeasurePressureCont(prs_mr, prs_osr);


  if (ret != 0)
  {
    goto START;
    //printf("DPS310 Init FAILED! ret = %d\n", ret);
  }
  else
  {
    //printf("DPS310 Init complete!\n");
  }
}
void DPS310Class::Reset()
{

}

void DPS310Class::Read_Data()
{
    static int count_ave = 0;
    static int ave[MOVING_AVERAGE_BUFFER_SIZE] = {0};
    //osDelay(100);

      
    //This function writes the results of continuous measurements to the arrays given as parameters
    //The parameters temperatureCount and pressureCount should hold the sizes of the arrays temperature and pressure when the function is called
    //After the end of the function, temperatureCount and pressureCount hold the numbers of values written to the arrays
    //Note: The Dps310 cannot save more than 32 results. When its result buffer is full, it won't save any new measurement results
    int16_t ret = DPS310MSP.Sensor.getContResults(&DPS310MSP.Data.temperature, DPS310MSP.Data.temperatureCount, &DPS310MSP.Data.pressure, DPS310MSP.Data.pressureCount);

    if (ret != 0)
    {
      //printf("DPS310 ERROR READ!!!\n");
    }
    else
    {
      //DEBUG_ALL("DPS310 pressure: %d Pascal\n", DPS310MSP.Data.pressure);
    
      ave[count_ave++] = DPS310MSP.Data.pressure;
      int temp = 0;
      for(int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++)
      {
        temp += ave[i];     
      }
      DPS310MSP.Data.pressure_ave = temp/MOVING_AVERAGE_BUFFER_SIZE;
      if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) DPS310MSP.Data.flag_receive = true;
      if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) count_ave = 0; 
      
#if defined(USE_LCD)
      static LCDMsgInQueueTypeDef msg;
      static char str1[20];
      static char str2[20];
      static char str3[20];
      msg.id = LCDMSGID_DPS310_DATA;
      snprintf(str1, sizeof(str1), "%d", DPS310MSP.Data.pressure_ave);        
      //snprintf(str2, sizeof(str2), "%d", DPS310MSP.Data.temperature);              
      msg.string1 = str1;
      msg.string2 = str2;
      osMessageQueuePut(lcd.queue_in, &msg, NULL, 0); 
#endif //defined(USE_LCD)      
    }
}
}//namespace DPS310Driver
