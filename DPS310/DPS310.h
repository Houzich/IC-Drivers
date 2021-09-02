/**
  ******************************************************************************
  * File Name          : DPS310.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DPS310_H
#define __DPS310_H
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#ifdef STM32F7
#include "stm32f7xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h" 
#endif
#if defined(RTE_CMSIS_RTOS2)
  #include "cmsis_os2.h"
#if defined(RTE_CMSIS_RTOS2_RTX5)
  #include "rtx_os.h"
#endif
#endif
#include "DPS310Sensor.h"
namespace DPS310Driver 
{ 
/* Exported constants --------------------------------------------------------*/

/**
  * @brief  DPS310 structure
  */
class DPS310Class
{
  class DataClass
  {
    public:
      uint8_t pressureCount;
      int32_t pressure;
      uint8_t temperatureCount;
      int32_t temperature;
      int32_t pressure_ave;
      bool flag_receive;
      DataClass()
      {
        pressureCount = 1;
        temperatureCount = 1;
        pressure_ave = 0;
        flag_receive = false;
      }
  };

  public:
    DPS310Class();          
    void Init();
    void Reset();
    void Read_Data();
  
    DPS310SensorClass Sensor;
    DataClass Data;
  };

  extern DPS310Class DPS310MSP;
} //namespace DPS310Driver 

/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#endif /*__DPS310_H*/
