/**
  ******************************************************************************
  * File Name          : LIS2DW12.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS2DW12_H
#define __LIS2DW12_H
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
#include "LIS2DW12Sensor.h"
namespace LIS2DW12Driver 
{ 
/* Exported constants --------------------------------------------------------*/
/**
* @brief  LIS2DW12 status
*/
typedef enum __LIS2DW12StatusEnumTypeDef{
	LIS2DW12Status_READY = 0x00,
}LIS2DW12Status;
/**
* @brief  LIS2DW12 error
*/
typedef enum __LIS2DW12ErrorEnumTypeDef{
	LIS2DW12Error_OK = 0x00,
}LIS2DW12Error;




/**
  * @brief  LIS2DW12 structure
  */
class LIS2DW12Class
{
  class DataClass
  {
    public:
//      bool event_tag[3] = {0};
//      bool event_double_tag[3] = {0};
//      int tap_threshold[3] = {0};
      int16_t acceleration_mg[3];
      int16_t acceleration_mg_ave[3];
      bool flag_receive;
      DataClass()
      {
        for(int i = 0; i < 3; i++)
        {
          acceleration_mg_ave[i] = 0;
          acceleration_mg[i] = 0;
        }

        flag_receive = false;
      }
  };
  
  public:      
//    osThreadId_t ThreadID;
//    uint64_t *pTaskStack;
//    const osThreadAttr_t *pThreadAttr;
  
  public:
    LIS2DW12Class();          
    void Init();
    void Reset();
    __NO_RETURN void Thread (void *arg);
    void Read_Data();
  
    LIS2DW12Sensor Sensor;
    DataClass Data;
    LIS2DW12Status status;
    LIS2DW12Error error;
  };

extern LIS2DW12Class LIS2DW12MSP;
} //namespace LIS2DW12Driver 
/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#endif /*__LIS2DW12_H*/
