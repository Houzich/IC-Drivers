/**
  ******************************************************************************
  * File Name          : MICROPHONES.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MICROPHONES_H
#define __MICROPHONES_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#if defined(RTE_CMSIS_RTOS2)
  #include "cmsis_os2.h"
#if defined(RTE_CMSIS_RTOS2_RTX5)
  #include "rtx_os.h"
#endif
#endif

namespace MICROPHONESDriver 
{ 
/* Exported constants --------------------------------------------------------*/
/**
  * @brief  MICROPHONES structure
  */
class MICROPHONESClass
{
  class DataClass
  {
    public:
      uint32_t uhADCxConvertedValue[2];
      uint32_t Sensor1_Voltage;
      uint32_t Sensor2_Voltage;
      uint32_t Sensor1_Voltage_ave;
      uint32_t Sensor2_Voltage_ave;
      bool flag_receive;
      DataClass()
      {
        flag_receive = false;
      }
  };
  

  public:
    MICROPHONESClass();          
    void Init();
    void Reset();
    void Start();
    void convertToVoltage();
  
    DataClass Data;
  };
extern MICROPHONESClass Microphones;
} //namespace MICROPHONESDriver 

/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /*__MICROPHONES_H*/
