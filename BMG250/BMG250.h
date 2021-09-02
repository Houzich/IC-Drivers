/**
  ******************************************************************************
  * File Name          : BMG250.h
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMG250_H
#define __BMG250_H
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
#include "BMG250Sensor.h"
#include "SPI.h"
namespace BMG250Driver 
{ 
/* Exported constants --------------------------------------------------------*/
/**
  * @brief  BMG250 structure
  */
class BMG250Class
{
  class DataClass
  {
    public:
      int16_t gyro[3];
      int16_t gyro_ave[3];
      bool flag_receive;
      DataClass()
      {
        flag_receive = false;
        for(int i = 0; i < 3; i++)
        {
          gyro[i] = 0;
          gyro_ave[i] = 0;
        }
      }
  };
  public:
    BMG250Class();          
    void Init();
    void Reset();
    void Read_Data();
  private:
    static void delay_ms(uint32_t period_ms);
    static int8_t  spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    static int8_t  spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    void  print_rslt(const char api_name[], int8_t rslt);
  public:
    DataClass Data;
    struct bmg250_dev gyro;
    /* Structure to set the gyro config */
    struct bmg250_cfg gyro_cfg;
    /* Structure to store the sensor data */
    struct bmg250_sensor_data gyro_data;
  };
extern BMG250Class BMG250MSP;
} //namespace BMG250Driver 

/* Exported Macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#endif /*__BMG250_H*/
