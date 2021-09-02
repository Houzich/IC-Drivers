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
#include  "LIS2DW12.h"
#include  "LIS2DW12Sensor.h"
#include "DEFAULT_SETTINGS.h"
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern uint8_t spi_tx_buffer[];
extern uint8_t spi_rx_buffer[];
namespace LIS2DW12Driver 
{ 
static axis3bit16_t data_raw_acceleration;
static uint8_t whoamI, rst;
static lis2dw12_ctrl4_int1_pad_ctrl_t  ctrl4_int1_pad;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

LIS2DW12Class LIS2DW12MSP;

LIS2DW12Class::LIS2DW12Class()
{
   Data.flag_receive = false;
}

void LIS2DW12Class::Init()
{
  /*
   *  Initialize mems driver interface
   */
  lis2dw12_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;

  osDelay(500);
  /*
   *  Check device ID
   */
  lis2dw12_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LIS2DW12_ID)
    while(1)
    {
      #if defined(USE_LCD)
      printf("LIS2DW12 Error Check device ID!!!\n");
      #endif //defined(USE_LCD)
      osDelay(1000);
      /* manage here device not found */
    }

  /*
   * Restore default configuration
   */
  lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lis2dw12_reset_get(&dev_ctx, &rst);
  } while (rst);

  /*
   *  Enable Block Data Update
   */
  lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /*
   * Set full scale
   */ 
  //lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);
  lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);

  /*
   * Configure filtering chain
   *
   * Accelerometer - filter path / bandwidth
   */ 
  lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);
  lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);

  /*
   * Configure power mode
   */   
  lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_HIGH_PERFORMANCE);
  //lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit);

  /*
   * Set Output Data Rate
   */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_25Hz);
  
  //нужно немного подождать, потому что первые значения снимает неправильные
  osDelay(100);
}

void LIS2DW12Class::Reset()
{

}

__NO_RETURN void LIS2DW12Class::Thread (void *arg){
  while(1){
    //printf("EXIT LIS2DW12_Thread!!!\n");
    osDelay(1000);
  }
}

void LIS2DW12Class::Read_Data()
{
    static lis2dw12_ctx_t dev_ctx;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
  
    uint8_t reg;
    
    /*
     * Read output only if new value is available
     */
    lis2dw12_flag_data_ready_get(&dev_ctx, &reg);
    static int count_ave = 0;
    static float ave[3][MOVING_AVERAGE_BUFFER_SIZE] = {0};
    if (reg)
    {
      /*
       * Read acceleration data
       */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
      Data.acceleration_mg[0] = data_raw_acceleration.i16bit[0];
      Data.acceleration_mg[1] = data_raw_acceleration.i16bit[1];
      Data.acceleration_mg[2] = data_raw_acceleration.i16bit[2];

      ave[0][count_ave] = Data.acceleration_mg[0];
      ave[1][count_ave] = Data.acceleration_mg[1];
      ave[2][count_ave] = Data.acceleration_mg[2];        
      count_ave++;
      
      float temp[3] = {0};
        for(int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++)
        {
          temp[0] += ave[0][i];     
          temp[1] += ave[1][i];     
          temp[2] += ave[2][i];     
        }
      Data.acceleration_mg_ave[0] = temp[0]/MOVING_AVERAGE_BUFFER_SIZE; 
      Data.acceleration_mg_ave[1] = temp[1]/MOVING_AVERAGE_BUFFER_SIZE; 
      Data.acceleration_mg_ave[2] = temp[2]/MOVING_AVERAGE_BUFFER_SIZE;   
      if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) Data.flag_receive = true;
      if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) count_ave = 0;  
      
     
      //printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
      //        LIS2DW12->Data->acceleration_mg[0], LIS2DW12->Data->acceleration_mg[1], LIS2DW12->Data->acceleration_mg[2]);
      
#if defined(USE_LCD)
      static LCDMsgInQueueTypeDef msg;
      static char str1[20];
      static char str2[20];
      static char str3[20];
      uint32_t mess_cnt = 0;
      mess_cnt  = osMessageQueueGetCount(lcd.queue_in);

      msg.id = LCDMSGID_LIS2DW12_DATA;
      snprintf(str1, sizeof(str1), "%d", Data.acceleration_mg_ave[0]);        
      snprintf(str2, sizeof(str2), "%d", Data.acceleration_mg_ave[1]);        
      snprintf(str3, sizeof(str3), "%d", Data.acceleration_mg_ave[2]);        
      msg.string1 = str1;
      msg.string2 = str2;
      msg.string3 = str3;
      osMessageQueuePut(lcd.queue_in, &msg, NULL, 0);
#endif //defined(USE_LCD)
      
    }

}
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{

//  tx_buffer[0] = reg;
//  memcpy(&tx_buffer[1],bufp, len);
//  SPIBRIDGEMsgQueueTypeDef msg_out;
//  SPIBRIDGEMsgQueueTypeDef msg_in;
//  msg_out.id = 1;
//  msg_out.len = len + 1;
//  msg_out.pbuff_in = rx_buffer;
//  msg_out.pbuff_out = tx_buffer;
//  osMessageQueuePut(LIS2DW12->Sensor->spibridge->queue_in, &msg_out, NULL, osWaitForever);
//  osMessageQueueGet(LIS2DW12->queue_in, &msg_in, NULL, osWaitForever); 
  spi_tx_buffer[0] = reg;
  memcpy(&spi_tx_buffer[1],bufp, len);
  SPI_Transfer(spi_tx_buffer, spi_rx_buffer, len + 1, 1);
  
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
//  tx_buffer[0] = reg|0x80;
//  memset(&tx_buffer[1],0xFF, len);
//  SPIBRIDGEMsgQueueTypeDef msg_out;
//  SPIBRIDGEMsgQueueTypeDef msg_in;
//  msg_out.id = 1;
//  msg_out.len = len + 1;
//  msg_out.pbuff_in = rx_buffer;
//  msg_out.pbuff_out = tx_buffer;
//  osStatus_t status;
//  status = osMessageQueuePut(LIS2DW12->Sensor->spibridge->queue_in, &msg_out, NULL, osWaitForever);
//  if(status != osOK)for(;;)printf("ERROR MessageQueuePut in LIS2DW12 platform_read\n");
//  status = osMessageQueueGet(LIS2DW12->queue_in, &msg_in, NULL, osWaitForever);
//  if(status != osOK)for(;;)printf("ERROR osMessageQueueGet in LIS2DW12 platform_read\n");
//  memcpy(bufp,&rx_buffer[1], len);
  spi_tx_buffer[0] = reg|0x80;
  memset(&spi_tx_buffer[1],0xFF, len);
  SPI_Transfer(spi_tx_buffer, spi_rx_buffer, len + 1, 1);
  memcpy(bufp,&spi_rx_buffer[1], len);
  return 0;
}


}
