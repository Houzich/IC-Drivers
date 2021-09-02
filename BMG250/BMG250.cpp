/*
 ******************************************************************************
 * @file    BMG250.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include  "Signaling.h"
#include  "BMG250.h"
#include  "BMG250Sensor.h"
#include  "SPI.h"
#include "DEFAULT_SETTINGS.h"
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern uint8_t spi_tx_buffer[];
extern uint8_t spi_rx_buffer[];
namespace BMG250Driver 
{ 
/* Extern variables ----------------------------------------------------------*/
#if !defined(STM32F7)
#define print_rslt(...)
#endif //!defined(STM32F7)
/* Private functions ---------------------------------------------------------*/

BMG250Class BMG250MSP;


BMG250Class::BMG250Class()
{
    Data.flag_receive = false;
}

void BMG250Class::Init()
{
     int8_t rslt;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    gyro.delay_ms = delay_ms;

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */
    gyro.dev_id = 0;
    gyro.read = spi_reg_read;
    gyro.write = spi_reg_write;
    gyro.intf = BMG250_SPI_INTF;
     
    rslt = bmg250_init(&gyro);
    print_rslt(" bmg250_init status", rslt);

    /* Setting the power mode as normal mode */
    gyro.power_mode = BMG250_GYRO_NORMAL_MODE;
    rslt = bmg250_set_power_mode(&gyro);
    print_rslt(" bmg250_set_power_mode status ", rslt);

    /* Read the set configuration from the sensor */
    rslt = bmg250_get_sensor_settings(&gyro_cfg, &gyro);
    print_rslt(" bmg250_get_sensor_settings status ", rslt);

    /* Gyro configuration settings */
    /* Output data rate in hertz */
    gyro_cfg.odr = BMG250_ODR_25HZ;

    /* Range(angular rate measurement rate)of the gyro in degrees per second */
    gyro_cfg.range = BMG250_RANGE_2000_DPS;

    /*Bandwidth settings for digital filter
     * For bandwidth = BMG250_BW_NORMAL_MODE, the gyro data is sampled at equidistant points in
     * the time defined by the ORD.
     * For bandwidth = BMG250_BW_OSR2_MODE, both stages of digital filter are used & data is oversampled
     * with an oversampling rate of 2. The ODR has to be 2 times higher than that of the normal mode.
     * For bandwidth = BMG250_BW_OSR4_MODE, both stages of digital filter are used & data is oversampled
     * with an oversampling rate of 4. The ODR has to be 4 times higher than that of the normal mode.
     */
    gyro_cfg.bw = BMG250_BW_OSR4_MODE;

    /* Set the gyro configurations */
    rslt = bmg250_set_sensor_settings(&gyro_cfg, &gyro);
    print_rslt(" bmg250_set_sensor_settings status ", rslt);
}
void BMG250Class::Reset()
{

}


/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void BMG250Class::delay_ms(uint32_t period_ms)
{
    /* Implement the delay routine according to the target machine */
  osDelay(period_ms);
}
int8_t BMG250Class::spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    spi_tx_buffer[0] = reg_addr;
    memcpy(&spi_tx_buffer[1],reg_data, length);
    SPI_Transfer(spi_tx_buffer, spi_rx_buffer, length + 1, 3);
    return BMG250_OK;
}


int8_t BMG250Class::spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  /* Implement the SPI read routine according to the target machine. */
  spi_tx_buffer[0] = reg_addr|0x80;
  memset(&spi_tx_buffer[1],0xFF, length);
  SPI_Transfer(spi_tx_buffer, spi_rx_buffer, length + 1, 3);
  memcpy(reg_data,&spi_rx_buffer[1], length);
  return BMG250_OK;
}

void BMG250Class::Read_Data()
{
    int8_t rslt;
    static int count_ave = 0;
    static float ave[3][MOVING_AVERAGE_BUFFER_SIZE] = {0};
    /* Reading the gyro data */
    rslt = bmg250_get_sensor_data(BMG250_DATA_SEL, &gyro_data, &gyro);
    if(rslt == BMG250_OK){

    BMG250MSP.Data.gyro[0] = gyro_data.x;
    BMG250MSP.Data.gyro[1] = gyro_data.y;
    BMG250MSP.Data.gyro[2] = gyro_data.z; 
      
    ave[0][count_ave] = BMG250MSP.Data.gyro[0];
    ave[1][count_ave] = BMG250MSP.Data.gyro[1];
    ave[2][count_ave] = BMG250MSP.Data.gyro[2];
      
    count_ave++;
    float temp[3] = {0};
      for(int i = 0; i < MOVING_AVERAGE_BUFFER_SIZE; i++)
      {
        temp[0] += ave[0][i];     
        temp[1] += ave[1][i];     
        temp[2] += ave[2][i];     
      }
    BMG250MSP.Data.gyro_ave[0] = temp[0]/MOVING_AVERAGE_BUFFER_SIZE; 
    BMG250MSP.Data.gyro_ave[1] = temp[1]/MOVING_AVERAGE_BUFFER_SIZE; 
    BMG250MSP.Data.gyro_ave[2] = temp[2]/MOVING_AVERAGE_BUFFER_SIZE;   
    if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) BMG250MSP.Data.flag_receive = true;
    if(count_ave == MOVING_AVERAGE_BUFFER_SIZE) count_ave = 0;        
#if defined(USE_LCD)
    static LCDMsgInQueueTypeDef msg;
    static char str1[20];
    static char str2[20];
    static char str3[20];
    uint32_t mess_cnt = 0;
    mess_cnt  = osMessageQueueGetCount(lcd.queue_in);
    msg.id = LCDMSGID_BMG250_DATA;
    snprintf(str1, sizeof(str1), "%d", BMG250MSP.Data.gyro_ave[0]);        
    snprintf(str2, sizeof(str2), "%d", BMG250MSP.Data.gyro_ave[1]);        
    snprintf(str3, sizeof(str3), "%d", BMG250MSP.Data.gyro_ave[2]);        
    msg.string1 = str1;
    msg.string2 = str2;
    msg.string3 = str3;
    osMessageQueuePut(lcd.queue_in, &msg, NULL, 0);
#endif //defined(USE_LCD)
  }
}


#if defined(STM32F7)
/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void BMG250Class::print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMG250_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMG250_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMG250_E_COM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMG250_E_INVALID_TEMPERATURE)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMG250_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
#endif  //STM32F7


}//namespace BMG250Driver
