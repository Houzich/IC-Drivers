/**
 ******************************************************************************
 * @file    DPS310Sensor.h
 * @author  CLab
 * @version V1.0.0
 * @date    15 November 2018
 * @brief   Abstract Class of an DPS310 Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __DPS310SENSOR_H
#define __DPS310SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include "SPI.h"
#include "dps310_consts.h"
/* Defines -------------------------------------------------------------------*/
/* Typedefs ------------------------------------------------------------------*/
/* Class Declaration ---------------------------------------------------------*/


/**
  * @brief  DPS310 structure
  */
class DPS310SensorClass
{
	public:
//    uint8_t tx_buffer[50];
//    uint8_t rx_buffer[50];
  
		//constructor
		DPS310SensorClass(void);
		//destructor
		~DPS310SensorClass(void);
		//begin
		void begin();
		//end
		void end(void);

		//general
		uint8_t getProductId(void);
		uint8_t getRevisionId(void);

		//Idle Mode
		int16_t standby(void);

		//Command Mode
		int16_t measureTempOnce(int32_t &result);
		int16_t measureTempOnce(int32_t &result, uint8_t oversamplingRate);
		int16_t startMeasureTempOnce(void);
		int16_t startMeasureTempOnce(uint8_t oversamplingRate);
		int16_t measurePressureOnce(int32_t &result);
		int16_t measurePressureOnce(int32_t &result, uint8_t oversamplingRate);
		int16_t startMeasurePressureOnce(void);
		int16_t startMeasurePressureOnce(uint8_t oversamplingRate);
		int16_t getSingleResult(int32_t &result);

		//Background Mode
		int16_t startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate);
		int16_t startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate);
		int16_t startMeasureBothCont(uint8_t tempMr, uint8_t tempOsr, uint8_t prsMr, uint8_t prsOsr);
		int16_t getContResults(int32_t *tempBuffer, uint8_t &tempCount, int32_t *prsBuffer, uint8_t &prsCount);

		//Interrupt Control
		int16_t setInterruptPolarity(uint8_t polarity);
		int16_t setInterruptSources(uint8_t fifoFull, uint8_t tempReady, uint8_t prsReady);
		int16_t getIntStatusFifoFull(void);
		int16_t getIntStatusTempReady(void);
		int16_t getIntStatusPrsReady(void);

		//function to fix a hardware problem on some devices
		int16_t correctTemp(void);

		//scaling factor table
		//for initialization see ifx_dps310.cpp
		static const int32_t scaling_facts[DPS310__NUM_OF_SCAL_FACTS];


	private:

		//enum for operating mode
		enum Mode
		{
			IDLE=0x00,
			CMD_PRS=0x01,
			CMD_TEMP=0x02,
			INVAL_OP_CMD_BOTH=0x03,		//invalid
			INVAL_OP_CONT_NONE=0x04, 	//invalid
			CONT_PRS=0x05,
			CONT_TMP=0x06,
			CONT_BOTH=0x07
		};
		Mode m_opMode;

		//flags
		uint8_t m_initFail;
		uint8_t m_productID;
		uint8_t m_revisionID;

		//settings
		uint8_t m_tempMr;
		uint8_t m_tempOsr;
		uint8_t m_prsMr;
		uint8_t m_prsOsr;
		uint8_t m_tempSensor;

		//compensation coefficients
		int32_t m_c0Half;
		int32_t m_c1;
		int32_t m_c00;
		int32_t m_c10;
		int32_t m_c01;
		int32_t m_c11;
		int32_t m_c20;
		int32_t m_c21;
		int32_t m_c30;
		//last measured scaled temperature
		//(necessary for pressure compensation)
		double m_lastTempScal;

		//measurement
		void init(void);
		int16_t readcoeffs(void);
		int16_t setOpMode(uint8_t background, uint8_t temperature, uint8_t pressure);
		int16_t setOpMode(uint8_t opMode);
		int16_t configTemp(uint8_t temp_mr, uint8_t temp_osr);
		int16_t configPressure(uint8_t prs_mr, uint8_t prs_osr);
		uint16_t calcBusyTime(uint16_t temp_rate, uint16_t temp_osr);
		int16_t getTemp(int32_t *result);
		int16_t getPressure(int32_t *result);
		int16_t getFIFOvalue(int32_t *value);
		int32_t calcTemp(int32_t raw);
		int32_t calcPressure(int32_t raw);

		//bus specific
		int16_t readByte(uint8_t regAddress);
		int16_t readByteSPI(uint8_t regAddress);
		int16_t readBlock(uint8_t regAddress, uint8_t length, uint8_t *buffer);
		int16_t readBlockSPI(uint8_t regAddress, uint8_t length, uint8_t *readbuffer);
		int16_t writeByte(uint8_t regAddress, uint8_t data);
		int16_t writeByte(uint8_t regAddress, uint8_t data, uint8_t check);
		int16_t writeByteSpi(uint8_t regAddress, uint8_t data, uint8_t check);
		int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift);
		int16_t writeByteBitfield(uint8_t data, uint8_t regAddress, uint8_t mask, uint8_t shift, uint8_t check);
		int16_t readByteBitfield(uint8_t regAddress, uint8_t mask, uint8_t shift);

};

   

#endif //__DPS310SENSOR_H
