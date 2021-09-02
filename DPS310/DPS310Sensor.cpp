/**
 ******************************************************************************
 * @file    DPS310Sensor.cpp
 * @author  Anton
 * @version V1.0.0
 * @date    
 * @brief   
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "DPS310Sensor.h"
#include "DPS310.h"
#include "Core.h"

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
/* Class Implementation ------------------------------------------------------*/
extern uint8_t spi_tx_buffer[];
extern uint8_t spi_rx_buffer[];

const int32_t DPS310SensorClass::scaling_facts[DPS310__NUM_OF_SCAL_FACTS]
	= {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
/**
 * Standard Constructor
 */
DPS310SensorClass::DPS310SensorClass(void)
{
	//assume that initialization has failed before it has been done
	m_initFail = 0U;
  m_opMode = IDLE;
}

/**
 * Standard Destructor
 */
DPS310SensorClass::~DPS310SensorClass(void)
{
	end();
}

/**
 * Standard SPI begin function
 *
 * &bus: 			SPI bus which connects MC to Dps310
 * chipSelect: 		Number of the CS line for the Dps310
 * threeWire: 		1 if Dps310 is connected with 3-wire SPI
 * 					0 if Dps310 is connected with 4-wire SPI (standard)
 */
void DPS310SensorClass::begin()
{
	//this flag will show if the initialization was successful
	m_initFail = 0U;
	osDelay(50);		//startup time of Dps310
	init();
}

/**
 * End function for Dps310
 * Sets the sensor to idle mode
 */
void DPS310SensorClass::end(void)
{
	standby();
}


////////		Declaration of other public functions starts here			////////


/**
 * returns the Product ID of the connected Dps310 sensor
 */
uint8_t DPS310SensorClass::getProductId(void)
{
	return m_productID;
}

/**
 * returns the Revision ID of the connected Dps310 sensor
 */
uint8_t DPS310SensorClass::getRevisionId(void)
{
	return m_revisionID;
}

/**
 * Sets the Dps310 to standby mode
 *
 * returns:		0 on success
 * 				-2 if object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::standby(void)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//set device to idling mode
	int16_t ret = setOpMode(IDLE);
	if(ret != DPS310__SUCCEEDED)
	{
		return ret;
	}
	//flush the FIFO
	ret = writeByteBitfield(1U, DPS310__REG_INFO_FIFO_FL);
	if(ret < 0)
	{
		return ret;
	}
	//disable the FIFO
	ret = writeByteBitfield(0U, DPS310__REG_INFO_FIFO_EN);
	return ret;
}


/**
 * performs one temperature measurement and writes result to the given address
 *
 * &result:		reference to a 32-Bit signed Integer value where the result will be written
 * 				It will not be written if result==NULL
 * returns: 	0 on success
 * 				-4 if the DPS310 is could not finish its measurement in time
 * 				-3 if the DPS310 is already busy
 * 				-2 if the object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::measureTempOnce(int32_t &result)
{
	return measureTempOnce(result, m_tempOsr);
}

/**
 * performs one temperature measurement and writes result to the given address
 * the desired precision can be set with oversamplingRate
 *
 * &result:				reference to a 32-Bit signed Integer where the result will be written
 * 						It will not be written if result==NULL
 * oversamplingRate: 	a value from 0 to 7 that decides about the precision
 * 						of the measurement
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements and combine the results
 * returns: 			0 on success
 * 						-4 if the DPS310 is could not finish its measurement in time
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 */
int16_t DPS310SensorClass::measureTempOnce(int32_t &result, uint8_t oversamplingRate)
{
	//Start measurement
	int16_t ret = startMeasureTempOnce(oversamplingRate);
	if(ret!=DPS310__SUCCEEDED)
	{
		return ret;
	}

	//wait until measurement is finished
	osDelay(calcBusyTime(0U, m_tempOsr)/DPS310__BUSYTIME_SCALING);
	osDelay(DPS310__BUSYTIME_FAILSAFE);

	ret = getSingleResult(result);
	if(ret!=DPS310__SUCCEEDED)
	{
		standby();
	}
	return ret;
}

/**
 * starts a single temperature measurement
 *
 * returns: 	0 on success
 * 				-3 if the DPS310 is already busy
 * 				-2 if the object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::startMeasureTempOnce(void)
{
	return startMeasureTempOnce(m_tempOsr);
}

/**
 * starts a single temperature measurement
 * The desired precision can be set with oversamplingRate
 *
 * oversamplingRate: 	a value from 0 to 7 that decides about the precision
 * 						of the measurement
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements and combine the results
 * returns: 			0 on success
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 */
int16_t DPS310SensorClass::startMeasureTempOnce(uint8_t oversamplingRate)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if(m_opMode!=IDLE)
	{
		return DPS310__FAIL_TOOBUSY;
	}

	if(oversamplingRate!=m_tempOsr)
	{
		//configuration of oversampling rate
		if(configTemp(0U, oversamplingRate) != DPS310__SUCCEEDED)
		{
			return DPS310__FAIL_UNKNOWN;
		}
	}

	//set device to temperature measuring mode
	return setOpMode(0U, 1U, 0U);
}

/**
 * performs one pressure measurement and writes result to the given address
 *
 * &result:		reference to a 32-Bit signed Integer value where the result will be written
 * 				It will not be written if result==NULL
 * returns: 	0 on success
 * 				-4 if the DPS310 is could not finish its measurement in time
 * 				-3 if the DPS310 is already busy
 * 				-2 if the object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::measurePressureOnce(int32_t &result)
{
	return measurePressureOnce(result, m_prsOsr);
}

/**
 * performs one pressure measurement and writes result to the given address
 * the desired precision can be set with oversamplingRate
 *
 * &result:				reference to a 32-Bit signed Integer where the result will be written
 * 						It will not be written if result==NULL
 * oversamplingRate: 	a value from 0 to 7 that decides about the precision
 * 						of the measurement
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements and combine the results
 * returns: 			0 on success
 * 						-4 if the DPS310 is could not finish its measurement in time
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 */
int16_t DPS310SensorClass::measurePressureOnce(int32_t &result, uint8_t oversamplingRate)
{
	//start the measurement
	int16_t ret = startMeasurePressureOnce(oversamplingRate);
	if(ret != DPS310__SUCCEEDED)
	{
		return ret;
	}

	//wait until measurement is finished
	osDelay(calcBusyTime(0U, m_prsOsr)/DPS310__BUSYTIME_SCALING);
	osDelay(DPS310__BUSYTIME_FAILSAFE);

	ret = getSingleResult(result);
	if(ret!=DPS310__SUCCEEDED)
	{
		standby();
	}
	return ret;
}

/**
 * starts a single pressure measurement
 *
 * returns: 	0 on success
 * 				-3 if the DPS310 is already busy
 * 				-2 if the object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::startMeasurePressureOnce(void)
{
	return startMeasurePressureOnce(m_prsOsr);
}

/**
 * starts a single pressure measurement
 * The desired precision can be set with oversamplingRate
 *
 * oversamplingRate: 	a value from 0 to 7 that decides about the precision
 * 						of the measurement
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements and combine the results
 * returns: 			0 on success
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 */
int16_t DPS310SensorClass::startMeasurePressureOnce(uint8_t oversamplingRate)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if(m_opMode != IDLE)
	{
		return DPS310__FAIL_TOOBUSY;
	}
	//configuration of oversampling rate, lowest measure rate to avoid conflicts
	if(oversamplingRate != m_prsOsr)
	{
		if(configPressure(0U, oversamplingRate))
		{
			return DPS310__FAIL_UNKNOWN;
		}
	}
	//set device to pressure measuring mode
	return setOpMode(0U, 0U, 1U);
}

/**
 * gets the result a single temperature or pressure measurement in °C or Pa
 *
 * &result:		reference to a 32-Bit signed Integer value where the result will be written
 * returns: 	0 on success
 * 				-4 if the DPS310 is still busy
 * 				-3 if the DPS310 is not in command mode
 * 				-2 if the object initialization failed
 * 				-1 on other fail
 */
int16_t DPS310SensorClass::getSingleResult(int32_t &result)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}

	//read finished bit for current opMode
	int16_t rdy;
	switch(m_opMode)
	{
	case CMD_TEMP: 	//temperature
		rdy = readByteBitfield(DPS310__REG_INFO_TEMP_RDY);
		break;
	case CMD_PRS: 	//pressure
		rdy = readByteBitfield(DPS310__REG_INFO_PRS_RDY);
		break;
	default: 	//DPS310 not in command mode
		return DPS310__FAIL_TOOBUSY;
	}

	//read new measurement result
	switch(rdy)
	{
	case DPS310__FAIL_UNKNOWN: 	//could not read ready flag
		return DPS310__FAIL_UNKNOWN;
	case 0: 						//ready flag not set, measurement still in progress
		return DPS310__FAIL_UNFINISHED;
	case 1: 						//measurement ready, expected case
		DPS310SensorClass::Mode oldMode = m_opMode;
		m_opMode = IDLE;				//opcode was automatically reseted by DPS310
		switch(oldMode)
		{
		case CMD_TEMP: 	//temperature
			return getTemp(&result);		//get and calculate the temperature value
		case CMD_PRS: 	//pressure
			return getPressure(&result);	//get and calculate the pressure value
		default:
			return DPS310__FAIL_UNKNOWN;	//should already be filtered above
		}
	}
	return DPS310__FAIL_UNKNOWN;
}

/**
 * starts a continuous temperature measurement
 * The desired precision can be set with oversamplingRate
 * The desired number of measurements per second can be set with measureRate
 *
 * measureRate: 		a value from 0 to 7 that decides about
 * 						the number of measurements per second
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements per second
 * oversamplingRate: 	a value from 0 to 7 that decides about
 * 						the precision of the measurements
 * 						If this value equals m, the DPS310 will perform
 * 						2^m internal measurements and combine the results
 * 						to one more exact measurement
 * returns: 			0 on success
 * 						-4 if measureRate or oversamplingRate is too high
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 * 	NOTE: 				If measure rate is n and oversampling rate is m,
 * 						the DPS310 performs 2^(n+m) internal measurements per second.
 * 						The DPS310 cannot operate with high precision and high speed
 * 						at the same time.
 * 						Consult the datasheet for more information.
 */
int16_t DPS310SensorClass::startMeasureTempCont(uint8_t measureRate, uint8_t oversamplingRate)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if(m_opMode != IDLE)
	{
		return DPS310__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if(calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return DPS310__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if(configTemp(measureRate, oversamplingRate))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//enable result FIFO
	if(writeByteBitfield(1U, DPS310__REG_INFO_FIFO_EN))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if(setOpMode(1U, 1U, 0U))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	return DPS310__SUCCEEDED;
}


/**
 * starts a continuous temperature measurement
 * The desired precision can be set with oversamplingRate
 * The desired number of measurements per second can be set with measureRate
 *
 * measureRate: 		a value from 0 to 7 that decides about
 * 						the number of measurements per second
 * 						If this value equals n, the DPS310 will perform
 * 						2^n measurements per second
 * oversamplingRate: 	a value from 0 to 7 that decides about the precision
 * 						of the measurements
 * 						If this value equals m, the DPS310 will perform
 * 						2^m internal measurements
 * 						and combine the results to one more exact measurement
 * returns: 			0 on success
 * 						-4 if measureRate or oversamplingRate is too high
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 * 	NOTE: 				If measure rate is n and oversampling rate is m,
 * 						the DPS310 performs 2^(n+m) internal measurements per second.
 * 						The DPS310 cannot operate with high precision and high speed
 * 						at the same time.
 * 						Consult the datasheet for more information.
 */
int16_t DPS310SensorClass::startMeasurePressureCont(uint8_t measureRate, uint8_t oversamplingRate)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if(m_opMode != IDLE)
	{
		return DPS310__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if(calcBusyTime(measureRate, oversamplingRate) >= DPS310__MAX_BUSYTIME)
	{
		return DPS310__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if(configPressure(measureRate, oversamplingRate))
		return DPS310__FAIL_UNKNOWN;
	//enable result FIFO
	if(writeByteBitfield(1U, DPS310__REG_INFO_FIFO_EN))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if(setOpMode(1U, 0U, 1U))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	return DPS310__SUCCEEDED;
}

/**
 * starts a continuous temperature and pressure measurement
 * The desired precision can be set with tempOsr and prsOsr
 * The desired number of measurements per second can be set with tempMr and prsMr
 *
 * tempMr				measure rate for temperature
 * tempOsr				oversampling rate for temperature
 * prsMr				measure rate for pressure
 * prsOsr				oversampling rate for pressure
 * returns: 			0 on success
 * 						-4 if precision or speed is too high
 * 						-3 if the DPS310 is already busy
 * 						-2 if the object initialization failed
 * 						-1 on other fail
 * 	NOTE: 				High precision and speed for both temperature and pressure
 * 						can not be reached at the same time.
 * 						Estimated time for temperature and pressure measurement
 * 						is the sum of both values.
 * 						This sum must not be more than 1 second.
 * 						Consult the datasheet for more information.
 */
int16_t DPS310SensorClass::startMeasureBothCont(uint8_t tempMr,
										 uint8_t tempOsr,
										 uint8_t prsMr,
										 uint8_t prsOsr)
{
	//abort if initialization failed
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in idling mode
	if(m_opMode!=IDLE)
	{
		return DPS310__FAIL_TOOBUSY;
	}
	//abort if speed and precision are too high
	if(calcBusyTime(tempMr, tempOsr) + calcBusyTime(prsMr, prsOsr)>=DPS310__MAX_BUSYTIME)
	{
		return DPS310__FAIL_UNFINISHED;
	}
	//update precision and measuring rate
	if(configTemp(tempMr, tempOsr))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//update precision and measuring rate
	if(configPressure(prsMr, prsOsr))
		return DPS310__FAIL_UNKNOWN;
	//enable result FIFO
	if(writeByteBitfield(1U, DPS310__REG_INFO_FIFO_EN))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//Start measuring in background mode
	if(setOpMode(1U, 1U, 1U))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	return DPS310__SUCCEEDED;
}

/**
 * Gets the results from continuous measurements and writes them to given arrays
 *
 * *tempBuffer: 	The start address of the buffer where the temperature results
 * 					are written
 * 					If this is NULL, no temperature results will be written out
 * &tempCount:		This has to be a reference to a number which contains
 * 					the size of the buffer for temperature results.
 * 					When the function ends, it will contain
 * 					the number of bytes written to the buffer
 * *prsBuffer: 		The start address of the buffer where the pressure results
 * 					are written
 * 					If this is NULL, no pressure results will be written out
 * &prsCount:		This has to be a reference to a number which contains
 * 					the size of the buffer for pressure results.
 * 					When the function ends, it will contain
 * 					the number of bytes written to the buffer
 * returns:			0 on success
 * 					-3 if DPS310 is not in background mode
 * 					-2 if the object initialization failed
 * 					-1 on other fail
 */
int16_t DPS310SensorClass::getContResults(int32_t *tempBuffer,
								   uint8_t &tempCount,
								   int32_t *prsBuffer,
								   uint8_t &prsCount)
{
	if(m_initFail)
	{
    DEBUG_ALL("DPS310__FAIL_INIT_FAILED\n");
		return DPS310__FAIL_INIT_FAILED;
	}
	//abort if device is not in background mode
	if(!(m_opMode & INVAL_OP_CONT_NONE))
	{
    DEBUG_ALL("DPS310__FAIL_TOOBUSY\n");
		return DPS310__FAIL_TOOBUSY;
	}

	//prepare parameters for buffer length and count
	uint8_t tempLen = tempCount;
	uint8_t prsLen = prsCount;
	tempCount = 0U;
	prsCount = 0U;

	//while FIFO is not empty
	while(readByteBitfield(DPS310__REG_INFO_FIFO_EMPTY) == 0)
	{
		int32_t result = 0;
		//read next result from FIFO
		int16_t type = getFIFOvalue(&result);
		switch(type)
		{
		case 0: //temperature
			//calculate compensated pressure value
			result = calcTemp(result);
			//if buffer exists and is not full
			//write result to buffer and increase temperature result counter
			if(tempBuffer != NULL)
			{
				if(tempCount<tempLen)
				{
					tempBuffer[tempCount++] = result;
				}
			}
			break;
		case 1: //pressure
			//calculate compensated pressure value
			result = calcPressure(result);
			//if buffer exists and is not full
			//write result to buffer and increase pressure result counter
			if(prsBuffer != NULL)
			{
				if(prsCount<prsLen)
				{
					prsBuffer[prsCount++] = result;
				}
			}
			break;
		case -1: //read failed
      //return DPS310__FAIL_UNFINISHED;
			break;	//continue while loop
					//if connection failed permanently,
					//while condition will become false
					//if read failed only once, loop will try again
		}
	}
	return DPS310__SUCCEEDED;
}

/**
 * Sets the active state of the Interrupt pin
 *
 * polarity: 	If this is 0, the interrupt pin of the Dps310 will be low-active
 * 				If this is 1, the interrupt pin of the Dps310 will be high-active
 * returns:		0 on success,
 * 				-1 on fail
 */
int16_t DPS310SensorClass::setInterruptPolarity(uint8_t polarity)
{
	return writeByteBitfield(polarity, DPS310__REG_INFO_INT_HL);
}

/**
 * Sets the sources that are able to cause interrupts
 *
 * fifoFull: 	if this is 1, an interrupt will be generated
 * 				when the FIFO is full
 * 				if this is 0, the FIFO will not generate any interrupts
 * tempReady: 	if this is 1, an interrupt will be generated
 * 				when a temperature measurement is finished
 * 				if this is 0, no interrupt will be generated
 * 				after finishing a temperature measurement
 * prsReady: 	if this is 1, an interrupt will be generated
 * 				when a pressure measurement is finished
 * 				if this is 0, no interrupt will be generated
 * 				after finishing a pressure measurement
 * returns: 	0 on success, -1 on fail
 */
int16_t DPS310SensorClass::setInterruptSources(uint8_t fifoFull, uint8_t tempReady, uint8_t prsReady)
{
	//mask parameters
	fifoFull &= DPS310__REG_MASK_INT_EN_FIFO >> DPS310__REG_SHIFT_INT_EN_FIFO;
	tempReady &= DPS310__REG_MASK_INT_EN_TEMP >> DPS310__REG_SHIFT_INT_EN_TEMP;
	prsReady &= DPS310__REG_MASK_INT_EN_PRS >> DPS310__REG_SHIFT_INT_EN_PRS;
	//read old value from register
	int16_t regData = readByte(DPS310__REG_ADR_INT_EN_FIFO);
	if(regData <0)
	{
		return DPS310__FAIL_UNKNOWN;
	}
	uint8_t toWrite = (uint8_t)regData;
	//update FIFO enable bit
	toWrite &= ~DPS310__REG_MASK_INT_EN_FIFO;	//clear bit
	toWrite |= fifoFull << DPS310__REG_SHIFT_INT_EN_FIFO;	//set new bit
	//update TempReady enable bit
	toWrite &= ~DPS310__REG_MASK_INT_EN_TEMP;
	toWrite |= tempReady << DPS310__REG_SHIFT_INT_EN_TEMP;
	//update PrsReady enable bit
	toWrite &= ~DPS310__REG_MASK_INT_EN_PRS;
	toWrite |= prsReady << DPS310__REG_SHIFT_INT_EN_PRS;
	//write updated value to register
	return writeByte(DPS310__REG_ADR_INT_EN_FIFO, toWrite);
}

/**
 * Gets the interrupt status flag of the FIFO
 *
 * Returns: 	1 if the FIFO is full and caused an interrupt
 * 				0 if the FIFO is not full or FIFO interrupt is disabled
 * 				-1 on fail
 */
int16_t DPS310SensorClass::getIntStatusFifoFull(void)
{
	return readByteBitfield(DPS310__REG_INFO_INT_FLAG_FIFO);
}

/**
 * Gets the interrupt status flag that indicates a finished temperature measurement
 *
 * Returns: 	1 if a finished temperature measurement caused an interrupt
 * 				0 if there is no finished temperature measurement
 * 					or interrupts are disabled
 * 				-1 on fail
 */
int16_t DPS310SensorClass::getIntStatusTempReady(void)
{
	return readByteBitfield(DPS310__REG_INFO_INT_FLAG_TEMP);
}

/**
 * Gets the interrupt status flag that indicates a finished pressure measurement
 *
 * Returns: 	1 if a finished pressure measurement caused an interrupt
 * 				0 if there is no finished pressure measurement
 * 					or interrupts are disabled
 * 				-1 on fail
 */
int16_t DPS310SensorClass::getIntStatusPrsReady(void)
{
	return readByteBitfield(DPS310__REG_INFO_INT_FLAG_PRS);
}

/**
 * Function to fix a hardware problem on some devices
 * You have this problem if you measure a temperature which is too high (e.g. 60°C when temperature is around 20°C)
 * Call correctTemp() directly after begin() to fix this issue
 */
int16_t DPS310SensorClass::correctTemp(void)
{
	if(m_initFail)
	{
		return DPS310__FAIL_INIT_FAILED;
	}
	writeByte(0x0E, 0xA5);
	writeByte(0x0F, 0x96);
	writeByte(0x62, 0x02);
	writeByte(0x0E, 0x00);
	writeByte(0x0F, 0x00);
	
	//perform a first temperature measurement (again)
	//the most recent temperature will be saved internally
	//and used for compensation when calculating pressure
	int32_t trash;
	measureTempOnce(trash);
	
	return DPS310__SUCCEEDED;
}



//////// 	Declaration of private functions starts here	////////


/**
 * Initializes the sensor.
 * This function has to be called from begin()
 * and requires a valid bus initialization.
 */
void DPS310SensorClass::init(void)
{
	int16_t prodId = readByteBitfield(DPS310__REG_INFO_PROD_ID);
	if(prodId != DPS310__PROD_ID)
	{
    DEBUG_ALL("DPS310 ERROR READ PROD ID!!!\n");
		//Connected device is not a Dps310
		m_initFail = 1U;
		return;
	}
	m_productID = prodId;

	int16_t revId = readByteBitfield(DPS310__REG_INFO_REV_ID);
	if(revId < 0)
	{
    DEBUG_ALL("DPS310 ERROR READ REV ID!!!\n");
		m_initFail = 1U;
		return;
	}
	m_revisionID = revId;

	//find out which temperature sensor is calibrated with coefficients...
	int16_t sensor = readByteBitfield(DPS310__REG_INFO_TEMP_SENSORREC);
	if(sensor < 0)
	{
    DEBUG_ALL("DPS310 ERROR FIND TEMPERATURE SENSOR!!!\n");
		m_initFail = 1U;
		return;
	}

	//...and use this sensor for temperature measurement
	m_tempSensor = sensor;
	if(writeByteBitfield((uint8_t)sensor, DPS310__REG_INFO_TEMP_SENSOR) < 0)
	{
    DEBUG_ALL("DPS310 ERROR USE SENSOR FOR TEMPERATURE MEASUREMENT!!!\n");
		m_initFail = 1U;
		return;
	}

	//read coefficients
	if(readcoeffs() < 0)
	{
    DEBUG_ALL("DPS310 ERROR READ COEFFICIENTS!!!\n");
		m_initFail = 1U;
		return;
	}

	//set to standby for further configuration
	//standby();

	//set measurement precision and rate to standard values;
	//configTemp(DPS310__TEMP_STD_MR, DPS310__TEMP_STD_OSR);
	//configPressure(DPS310__PRS_STD_MR, DPS310__PRS_STD_OSR);

	//perform a first temperature measurement
	//the most recent temperature will be saved internally
	//and used for compensation when calculating pressure
	//int32_t trash;
	//measureTempOnce(trash);

	//make sure the DPS310 is in standby after initialization
	//standby();	

	// Fix IC with a fuse bit problem, which lead to a wrong temperature 
	// Should not affect ICs without this problem
	//correctTemp();
}


/**
 * reads the compensation coefficients from the DPS310
 * this is called once from init(), which is called from begin()
 *
 * returns: 	0 on success, -1 on fail
 */
int16_t DPS310SensorClass::readcoeffs(void)
{
	uint8_t buffer[DPS310__REG_LEN_COEF];
	//read COEF registers to buffer
	int16_t ret = readBlock(DPS310__REG_ADR_COEF,
							DPS310__REG_LEN_COEF,
							buffer);
	//abort if less than REG_LEN_COEF bytes were read
	if(ret < DPS310__REG_LEN_COEF)
	{
		return DPS310__FAIL_UNKNOWN;
	}

	//compose coefficients from buffer content
	m_c0Half =    ((uint32_t)buffer[0] << 4)
				| (((uint32_t)buffer[1] >> 4) & 0x0F);
	//this construction recognizes non-32-bit negative numbers
	//and converts them to 32-bit negative numbers with 2's complement
	if(m_c0Half & ((uint32_t)1 << 11))
	{
		m_c0Half -= (uint32_t)1 << 12;
	}
	//c0 is only used as c0*0.5, so c0_half is calculated immediately
	m_c0Half = m_c0Half / 2U;

	//now do the same thing for all other coefficients
	m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	if(m_c1 & ((uint32_t)1 << 11))
	{
		m_c1 -= (uint32_t)1 << 12;
	}

	m_c00 =   ((uint32_t)buffer[3] << 12)
			| ((uint32_t)buffer[4] << 4)
			| (((uint32_t)buffer[5] >> 4) & 0x0F);
	if(m_c00 & ((uint32_t)1 << 19))
	{
		m_c00 -= (uint32_t)1 << 20;
	}

	m_c10 =   (((uint32_t)buffer[5] & 0x0F) << 16)
			| ((uint32_t)buffer[6] << 8)
			| (uint32_t)buffer[7];
	if(m_c10 & ((uint32_t)1<<19))
	{
		m_c10 -= (uint32_t)1 << 20;
	}

	m_c01 =   ((uint32_t)buffer[8] << 8)
			| (uint32_t)buffer[9];
	if(m_c01 & ((uint32_t)1 << 15))
	{
		m_c01 -= (uint32_t)1 << 16;
	}

	m_c11 =   ((uint32_t)buffer[10] << 8)
			| (uint32_t)buffer[11];
	if(m_c11 & ((uint32_t)1 << 15))
	{
		m_c11 -= (uint32_t)1 << 16;
	}

	m_c20 =   ((uint32_t)buffer[12] << 8)
			| (uint32_t)buffer[13];
	if(m_c20 & ((uint32_t)1 << 15))
	{
		m_c20 -= (uint32_t)1 << 16;
	}

	m_c21 =   ((uint32_t)buffer[14] << 8)
			| (uint32_t)buffer[15];
	if(m_c21 & ((uint32_t)1 << 15))
	{
		m_c21 -= (uint32_t)1 << 16;
	}

	m_c30 =   ((uint32_t)buffer[16] << 8)
			| (uint32_t)buffer[17];
	if(m_c30 & ((uint32_t)1 << 15))
	{
		m_c30 -= (uint32_t)1 << 16;
	}

	return DPS310__SUCCEEDED;
}

/**
 * Sets the Operation Mode of the Dps310
 *
 * background:		determines the general behavior of the Dps310
 *  				0 enables command mode (only measure on commands)
 * 					1 enables background mode (continuous work in background)
 * temperature: 	set 1 to measure temperature
 * pressure: 		set 1 to measure pressure
 * return:			0 on success, -1 on fail
 *
 * NOTE!
 * You cannot set background to 1 without setting temperature and pressure
 * You cannot set both temperature and pressure when background mode is disabled
 */
int16_t DPS310SensorClass::setOpMode(uint8_t background, uint8_t temperature, uint8_t pressure)
{
	uint8_t opMode =  (background & DPS310__LSB) << 2U
					| (temperature & DPS310__LSB) << 1U
					| (pressure & DPS310__LSB);
	return setOpMode(opMode);
}


/**
 * Sets the Operation Mode of the Dps310
 *
 * opMode: 			the new OpMode that has to be set
 * return: 			0 on success, -1 on fail
 *
 * NOTE!
 * You cannot set background to 1 without setting temperature and pressure
 * You cannot set both temperature and pressure when background mode is disabled
 */
int16_t DPS310SensorClass::setOpMode(uint8_t opMode)
{
	//Filter irrelevant bits
	opMode &= DPS310__REG_MASK_OPMODE >> DPS310__REG_SHIFT_OPMODE;
	//Filter invalid OpModes
	if(opMode == INVAL_OP_CMD_BOTH || opMode == INVAL_OP_CONT_NONE)
	{
		return DPS310__FAIL_UNKNOWN;
	}
	//Set OpMode
	if(writeByte(DPS310__REG_ADR_OPMODE, opMode))
	{
		return DPS310__FAIL_UNKNOWN;
	}
	m_opMode = (DPS310SensorClass::Mode)opMode;
	return DPS310__SUCCEEDED;
}

/**
 * Configures temperature measurement
 *
 * tempMr: 	the new measure rate for temperature
 * 				This can be a value from 0U to 7U.
 * 				Actual measure rate will be 2^tempMr,
 * 				so this will be a value from 1 to 128.
 * tempOsr: 	the new oversampling rate for temperature
 * 				This can be a value from 0U to 7U.
 * 				Actual measure rate will be 2^tempOsr,
 * 				so this will be a value from 1 to 128.
 * returns: 	0 normally or -1 on fail
 */
int16_t DPS310SensorClass::configTemp(uint8_t tempMr, uint8_t tempOsr)
{
	//mask parameters
	tempMr &= DPS310__REG_MASK_TEMP_MR >> DPS310__REG_SHIFT_TEMP_MR;
	tempOsr &= DPS310__REG_MASK_TEMP_OSR >> DPS310__REG_SHIFT_TEMP_OSR;

	//set config register according to parameters
	uint8_t toWrite = tempMr << DPS310__REG_SHIFT_TEMP_MR;
	toWrite |= tempOsr << DPS310__REG_SHIFT_TEMP_OSR;
	//using recommended temperature sensor
	toWrite |=    DPS310__REG_MASK_TEMP_SENSOR
				& (m_tempSensor << DPS310__REG_SHIFT_TEMP_SENSOR);
	int16_t ret = writeByte(DPS310__REG_ADR_TEMP_MR, toWrite);
	//abort immediately on fail
	if(ret != DPS310__SUCCEEDED)
	{
		return DPS310__FAIL_UNKNOWN;
	}

	//set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3)
	if(tempOsr > DPS310__OSR_SE)
	{
		ret=writeByteBitfield(1U, DPS310__REG_INFO_TEMP_SE);
	}
	else
	{
		ret=writeByteBitfield(0U, DPS310__REG_INFO_TEMP_SE);
	}

	if(ret == DPS310__SUCCEEDED)
	{	//save new settings
		m_tempMr = tempMr;
		m_tempOsr = tempOsr;
	}
	else
	{
		//try to rollback on fail avoiding endless recursion
		//this is to make sure that shift enable and oversampling rate
		//are always consistent
		if(tempMr != m_tempMr || tempOsr != m_tempOsr)
		{
			configTemp(m_tempMr, m_tempOsr);
		}
	}
	return ret;
}

/**
 * Configures pressure measurement
 *
 * prsMr: 		the new measure rate for pressure
 * 				This can be a value from 0U to 7U.
 * 				Actual measure rate will be 2^prs_mr,
 * 				so this will be a value from 1 to 128.
 * prsOs: 	the new oversampling rate for temperature
 * 				This can be a value from 0U to 7U.
 * 				Actual measure rate will be 2^prsOsr,
 * 				so this will be a value from 1 to 128.
 * returns: 	0 normally or -1 on fail
 */
int16_t DPS310SensorClass::configPressure(uint8_t prsMr, uint8_t prsOsr)
{
	//mask parameters
	prsMr &= DPS310__REG_MASK_PRS_MR >> DPS310__REG_SHIFT_PRS_MR;
	prsOsr &= DPS310__REG_MASK_PRS_OSR >> DPS310__REG_SHIFT_PRS_OSR;

	//set config register according to parameters
	uint8_t toWrite = prsMr << DPS310__REG_SHIFT_PRS_MR;
	toWrite |= prsOsr << DPS310__REG_SHIFT_PRS_OSR;
	int16_t ret = writeByte(DPS310__REG_ADR_PRS_MR, toWrite);
	//abort immediately on fail
	if(ret != DPS310__SUCCEEDED)
	{
		return DPS310__FAIL_UNKNOWN;
	}

	//set PM SHIFT ENABLE if oversampling rate higher than eight(2^3)
	if(prsOsr > DPS310__OSR_SE)
	{
		ret = writeByteBitfield(1U, DPS310__REG_INFO_PRS_SE);
	}
	else
	{
		ret = writeByteBitfield(0U, DPS310__REG_INFO_PRS_SE);
	}

	if(ret == DPS310__SUCCEEDED)
	{	//save new settings
		m_prsMr = prsMr;
		m_prsOsr = prsOsr;
	}
	else
	{	//try to rollback on fail avoiding endless recursion
		//this is to make sure that shift enable and oversampling rate
		//are always consistent
		if(prsMr != m_prsMr || prsOsr != m_prsOsr)
		{
			configPressure(m_prsMr, m_prsOsr);
		}
	}
	return ret;
}

/**
 * calculates the time that the DPS310 needs for 2^mr measurements
 * with an oversampling rate of 2^osr
 *
 * mr: 		Measure rate for temperature or pressure
 * osr: 	Oversampling rate for temperature or pressure
 * returns: time that the DPS310 needs for this measurement
 * 			a value of 10000 equals 1 second
 * 	NOTE! 	The measurement time for temperature and pressure
 * 			in sum must not be more than 1 second!
 * 			Timing behavior of pressure and temperature sensors
 * 			can be considered as equal.
 */
uint16_t DPS310SensorClass::calcBusyTime(uint16_t mr, uint16_t osr)
{
	//mask parameters first
	mr &= DPS310__REG_MASK_TEMP_MR >> DPS310__REG_SHIFT_TEMP_MR;
	osr &= DPS310__REG_MASK_TEMP_OSR >> DPS310__REG_SHIFT_TEMP_OSR;
	//formula from datasheet (optimized)
	return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

/**
 * Gets the next temperature measurement result in degrees of Celsius
 *
 * result: 	address where the result will be written
 * returns:	0 on success
 * 			-1 on fail;
 */
int16_t DPS310SensorClass::getTemp(int32_t *result)
{
	uint8_t buffer[3] = {0};
	//read raw pressure data to buffer

	int16_t i = readBlock(DPS310__REG_ADR_TEMP,
							DPS310__REG_LEN_TEMP,
							buffer);
	if(i != DPS310__REG_LEN_TEMP)
	{
		//something went wrong
		return DPS310__FAIL_UNKNOWN;
	}

	//compose raw temperature value from buffer
	int32_t temp =    (uint32_t)buffer[0] << 16
					| (uint32_t)buffer[1] << 8
					| (uint32_t)buffer[2];
	//recognize non-32-bit negative numbers
	//and convert them to 32-bit negative numbers using 2's complement
	if(temp & ((uint32_t)1 << 23))
	{
		temp -= (uint32_t)1 << 24;
	}

	//return temperature
	*result = calcTemp(temp);
	return DPS310__SUCCEEDED;
}

/**
 * Gets the next pressure measurement result in Pa
 *
 * result: 	address where the result will be written
 * returns: 0 on success
 * 			-1 on fail;
 */
int16_t DPS310SensorClass::getPressure(int32_t *result)
{
	uint8_t buffer[3] = {0};
	//read raw pressure data to buffer
	int16_t i = readBlock(DPS310__REG_ADR_PRS,
							DPS310__REG_LEN_PRS,
							buffer);
	if(i != DPS310__REG_LEN_PRS)
	{
		//something went wrong
		//negative pressure is not allowed
		return DPS310__FAIL_UNKNOWN;
	}

	//compose raw pressure value from buffer
	int32_t prs =   (uint32_t)buffer[0] << 16
					| (uint32_t)buffer[1] << 8
					| (uint32_t)buffer[2];
	//recognize non-32-bit negative numbers
	//and convert them to 32-bit negative numbers using 2's complement
	if(prs & ((uint32_t)1 << 23))
	{
		prs -= (uint32_t)1 << 24;
	}

	*result = calcPressure(prs);
	return DPS310__SUCCEEDED;
}

/**
 * reads the next raw value from the Dps310 FIFO
 *
 * value: 	address where the value will be written
 * returns:	-1 on fail
 * 			0 if result is a temperature raw value
 * 			1 if result is a pressure raw value
 */
int16_t DPS310SensorClass::getFIFOvalue(int32_t* value)
{
	//abort on invalid argument
	if(value == NULL)
	{
		return DPS310__FAIL_UNKNOWN;
	}

	uint8_t buffer[DPS310__REG_LEN_PRS] = {0};
	//always read from pressure raw value register
	int16_t i = readBlock(DPS310__REG_ADR_PRS,
							DPS310__REG_LEN_PRS,
							buffer);
	if(i != DPS310__REG_LEN_PRS)
	{
		//something went wrong
		//return error code
		return DPS310__FAIL_UNKNOWN;
	}
	//compose raw pressure value from buffer
	*value =  (uint32_t)buffer[0] << 16
			| (uint32_t)buffer[1] << 8
			| (uint32_t)buffer[2];
	//recognize non-32-bit negative numbers
	//and convert them to 32-bit negative numbers using 2's complement
	if(*value & ((uint32_t)1 << 23))
	{
		*value -= (uint32_t)1 << 24;
	}

	//least significant bit shows measurement type
	return buffer[2] & DPS310__LSB;
}

/**
 * Calculates a scaled and compensated pressure value from raw data
 * raw: 	raw temperature value read from Dps310
 * returns: temperature value in °C
 */
int32_t DPS310SensorClass::calcTemp(int32_t raw)
{
	double temp = raw;
	
	//scale temperature according to scaling table and oversampling
	temp /= scaling_facts[m_tempOsr];

	//update last measured temperature
	//it will be used for pressure compensation
	m_lastTempScal = temp;

	//Calculate compensated temperature
	temp = m_c0Half + m_c1 * temp;

	//return temperature
	return (int32_t)temp;
}

/**
 * Calculates a scaled and compensated pressure value from raw data
 * raw: 	raw pressure value read from Dps310
 * returns: pressure value in Pa
 */
int32_t DPS310SensorClass::calcPressure(int32_t raw)
{
	double prs = raw;

	//scale pressure according to scaling table and oversampling
	prs /= scaling_facts[m_prsOsr];

	//Calculate compensated pressure
	prs =   m_c00
			+ prs * (m_c10 + prs * (m_c20 + prs * m_c30))
			+ m_lastTempScal * (m_c01 + prs * (m_c11 + prs * m_c21));

	//return pressure
	return (int32_t)prs;
}

/**
 * reads a byte from dps310
 *
 * regAdress: 	Address that has to be read
 * returns: 	register content or -1 on fail
 */
int16_t DPS310SensorClass::readByte(uint8_t regAddress)
{
	return readByteSPI(regAddress);
}

/**
 * reads a byte from dps310 via SPI
 * this function is automatically called by readByte
 * if Dps310 is connected via SPI
 *
 * regAdress: 	Address that has to be read
 * returns: 	register content or -1 on fail
 */
int16_t DPS310SensorClass::readByteSPI(uint8_t regAddress)
{
  int16_t ret = 0;
//  tx_buffer[0] = (regAddress | DPS310__SPI_READ_CMD);
//  tx_buffer[1] = 0xFF;

//  SPIBRIDGEMsgQueueTypeDef msg_out;
//  SPIBRIDGEMsgQueueTypeDef msg_in;
//  msg_out.id = 2;
//  msg_out.len = 2;
//  msg_out.pbuff_in = rx_buffer;
//  msg_out.pbuff_out = tx_buffer;
//  osMessageQueuePut(spi_bridge.queue_in, &msg_out, NULL, osWaitForever);
//  osMessageQueueGet(Core.DPS310->queue_in, &msg_in, NULL, osWaitForever);
  
    spi_tx_buffer[0] = (regAddress | DPS310__SPI_READ_CMD);
    spi_tx_buffer[1] = 0xFF;
    SPI_Transfer(spi_tx_buffer, spi_rx_buffer, 2, 2);
    ret = spi_rx_buffer[1];
	return ret;
}

/**
 * reads a block from dps310
 *
 * regAdress: 	Address that has to be read
 * length: 		Length of data block
 * buffer: 	Buffer where data will be stored
 * returns: 	number of bytes that have been read successfully
 * 				NOTE! This is not always equal to length
 * 					  due to rx-Buffer overflow etc.
 */
int16_t DPS310SensorClass::readBlock(uint8_t regAddress, uint8_t length, uint8_t *buffer)
{
  int16_t ret = readBlockSPI(regAddress, length, buffer);
	return ret;
}

/**
 * reads a block from dps310 via SPI
 *
 * regAdress: 	Address that has to be read
 * length: 		Length of data block
 * readbuffer: 	Buffer where data will be stored
 * returns: 	number of bytes that have been read successfully
 * 				NOTE! This is not always equal to length
 * 					  due to rx-Buffer overflow etc.
 */
int16_t DPS310SensorClass::readBlockSPI(uint8_t regAddress, uint8_t length, uint8_t *buffer)
{
//  int32_t ret = DPS310__FAIL_UNKNOWN; //error
//  tx_buffer[0] = (regAddress | DPS310__SPI_READ_CMD);
//  for(int i = 0; i < length; i++){
//    tx_buffer[i + 1] = 0xFF;
//  }
//  SPIBRIDGEMsgQueueTypeDef msg_out;
//  SPIBRIDGEMsgQueueTypeDef msg_in;
//  msg_out.id = 2;
//  msg_out.len = length + 1;
//  msg_out.pbuff_in = rx_buffer;
//  msg_out.pbuff_out = tx_buffer;
//  osMessageQueuePut(spi_bridge.queue_in, &msg_out, NULL, osWaitForever);
//  osMessageQueueGet(Core.DPS310->queue_in, &msg_in, NULL, osWaitForever);   
//  memcpy(buffer, &rx_buffer[1], length);
  
  spi_tx_buffer[0] = (regAddress | DPS310__SPI_READ_CMD);
  memset(&spi_tx_buffer[1],0xFF, length);
  SPI_Transfer(spi_tx_buffer, spi_rx_buffer, length + 1, 2);
  memcpy(buffer, &spi_rx_buffer[1], length);
  
	//return received data
	return length;
}

/**
 * writes a given byte to a given register of dps310 without checking
 *
 * regAdress: 	Address of the register that has to be updated
 * data:		Byte that will be written to the register
 * return:		0 if byte was written successfully
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::writeByte(uint8_t regAddress, uint8_t data)
{
	return writeByte(regAddress, data, 0U);
}

/**
 * writes a given byte to a given register of dps310
 *
 * regAdress: 	Address of the register that has to be updated
 * data:		Byte that will be written to the register
 * check: 		If this is true, register content will be read after writing
 * 				to check if update was successful
 * return:		0 if byte was written successfully
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::writeByte(uint8_t regAddress, uint8_t data, uint8_t check)
{
	return writeByteSpi(regAddress, data, check);
}

/**
 * writes a given byte to a given register of dps310 via SPI
 *
 * regAdress: 	Address of the register that has to be updated
 * data:		Byte that will be written to the register
 * check: 		If this is true, register content will be read after writing
 * 				to check if update was successful
 * return:		0 if byte was written successfully
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::writeByteSpi(uint8_t regAddress, uint8_t data, uint8_t check)
{

//  tx_buffer[0] = (regAddress | DPS310__SPI_WRITE_CMD);
//  tx_buffer[1] = data;
//  SPIBRIDGEMsgQueueTypeDef msg_out;
//  SPIBRIDGEMsgQueueTypeDef msg_in;
//  msg_out.id = 2;
//  msg_out.len = 2;
//  msg_out.pbuff_in = rx_buffer;
//  msg_out.pbuff_out = tx_buffer;
//  osMessageQueuePut(spi_bridge.queue_in, &msg_out, NULL, osWaitForever);
//  osMessageQueueGet(Core.DPS310->queue_in, &msg_in, NULL, osWaitForever); 
//  if (check == 0)
//  {
//      //no checking necessary
//      return DPS310__SUCCEEDED;
//  }  
//  //checking necessary
//  if (readByte(regAddress) == data)
//  {
//      //check passed
//      return DPS310__SUCCEEDED;
//  }
//  else
//  {
//      //check failed
//      return DPS310__FAIL_UNKNOWN;
//  }
//	return DPS310__FAIL_UNKNOWN;

  
  
    spi_tx_buffer[0] = (regAddress | DPS310__SPI_WRITE_CMD);
    spi_tx_buffer[1] = data;
    SPI_Transfer(spi_tx_buffer, spi_rx_buffer, 2, 2);
  return DPS310__SUCCEEDED;
}

/**
 * updates some given bits of a given register of dps310 without checking
 *
 * regAdress: 	Address of the register that has to be updated
 * data:		BitValues that will be written to the register
 * shift:		Amount of bits the data byte is shifted (left) before being masked
 * mask: 		Masks the bits of the register that have to be updated
 * 				Bits with value 1 are updated
 * 				Bits with value 0 are not changed
 * return:		0 if byte was written successfully
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::writeByteBitfield(uint8_t data,
										uint8_t regAddress,
										uint8_t mask,
										uint8_t shift)
{
	return writeByteBitfield(data, regAddress, mask, shift, 0U);
}

/**
 * updates some given bits of a given register of dps310
 *
 * regAdress: 	Address of the register that has to be updated
 * data:		BitValues that will be written to the register
 * shift:		Amount of bits the data byte is shifted (left) before being masked
 * mask: 		Masks the bits of the register that have to be updated
 * 				Bits with value 1 are updated
 * 				Bits with value 0 are not changed
 * check: 		enables/disables check after writing
 * 				0 disables check
 * 				if check fails, -1 will be returned
 * return:		0 if byte was written successfully
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::writeByteBitfield(uint8_t data,
										uint8_t regAddress,
										uint8_t mask,
										uint8_t shift,
										uint8_t check)
{
	int16_t old = readByte(regAddress);
	if(old < 0)
	{
		//fail while reading
		return old;
	}
	return writeByte(regAddress, ((uint8_t)old & ~mask)|((data << shift) & mask), check);
}

/**
 * reads some given bits of a given register of dps310
 *
 * regAdress: 	Address of the register that has to be updated
 * mask: 		Masks the bits of the register that have to be updated
 * 				Bits masked with value 1 are read
 * 				Bits masked with value 0 are set 0
 * shift:		Amount of bits the data byte is shifted (right) after being masked
 * return:		read and processed bits
 * 				or -1 on fail
 */
int16_t DPS310SensorClass::readByteBitfield(uint8_t regAddress, uint8_t mask, uint8_t shift)
{
	int16_t ret = readByte(regAddress);
	if(ret<0)
	{
		return ret;
	}
	return (((uint8_t)ret) & mask) >> shift;
}
