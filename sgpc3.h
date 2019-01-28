/*
 * Copyright (c) 2017-2018, Sensirion AG
 * Copyright (c) 2019, Giovanni Bonomini
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 */

#ifndef SGPC3_H
#define SGPC3_H

#include <stdint.h>


#define SGPC3_I2C_ADDR																0x58
#define SGPC3_LOW_POWER																0
#define SGPC3_ULTRA_LOW_POWER													1
#define SGPC3_LOW_POWER_INTERMEASURE_DELAY  					2000
#define SGPC3_ULTRA_LOW_POWER_INTERMEASURE_DELAY 			30000

#ifndef SGPC3_ERROR_BUF_LENGTH
	#define SGPC3_ERROR_BUF_LENGTH 255
#endif

//! DownTime
/*! Total sensor down-time, for the correct sensor initalization */
enum downTime_t
{
	LT_5_MIN = 0,
	LT_1_HOUR = 1,
	LT_24_HOUR = 2,
	LT_1_WEEK = 3,
	LT_FOREVER = 4,
};
		
//!  Sensiron SGPC3 VOC Sensor Class. 
/*!
  Class for SGCP3 VOC sensor, implent low-power and ultra-power profile with the correct initialization.
*/

class SGPC3
{
	public:
	 
		//! Constructor
		/*!
			Initialize the sensor mode in low power.
		*/  
		#ifdef ARDUINO_ARCH_ESP32
			SGPC3(int sda, int scl, uint32_t frequency);
		#else
			SGPC3();
		#endif
		
		//! Get the product type.
		/*!
			\return product type
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		uint8_t getProductType();
		
		/*! Get the product version.
			\return product version
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		uint8_t getVersion();
		
		//! Set the low power mode to be used in the initialization, or if the sensor is initialized change the power mode.
		/*!
			\return low power result (0 = 0K)
		*/
		int lowPower();
		
		//! Set the ultra low power mode to be used in the initialization, or if the sensor is initialized change the power mode.
		/*!
			\return ultra low power result (0 = 0K)
		*/
		int ultraLowPower();
		
	   //! Perform sensor self test, must run befor initialization
		/*!
			\return selftest result (0 = OK)
		*/
		int selfTest();
		
		//! Set the baseline value to be used in the initialization.
		/*!
			\param value value to be used
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		void setBaselineValue(uint16_t value);

		//! Read from the sensor the factory baseline value.
		/*!
			\param factoryBaseline poiter to uint16_t where to store the factory baseline value
			\return function result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		int getFactoryBaseline(uint16_t* factoryBaseline);
		
		//! Sensor Initialization function,intialize sensor in low power mode with no baseline initialization
		/*!
			\return initialization result (0 = 0K)
		*/
		int initSGPC3();
		
		//! Sensor Initialization function with the power mode and the baseline setted before, by default initalize the sensor in low power mode without baseline waiting 184s of init time
		/*!
			\param downTime the down-time of the sensor from the last power on.
			\return init result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		int initSGPC3(downTime_t downTime);
		
		//! Check if the sensor is initialized
		/*!
			\return true if is initialized
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		bool isInitialized();
		
		//! Set power the power mode selected before
		/*!
			\return power mode activation result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		int setPowerMode();
		
	   //! Set the humidity value for the measure compensation
		/*!
			\param humidity_8_8 absolute humidity in format 8.8 unsigned
			\return compensation result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		int setAbsHumidityCompensation(uint16_t humidity_8_8);
		
	   //! Disable the compensation of the measure in function of humidity the humidity value for the measure compensation
		/*!
			\return compensation result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		int disableAbsHumidityCompensation();	
		
	   //! Perform TVOC measure
		/*!
			\return function result (0 = OK)
		*/
		int measureIAQ();
		
	   //! Get the TVOC measured value
		/*!
			\return TVOC measured value
		*/
		uint16_t getTVOC();
		
		//! Get the timestamp of the last TVOC measure
		/*!
			\return measure timestamp
		*/
		unsigned long getLastMeasureTimestamp();
		
	   //! Read base line value from the sensor
		/*!
			\return function result (0 = OK)
		*/
		int getBaseline();
		
		//! Retrieve the actual baseline in use, this value is updated from the getBaseline function
		/*!
			\return baseline value
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
		uint16_t getBaselineValue();
		
		//! Get the last error message
		/*!
			\return last error message
		*/
		const char* getError() const;
	
		//! Reamining time before the next measure in ms
		/*!
			\return ms remaining before the TVOC update
		*/
		int remainingWaitTimeMS();

private:
	//! Read the sensor specs.
		/*!
			\return function result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
	int readFeatureSet();
	
	//! Write the baseline value to the sensor.
		/*!
			\return function result (0 = OK)
			\sa QTstyle_Test(), ~QTstyle_Test(), testMeToo() and publicVar() //TODO: aggiungere reference
		*/
	int setBaseline();
		
	//! Type of sensor.
   uint8_t productType       = 0;
	//! Version of sensor.
   uint8_t productVersion    = 0;
   //! Sensor Power Mode
	/*!
		Can be low power or ultra low power, default mode is low power, is initialized in sensor initialization.
	*/
	uint8_t powerMode         = SGPC3_LOW_POWER;
	//! Actual baseline used values, depend on the active mode, the value is mode depended.
	uint16_t baseline =	0;
	//! Sensor initialization state.
	bool initialized = false;
	//! Correct time between measure
	/*!
		The time between measure is 2s in low power mode and 30s in ultra low power, default mode is low power.
	*/
	unsigned long measureDelay = 0;
	//! Timestamp of the last acquired TVOC.
   unsigned long measurementTimestamp = 0;
	//! TVOC acquired values in ppm.
	uint16_t TVOC        = 0;
	//! Buffer for error messages.
	char mErrorBuf[SGPC3_ERROR_BUF_LENGTH];
	//! Buffer for read sensor data
  uint8_t mDataBuf[3];
	
	//helper
	//! i2c write on interface
	/*!
		\param addr I2c address.
		\param data write buffer pointer.
		\param count number of byte to write from data buffer.
		\return The write result(0 = OK)
	*/
   static int8_t i2c_write(uint8_t addr, const uint8_t* data, uint16_t count);
	
	//! i2c read on interface
	/*!
		\param addr I2c address.
		\param data read buffer pointer.
		\param count number of byte to read from interface.
		\return The read result(0 = OK)
	*/
   int8_t i2c_read(uint8_t addr, uint8_t* data, uint16_t count);
	
	//! crc8 computation
	/*!
		\param data buffer pointer.
		\param len number of byte in the data buffer to compute crc.
		\return crc result
	*/
   uint8_t crc8(const uint8_t* data, uint8_t len);
	
	// error handling
	//! Set the error message
	/*!
		\param error message.
	*/
#ifdef ARDUINO_ARCH_ESP32
	void setError(const char* error);
#else
	void setError(const __FlashStringHelper* error);
#endif
	 
};

#endif /* SGPC3_H */
