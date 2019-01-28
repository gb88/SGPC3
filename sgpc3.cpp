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

/*
 * Not implemented:
	Get raw Ethanol signal 0x204d
	Get sensor raw signal (Ethanol_signal) and the TVOC air quality signal 0x2046,
 */

#include <Arduino.h>
#include <Wire.h>

#include <string.h>

#include "sgpc3.h"

#ifdef ARDUINO_ARCH_ESP32
	#include <pgmspace.h>
#else
	#include <avr/pgmspace.h>
#endif
#ifdef ARDUINO_ARCH_ESP32
	SGPC3::SGPC3(int sda, int scl, uint32_t frequency)
	{
		Wire.begin(sda,scl,frequency);
		powerMode = SGPC3_LOW_POWER;
		measureDelay = SGPC3_LOW_POWER_INTERMEASURE_DELAY;
	}
#else
	SGPC3::SGPC3()
	{
		Wire.begin();
		powerMode = SGPC3_LOW_POWER;
		measureDelay = SGPC3_LOW_POWER_INTERMEASURE_DELAY;
	}
#endif

int SGPC3::readFeatureSet()
{
    uint8_t cmd[2] = { 0x20, 0x2f };

    if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_write");
			#else
				setError(F("error in i2c_write"));
			#endif
        return -1;
    }
    delay(10); //10ms as specified in datasheet

    uint8_t data[3] = { 0 };
    int ret = i2c_read(SGPC3_I2C_ADDR, data, 3);
    if (ret == -1) {
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_read");
			#else
				setError(F("error in i2c_read"));
			#endif
        return -3;
    }

    // check CRC
    if (crc8(data, 2) != data[2]) {
		#ifdef ARDUINO_ARCH_ESP32
        setError("CRC mismatch");
		#else
				setError(F("CRC mismatch"));
		#endif
        return -4;
    }

    // 0 = SGP30, 1 = SGPC3
    productType = (data[0] & 0xF0) >> 4;
    productVersion = data[1] & 0xFF;

    return 0;
}

uint8_t SGPC3::getProductType()
{
    return productType;
}

uint8_t SGPC3::getVersion()
{
    return productVersion;
}

int SGPC3::lowPower()
{
   powerMode = SGPC3_LOW_POWER;
	measureDelay = SGPC3_LOW_POWER_INTERMEASURE_DELAY;
	if (initialized) {
		return setPowerMode();
	}
	else
		return 0;
}

int SGPC3::ultraLowPower()
{
   powerMode = SGPC3_ULTRA_LOW_POWER;
	measureDelay = SGPC3_ULTRA_LOW_POWER_INTERMEASURE_DELAY;
	if (initialized) {
		return setPowerMode();
	}
	else
		return 0;
}

int SGPC3::selfTest()
{
	if (!initialized) {
		uint8_t cmd[2] = { 0x20, 0x32};
		if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
		  #ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_write");
			#else
				setError(F("error in i2c_write"));
			#endif
      return -1;
		}
		delay(220); //220ms as specified in datasheet
		int ret = i2c_read(SGPC3_I2C_ADDR, mDataBuf, 3);
		if (ret == -1) 
		{
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_read");
			#else
				setError(F("error in i2c_read"));
			#endif
			return -2;
		}
		if (crc8(mDataBuf, 2) != mDataBuf[2]) 
		{
			#ifdef ARDUINO_ARCH_ESP32
					setError("CRC mismatch");
			#else
					setError(F("CRC mismatch"));
			#endif
			return -3;
		}
		if((mDataBuf[0] == 0xD4) &&(mDataBuf[0] == 0x00))
		{
			return 0; //test ok
		}
		else{
			return -4;
		}
	}
	else
	{
		return -5;
	}
}

void SGPC3::setBaselineValue(uint16_t value)
{
    baseline = value;
}

int SGPC3::setBaseline()
{
   uint8_t cmd[5] = { 0x20, 0x1e,0x00, 0x00,0x00};
	 cmd[2] = (uint8_t)(baseline >> 8);
	 cmd[3] = (uint8_t)baseline;
	 //crc of the parameter
	 cmd[4] = crc8(&cmd[2],2);
    if (i2c_write(SGPC3_I2C_ADDR, cmd, 5)) {
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_write");
			#else
				setError(F("error in i2c_write"));
			#endif
      return -1;
    }
    delay(10); //10ms as specified in datasheet
    return 0;
}

int SGPC3::getFactoryBaseline(uint16_t* factoryBaseline)
{
	uint8_t cmd[2] = { 0x20, 0x23 };

	if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_write");
		#else
			setError(F("error in i2c_write"));
		#endif
	  return -1;
	}
	delay(10);
	int ret = i2c_read(SGPC3_I2C_ADDR, mDataBuf, 3);
	if (ret == -1) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_read");
		#else
			setError(F("error in i2c_read"));
		#endif
	  return -2;
	}

	if (crc8(mDataBuf, 2) != mDataBuf[2]) {
	  #ifdef ARDUINO_ARCH_ESP32
      setError("CRC mismatch");
		#else
			setError(F("CRC mismatch"));
		#endif
	  return -3;
	}
	uint16_t temp;
	temp = mDataBuf[0];
	temp <<= 8;
	temp |= mDataBuf[1];
	*factoryBaseline = temp;
   return 0;
}

int SGPC3::initSGPC3()
{
	int ret = readFeatureSet();
	if(ret != 0)
	 return -1;
	uint8_t cmd[2] = { 0x20, 0xae };
		
	// run set power mode
	ret = setPowerMode();
	if(ret != 0)
	 return -2;

	// run init air quality
	if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_write");
		#else
			setError(F("error in i2c_write"));
		#endif
	  return -3;
	}
   delay(10);
	initialized = true;
   return ret;
}

int SGPC3::initSGPC3(downTime_t downTime)
{
	int ret = readFeatureSet();
	unsigned long delay_time;
	if(ret != 0)
	 return -1;
	//default LT_FOREVER
	uint8_t cmd[2] = { 0x20, 0xae };
		
	// run set power mode
	ret = setPowerMode();
	if(ret != 0)
	 return -2;
	if(downTime  == LT_5_MIN)
		cmd[1] = 0x89;
 	else if(downTime  == LT_1_HOUR)
		cmd[1] = 0x24;
	else if(downTime  == LT_24_HOUR)
		cmd[1] = 0x03;
	else if(downTime  == LT_1_WEEK)
		cmd[1] = 0x6a;
	
	// run init air quality
	if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_write");
		#else
			setError(F("error in i2c_write"));
		#endif
	  return -3;
	}
	
	delay(10);
	if(downTime != LT_FOREVER)
	{
		ret = setBaseline();
		if(ret != 0)
		return -3;
	}
	
	if(downTime  == LT_1_HOUR)
		delay_time = 16000;
	else if(downTime  == LT_24_HOUR)
		delay_time = 64000;
	else
		delay_time = 184000;
	
	if(downTime  != LT_5_MIN)
		delay(delay_time);
	initialized = true;
   return ret;
}

bool SGPC3::isInitialized()
{
    return initialized;
}


int SGPC3::setPowerMode()
{
    uint8_t cmd[5] = { 0x20, 0x9f,0x00, 0x01,0x00};
    if(powerMode == SGPC3_ULTRA_LOW_POWER)
	 {
		cmd[3] = 0x00;
	 }
	 //crc of the parameter
	 cmd[4] = crc8(&cmd[2],2);
    if (i2c_write(SGPC3_I2C_ADDR, cmd, 5)) {
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_write");
			#else
				setError(F("error in i2c_write"));
			#endif
      return -1;
    }
    delay(10); //10ms as specified in datasheet
    return 0;
}

int SGPC3::setAbsHumidityCompensation(uint16_t humidity_8_8) //g/m3 
{
    uint8_t cmd[5] = { 0x20, 0x61,0x00, 0x00,0x00};
	 cmd[2] = (uint8_t)(humidity_8_8 >> 8);
	 cmd[3] = (uint8_t)humidity_8_8;
	 //crc of the parameter
	 cmd[4] = crc8(&cmd[2],2);
    if (i2c_write(SGPC3_I2C_ADDR, cmd, 5)) {
        return -1;
    }
    delay(10); //10ms as specified in datasheet
    return 0;
}

int SGPC3::disableAbsHumidityCompensation()
{
    uint8_t cmd[5] = { 0x20, 0x61,0x00, 0x00,0x00};
	 //crc of the parameter
	 cmd[4] = crc8(&cmd[2],2);
    if (i2c_write(SGPC3_I2C_ADDR, cmd, 5)) {
			#ifdef ARDUINO_ARCH_ESP32
				setError("error in i2c_write");
			#else
				setError(F("error in i2c_write"));
			#endif
      return -1;
    }
    delay(10); //10ms as specified in datasheet
    return 0;
}

int SGPC3::measureIAQ()
{
	if (!initialized) {
		#ifdef ARDUINO_ARCH_ESP32
      setError("ESS not initialized");
		#else
			setError(F("ESS not initialized"));
		#endif
	  return -1;
	}

	uint8_t cmd[2] = { 0x20, 0x08 };

	if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_write");
		#else
			setError(F("error in i2c_write"));
		#endif
	  return -1;
	}

	delay(50);

	int ret = i2c_read(SGPC3_I2C_ADDR, mDataBuf, 3);
	if (ret == -1) {
		#ifdef ARDUINO_ARCH_ESP32
			setError("error in i2c_read");
		#else
			setError(F("error in i2c_read"));
		#endif
	  return -2;
	}

	if (crc8(mDataBuf, 2) != mDataBuf[2]) {
		#ifdef ARDUINO_ARCH_ESP32
      setError("CRC mismatch");
		#else
			setError(F("CRC mismatch"));
		#endif
	  return -3;
	}
	// keep track of timing
	measurementTimestamp = millis();
   TVOC = (uint16_t)(mDataBuf[0] << 8) | mDataBuf[1];
   return 0;
}

uint16_t SGPC3::getTVOC()
{
    return TVOC;
}

unsigned long SGPC3::getLastMeasureTimestamp()
{
    return measurementTimestamp;
}

int SGPC3::getBaseline()
{
	uint8_t cmd[2] = { 0x20, 0x15 };

	if (i2c_write(SGPC3_I2C_ADDR, cmd, 2)) {
	  return -1;
	}
	delay(10);
	int ret = i2c_read(SGPC3_I2C_ADDR, mDataBuf, 3);
	if (ret == -1) {
	  return -2;
	}

	if (crc8(mDataBuf, 2) != mDataBuf[2]) {
	  #ifdef ARDUINO_ARCH_ESP32
      setError("CRC mismatch");
		#else
			setError(F("CRC mismatch"));
		#endif
	  return -3;
	}
	uint16_t temp;
	temp = mDataBuf[0];
	temp <<= 8;
	temp |= mDataBuf[1];
	baseline = temp;
   return 0;
}

uint16_t SGPC3::getBaselineValue()
{
   return baseline;
}
#ifdef ARDUINO_ARCH_ESP32
void SGPC3::setError(const char* error)
{
    strcpy(mErrorBuf, error);
}
#else
void SGPC3::setError(const __FlashStringHelper* error)
{
  int cursor = 0;
  const char *ptr = ( const char * ) error;
  while( ( mErrorBuf[ cursor ] = pgm_read_byte_near( ptr + cursor ) ) != '\0' ) ++cursor;
  //strcpy_P(mErrorBuf, error);
}
#endif

const char* SGPC3::getError() const
{
    return mErrorBuf;
}

int SGPC3::remainingWaitTimeMS()
{
    unsigned long deltaT = millis() - measurementTimestamp;
    if (deltaT > measureDelay) {
        // we're already late, don't wait any longer
        return 0;
    }
    return (measureDelay - deltaT);
}

//helper
int8_t SGPC3::i2c_read(uint8_t addr, uint8_t* data, uint16_t count)
{
    Wire.requestFrom(addr, count);
    if (Wire.available() != count) {
      return -1;
    }
    for (int i = 0; i < count; ++i) {
        data[i] = Wire.read();
    }
    return 0;
}

int8_t SGPC3::i2c_write(uint8_t addr, const uint8_t* data, uint16_t count)
{
    Wire.beginTransmission(addr);
    for (int i = 0; i < count; ++i) {
        if (Wire.write(data[i]) != 1) {				
            return false;
        }
    }
    if (Wire.endTransmission() != 0) {
        return -1;
    }
    return 0;
}

uint8_t SGPC3::crc8(const uint8_t* data, uint8_t len)
{
  // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/

  uint8_t crc = 0xff;
  uint8_t byteCtr;
  for (byteCtr = 0; byteCtr < len; ++byteCtr) {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}