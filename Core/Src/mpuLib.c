/*
 * mpuLib.cpp
 *
 *  Created on: Jan 8, 2021
 *      Author: elvez
 */



#include <stdbool.h>
#include <stdlib.h>

#include <math.h>
#include <mpuLib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "userHW.h"
#include "main.h"

RawData_TypeDef rawDataAcc;
ScaledData_TypeDef scaledDataAcc;
RawData_TypeDef rawDataGyro;
ScaledData_TypeDef scaledDataGyro;
I2C_HandleTypeDef mpuI2C;
float accelScalingFactor, gyroScalingFactor;
float accXBias = 0.0f;
float accYBias = 0.0f;
float accZBias = 0.0f;

void mpuInit(I2C_HandleTypeDef *i2cHandle)
{
	memcpy(&mpuI2C, i2cHandle, sizeof(*i2cHandle));
}

void mpuConfig(MPU_ConfigTypeDef configuration)
{
	uint8_t buffer_ = 0;

	writeData(pwrMgmtReg_, 0x80);
	HAL_Delay(100);
	buffer_ = configuration.clockSource & 0x07;
	buffer_ |= (configuration.sleepMode_ << 6) &0x40;
	writeData(pwrMgmtReg_, buffer_);
	HAL_Delay(100);

	buffer_ = 0;
	buffer_ = configuration.configDLPF & 0x07;
	writeData(configReg_, buffer_);

	buffer_ = 0;
	buffer_ = (configuration.gyroFS_ << 3) & 0x18;
	writeData(configGyroReg_, buffer_);

	buffer_ = 0;
	buffer_ = (configuration.accFS_ << 3) & 0x18;
	writeData(configAccReg_, buffer_);
	setSampleRateDivider(0x04);

	switch (configuration.accFS_)
	{
		case AFS_2G:
			accelScalingFactor = (2000.0f/32768.0f);
			break;

		case AFS_4G:
			accelScalingFactor = (4000.0f/32768.0f);
			break;

		case AFS_8G:
			accelScalingFactor = (8000.0f/32768.0f);
			break;

		case AFS_16G:
			accelScalingFactor = (16000.0f/32768.0f);
			break;

		default:
			break;
	}

	switch (configuration.gyroFS_)
	{
		case FS_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;

		case FS_500:
			gyroScalingFactor = 500.0f/32768.0f;
			break;

		case FS_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;

		case FS_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;

		default:
			break;
	}
}

RawData_TypeDef getRawAccData()
{
	uint8_t buffer_[2];
	uint8_t acceArr[6], gyroArr[6];

	readData(intStatusReg_, &buffer_[1],1);
	if((buffer_[1] && 0x01))
	{
		readData(xOutAcc_, acceArr, 6);

		rawDataAcc.xData = ((acceArr[0] << 8) + acceArr[1]);
		rawDataAcc.yData = ((acceArr[2] << 8) + acceArr[3]);
		rawDataAcc.zData = ((acceArr[4] << 8) + acceArr[5]);

		readData(xOutGyro_, gyroArr, 6);
		gyroRW[0] = ((gyroArr[0] << 8) + gyroArr[1]);
		gyroRW[1] = ((gyroArr[2] << 8) + gyroArr[3]);
		gyroRW[2] = ((gyroArr[4] << 8) + gyroArr[5]);
	}
	return rawDataAcc;
}

RawData_TypeDef getRawGyroData()
{
	rawDataGyro.xData = gyroRW[0];
	rawDataGyro.yData = gyroRW[1];
	rawDataGyro.zData = gyroRW[2];

	return rawDataGyro;
}

ScaledData_TypeDef getScaledAccData()
{
	RawData_TypeDef accelRData;
	accelRData = getRawAccData();

	scaledDataAcc.xData = ((accelRData.xData + 0.0f)*accelScalingFactor);
	scaledDataAcc.yData = ((accelRData.yData + 0.0f)*accelScalingFactor);
	scaledDataAcc.zData = ((accelRData.zData + 0.0f)*accelScalingFactor);

	return scaledDataAcc;
}

ScaledData_TypeDef getScaledGyroData()
{
	RawData_TypeDef gyroRaw;
	gyroRaw = getRawGyroData();

	scaledDataGyro.xData = (gyroRaw.xData)*gyroScalingFactor;
	scaledDataGyro.yData = (gyroRaw.yData)*gyroScalingFactor;
	scaledDataGyro.zData = (gyroRaw.zData)*gyroScalingFactor;

	return scaledDataGyro;
}

uint8_t getSamplerateDivider()
{
	uint8_t buffer_ = 0;

	readData(smplrtDivReg_, &buffer_, 1);
	return buffer_;
}

void setSampleRateDivider(uint8_t SMPRTvalue)
{
	writeData(smplrtDivReg_, SMPRTvalue);
}

uint8_t getFrameSync()
{
	uint8_t buffer_ = 0;

	readData(configReg_, &buffer_, 1);
	buffer_ &= 0x38;
	return (buffer_>>3);
}

void setFrameSync(enum frameSync extSync_)
{
	uint8_t buffer_ = 0;

	readData(configReg_, &buffer_,1);
	buffer_ &= ~0x38;
	buffer_ |= (extSync_ <<3);
	writeData(configReg_, buffer_);
}

void readData(uint8_t addr, uint8_t *i2cBuf, uint8_t len)
{
	uint8_t txBuffer[2];
	uint8_t MPUADDR = (baseAddr_<<1);

	txBuffer[0] = addr;
	HAL_I2C_Master_Transmit(&mpuI2C, MPUADDR, txBuffer, 1, 10);
	HAL_I2C_Master_Receive(&mpuI2C, MPUADDR, i2cBuf, len, 100);
}

void writeData(uint8_t addr, uint8_t data)
{
	uint8_t i2cData[2];
	uint8_t MPUADDR = (baseAddr_<<1);

	i2cData[0] = addr;
	i2cData[1] = data;
	HAL_I2C_Master_Transmit(&mpuI2C, MPUADDR, i2cData, 2, 100);
}

void getAccCalib(ScaledData_TypeDef calibrationData)
{
	ScaledData_TypeDef accelScaled;
	accelScaled = getScaledAccData();

	calibrationData.xData = (accelScaled.xData) - accXBias; // x-Axis
	calibrationData.yData = (accelScaled.yData) - accYBias;// y-Axis
	calibrationData.zData = (accelScaled.zData) - accZBias;// z-Axis
}

void accelCalib(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
	accXBias		= (xMax + xMin)/2.0f;
	accYBias		= (yMax + yMin)/2.0f;
	accZBias		= (zMax + zMin)/2.0f;
}






