/*
 * mpuLib.h
 *
 *  Created on: Jan 8, 2021
 *      Author: elvez
 */

#ifndef INC_MPULIB_H_
#define INC_MPULIB_H_


#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "userHW.h"
#include "main.h"


#define whoAmI_				0x75
#define baseAddr_			0x68
#define pwrMgmtReg_			0x6B
#define configReg_			0x1A
#define configGyroReg_		0x1B
#define configAccReg_		0x1C
#define smplrtDivReg_		0x19
#define intStatusReg_		0x3A
#define xOutAcc_			0x3B
#define tempOutReg_			0x41
#define xOutGyro_			0x43
#define fifoEnReg_	 		0x23
#define intEnableReg_ 		0x38
#define i2cCMACO_	 		(fifoEnReg_)
#define userCtrlReg_		0x6A
#define fifoCountReg_	 	0x72
#define fifoRWReg_	 		0x74


typedef struct
{
	uint8_t 	clockSource;
	uint8_t 	gyroFS_;
	uint8_t		accFS_;
	uint8_t 	configDLPF;
	bool 		sleepMode_;

}MPU_ConfigTypeDef;

enum clockSelect
{
	INTERNAL_8MHZ 		= 0x00,
	X_AXIS_REF			= 0x01,
	Y_AXIS_REF			= 0x02,
	Z_AXIS_REF			= 0x03,
	EXTERNAL_32KHZ		= 0x04,
	EXTERNAL_19_2MHZ	= 0x05,
	TIM_GENT_INREST		= 0x07
};

enum gyroConfig
{
	FS_250 	= 0x00,
	FS_500 	= 0x01,
	FS_1000 = 0x02,
	FS_2000	= 0x03
};

enum accelConfig
{
	AFS_2G	= 0x00,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum digitalLPF
{
	DLPF_260A_256G_Hz	= 0x00,
	DLPF_184A_188G_Hz 	= 0x01,
	DLPF_94A_98G_Hz 	= 0x02,
	DLPF_44A_42G_Hz 	= 0x03,
	DLPF_21A_20G_Hz 	= 0x04,
	DLPF_10_Hz 			= 0x05,
	DLPF_5_Hz 			= 0x06
};

enum frameSync
{
	INPUT_DISABLE   = 0x00,
	TEMP_OUT_L		= 0x01,
	GYRO_XOUT_L		= 0x02,
	GYRO_YOUT_L		= 0x03,
	GYRO_ZOUT_L		= 0x04,
	ACCEL_XOUT_L	= 0x05,
	ACCEL_YOUT_L	= 0x06,
	ACCEL_ZOUT_L	= 0x07
};

typedef struct
{
	int16_t xData;
	int16_t yData;
	int16_t zData;
}RawData_TypeDef;

typedef struct
{
	float xData;
	float yData;
	float zData;
}ScaledData_TypeDef;


int16_t gyroRW[3];

void mpuInit(I2C_HandleTypeDef *i2cHandle);

void mpuConfig(MPU_ConfigTypeDef configuration);

RawData_TypeDef getRawAccData(void);

ScaledData_TypeDef getScaledAccData(void);

RawData_TypeDef getRawGyroData(void);

ScaledData_TypeDef getScaledGyroData(void);


uint8_t getSamplerateDivider(void);

void setSampleRateDivider(uint8_t SMPRTvalue);

uint8_t getFrameSync(void);

void setFrameSync(enum frameSync extSync_);

void readData(uint8_t addr, uint8_t *i2cBuf, uint8_t len);

void writeData(uint8_t addr, uint8_t data);

void getAccCalib(ScaledData_TypeDef calibrationData);

void accelCalib(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);







#endif /* INC_MPULIB_H_ */
