/*
 * userHW.h
 *
 *  Created on: Jan 5, 2021
 *      Author: Pravesh Narayan
 */

#ifndef INC_USERHW_H_
#define INC_USERHW_H_
#define USBDEVICE_
//#define UARTDEVICE_


#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

#define infoLED_	GPIO_PIN_13
#define infoPort_	GPIOC
#define	userButton_	GPIO_PIN_0
#define userPort_	GPIOA
#define stateOn_	GPIO_PIN_RESET
#define stateOff_	GPIO_PIN_SET
#define pI2C_		&hi2c1

#ifdef	UARTDEVICE_
#define pUART_		&huart1
#endif

typedef struct iicData
{
	I2C_HandleTypeDef* typeDef_;
	uint16_t baseAddress_;
	uint8_t* pData_;
	uint16_t dataSize_;
	uint32_t timeout_;
} iicData;

void infoLED(uint8_t turns, uint32_t delay);

void LEDOn(void);

void LEDOff(void);

GPIO_PinState isButtonPressed(void);

void sendMessage(char* format,...);

void iicTransmit(iicData transData);

void rxUart(uint8_t* data, uint16_t len);


#endif /* INC_USERHW_H_ */
