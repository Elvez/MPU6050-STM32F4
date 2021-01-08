/*
 * userHW.c
 *
 *  Created on: Jan 5, 2021
 *      Author: Pravesh Narayan
 */


#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "userHW.h"

void sendMessage(char* format, ...)
{
	char message[100];
	va_list args;
	va_start(args, format);
	vsprintf(message, format, args);
#ifdef USBDEVICE_
	CDC_Transmit_FS((uint8_t*)message, strlen(message));
#endif
#ifdef UARTDEVICE_
	HAL_UART_Transmit(pUART_, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
#endif
	va_end(args);
}

void LEDOn()
{
	HAL_GPIO_WritePin(infoPort_, infoLED_, stateOn_);
}

void LEDOff()
{
	HAL_GPIO_WritePin(infoPort_, infoLED_, stateOff_);
}

GPIO_PinState isButtonPressed()
{
	GPIO_PinState state_;

	state_ = HAL_GPIO_ReadPin(userPort_, userButton_);
	return state_;
}

void infoLED(uint8_t turns, uint32_t delay)
{
	for(uint8_t iter = 0; iter < 2*turns; iter++)
	{
		HAL_GPIO_TogglePin(infoPort_, infoLED_);
		HAL_Delay(delay);
	}
	LEDOff();
}

void iicTransmit(iicData transData)
{
	transData.timeout_ = HAL_MAX_DELAY;

	HAL_I2C_Master_Transmit(transData.typeDef_, transData.baseAddress_, transData.pData_, transData.dataSize_, transData.timeout_);
}

void rxUart(uint8_t* data, uint16_t len)
{
#ifdef UARTDEVICE_
	HAL_UART_Receive(pUART_, data, len, HAL_MAX_DELAY);
#endif
}
