/**
  ******************************************************************************

  EEPROM.h Using the HAL I2C Functions
  Author:   ControllersTech
  Updated:  Feb 16, 2021

  ******************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ******************************************************************************
*/

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef struct {
    uint32_t baudrate;
    uint32_t parity;
    uint32_t wordLength;
    uint32_t stopBits;
} UART_Params;

extern UART_Params uartParams;
extern uint8_t uartParamsData[sizeof(UART_Params)];

void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_PageErase (uint16_t page);

void EEPROM_Write_NUM (uint16_t page, uint16_t offset, float  fdata);
float EEPROM_Read_NUM (uint16_t page, uint16_t offset);

void EEPROM_Write_UART_Params(uint8_t page, uint16_t address, UART_Params* params);
void EEPROM_Read_UART_Params(uint8_t page, uint16_t address, UART_Params* params);


#endif /* INC_EEPROM_H_ */
