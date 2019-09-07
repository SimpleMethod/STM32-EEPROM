/*
 * eeprom.h
 *
 *  Created on: 10.07.2019
 *      Author: SimpleMethod
 *
 *Copyright 2019 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */


#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stm32f4xx_hal.h"
#include "main.h"

typedef struct {
	I2C_HandleTypeDef* i2c_handel;
	uint16_t* write_address;
	uint16_t* read_address;
	uint16_t i2c_memSize;
	uint16_t block_size;

} EEprom_HandleTypeDef;

EEprom_HandleTypeDef eeprom_init(I2C_HandleTypeDef* i2c_handel,
		uint16_t write_address[], uint16_t read_address[], uint16_t i2c_memSize,
		uint16_t block_size);
HAL_StatusTypeDef eeprom_write_pages(EEprom_HandleTypeDef *eeprom,
		uint8_t pData[], uint16_t Size);
HAL_StatusTypeDef eeprom_write(EEprom_HandleTypeDef *eeprom,
		uint16_t block_select, uint16_t mem_address, uint8_t *pData,
		uint16_t Size);
HAL_StatusTypeDef eeprom_read(EEprom_HandleTypeDef *eeprom,
		uint16_t block_select, uint16_t mem_address, uint8_t *pData,
		uint16_t Size);

#endif /* INC_EEPROM_H_ */
