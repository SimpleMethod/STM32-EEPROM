/*
 * eeprom.c
 *
 *  Created on: 11.07.2019
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
#include "eeprom.h"

EEprom_HandleTypeDef eeprom_init(I2C_HandleTypeDef* i2c_handel,
		uint16_t write_address[], uint16_t read_address[], uint16_t i2c_memSize,
		uint16_t block_size) {
	EEprom_HandleTypeDef EEprom_;

	EEprom_.i2c_handel = i2c_handel;
	EEprom_.write_address = write_address;
	EEprom_.read_address = read_address;
	EEprom_.i2c_memSize = i2c_memSize;
	EEprom_.block_size = block_size;
	return EEprom_;
}

HAL_StatusTypeDef eeprom_write_pages(EEprom_HandleTypeDef *eeprom,
		uint8_t *pData, uint16_t Size) {
	uint8_t address = 0, segment = 0;
	uint16_t counter = 0;
	while (counter < Size - 1) {
		if (counter == eeprom->block_size * segment) {
			segment++;
			address = 0;
		}
		if(HAL_I2C_Mem_Write_DMA(eeprom->i2c_handel,eeprom->write_address[0+segment-1], 0x00+address,eeprom->i2c_memSize, &pData[counter], sizeof(pData[counter]))==HAL_ERROR)
		{
			return HAL_ERROR;
		}
		counter++;
		address++;
		HAL_Delay(3);
	}
return HAL_OK;
}

HAL_StatusTypeDef eeprom_write(EEprom_HandleTypeDef *eeprom,
		uint16_t block_select, uint16_t MemAddress, uint8_t *pData,
		uint16_t Size) {
	return HAL_I2C_Mem_Write_DMA(eeprom->i2c_handel,
			eeprom->write_address[block_select], MemAddress,
			eeprom->i2c_memSize, pData, Size);
}

HAL_StatusTypeDef eeprom_read(EEprom_HandleTypeDef *eeprom,
		uint16_t block_select, uint16_t MemAddress, uint8_t *pData,
		uint16_t Size) {
	return HAL_I2C_Mem_Read_DMA(eeprom->i2c_handel,
			eeprom->read_address[block_select], MemAddress, eeprom->i2c_memSize,
			*(&pData), Size);
}
