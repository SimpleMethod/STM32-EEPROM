/*
 * eeprom.h
 *
 *  Created on: 10.07.2019
 *      Author: Pathfinder
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
