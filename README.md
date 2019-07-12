
# STM32 EEPROM

C-language library for serial EEPROM  on STM32. Supports Microchip's 24AAXX serial EEPROM memory with memory block organization. **The library supports only DMA mode.** 


## Access times

**The time for writing 1024 words is only 4.038 seconds.**
![enter image description here](https://raw.githubusercontent.com/SimpleMethod/STM32-EEPROM/master/images/Allocation%20of%20the%20whole%20memory.png)

**The time for reading is only 22.94 ms, it's really fast ( ಠ ͜ʖಠ)**
![enter image description here](https://raw.githubusercontent.com/SimpleMethod/STM32-EEPROM/master/images/Reading%20the%20whole%20memory.png)

## Pins output
```markdown
| Pin Nb      | PINs           | FUNCTIONs      | LABELs               |
|-------------|----------------|----------------|----------------------|
| 2           | PC13-ANTI_TAMP | GPIO_EXTI13    | B1 [Blue PushButton] |
| 16          | PA2            | USART2_TX      | USART_TX             |
| 17          | PA3            | USART2_RX      | USART_RX             |
| 21          | PA5            | GPIO_Output    | LD2 [Green Led]      |
| 61          | PB8            | I2C1_SCL       |                      |
| 62          | PB9            | I2C1_SDA       |                      |
| PERIPHERALS | MODES          | FUNCTIONS      | PINS                 |
| I2C1        | I2C            | I2C1_SCL       | PB8                  |
| I2C1        | I2C            | I2C1_SDA       | PB9                  |
| SYS         | SysTick        | SYS_VS_Systick | VP_SYS_VS_Systick    |
| USART2      | Asynchronous   | USART2_RX      | PA3                  |
| USART2      | Asynchronous   | USART2_TX      | PA2                  |
```
## DMA configuration
```markdown
| DMA request | Stream       | Direction            | Priority  |
|-------------|--------------|----------------------|-----------|
| I2C1_RX     | DMA1_Stream0 | Peripheral to memory | Very High |
| I2C1_TX     | DMA1_Stream1 | Memory To Peripheral | Very High |
```

## Addresses for EEPROM 24AA08 memory in organization 4x256kx8bit

| Operation | Control Code | Block Select | Block Select as a binary | Read/Write | Result in binary form | Result in hexadecimal form |
|-----------|--------------|--------------|--------------------------|------------|-----------------------|----------------------------|
| Write     | 1010         | 0            | 000                      | 0          | 10100000              | A0                         |
| Read      | 1010         | 0            | 000                      | 1          | 10100001              | A1                         |
| Write     | 1010         | 1            | 001                      | 0          | 10100010              | A2                         |
| Read      | 1010         | 1            | 001                      | 1          | 10100011              | A3                         |
| Write     | 1010         | 2            | 010                      | 0          | 10100100              | A4                         |
| Read      | 1010         | 2            | 010                      | 1          | 10100101              | A5                         |
| Write     | 1010         | 3            | 011                      | 0          | 10100110              | A6                         |
| Read      | 1010         | 3            | 011                      | 1          | 10100111              | A7                         |


## Example of use

	char memory_dump[1024] = {0}; //8kbit
	char example_1 = *"T";
	char example_2[] ="9A8B7C6D5E4F3G2H1I0J...............................................................................0...................................................................................................1...................................................................................................2...................................................................................................3...................................................................................................4...................................................................................................5...................................................................................................6...................................................................................................7...................................................................................................8...................................................................................................9.....................END";
	EEprom_HandleTypeDef EEprom_;

	uint16_t write_address[] = { 0xA0, 0xA2, 0xA4, 0xA6 };

	uint16_t read_address[] = { 0xA1, 0xA3, 0xA5, 0xA7 };

	EEprom_ = eeprom_init(&hi2c1, write_address, read_address,I2C_MEMADD_SIZE_8BIT, 256);

	eeprom_write(&EEprom_, 0, 0x00, (uint8_t*) &example_1,sizeof(example_1));

	eeprom_write_pages(&EEprom_, (uint8_t*) &example_2,sizeof(example_2));

	HAL_Delay(10);
	eeprom_read(&EEprom_, 0, 0x00, (uint8_t*) &memory_dump,sizeof(memory_dump));
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2, (uint8_t*) &memory_dump, sizeof(memory_dump),0xffffff);



## List of available functions 

    EEprom_HandleTypeDef eeprom_init(I2C_HandleTypeDef* i2c_handel,uint16_t write_address[], uint16_t read_address[], uint16_t i2c_memSize,uint16_t block_size);
    HAL_StatusTypeDef eeprom_write_pages(EEprom_HandleTypeDef *eeprom,uint8_t pData[], uint16_t Size);
    HAL_StatusTypeDef eeprom_write(EEprom_HandleTypeDef *eeprom,uint16_t block_select, uint16_t mem_address, uint8_t *pData,uint16_t Size);
    HAL_StatusTypeDef eeprom_read(EEprom_HandleTypeDef *eeprom,uint16_t block_select, uint16_t mem_address, uint8_t *pData,uint16_t Size);



