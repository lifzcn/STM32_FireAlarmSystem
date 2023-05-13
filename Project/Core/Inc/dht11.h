#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim.h"

#define DHT11_DQ_OUT_HIGH HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET)
#define DHT11_DQ_OUT_LOW HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET)
#define DHT11_DQ_IN HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)

void DS18B20_IO_IN();
void DS18B20_IO_OUT();

uint8_t DHT11_Init();
uint8_t DHT11_Read_Data(uint16_t *temp, uint16_t *humi);
uint8_t DHT11_Check();
void DHT11_Rst();

#endif
