#include "dht11.h"

void DHT11_IO_IN()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.Pin = DHT11_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStructure);
}

void DHT11_IO_OUT()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.Pin = DHT11_Pin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStructure);
}

void DHT11_Rst()
{
	DHT11_IO_OUT();
	DHT11_DQ_OUT_LOW;
	HAL_Delay(20);
	DHT11_DQ_OUT_HIGH;
	HAL_Delay_us(30);
}

uint8_t DHT11_Check()
{
	uint8_t retry = 0;
	DHT11_IO_IN();
	while (DHT11_DQ_IN && retry < 100)
	{
		retry++;
		HAL_Delay_us(1);
	};
	if (retry >= 100)
	{
		return 1;
	}
	else
	{
		retry = 0;
	}
	while (!DHT11_DQ_IN && retry < 100)
	{
		retry++;
		HAL_Delay_us(1);
	};
	if (retry >= 100)
	{
		return 1;
	}
	return 0;
}

uint8_t DHT11_Read_Bit()
{
	uint8_t retry = 0;
	while (DHT11_DQ_IN && retry < 100)
	{
		retry++;
		HAL_Delay_us(1);
	}
	retry = 0;
	while (!DHT11_DQ_IN && retry < 100)
	{
		retry++;
		HAL_Delay_us(1);
	}
	HAL_Delay_us(40);
	if (DHT11_DQ_IN)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t DHT11_Read_Byte()
{
	uint8_t i, dat;
	dat = 0;
	for (i = 0; i < 8; i++)
	{
		dat <<= 1;
		dat |= DHT11_Read_Bit();
	}
	return dat;
}

uint8_t DHT11_Read_Data(uint16_t *temp, uint16_t *humi)
{
	uint8_t buf[5];
	uint8_t i;
	DHT11_Rst();
	if (DHT11_Check() == 0)
	{
		for (i = 0; i < 5; i++)
		{
			buf[i] = DHT11_Read_Byte();
		}
		if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
		{
			*humi = (buf[0] << 8) + buf[1];
			*temp = (buf[2] << 8) + buf[3];
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

uint8_t DHT11_Init()
{
	DHT11_Rst();
	return DHT11_Check();
}
