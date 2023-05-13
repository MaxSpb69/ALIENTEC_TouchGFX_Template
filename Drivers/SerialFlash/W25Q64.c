/*
 * AT25DF321.c
 *
 *  Created on: Jun 16, 2021
 *      Author: maxsp
 */

#include "stm32f4xx_hal.h"
#include "main.h"

#define FLASH_SPI	hspi1

extern SPI_HandleTypeDef FLASH_SPI;


static volatile uint8_t isReceivingData = 0;


void DataReader_ReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
{

	uint8_t byte;
	uint32_t l;

	if(isReceivingData == 0)
	{

		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
		byte = 0x0b;
		HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);

		byte = ((address24 & 0xFF0000) >> 16);
		HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
		byte = ((address24 & 0xFF00) >> 8);
		HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
		byte = (address24 & 0xFF);
		HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
		byte = 0xa5;
		HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);

		HAL_SPI_Receive(&FLASH_SPI, buffer, length, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
	}
}

void DataReader_StartDMAReadData(uint32_t address24, uint8_t* buffer, uint32_t length)
{
//	DataReader_ReadData(address24, buffer, length);
//	return;

	uint8_t byte;

	isReceivingData = 1;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

	byte = 0x0b;
	HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);

	byte = ((address24 & 0xFF0000) >> 16);
	HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
	byte = ((address24 & 0xFF00) >> 8);
	HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
	byte = (address24 & 0xFF);
	HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);
	byte = 0xa5;
	HAL_SPI_Transmit(&FLASH_SPI, &byte, 1, HAL_MAX_DELAY);

	HAL_SPI_Receive_DMA(&FLASH_SPI, buffer, length);


}




void DataReader_WaitForReceiveDone(void)
{
	while(isReceivingData);
}


void SPI_DMA_Transfer_Complete_Callback(void)
{
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
		isReceivingData = 0;
}


HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == &FLASH_SPI)
		SPI_DMA_Transfer_Complete_Callback();
}



void CopyFlashToSdram(uint32_t len)
{

	uint32_t num_full_blocks, last_block_len;
	uint16_t i;

	num_full_blocks = len / 65535;
	last_block_len = len - (num_full_blocks * 65535);
	if(num_full_blocks > 0)
	{
		for (i = 0; i < num_full_blocks; i++)
		{
			while(isReceivingData);
			DataReader_StartDMAReadData(i * 65535 , (uint8_t*)(0xc0300000 + (i * 65535)), 65535);
		}
	}

	while(isReceivingData);

	if(last_block_len > 0)
	{
		DataReader_StartDMAReadData(i * 65535 , (uint8_t*)(0xc0300000 + (i * 65535)), last_block_len);
		while(isReceivingData);
	}

}
