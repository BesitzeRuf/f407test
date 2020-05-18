
#include "stdint.h"
#include "xprintf.h"

#include "sdcard.h"

extern UART_HandleTypeDef huart3;

extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

// Отладка
extern TIM_HandleTypeDef htim3;
extern uint8_t report;
extern uint32_t reportData;

uint8_t testflag = 0;

void SDSelect(SDCard_HandleTypeDef sd)
{
	HAL_GPIO_WritePin(sd.cs_port, sd.cs_pin, GPIO_PIN_RESET);
}

void SDUnselect(SDCard_HandleTypeDef sd)
{
	HAL_GPIO_WritePin(sd.cs_port, sd.cs_pin, GPIO_PIN_SET);
}


void SDSendCmd(SDCard_HandleTypeDef sd, uint8_t cmd, uint32_t arg, uint8_t crc)
{
	uint8_t data = cmd;
	
	// TODO: сделать таймаут
	SDWaitNotBusy(&sd);

	// Подаём низкий уровень на CS
	//HAL_GPIO_WritePin(sd.cs_port, sd.cs_pin, GPIO_PIN_RESET);
	
	// Устанавливаем первый бит команды в 1
	data |= 0x40;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);
	
	// Побайтово отправляем аргумент
	data = arg >> 24;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);

	data = arg >> 16;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);

	data = arg >> 8;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);

	data = arg;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);
	
	// Отправляем CRC
	data = crc;
	HAL_SPI_Transmit(&sd.spi, &data, 1, 100);
}

uint8_t SDGetAnswer(SDCard_HandleTypeDef sd)
{
	uint8_t cmd = 0xFF;
	uint8_t response = 0;
	do
	{
		HAL_SPI_TransmitReceive(&sd.spi, &cmd, &response, 1, 100);
		//xprintf("Response: %d - %d\n", response, testflag);
	}
	while (response == 0xFF);

	// Подаём высокий уровень на CS
	//HAL_GPIO_WritePin(sd.cs_port, sd.cs_pin, GPIO_PIN_SET);

	return response;
}

void SDGetAnswerArr(SDCard_HandleTypeDef sd, uint8_t * buff, uint16_t byteCount)
{
	uint8_t cmd = 0xFF;
	
	for(uint16_t i = 0; i < byteCount; i++)
	{
		HAL_SPI_TransmitReceive(&sd.spi, &cmd, buff + i, 1, 100);
	}
}

void SDGetAnswerArrDMA(SDCard_HandleTypeDef * sd, uint8_t * buff, uint16_t byteCount)
{
	sd->isReceiving = 1;
	makeSPIDMARequest(buff, byteCount);
	while (sd->isReceiving == 1);
}

void SDGetAnswerArrDbg(SDCard_HandleTypeDef sd, uint8_t * buff, uint16_t byteCount)
{
	uint8_t cmd = 0xFF;
	
	for(uint16_t i = 0; i < byteCount; i++)
	{
		HAL_SPI_TransmitReceive(&sd.spi, &cmd, buff + i, 1, 100);
		//HAL_UART_Transmit(&huart3, buff + i, 1, 100);
	}
}


uint8_t SDWaitNotBusy(SDCard_HandleTypeDef *sd)
{
	uint8_t busy;
	do
	{
		SDGetAnswerArr(*sd, &busy, 1);
	} while(busy != 0xFF);

    return 0; 
}

void SDWaitDataToken(SDCard_HandleTypeDef *sd, uint8_t token)
{
	uint8_t fb;
	// make sure FF is transmitted during receive
	uint8_t tx = 0xFF;
	for(;;)
	{
		HAL_SPI_TransmitReceive(&sd->spi, &tx, &fb, 1, 100);
		
		if(fb == token)
		{
			break;
		}

		if(fb != 0xFF)
		{
			sd->errCode = ERR_WAIT_TOKEN;
			//xputs("Token errror");
			return;
    }
	}

}

uint8_t SDInit(SDCard_HandleTypeDef * sd)
{
	uint8_t cmd = 0xFF;
	uint8_t out[4];

	// Подаём высокий уровень на CS
	SDUnselect(*sd);
	
	for (uint8_t i = 0; i < 10; i++)
	{
		HAL_SPI_Transmit(&sd->spi, &cmd, 1, 100);
	}

	// Подаём низкий уровень на CS
	SDSelect(*sd);
	
	// Ждём готовности карты
	SDWaitNotBusy(sd);
	
	// Передаём IDLE_COMMAND
	SDSendCmd(*sd, SD_IDLE_CMD, 0, SD_IDLE_COMMAND_CRC);
	sd->lastResponse = SDGetAnswer(*sd);
	
	if (!processResult(sd, 0x01, ERR_INIT_ON_IDLE_CMD, 1))
	{
		return 0;
	}
	
	// Делаем паузу, передаём 0xFF
	//HAL_SPI_Transmit(&(sd->spi), &cmd, 1, 100);

	// Ждём готовности карты
	SDWaitNotBusy(sd);


	// Передаём CMD8
	SDSendCmd(*sd, SD_SEND_IF_COND_CMD, 0x000001AA, SD_SEND_IF_COND_CMD_CRC);
	sd->lastResponse = SDGetAnswer(*sd);

	// Если команда не распознана, то карта старого образца, инициализируем по отдельному алгоритму
	if (sd->lastResponse == 0x05)
	{
		sd->type = SD_TYPE_OLD;
	}
	else
	{
		if (!processResult(sd, 0x01, ERR_INIT_ON_IF_COND_CMD, 0))
		{
			return 0;
		}

		SDGetAnswerArr(*sd, out, 4);

		for (uint8_t i = 0; i < 4; i++)
		{
			sd->buff[i] = out[i];
		}
	}
	
	SDUnselect(*sd);
	HAL_Delay(1);
	SDSelect(*sd);
	
	if (sd->type == SD_TYPE_OLD)
	{
		// Передаём READ_OCR_CMD
		SDSendCmd(*sd, SD_READ_OCR_CMD, 0, SD_READ_OCR_CMD_CRC);
		sd->lastResponse = SDGetAnswer(*sd);
		if (!processResult(sd, 0x01, ERR_INIT_ON_OCR_CMD_OLD_SD, 0))
		{
			return 0;
		}
		
		//uint8_t busy;
		//uint8_t tx = 0xFF;

		/*do
		{
			HAL_SPI_TransmitReceive(&sd->spi, &tx, &busy, 1, 100);
		}
		while(busy == 0xFF);*/

		SDGetAnswerArr(*sd, out, 4);
	
		for (uint8_t i = 0; i < 4; i++)
		{
			sd->buff2[i] = out[i];
		}

		SDUnselect(*sd);
		HAL_Delay(1);
		SDSelect(*sd);

		while(1)
		{
			// Передаём SEND_OP_COND_CMD
			SDSendCmd(*sd, SD_SEND_OP_COND_CMD, 0x00000000, SD_SEND_OP_COND_CMD_CRC);
			sd->lastResponse = SDGetAnswer(*sd);

			if (sd->lastResponse == 0x00)
			{
				break;
			}
			else if (sd->lastResponse != 0x01)
			{
				sd->errCode = ERR_INIT_ON_CMD1_OLD_SD;
				// Подаём высокий уровень на CS
				SDUnselect(*sd);
				return 0;
			}
		}
	}
	else
	{
		while(1)
		{
			// Передаём APP_CMD
			SDSendCmd(*sd, SD_APP_CMD, 0, SD_APP_CMD_CRC);
			sd->lastResponse = SDGetAnswer(*sd);
			
			if (!processResult(sd, 0x01, ERR_INIT_ON_APP_CMD, 1))
			{
				return 0;
			}
		
			// Передаём SEND_OP_COND_CMD
			SDSendCmd(*sd, SD_SEND_OP_COND_CMD, 0x40000000, SD_SEND_OP_COND_CMD_CRC);
			sd->lastResponse = SDGetAnswer(*sd);

			if (sd->lastResponse == 0x00)
			{
				break;
			}
			else if (sd->lastResponse != 0x01)
			{
				// Подаём высокий уровень на CS
				SDUnselect(*sd);
				return 0;
			}
			//HAL_UART_Transmit(&huart, &response, 1, 100);
		}
	
		SDUnselect(*sd);
		HAL_Delay(1);
		SDSelect(*sd);

		// Передаём READ_OCR_CMD
		SDSendCmd(*sd, SD_READ_OCR_CMD, 0, SD_READ_OCR_CMD_CRC);
		sd->lastResponse = SDGetAnswer(*sd);

		if (!processResult(sd, 0x00, ERR_INIT_ON_OCR_CMD_NEW_SD, 0))
		{
			return 0;
		}

		SDGetAnswerArr(*sd, out, 4);
	
		for (uint8_t i = 0; i < 4; i++)
		{
			sd->buff2[i] = out[i];
		}
	
		// Сохраняем значение бита 30: 0 - SD, 1 - SDHC
		sd->isHC = ((out[0] & 64) > 0) ? 1 : 0;

		SDUnselect(*sd);
		HAL_Delay(1);
		SDSelect(*sd);
	}

	// Устанавливаем размер блока 512 байт
	SDSendCmd(*sd, SD_SET_BLOCKLEN_CMD, 512, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_INIT_ON_SET_BLOCKLEN, 1))
	{
		return 0;
	}

	// Подаём высокий уровень на CS
	SDUnselect(*sd);

	return sd->lastResponse;
}

uint8_t readSingleBlock(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff)
{
	uint8_t crc[2];
	
	SDSelect(*sd);
	SDSendCmd(*sd, SD_SEND_SINGLE_BLOCK_CMD, blockNumber, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_SEND_BLOCK, 0))
	{
		return 0;
	}
	
	SDWaitDataToken(sd, SD_DATA_TOKEN_CMD17);
	
	if (sd->errCode > 0)
	{
		SDUnselect(*sd);
		return 0;
	}

	SDGetAnswerArr(*sd, buff, 512);

	SDGetAnswerArr(*sd, crc, 2);	

	SDUnselect(*sd);

	return 0;
}

uint8_t readSingleBlockDMA(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff)
{
	//__HAL_TIM_SET_COUNTER(&htim3, 0);
	uint8_t crc[2];
	
	SDSelect(*sd);
	SDSendCmd(*sd, SD_SEND_SINGLE_BLOCK_CMD, blockNumber, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_SEND_BLOCK, 0))
	{
		return 0;
	}
	
	SDWaitDataToken(sd, SD_DATA_TOKEN_CMD17);
	
	if (sd->errCode > 0)
	{
		SDUnselect(*sd);
		return 0;
	}

	SDGetAnswerArrDMA(sd, buff, 512);

	SDGetAnswerArr(*sd, crc, 2);	

	SDUnselect(*sd);

	//reportData = __HAL_TIM_GET_COUNTER(&htim3);
	//report = 1;

	return 0;
}

uint8_t readMultipleBlocks(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint16_t blockCount, uint8_t * buff)
{
	// Отправляем команду начала чтения нескольких блоков
	SDSelect(*sd);
	SDSendCmd(*sd, SD_SEND_MULTIPLE_BLOCK_CMD, blockNumber, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_SEND_BLOCK, 0))
	{
		return 0;
	}
	
	SDUnselect(*sd);

	// Читаем данные
	uint8_t crc[2];
	for (uint16_t i = 0; i < blockCount; i++)
	{
		SDSelect(*sd);
	
		SDWaitDataToken(sd, SD_DATA_TOKEN_CMD17);
	
		if (sd->errCode > 0)
		{
			SDUnselect(*sd);
			return 0;
		}
	
		SDGetAnswerArr(*sd, buff, 512);
		buff += 512;

		SDGetAnswerArr(*sd, crc, 2);	

		SDUnselect(*sd);
	}
	
	// Отправляем команду окончания чтения
	SDSelect(*sd);
	
	SDSendCmd(*sd, SD_SEND_READ_STOP_CMD, 0x00, SD_READ_STOP_CMD_CRC);
	
	uint8_t tmp;
	// Принимаем 1 байт, его значение не важно
	SDGetAnswerArr(*sd, &tmp, 1);

	//xprintf("Block: %d, count: %d\n", blockNumber, blockCount);	
	
	sd->lastResponse = SDGetAnswer(*sd);
	
	if (!processResult(sd, 0x00, ERR_STOP_READ, 0))
	{
		return 0;
	}
	
	SDUnselect(*sd);
	
	return 0;
}

uint8_t readMultipleBlocksDMA(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint16_t blockCount, uint8_t * buff)
{
	// Отправляем команду начала чтения нескольких блоков
	SDSelect(*sd);
	SDSendCmd(*sd, SD_SEND_MULTIPLE_BLOCK_CMD, blockNumber, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_SEND_BLOCK, 0))
	{
		return 0;
	}
	
	SDUnselect(*sd);

	// Читаем данные
	uint8_t crc[2];
	for (uint16_t i = 0; i < blockCount; i++)
	{
		SDSelect(*sd);
	
		SDWaitDataToken(sd, SD_DATA_TOKEN_CMD17);
	
		if (sd->errCode > 0)
		{
			SDUnselect(*sd);
			return 0;
		}
	
		SDGetAnswerArrDMA(sd, buff, 512);
		buff += 512;

		SDGetAnswerArr(*sd, crc, 2);	

		SDUnselect(*sd);
	}
	
	// Отправляем команду окончания чтения
	SDSelect(*sd);
	SDSendCmd(*sd, SD_SEND_READ_STOP_CMD, 0x00, SD_READ_STOP_CMD_CRC);

	// Принимаем 1 байт, его значение не важно
	uint8_t tmp;
	// Принимаем 1 байт, его значение не важно
	SDGetAnswerArr(*sd, &tmp, 1);
	
	testflag = 1;
	sd->lastResponse = SDGetAnswer(*sd);
	testflag = 0;
	
	if (!processResult(sd, 0x00, ERR_STOP_READ, 0))
	{
		return 0;
	}
	
	SDUnselect(*sd);
	
	return 0;
}


uint8_t writeSingleBlock(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff)
{
	SDSelect(*sd);

	SDSendCmd(*sd, SD_WRITE_BLOCK_CMD, blockNumber, 0xFF);
	sd->lastResponse = SDGetAnswer(*sd);

	if (!processResult(sd, 0x00, ERR_WRITE_BLOCK, 0))
	{
		return 0;
	}

	uint8_t dataToken = SD_DATA_TOKEN_CMD24;
	uint8_t crc[2] = { 0xFF, 0xFF };
	HAL_SPI_Transmit(&sd->spi, &dataToken, sizeof(dataToken), 100);
	HAL_SPI_Transmit(&sd->spi, (uint8_t*)buff, 512, 1000);
	HAL_SPI_Transmit(&sd->spi, crc, sizeof(crc), 100); 
	
	uint8_t dataResponse;
	SDGetAnswerArr(*sd, &dataResponse, 1);
	sd->lastResponse = dataResponse;
	
	if ((sd->lastResponse & 0x1F) != 0x05)
	{
		sd->errCode = ERR_WRITE_BLOCK_CRC;
		SDUnselect(*sd);
		return 0;
	}
	
	HAL_UART_Transmit(&huart3, &sd->lastResponse, 1, 100);
	
	SDWaitNotBusy(sd);

	SDUnselect(*sd);
	return 0;

}

uint8_t processResult(SDCard_HandleTypeDef *sd, uint8_t expectedValue, uint8_t errValue, uint8_t toggleCS)
{
	if (sd->lastResponse != expectedValue)
	{
		// Подаём высокий уровень на CS
		SDUnselect(*sd);
		// Устанавливаем код ошибки
		sd->errCode = errValue;
		return 0;
	}

	if (toggleCS)
	{
		// Дёргаем "туда-сюда" пин CS
		SDUnselect(*sd);
		HAL_Delay(1);
		SDSelect(*sd);
	}

	return 1;
}

void makeSPIDMARequest(uint8_t * buff, uint16_t count)
{
	static uint8_t _filler = 0xFF;

	// Отключаем RX
	__HAL_DMA_DISABLE(&hdma_spi2_rx);
	// Отключаем TX
	__HAL_DMA_DISABLE(&hdma_spi2_tx);

	SET_BIT(hspi2.Instance->CR2, SPI_CR2_RXDMAEN);	
	SET_BIT(hspi2.Instance->CR2, SPI_CR2_TXDMAEN);	

	// Снимаем флаги прерываний
	__HAL_DMA_CLEAR_FLAG(&hdma_spi2_rx, DMA_FLAG_HTIF3_7 | DMA_FLAG_TCIF3_7 | DMA_FLAG_FEIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_DMEIF3_7);
	__HAL_DMA_CLEAR_FLAG(&hdma_spi2_tx, DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4);
	

	//xputs("Here 0.1\n");

	// Настраиваем RX
	hdma_spi2_rx.Instance->PAR = (uint32_t)(&SPI2->DR); //заносим адрес регистра DR в CPAR
  hdma_spi2_rx.Instance->M0AR = (uint32_t)buff; //заносим адрес данных в регистр CMAR
  hdma_spi2_rx.Instance->NDTR = count; //количество передаваемых данных

	hdma_spi2_rx.Instance->CR |= DMA_IT_TC/* | DMA_IT_TE*/;
	__HAL_DMA_ENABLE(&hdma_spi2_rx); //включаем прием данных
	
	//xputs("Here 0.2\n");

	// Настраиваем TX
	hdma_spi2_tx.Instance->PAR = (uint32_t)(&SPI2->DR); //заносим адрес регистра DR в CPAR
  hdma_spi2_tx.Instance->M0AR = (uint32_t)(&_filler); //заносим адрес данных в регистр CMAR
  hdma_spi2_tx.Instance->NDTR = count; //количество передаваемых данных	

	__HAL_DMA_ENABLE(&hdma_spi2_tx); //включаем передачу данных
	//xputs("Here 0.3\n");
}
