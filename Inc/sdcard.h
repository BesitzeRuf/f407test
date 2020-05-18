#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "stm32f4xx_hal.h"
#include "stdint.h"

#define SD_IDLE_CMD 0x00
#define SD_CMD1 0x01
#define SD_SEND_IF_COND_CMD 0x08
#define SD_SEND_READ_STOP_CMD 0x0C
#define SD_SET_BLOCKLEN_CMD 0x10
#define SD_SEND_SINGLE_BLOCK_CMD 0x11	// CMD17
#define SD_SEND_MULTIPLE_BLOCK_CMD 0x12	// CMD18
#define SD_WRITE_BLOCK_CMD 0x18
#define SD_SEND_OP_COND_CMD 0x29
#define SD_APP_CMD 0x37
#define SD_READ_OCR_CMD 0x3A

#define SD_IDLE_COMMAND_CRC 0x95
#define SD_SEND_IF_COND_CMD_CRC 0x87
#define SD_APP_CMD_CRC (0x7F << 1) | 1
#define SD_SEND_OP_COND_CMD_CRC (0x7F << 1) | 1
#define SD_READ_OCR_CMD_CRC (0x7F << 1) | 1
#define SD_READ_STOP_CMD_CRC (0x7F << 1) | 1

#define SD_TYPE_OLD 1
#define SD_TYPE_NEW 2

#define ERR_INIT_ON_IDLE_CMD 1
#define ERR_INIT_ON_IF_COND_CMD 2
#define ERR_INIT_ON_OCR_CMD_OLD_SD 3
#define ERR_INIT_ON_OCR_CMD_NEW_SD 4
#define ERR_INIT_ON_CMD1_OLD_SD 5
#define ERR_INIT_ON_APP_CMD 6
#define ERR_INIT_ON_SET_BLOCKLEN 7
#define ERR_SEND_BLOCK 8
#define ERR_WRITE_BLOCK 9
#define ERR_WRITE_BLOCK_CRC 10
#define ERR_WAIT_TOKEN 11
#define ERR_STOP_READ 12

#define SD_DATA_TOKEN_CMD17 0xFE
#define SD_DATA_TOKEN_CMD18 0xFE
#define SD_DATA_TOKEN_CMD24 0xFE

/**
  * @brief  TIM Time Base Handle Structure definition 
  */ 
typedef struct
{
	SPI_HandleTypeDef spi;
	GPIO_TypeDef *		cs_port;
	uint16_t					cs_pin;
	
	volatile uint8_t isReceiving; 
	
	uint8_t lastResponse;
	uint8_t errCode;
	uint8_t isHC;
	uint8_t type;
	
	uint8_t buff[4];
	uint8_t buff2[4];
	
	

}SDCard_HandleTypeDef;

void SDSelect(SDCard_HandleTypeDef sd);
void SDUnselect(SDCard_HandleTypeDef sd);

void SDSendCmd(SDCard_HandleTypeDef sd, uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SDGetAnswer(SDCard_HandleTypeDef sd);
void SDGetAnswerArr(SDCard_HandleTypeDef sd, uint8_t * buff, uint16_t byteCount);
void SDGetAnswerArrDMA(SDCard_HandleTypeDef * sd, uint8_t * buff, uint16_t byteCount); 
void SDGetAnswerArrDbg(SDCard_HandleTypeDef sd, uint8_t * buff, uint16_t byteCount);

uint8_t SDWaitNotBusy(SDCard_HandleTypeDef *sd);
void SDWaitDataToken(SDCard_HandleTypeDef *sd, uint8_t token);


uint8_t SDInit(SDCard_HandleTypeDef *sd);

uint8_t readSingleBlock(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff);
uint8_t readSingleBlockDMA(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff); 
uint8_t readMultipleBlocks(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint16_t blockCount, uint8_t * buff);
uint8_t readMultipleBlocksDMA(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint16_t blockCount, uint8_t * buff);
uint8_t writeSingleBlock(SDCard_HandleTypeDef *sd, uint32_t blockNumber, uint8_t * buff);

uint8_t processResult(SDCard_HandleTypeDef *sd, uint8_t expectedValue, uint8_t errValue, uint8_t toggleCS);

void makeSPIDMARequest(uint8_t * buff, uint16_t count); 


#endif // __SDCARD_H__
