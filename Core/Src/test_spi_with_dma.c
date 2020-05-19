/**
 *******************************************************************************
 * @file    test_spi_with_dma.c
 * @author  BesitzeRuf
 * @version
 * @date	 
 * @brief
 *******************************************************************************
 */

#include "test_spi_with_dma.h"
#include "test_spi_with_dma_data.h"
#include "main.h"
#include "stm32f4xx.h"

static SPI_HandleTypeDef hspi2;
static DMA_HandleTypeDef hdma_spi2_rx;
static DMA_HandleTypeDef hdma_spi2_tx;

static void this_InitGpio();
static void this_InitDma();
static void this_InitSpi();


static  uint8_t daraRX[60010U];

void Test_SpiWithDmaUsingHalLibrary()
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();


  this_InitGpio();
  this_InitDma();
  this_InitSpi();


  /* Connect RX with TX pin */
  const uint16_t TransmitCount =  sizeof(daraRX);
  HAL_StatusTypeDef eHalResult = HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)byte_array_hex, (uint8_t *)daraRX, TransmitCount);

  while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
  {
  }

  if (eHalResult != HAL_OK)
  {
    Error_Handler();
  }

  while(1)
  {
    __NOP();
  }
}









static void this_InitGpio()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /**SPI2 GPIO Configuration
  PC2     ------> SPI2_MISO
  PC3     ------> SPI2_MOSI
  PB10     ------> SPI2_SCK
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}



static void this_InitSpi()
{
  __HAL_RCC_SPI2_CLK_ENABLE();

  hspi2.Instance                = SPI2;
  hspi2.Init.Mode               = SPI_MODE_MASTER;
  hspi2.Init.Direction          = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize           = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity        = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase           = SPI_PHASE_1EDGE;
  hspi2.Init.NSS                = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode             = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial      = 7;

  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}


static void this_InitDma()
{

  /* Configure the DMA handler for Transmission process */
  hdma_spi2_tx.Instance                 = DMA1_Stream4;
  hdma_spi2_tx.Init.Channel             = DMA_CHANNEL_0;
  hdma_spi2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_spi2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_spi2_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_spi2_tx.Init.Mode                = DMA_NORMAL;
  hdma_spi2_tx.Init.Priority            = DMA_PRIORITY_LOW;
  hdma_spi2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_spi2_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdma_spi2_tx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_spi2_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

  /* Configure the DMA handler for Receiving process */
  hdma_spi2_rx.Instance                 = DMA1_Stream3;
  hdma_spi2_rx.Init.Channel             = DMA_CHANNEL_0;
  hdma_spi2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_spi2_rx.Init.PeriphInc           = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_spi2_rx.Init.Mode                = DMA_PFCTRL;
  hdma_spi2_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
  hdma_spi2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  hdma_spi2_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_1QUARTERFULL;
  hdma_spi2_rx.Init.MemBurst            = DMA_MBURST_INC4;
  hdma_spi2_rx.Init.PeriphBurst         = DMA_PBURST_INC4;


  if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
  {
    Error_Handler();
  }



  /* Associate the initialized DMA handles to the the SPI handle */
  __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi2_tx);
  __HAL_LINKDMA(&hspi2, hdmarx, hdma_spi2_rx);

  /* NVIC for DMA TX (priority is lower than for DMA RX)*/
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

  /* NVIC for DMA RX */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  /* Priority is lower than for DMA RX/TX */
  HAL_NVIC_SetPriority(SPI2_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);

}



/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmarx);


}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hspi2.hdmatx);
}


void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi2);


}
