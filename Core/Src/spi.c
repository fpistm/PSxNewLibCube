/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#include "stm32f4xx_ll_spi.h"
// Defines a default timeout delay in milliseconds for the SPI transfer
#ifndef SPI_TRANSFER_TIMEOUT
  #define SPI_TRANSFER_TIMEOUT 1000
#elif SPI_TRANSFER_TIMEOUT <= 0
  #error "SPI_TRANSFER_TIMEOUT cannot be less or equal to 0!"
#endif
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  // Set up the speed, data order and data mode
  // static SPISettings spiSettings (250000, LSBFIRST, SPI_MODE3);

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* In order to set correctly the SPI polarity we need to enable the peripheral */
  __HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief This function is implemented by user to send/receive data over
  *         SPI interface
  * @param  obj : pointer to spi_t structure
  * @param  tx_buffer : tx data to send before reception
  * @param  rx_buffer : rx data to receive if not numm
  * @param  len : length in byte of the data to send and receive
  * @retval status of the send operation (0) in case of error
  */
HAL_StatusTypeDef spi_transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer,
                               uint16_t len)
{
  HAL_StatusTypeDef ret = HAL_ERROR;
  uint32_t tickstart, size = len;
  SPI_TypeDef *_SPI = SPI1;
  uint8_t *tx_buf = (uint8_t *)tx_buffer;

  if (len != 0) {
    tickstart = HAL_GetTick();

    while (size--) {
      while (!LL_SPI_IsActiveFlag_TXE(_SPI));
      LL_SPI_TransmitData8(_SPI, tx_buf ? *tx_buf++ : 0XFF);

      while (!LL_SPI_IsActiveFlag_RXNE(_SPI));
      if (rx_buffer) {
        *rx_buffer++ = LL_SPI_ReceiveData8(_SPI);
      } else {
        LL_SPI_ReceiveData8(_SPI);
      }
      if ((SPI_TRANSFER_TIMEOUT != HAL_MAX_DELAY) &&
          (HAL_GetTick() - tickstart >= SPI_TRANSFER_TIMEOUT)) {
        ret = HAL_TIMEOUT;
        break;
      }
    }

    /* Wait for end of transfer */
    while (LL_SPI_IsActiveFlag_BSY(_SPI));
  }
  return ret;
}

uint8_t transfer(uint8_t data)
{
  spi_transfer(&data, &data, sizeof(uint8_t));
  return data;
}
/* USER CODE END 1 */
