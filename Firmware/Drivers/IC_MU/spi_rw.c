#include "spi_rw.h"





 void delay_clk(void)
{
  volatile int i = 2000;
  while(i--)
    __NOP();
}


void delay__ms(int cnt)
{
	volatile int i=0,j=0;
	
	for(i =0 ;i<1000;i++)
		for(j=0;j<cnt;j++);
}
	






void SPI_Init(SPI_HandleTypeDef * hspi_handler)
{
   /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  SPI_HandleTypeDef * hspi = hspi_handler;
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.CRCPolynomial = 7;
__HAL_SPI_DISABLE(hspi);
  HAL_SPI_DeInit(hspi);
  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    Error_Handler();
  }
	hspi->Instance->DR;
  //SET_BIT(SPI1->CR2, SPI_RXFIFO_THRESHOLD);
  __HAL_SPI_ENABLE(hspi);
}


void SPI_I2S_SendData8(SPI_TypeDef* SPIx, uint8_t Data)
{
  uint32_t spixbase = 0x00;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));

  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  *(__IO uint8_t *) spixbase = Data;
}


uint8_t SPI_I2S_ReceiveData8(SPI_TypeDef* SPIx)
{
  uint32_t spixbase = 0x00;
  
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH_EXT(SPIx));
  
  spixbase = (uint32_t)SPIx; 
  spixbase += 0x0C;
  
  return *(__IO uint8_t *) spixbase;
}

uint16_t SPI_RW(spi_hardware_t *spi_hardware,uint16_t data)
{
  /* Loop while DR register in not emplty */
  while(__HAL_SPI_GET_FLAG(spi_hardware->spi_handle, SPI_FLAG_TXE) == RESET);
  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData8(spi_hardware->spi_handle->Instance, data);
  /* Wait to receive a byte */
  while((__HAL_SPI_GET_FLAG(spi_hardware->spi_handle, SPI_FLAG_RXNE) == RESET));
  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData8(spi_hardware->spi_handle->Instance);  
}


