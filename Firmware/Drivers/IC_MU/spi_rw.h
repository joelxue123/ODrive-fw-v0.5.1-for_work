#ifndef __SPI_RW_H__
#define __SPI_RW_H__ 

#include "stdbool.h"
#include "stdint.h"

// drivers

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	SPI_HandleTypeDef *spi_handle;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;

}spi_hardware_t;


void delay_clk(void);
void SPI_Init(SPI_HandleTypeDef * hspi_handler);
uint16_t SPI_RW(spi_hardware_t *spi_hardware,uint16_t data);
void delay__ms(int cnt);



#ifdef __cplusplus
}
#endif


#endif


