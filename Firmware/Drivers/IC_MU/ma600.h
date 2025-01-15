#ifndef __MA600_H__
#define __MA600_H__ 

#include "stdbool.h"
#include "stdint.h"
#include "spi_rw.h"

// drivers

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif



#define MA600_CMD_REG_READ		0XD2   //
#define MA600_CMD_REG_WRITE		0Xea54	 //
#define MA600_CMD_STORE_NVM  0Xea55
#define MA600_CMD_NVM_0  0Xea00
#define MA600_CMD_NVM_1  0Xea01







void MA600_Init(spi_hardware_t *spi_hardware);
uint16_t readMagAlphaAngle(spi_hardware_t *spi_hardware);
uint8_t readMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address);
uint8_t writeMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address, uint8_t value);




#ifdef __cplusplus
}
#endif


#endif


