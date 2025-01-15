#include "ma600.h"


uint16_t readMagAlphaAngle(spi_hardware_t *spi_hardware);
uint8_t readMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address);
uint8_t writeMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address, uint8_t value);


uint8_t readbackRegValue;

void MA600_Init(spi_hardware_t *spi_hardware)
{
	
  uint8_t regAddress;
  uint8_t regValue;
  bool error;
	
	//Example of MagAlpha Register Settings (Set Reg 0 to 0x80)
  regAddress = 0;
  regValue = 0x80;

  SPI_Init(spi_hardware->spi_handle);

  //Read the initial register value
  readbackRegValue=readMagAlphaRegister(spi_hardware,regAddress);
  //write the register with the desired value
  readbackRegValue=writeMagAlphaRegister(spi_hardware,regAddress, regValue);

  //remove warning during compilation
  (void)readbackRegValue;
  (void)error;
}

uint16_t readMagAlphaAngle(spi_hardware_t *spi_hardware)
{
    uint8_t txData[2];
    uint8_t rxData[2];
    uint8_t tx_cnt = 0;
    uint8_t rx_cnt = 0;


    txData[1]=0;
    txData[0]=0;
    uint16_t angleSensor;
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);

    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);

    angleSensor=rxData[0]<<8 | rxData[1];
    return angleSensor;
}

uint8_t readMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address)
{

    uint8_t txData[4];
    uint8_t rxData[4];
    uint8_t tx_cnt = 0;
    uint8_t rx_cnt = 0;

    txData[0]=MA600_CMD_REG_READ;
    txData[1]=address;
    txData[2]=0x00;
    txData[3]=0x00;
    uint8_t registerReadbackValue;


    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);

    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);

    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);


  registerReadbackValue=rxData[2];
  return registerReadbackValue;
}

uint8_t writeMagAlphaRegister(spi_hardware_t *spi_hardware,uint8_t address, uint8_t value)
{
    uint8_t txData[6];
    uint8_t rxData[6];
    uint8_t tx_cnt = 0;
    uint8_t rx_cnt = 0;

    txData[0]=MA600_CMD_REG_WRITE&0xff;
    txData[1]=(MA600_CMD_REG_WRITE>>8)&0xff;
    txData[2]=address;
    txData[3]=value;
    txData[4]=0x00;
    txData[5]=0x00;
    uint8_t registerReadbackValue;


    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);

    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++] = SPI_RW(spi_hardware,txData[tx_cnt++]);

    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);

    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
    delay__ms(1);
    HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
    delay__ms(1);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);
    rxData[rx_cnt++]  = SPI_RW(spi_hardware,txData[tx_cnt++]);

  registerReadbackValue=rxData[4];
  return registerReadbackValue;
}

