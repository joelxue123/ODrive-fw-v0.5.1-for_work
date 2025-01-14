#include "icmu.h"

int32_t MUValue = 0;
int32_t test_cnt = 0;
uint8_t buf[9] = {0, 0, 0};
uint8_t spi1_send[4] = {0xA6,0x00,0x00,0x00};
int16_t Ang_reg[1024] = {0};
int16_t Ang_reg_Index = 0;
int16_t Ang_reg_Index_Last = 0;
int16_t Ang_reg_Sum = 0;
int16_t Ang_reg_Count = 0;
int16_t MECode_offset_ICHaus = 0;
uint8_t spi1_reg_buf[4] = {0};
volatile uint8_t spi1_reg[4] = {0};
volatile int32_t *p_MU_Value = (int32_t*)(&spi1_reg[0]);
volatile int32_t MU_Value = 0,MU_Value_last = 0;
volatile int32_t Speed_T = 0;
volatile int32_t Speed_DltAng_pls = 0;
volatile int32_t MU_Vlaue_last = 0;
volatile uint8_t Flag_MU_read = 0;




static int Flag_error_read_eeprom = 0;



uint8_t icmu_spi_init(spi_hardware_t *spi_hardware)
{
  uint8_t icmu_reg = 0;  

  SPI_Init(spi_hardware->spi_handle);


  icmu_reg = icmu_read_reg(spi_hardware,0x77);
	
	return (icmu_reg); 
}
uint8_t icmu_activiate(uint8_t pactive, uint8_t ractive)
{
  SPI1_CS_EN;
  //if (CMD_ACTIVATE == SPI_RW(CMD_ACTIVATE))
  {
    //if (0x20 == SPI_RW(0x80+ pactive + (ractive<<1)))
    {
      SPI1_CS_DIS;
      return 1;
    }
  }
  SPI1_CS_DIS;
  return 0;
}
uint32_t icmu_sdtransmission(void)
{
  uint32_t temp = 0;			
  SPI1_CS_EN;
  //if (CMD_SD_TRANS == SPI_RW(CMD_SD_TRANS))
  {
   // buf[0] = SPI_RW(0);
   // buf[1] = SPI_RW(0);
   // buf[2] = SPI_RW(0);
  }
  temp = buf[0];
  temp = (temp<<8) | buf[1];
  temp = (temp<<8) | buf[2];
  temp = temp>>5;
  SPI1_CS_DIS;
  return  temp;
}


uint8_t icmu_sdstatus(void)
{
  SPI1_CS_EN;
  //if (CMD_SD_STATUS == SPI_RW(CMD_SD_STATUS))
  {
    //if (SPI_RW(0) == 0x80)
    {
      SPI1_CS_DIS;
      return 1;
    }
    //else
    {
      SPI1_CS_DIS;
      return 0;
    }
  }
  return 0;
}



uint8_t icmu_read_reg(spi_hardware_t *spi_hardware,uint8_t addr)
{
  uint8_t reg = 0;
  uint8_t sta = 0;
	uint8_t i =0;
	
	Flag_error_read_eeprom = 1;
	
	
  HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
	delay__ms(1);
  HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
	delay__ms(2);

  if (CMD_REG_READ == SPI_RW(spi_hardware,CMD_REG_READ))
  {

    if (addr == SPI_RW(spi_hardware,addr))
    {
			do
			{
				HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
				delay_clk();
				HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_RESET);
				delay_clk();
				if (CMD_REG_STATUS == SPI_RW(spi_hardware,CMD_REG_STATUS))  // if (CMD_REG_STATUS == SPI_RW(CMD_REG_STATUS))
				{

					sta = SPI_RW(spi_hardware,0);
					reg = SPI_RW(spi_hardware,0);
					if( sta & 0x01)
					{
						if(reg & 0x40)
						{
						}
						else
						{
							Flag_error_read_eeprom =0;
						}
						//g_CmdMap[0x12b] = reg;
						break;
					}
          else
          {
            reg = 0;
          }
				}
				
			}while( i++ < 3);
			
    }
  }
  HAL_GPIO_WritePin(spi_hardware->cs_port, spi_hardware->cs_pin, GPIO_PIN_SET);
  return reg;
}

uint8_t icmu_write_reg(uint8_t addr, uint8_t data)
{
  SPI1_CS_EN;
  //if (CMD_REG_WRITE == SPI_RW(CMD_REG_WRITE))
  {
    //if (addr == SPI_RW(addr))
    {
    //  if (data == SPI_RW(data))
      {
        SPI1_CS_DIS;
        return 1;
      }
    }
  }
  SPI1_CS_DIS;
  return 0;
}

uint8_t icmu_reg_status(void)
{
  SPI1_CS_EN;
  //if (CMD_SD_STATUS == SPI_RW(CMD_SD_STATUS))
  {
  //  if (SPI_RW(0) == 0x80)
    {
      SPI1_CS_DIS;
      return 1;
    }
    //else
    {
      SPI1_CS_DIS;
      return 0;
    }
  }
  SPI1_CS_DIS;
  return 0;
}


/*
0x01 WRITE_ALL Write internal configuration and Offset values to EEPROM
0x02 WRITE_OFF Write internal Offset values to EEPROM
0x03 ABS_RESET Reset of Absolute value (including ABZ-part)
0x04 NON_VER Verification of actual position by doing a nonius calculation
0x05 MT_RESET New read in and synchronisation of multiturn value
0x06 MT_VER Read in of multiturn and verification of counted multiturn value
0x07 SOFT_RESET startup with read in of EEPROM
0x08 SOFT_PRES Set output to preset
0x09 SOFT_E2P_PRES Set output to preset and save offset values to EEPROM
0x0A E2P_COM start EEPROM communication
0x0B EVENT_COUNT increment event counter by 1
0x0C SWITCH A variant of WRITE_ALL to write configurations of MODEA and RPL which inhibit register communications
0x0D CRC_VER Verification of CRC16 and CRC8
0x0E CRC_CALC Recalculate internal CRC16 and CRC8 values
0x0F SET_MTC Set MTC-Pin *)
0x10 RES_MTC Reset MTC-Pin *)
0xFF no function
Note: *) MODE_MT=0x00
*/
void icmu_cmu_mu(uint8_t cmd)
{
	

  icmu_write_reg(0x75,cmd);
}

















int32_t error_encoder = 0;



void clear_eeprom_erro(void)
{
  Flag_error_read_eeprom =0;

}


void icmu_eerpom_erro_detect(void)
{
	if(Flag_error_read_eeprom)
	{

	}
}


