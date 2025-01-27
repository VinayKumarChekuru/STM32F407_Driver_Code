/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jan 21, 2025
 *      Author: vinaykumar
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include"stm32f4xx.h"
#include<stdint.h>


#define SPI_RXNE_FLAG                (1<<0)
#define SPI_TXE_FLAG                 (1<<1)
#define SPI_CHSIDE_FLAG              (1<<2)
#define SPI_UDR_FLAG                 (1<<3)
#define SPI_CRCERR_FLAG              (1<<4)
#define SPI_MODF_FLAG                (1<<5)
#define SPI_OVR_FLAG                 (1<<6)
#define SPI_BSY_FLAG                 (1<<7)
#define SPI_FRE_FLAG                 (1<<8)

#define SPI_DFF_FLAG                 (1<11)



#define SPI_DEVICEMODE_Master       1
#define SPI_DEVICEMODE_Slave        0

#define SPI_BUSCONFIG_FD            1
#define SPI_BUSCONFIG_HD            2
#define SPI_BUSCONFIG_S_RX_ONLY     3

#define SPI_SCLK_SPEED_DIV2         0
#define SPI_SCLK_SPEED_DIV4         1
#define SPI_SCLK_SPEED_DIV8         2
#define SPI_SCLK_SPEED_DIV16        3
#define SPI_SCLK_SPEED_DIV32        4
#define SPI_SCLK_SPEED_DIV64        5
#define SPI_SCLK_SPEED_DIV128       6
#define SPI_SCLK_SPEED_DIV256       7


#define SPI_DFF_8BIT                0
#define SPI_DFF_16BIT               1

#define SPI_CPOL_LOW                0
#define SPI_CPOL_HIGH               1

#define SPI_CPHA_LOW                0
#define SPI_CPHA_HIGH               1

#define SPI_SSM_DI                  0
#define SPI_SSM_EN                  1


#define SPI_MODE0                   0 //Both CPOL and CPHA are 0's
#define SPI_MODE1                   1 //CPOL=0 & CPHA=1
#define SPI_MODE2                   2 //CPOL=1 & CPHA=0
#define SPI_MODE3                   3 //CPOL=1 & CPHA=1










/*
 * SPI configuration structure
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * SPI Handle structure
 */

typedef struct
{
	SPI_Config_t SPIConfig;
	SPI_RegDef_t *pSPIx;
}SPI_Handle_t;

/*
 * SPI Prototypes decleration
 */


/*
 * SPI clock control
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * SPI Initialization and GPIO Deinitialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive

 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * SPI interrupts
 */


void SPI_IRQItConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
