/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jan 21, 2025
 *      Author: vinaykumar
 */

#include<stdint.h>

#include"stm32f407xx_spi_driver.h"

uint8_t SPI_Get_Flag_Status(SPI_RegDef_t *pSPIx, uint32 Flag)
{
	if((pSPIx->SR) & Flag)
	{
		return SET;
	}
	return RESET;
}


/*
 * SPI clock control
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI3_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI4_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI5_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI6_CLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI3_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI4_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI5_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI6_CLK_DI();
		}
	}
}

/*
 * SPI Initialization and GPIO De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
//	uint32_t temp = (uint32_t)(0xFFF0 + pSPIHandle->SPIConfig.SPI_CPHA )
//     pSPIHandle->pSPIx->CR1 |= (0<<pSPIHandle->SPIConfig.SPI_CPHA);

	uint16_t tempReg = 0;

	//Configure device as master or slave

	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode<<2;

	//Configure the bus config

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
	{
		//Clear BIDIMODE bit in CR1
		tempReg &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD)
	{
		// Set BIDIMODE bit in CR1
		tempReg |= (1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_S_RX_ONLY)
	{
         //	Set BIDIMODE bit in CR1
		  tempReg &= ~(1<<SPI_CR1_BIDIMODE);

		  //Set RXONLY bit in CR1

		  tempReg |= (1<<SPI_CR1_RXONLY);
	}

	//Configure SPI CLK frequency
     //Set the Sclk as fPCLK/2

		tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//Configure the size of DFF

		tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//Configure CPOL and CPHA

		tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);
		tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configure Software slave management

		tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM) ;

		pSPIHandle->pSPIx->CR1 = tempReg ;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
  if(pSPIx == SPI1)
  {
	  SPI1_RST();
  }
  else if(pSPIx == SPI2)
  {
	  SPI2_RST();
  }
  else if(pSPIx == SPI3)
  {
	  SPI3_RST();
  }
  else if(pSPIx == SPI4)
  {
	  SPI4_RST();
  }
  else if(pSPIx == SPI5)
  {
	  SPI5_RST();
  }
  else if(pSPIx == SPI6)
  {
	  SPI6_RST();
  }
}


/*
 * Data send and receive

 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
  while(len > 0)
  {
	  //waits until TXE is empty
	  while(SPI_Get_Flag_Status(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

	  if((pSPIx->CR1) & SPI_DFF_FLAG)
	  {
		  //16 bit data transfer

		  //load value of pTxBuffer into DR register
		  pSPIx->DR = *((uint16_t*)pTxBuffer);
		  len--;
		  len--;
		  (uint16_t*)pTxBuffer++;
	  }
	  else
	  {
		  //8 bit data transfer
		  pSPIx->DR = *pTxBuffer;
		  len--;
		  pTxBuffer++;

	  }
  }
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}

/*
 * SPI interrupts
 */


void SPI_IRQItConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}
