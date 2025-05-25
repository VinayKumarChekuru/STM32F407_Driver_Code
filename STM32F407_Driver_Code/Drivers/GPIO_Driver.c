/*
 * GPIO_Driver.c
 *
 *  Created on: Mar 8, 2025
 *      Author: vinaykumar
 */
#include"GPIO_Driver.h"

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_Clk_En();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_Clk_En();
		}
		else if(pGPIOx == GPIOC)
				{
			GPIOC_Clk_En();
				}
		else if(pGPIOx == GPIOD)
				{
			GPIOD_Clk_En();
				}
		else if(pGPIOx == GPIOE)
				{
			GPIOE_Clk_En();
				}
		else if(pGPIOx == GPIOF)
				{
			GPIOF_Clk_En();
				}
		else if(pGPIOx == GPIOG)
				{
			GPIOG_Clk_En();
				}
		else if(pGPIOx == GPIOH)
				{
			GPIOH_Clk_En();
				}
		else if(pGPIOx == GPIOI)
				{
			GPIOI_Clk_En();
				}
	}
	else if(pGPIOx == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_Clk_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_Clk_DI();
		}
		else if(pGPIOx == GPIOC)
				{
			GPIOC_Clk_DI();
				}
		else if(pGPIOx == GPIOD)
				{
			GPIOD_Clk_DI();
				}
		else if(pGPIOx == GPIOE)
				{
			GPIOE_Clk_DI();
				}
		else if(pGPIOx == GPIOF)
				{
			GPIOF_Clk_DI();
				}
		else if(pGPIOx == GPIOG)
				{
			GPIOG_Clk_DI();
				}
		else if(pGPIOx == GPIOH)
				{
			GPIOH_Clk_DI();
				}
		else if(pGPIOx == GPIOI)
				{
			GPIOI_Clk_DI();
				}
	}
}

void GPIO_Init(GPIO_RegDef_t *pGPIOX,
		uint8_t GPIO_PinNumber,
		uint8_t GPIO_PinMode,
		uint8_t GPIO_PinSpeed,
		uint8_t GPIO_PinPuPdControl,
		uint8_t GPIO_PinOPType,
		uint8_t GPIO_PinAltFunMode)
{
	//set mode of a GPIO pin
	uint32_t tempReg = 0;
	tempReg |= (GPIO_PinMode << 2*GPIO_PinNumber);
    pGPIOX->MODER &= ~(3 << 2*GPIO_PinNumber); //It will claer the bit field of MOder
    pGPIOX->MODER |= tempReg;

    //set GPIO pin speed
    tempReg = 0;
    tempReg |= (GPIO_PinSpeed << 2*GPIO_PinNumber);
    pGPIOX->OSPEEDR &= ~(3 << 2*GPIO_PinNumber); //It will claer the bit field of MOder
    pGPIOX->OSPEEDR |= tempReg;

    //set pull up pull down control

    tempReg = 0;
    tempReg |= (GPIO_PinPuPdControl << 2*GPIO_PinNumber);
    pGPIOX->PUPDR &= ~(3 << 2*GPIO_PinNumber); //It will claer the bit field of MOder
    pGPIOX->PUPDR |= tempReg;


    //set Output type
    tempReg = 0;
    tempReg |= ( GPIO_PinOPType << GPIO_PinNumber);
    pGPIOX->OTYPER &= ~(1 << GPIO_PinNumber); //It will claer the bit field of MOder
    pGPIOX->OTYPER |= tempReg;


    //set alternate functionality

    if(GPIO_PinNumber <= 7)
    {
    	//alternate function low register
    	tempReg = 0;
    	tempReg |= (GPIO_PinAltFunMode  << 4*GPIO_PinNumber);
    	pGPIOX->AFRL &= ~(15 << GPIO_PinNumber); //It will claer the bit field of MOder
    	pGPIOX->AFRL |= tempReg;
    }
    else if(GPIO_PinNumber > 7)
    {
    	//alternate functionality High Register
    	tempReg = 0;
    	tempReg |= (GPIO_PinAltFunMode  << 4*GPIO_PinNumber);
    	pGPIOX->AFRH &= ~(15 << GPIO_PinNumber); //It will claer the bit field of MOder
    	pGPIOX->AFRH |= tempReg;
    }



}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOX)
{
	if(pGPIOX == GPIOA)
	{
		RCC->AHB1RSTR |= (1<< 0);
	}
	else if(pGPIOX == GPIOB)
	{
		RCC->AHB1RSTR |= (1<< 1);
	}
	else if(pGPIOX == GPIOC)
	{
		RCC->AHB1RSTR |= (1<< 2);
	}
	else if(pGPIOX == GPIOD)
	{
		RCC->AHB1RSTR |= (1<< 3);
	}
	else if(pGPIOX == GPIOE)
	{
		RCC->AHB1RSTR |= (1<< 4);
	}
	else if(pGPIOX == GPIOF)
	{
		RCC->AHB1RSTR |= (1<< 5);
	}
	else if(pGPIOX == GPIOG)
	{
		RCC->AHB1RSTR |= (1<< 6);
	}
	else if(pGPIOX == GPIOH)
	{
		RCC->AHB1RSTR |= (1<< 7);
	}
	else if(pGPIOX == GPIOI)
	{
		RCC->AHB1RSTR |= (1<< 8);
	}
}

uint8_t GPIO_ReadFromIn_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (((pGPIOx->IDR)>> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromIn_Port(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value  = pGPIOx->IDR ;
	return value;
}

void GPIO_WriteToOut_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_Num,uint8_t value)
{
	if(value == 1)
	{
		pGPIOx->ODR |= (value << Pin_Num);
	}
	else if(value == 0)
	{
		pGPIOx->ODR &= ~(value << Pin_Num);
	}
}
void GPIO_WriteToOut_Port(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}
