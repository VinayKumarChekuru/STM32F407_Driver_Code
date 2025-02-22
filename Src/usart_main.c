/*
 * usart_main.c
 *
 *  Created on: Feb 6, 2025
 *      Author: vinaykumar
 */


#include <stdint.h>
#include<string.h>
#include<stm32f407xx_gpio_driver.h>
#include"stm32f407xx_usart_driver.h"
#include"stm32f4xx.h"
#include"main.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main()
{
	USART_Handle_t usart;
	usart.pUSARTx = USART1;
	usart.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_CTS_RTS;
	usart.USART_Config.USART_MODE = USART_MODE_TXRX;
	usart.USART_Config.USART_NoOfStopBits = USART_1_STOP_BIT;
	usart.USART_Config.USART_ParityControl = USART_PARITYCONTR_DI;
	usart.USART_Config.USART_WordLength = USART_WORD_LEN_8BITS;
	USART_Init(&usart);

	while(1);
}
