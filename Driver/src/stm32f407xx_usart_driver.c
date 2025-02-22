/*
 * stm32f407xx_uart_driver.c
 *
 *  Created on: Feb 5, 2025
 *      Author: JIIN17350723
 */


#include"stm32f407xx_usart_driver.h"

#define UART_TXE_FLAG      1<<7
#define UART_TCE_FLAG      1<<6

#define UART_RXNE_FLAG     1<<5

uint8_t USART_Get_TC_FlagStatus(USART_RegDef_t *pUSART, uint32_t TCE_Flag)
{
	if((pUSART->SR) & (UART_TCE_FLAG))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


//Checking transmitter Data Buffer is empty or not
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSART, uint32_t TXE_flag)
{
	if((pUSART->SR) & TXE_flag)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



//USART Peripheral Clock Enable or Disable

void USART_ClockControl(USART_RegDef_t *pUSARTx , uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_CLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_CLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_CLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_CLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_CLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_CLK_EN();
		}

	}
	else if(EnorDi == DISABLE)
	{
	     if(pUSARTx == USART1)
			{
               USART1_CLK_DI();
			}
			else if(pUSARTx == USART2)
			{
				USART2_CLK_DI();
			}
			else if(pUSARTx == USART3)
			{
				USART3_CLK_DI();
			}
			else if(pUSARTx == USART6)
			{
				USART6_CLK_DI();
			}
			else if(pUSARTx == UART4)
			{
				UART4_CLK_DI();
			}
			else if(pUSARTx == UART5)
			{
				UART5_CLK_DI();
			}

		}
}

//// USART Initialization and Deinitialization
//
//void USART_Init(USART_Handle_t *pUSARTHandle)
//{
//     //Set USART MODE
//	if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_ONLY_TX)
//	{
//		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TE); // Enabling Transmitter mode
//		pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_RE); //Disabling Receiver mode
//	}
//	else if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_ONLY_RX)
//	{
//		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_RE); // Enabling Receiver mode
//	    pUSARTHandle->pUSARTx->CR1 &= ~(1<<USART_CR1_TE); //Disabling Transmitter mode
//	}
//	else if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_TXRX)
//	{
//		pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_RE); // Enabling Receiver mode
//	    pUSARTHandle->pUSARTx->CR1 |= (1<<USART_CR1_TE); //Enabling Transmitter mode
//	}
//
//	//Set the Baud rate
//	uint32_t temp = 0, mantisa=0 , USART_DIV=0;
//	double fraction=0;
//	temp |= pUSARTHandle->pUSARTx->CR1;
//	uint8_t OVER8_Val =  (temp) & (1<< USART_CR1_OVER8);
//
//	if(pUSARTHandle->pUSARTx == USART1 || pUSARTHandle->pUSARTx == USART6 ) //Both USART1 & 6 are connected to APB2 Bus
//	{
//		 USART_DIV = (APB2_CLK_SPEED) / ((8U * (2U - (OVER8_Val)) * (pUSARTHandle->USART_Config.USART_Baud)));
//
//
//	}
//	else
//	{
//		 USART_DIV = (APB1_CLK_SPEED) / ((8U * (2U - (OVER8_Val)) * (pUSARTHandle->USART_Config.USART_Baud)));
//
//	}
//	mantisa = (uint32_t)USART_DIV;
//	fraction = USART_DIV - mantisa;
//	fraction &= (0x000F);
//	pUSARTHandle->pUSARTx->BRR |= fraction; //Setting fraction part of the BRR register
//	pUSARTHandle->pUSARTx->BRR |=(mantise << 4); //Setting Mantise Part
//
//
//	//Set Number of stop bits
//	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << 12);
//
//
//	//Set the Word length
//	pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.USART_WordLength << 12);
//
//	//Set Parity control
//	pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->USART_Config.USART_ParityControl << 10);
//
//	//Set Hardware flow control
//	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
//	{
//	    pUSARTHandle->pUSARTx->CR3 |= (1 << 9); //enable CTS
//	    pUSARTHandle->pUSARTx->CR3 &= ~(1 << 8); //Disable RTS
//	}
//	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
//	{
//		 pUSARTHandle->pUSARTx->CR3 |= (1 << 8); //Enable RTS
//		 pUSARTHandle->pUSARTx->CR3 &= ~(1 << 9); //Disable CTS
//	}
//	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
//	{
//		 pUSARTHandle->pUSARTx->CR3 |= (1 << 8); //Enable RTS
//		 pUSARTHandle->pUSARTx->CR3 |= (1 << 9); //enable CTS
//	}
//
//
//}




void USART_Init(USART_Handle_t *pUSARTHandle)
{
	USART_ClockControl(pUSARTHandle->pUSARTx, ENABLE);
	//============Configure the USART CR1 Register==============//

	uint32_t tempReg = 0;


	 //Set USART MODE
		if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_ONLY_TX)
		{
			tempReg |= (1<<USART_CR1_TE); // Enabling Transmitter mode
			tempReg &= ~(1<<USART_CR1_RE); //Disabling Receiver mode
		}
		else if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_ONLY_RX)
		{
			tempReg |= (1<<USART_CR1_RE); // Enabling Receiver mode
			tempReg &= ~(1<<USART_CR1_TE); //Disabling Transmitter mode
		}
		else if(pUSARTHandle->USART_Config.USART_MODE == USART_MODE_TXRX)
		{
			tempReg |= (1<<USART_CR1_RE); // Enabling Receiver mode
			tempReg |= (1<<USART_CR1_TE); //Enabling Transmitter mode
		}

		//Set the Word length
		tempReg |= (pUSARTHandle->USART_Config.USART_WordLength << 12);

		//ParityControl

	    if(pUSARTHandle->USART_Config.USART_ParityControl = (USART_PARITYCONTR_EN_EVEN))
	    {
	    	tempReg |= ( 1<< USART_CR1_PCE);  //Enabling Parity
	    	tempReg &= ~(1<< USART_CR1_PS);  //Setting Even Parity

	    }
	    else if (pUSARTHandle->USART_Config.USART_ParityControl = USART_PARITYCONTR_EN_ODD)
	    {
	    	tempReg |= ( 1<< USART_CR1_PCE);  //Enabling Parity
	        tempReg |= (1<< USART_CR1_PS);  //Setting Odd Parity
	    }
 //Program CR1 Register

	    pUSARTHandle->pUSARTx->CR1 = tempReg;


	    //============Configure the USART CR2 Register==============//

	    tempReg = 0;

	    //Setting NUmber of Stop bits
	    tempReg = (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP );

	    pUSARTHandle->pUSARTx->CR2 = tempReg;

	    //============Configure the USART CR3 Register==============//

	    tempReg = 0;
	    //Set Hardware flow control
	    if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	    	{
	    	tempReg |= (1 << USART_CR3_CTSE); //enable CTS
	    	tempReg &= ~(1 << USART_CR3_RTSE); //Disable RTS
	    	}
	     else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	    	{
	    	 tempReg |= (1 << USART_CR3_RTSE); //Enable RTS
	    	 tempReg&= ~(1 << USART_CR3_CTSE); //Disable CTS
	    	}
	       else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	    	{
	    	   tempReg |= (1 << USART_CR3_RTSE); //Enable RTS
	    	   tempReg |= (1 << USART_CR3_CTSE); //enable CTS
	    	}
	    pUSARTHandle->pUSARTx->CR3 = tempReg;



}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{

	if(pUSARTx == USART1)
	{
		USART1_RST();
	}
	else if(pUSARTx == USART2)
	{
		USART2_RST();
	}
	else if(pUSARTx == USART3)
	{
		USART2_RST();
	}
	else if(pUSARTx == UART4)
	{
		UART4_RST();
	}
	else if(pUSARTx == UART5)
	{
		UART5_RST();
	}
	else if(pUSARTx == USART6)
	{
		USART6_RST();
	}

}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, UART_TXE_FLAG) == FLAG_RESET);

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITYCONTR_DI)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_Get_TC_FlagStatus(pUSARTHandle->pUSARTx,UART_TCE_FLAG));
}




void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint32_t tempReg = 0;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx , UART_RXNE_FLAG)));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_LEN_9BITS)
		{
			//We are going to receive 9bit data in a frame
			tempReg |= (1 << USART_CR1_M);

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITYCONTR_DI)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)(0x1FF));

				//Now increment the pRxBuffer two times
				pRxBuffer++;

			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITYCONTR_DI)
			{
				//No parity is used , so all 8bits will be of user data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)(0xFF));

			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (0x7F));
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}











