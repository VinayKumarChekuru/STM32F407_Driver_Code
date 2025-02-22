/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Feb 5, 2025
 *      Author: JIIN17350723
 */

#ifndef INC_STM32F407XX_UART_DRIVER_H_
#define INC_STM32F407XX_UART_DRIVER_H_

#include"stm32f4xx.h"

#define USART_MODE_ONLY_TX        0
#define USART_MODE_ONLY_RX        1
#define USART_MODE_TXRX           2


#define USART_WORD_LEN_8BITS       0
#define USART_WORD_LEN_9BITS      1

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


#define USART_1_STOP_BIT           0
#define USART_0_5_STOP_BIT         1
#define USART_2_STOP_BIT           2
#define USART_1_5_STOP_BIT         3



#define USART_PARITYCONTR_DI       0
#define USART_PARITYCONTR_EN_EVEN  1
#define USART_PARITYCONTR_EN_ODD   2

#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3





/*
 * Configurable structure of USART
 */

typedef struct
{
	uint8_t USART_MODE;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t  USART_Config;

}USART_Handle_t;

//USART Peripheral Clock Enable or Disable

void USART_ClockControl(USART_RegDef_t *pUSARTx , uint8_t EnorDi);

// USART Initialization and Deinitialization

void USART_Init(USART_Handle_t *pUSARTHandle);

void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTx, uint8_t *pBuffer, uint32_t Len);




#endif /* INC_STM32F407XX_UART_DRIVER_H_ */
