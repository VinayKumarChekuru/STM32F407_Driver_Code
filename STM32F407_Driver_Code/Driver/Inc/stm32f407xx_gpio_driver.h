/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Dec 22, 2024
 *      Author: vinay
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include"stm32f4xx.h"
#include<stdint.h>


/*
 * GPIO pin Possible modes
 */

#define GPIO_MODE_IN           0
#define GPIO_MODE_OUT          1
#define GPIO_MODE_ALTFN        2
#define GPIO_MODE_ANALOG       3
#define GPIO_MODE_IT_FT        4
#define GPIO_MODE_IT_RT        5
#define GPIO_MODE_IT_FRT       6


/*
 * GPIO possible output types
 */

#define OUTPUT_PUPL            0
#define OUTPUT_OPDR            1

/*
 * GPIO possible output speeds
 */
#define LW_SPD                 0
#define MD_SPD                 1
#define HG_SPD                 2
#define VHG_SPD                3

/*
 * GPIO possible Pull up Pull down values
 */
#define NO_PUPD               0
#define PULL_UP               1
#define PULL_DWN              2


/*
 * GPIO possible Pin Numbers
 */
#define GIPO_PIN_NO_0          0
#define GIPO_PIN_NO_1          1
#define GIPO_PIN_NO_2          2
#define GIPO_PIN_NO_3          3
#define GIPO_PIN_NO_4          4
#define GIPO_PIN_NO_5          5
#define GIPO_PIN_NO_6          6
#define GIPO_PIN_NO_7          7
#define GIPO_PIN_NO_8          8
#define GIPO_PIN_NO_9          9
#define GIPO_PIN_NO_10         10
#define GIPO_PIN_NO_11         11
#define GIPO_PIN_NO_12         12
#define GIPO_PIN_NO_13         13
#define GIPO_PIN_NO_14         14
#define GIPO_PIN_NO_15         15




typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

//=================================================================

/*
 * Prototypes of APIs which are supported by GPIO_Driver code
 */

//=====================================================================

/*
 * GPIO Initialization and GPIO Deinitialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * GPIO clock control
 */
void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Data read and write Macros
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


void GPIO_IRQItConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);





#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
