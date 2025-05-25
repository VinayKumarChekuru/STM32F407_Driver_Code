/*
 * GPIO_Driver.h
 *
 *  Created on: Mar 3, 2025
 *      Author: vinaykumar
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include"stm32f407xx.h"

#define GPIO_PIN_0           0
#define GPIO_PIN_1           1
#define GPIO_PIN_2           2
#define GPIO_PIN_3           3
#define GPIO_PIN_4           4
#define GPIO_PIN_5           5
#define GPIO_PIN_6           6
#define GPIO_PIN_7           7
#define GPIO_PIN_8           8
#define GPIO_PIN_9           9
#define GPIO_PIN_10          10
#define GPIO_PIN_11          11
#define GPIO_PIN_12          12
#define GPIO_PIN_13          13
#define GPIO_PIN_14          14
#define GPIO_PIN_15          15

#define GPIO_MODE_IN          0
#define GPIO_MODE_OUT         1
#define GPIO_MODE_ATLFN       2
#define GPIO_MODE_ANALOG      3


#define GPIO_MODE_IN          0
#define GPIO_MODE_OUT         1
#define GPIO_MODE_ALTFUN      2
#define GPIO_MODE_ANALOG      3


#define GPIO_LOW_SPEED       0
#define GPIO_MED_SPEED       1
#define GPIO_HIGH_SPEED      2
#define GPIO_VHIGH_SPEED     3

#define GPIO_NOPUPD          0
#define GPIO_PU              1
#define GPIO_PD              2
#define GPIO_RESERVED        3

#define GPIO_PUSHPULL        0
#define GPIO_OPENDR          1


#define GPIO_AF0             0
#define GPIO_AF1             1
#define GPIO_AF2             2
#define GPIO_AF3             3
#define GPIO_AF4             4
#define GPIO_AF5             5
#define GPIO_AF6             6
#define GPIO_AF7             7
#define GPIO_AF8             8
#define GPIO_AF9             9
#define GPIO_AF10            10
#define GPIO_AF11            11
#define GPIO_AF12            12
#define GPIO_AF13            13
#define GPIO_AF14            14
#define GPIO_AF15            15

//Clock Enable or Disable API

void GPIO_ClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//GPIO Init and GPIO Deints


/*
 *  GPIO_PinNumber;
    GPIO_PinMode;
    GPIO_PinSpeed;
	GPIO_PinPuPdControl;
	GPIO_PinOPType;
    GPIO_PinAltFunMode;
 */
void GPIO_Init(GPIO_RegDef_t *pGPIOX,uint8_t GPIO_PinNumber,uint8_t GPIO_PinMode,uint8_t GPIO_PinSpeed, uint8_t GPIO_PinPuPdControl,uint8_t GPIO_PinOPType,uint8_t GPIO_PinAltFunMode);

void GPIO_DeInit(GPIO_RegDef_t *pGPIOX);

//Read Write

uint8_t GPIO_ReadFromIn_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromIn_Port(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOut_Pin(GPIO_RegDef_t *pGPIOx,uint8_t Pin_Num,uint8_t value);
void GPIO_WriteToOut_Port(GPIO_RegDef_t *pGPIOx, uint16_t value);




#endif /* GPIO_DRIVER_H_ */
