/*
 * stm32f407xx.h
 *
 *  Created on: Feb 19, 2025
 *      Author: vinaykumar
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include<stdint.h>

#define APB1_BASE_ADDR          0x40000000U
#define APB2_BASE_ADDR          0x40010000U
#define AHB1_BASE_ADDR          0x40020000U
#define AHB2_BASE_ADDR          0x50000000U
#define AHB3_BASE_ADDR          0xA0000000U



/*
 * Base Addresses of all peripherals hanged on AHB1 bus
 */
#define GPIOA_BASE_ADDR        (AHB1_BASE_ADDR+(0X0000))
#define GPIOB_BASE_ADDR        (AHB1_BASE_ADDR+(0X0400))
#define GPIOC_BASE_ADDR        (AHB1_BASE_ADDR+(0X0800))
#define GPIOD_BASE_ADDR        (AHB1_BASE_ADDR+(0X0C00))
#define GPIOE_BASE_ADDR        (AHB1_BASE_ADDR+(0X1000))
#define GPIOF_BASE_ADDR        (AHB1_BASE_ADDR+(0X1400))
#define GPIOG_BASE_ADDR        (AHB1_BASE_ADDR+(0X1800))
#define GPIOH_BASE_ADDR        (AHB1_BASE_ADDR+(0X1C00))
#define GPIOI_BASE_ADDR        (AHB1_BASE_ADDR+(0X2000))


#define RCC_BASE_ADDR          0x40023800U


// Register Definition struct for GPIO ports

typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t LCKR;
	uint32_t AFR[2];

}GPIO_RegDef_t;


typedef struct
{
   uint32_t CR;
   uint32_t PLLCFGR;
   uint32_t CFGR;
   uint32_t CIR;
   uint32_t AHB1RSTR;
   uint32_t AHB2RSTR;
   uint32_t AHB3RSTR;
   uint32_t RESERVED0;
   uint32_t APB1RSTR;
   uint32_t APB2RSTR;
   uint32_t RESERVED1[2];
   uint32_t AHB1ENR;
   uint32_t AHB2ENR;
   uint32_t AHB3ENR;
   uint32_t RESERVED2;
   uint32_t APB1ENR;
   uint32_t APB2ENR;
   uint32_t RESERVED3[2];
   uint32_t AHB1LPENR;
   uint32_t AHB2LPENR;
   uint32_t AHB3LPENR;
   uint32_t RESERVED4;
   uint32_t APB1LPENR;
   uint32_t APB2LPENR;
  uint32_t  RESERVED5[2];
   uint32_t BDCR;
   uint32_t CSR;
  uint32_t  RESERVED6[2];
   uint32_t SSCGR;
   uint32_t PLLI2SCFGR;
   uint32_t PLLSAICFGR;
   uint32_t DCKCFGR;
   uint32_t CKGATENR;
   uint32_t DCKCFGR2;

} RCC_RegDef_t;




//Pointers to hold base address of RegDef struct

#define GPIOA         (GPIO_RegDef_t*)GPIOA_BASE_ADDR
#define GPIOB         (GPIO_RegDef_t*)GPIOB_BASE_ADDR
#define GPIOC         (GPIO_RegDef_t*)GPIOC_BASE_ADDR
#define GPIOD         (GPIO_RegDef_t*)GPIOD_BASE_ADDR
#define GPIOE         (GPIO_RegDef_t*)GPIOE_BASE_ADDR
#define GPIOF         (GPIO_RegDef_t*)GPIOF_BASE_ADDR
#define GPIOG         (GPIO_RegDef_t*)GPIOG_BASE_ADDR
#define GPIOH         (GPIO_RegDef_t*)GPIOH_BASE_ADDR
#define GPIOI         (GPIO_RegDef_t*)GPIOI_BASE_ADDR


//Pointer for RCC Base Addr

#define RCC         (RCC_RegDef_t*)RCC_BASE_ADDR

//Clock Enable and Disable macros of GPIO

#define GPIOA_Clk_En()         (RCC->AHB1ENR |= (1<<0))


#define ENABLE      1
#define DISABLE     0




#endif /* STM32F407XX_H_ */
