/*
 * stm32f4xx.h
 *
 *  Created on: Dec 20, 2024
 *      Author: vinay
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include<stdint.h>


#define __vo                     volatile
#define __ui                     uint32_t



/*
 * Some Processor specific registers
 */

#define IRQ_NO_EXTI0             6
#define IRQ_NO_EXTI1             7
#define IRQ_NO_EXTI2             8
#define IRQ_NO_EXTI3             9
#define IRQ_NO_EXTI4             10
#define IRQ_NO_EXTI9_5           23
#define IRQ_NO_EXTI15_10         40
/*
 * ARM CORTEX-M4 ISER registers
 */
#define NVIC_ISER0               ((__vo __ui*)0xE000E100)
#define NVIC_ISER1               ((__vo __ui*)0xE000E104)
#define NVIC_ISER2               ((__vo __ui*)0xE000E108)
#define NVIC_ISER3               ((__vo __ui*)0xE000E10C)


/*
 * ARM CORTEX-M4 ICER registers
 */
#define NVIC_ICER0                ((__vo __ui*)(0XE000E180))
#define NVIC_ICER1                ((__vo __ui*)(0XE000E184))
#define NVIC_ICER2                ((__vo __ui*)(0XE000E188))
#define NVIC_ICER3                ((__vo __ui*)(0XE000E18C))

/*
 * Base address of interrupt priority register
 */

#define NVIC_PR_BASE_ADDR    ((__vo __ui*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED           4



/*
 * Some generic macros
 */
#define ENABLE                   1
#define DISABLE                  0
#define SET                      ENABLE
#define RESET                    DISABLE
#define GPIO_PIN_SET             ENABLE
#define GPIO_PIN_RESET           DISABLE



/* Base addresses of different memories */

#define SRAM_BASE_ADDR           0x20000000U
#define SRAM1_BASE_ADDR          SRAM_BASE_ADDR
#define END_OF_SRAM1             (SRAM1_BASE_ADDR + (112*1024))U
#define SRAM2_BASE_ADDR          END_OF_SRAM1
#define FLASH_MEM_BASE_ADDR      0x08000000U
#define ROM_BASE_ADDR            0x1FFF0000U


/* Base addresses of different buses */

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

/*
 * Base addresses of SPI pheripheral
 */
#define SPI1_BASE_ADDR          (APB2_BASE_ADDR + (0x3000))
#define SPI2_BASE_ADDR          (APB1_BASE_ADDR + (0x3800))
#define SPI3_BASE_ADDR          (APB1_BASE_ADDR + (0x3C00))
#define SPI4_BASE_ADDR          (APB2_BASE_ADDR + (0x3400))
#define SPI5_BASE_ADDR          (APB2_BASE_ADDR + (0x5000))
#define SPI6_BASE_ADDR          (APB2_BASE_ADDR + (0x5400))

/*
 * Base addresses of SPI pheripheral
 */
#define I2C1_BASE_ADDR          (APB1_BASE_ADDR + (0x5400))
#define I2C2_BASE_ADDR          (APB1_BASE_ADDR + (0x5800))
#define I2C3_BASE_ADDR          (APB1_BASE_ADDR + (0x5C00))



#define RCC_BASE_ADDR           0x40023800U


#define EXTI_BASE_ADDR          0x40013C00U
#define SYSCFG_BASE_ADDR        0x40013800U
/*
 * GPIO Register structure definition
 */

typedef struct
{
	__vo __ui MODER;
	__vo __ui OTYPER;
	__vo __ui OSPEEDR;
	__vo __ui PUPDR;
	__vo __ui IDR;
	__vo __ui ODR;
	__vo __ui BSRR;
	__vo __ui LCKR;
	__vo __ui AFRL;
	__vo __ui AFRH;

}GPIO_RegDef_t;

typedef struct
{
	__vo __ui CR;
	__vo __ui PLLCGFR;
	__vo __ui CFGR;
	__vo __ui CIR;
	__vo __ui AHB1RSTR;
	__vo __ui AHB2RSTR;
	__vo __ui AHB3RSTR;
	__vo __ui RESERVED0;
	__vo __ui APB1RSTR;
	__vo __ui APB2RSTR;
	__vo __ui RESERVED1[2];
	__vo __ui AHB1ENR;
	__vo __ui AHB2ENR;
	__vo __ui AHB3ENR;
	__vo __ui RESERVED2;
	__vo __ui APB1ENR;
	__vo __ui APB2ENR;
	__vo __ui RESERVED3[2];
	__vo __ui AHB1LPENR;
	__vo __ui AHB2LPENR;
	__vo __ui AHB3LPENR;
	__vo __ui RESERVED4;
	__vo __ui APB1LPENR;
	__vo __ui APB2LPENR;
	__vo __ui RESERVED5[2];
	__vo __ui BDCR;
	__vo __ui CSR;
	__vo __ui RESERVED6[2];
	__vo __ui SSCGR;
	__vo __ui PLLI2SCFGR;
	__vo __ui PLLSAICFGR;
	__vo __ui RCC_DCKCFGR;
}Rcc_RegDef_t;


/*
 * EXTI register definition structure
 */

typedef struct
{
	__vo __ui EXTI_IMR;
	__vo __ui EXTI_EMR;
	__vo __ui EXTI_RTSR;
	__vo __ui EXTI_FTSR;
	__vo __ui EXTI_SWIER;
	__vo __ui EXTI_PR;

}EXTI_RegDef_t;



typedef struct //need to add some things
{
	__vo __ui SYSCFG_MEMRMP;
	__vo __ui SYSCFG_PMC;
	__vo __ui SYSCFG_EXTICR[4];
	__ui RESERVED[2];
}SYSCFG_RefDef_t;

typedef struct
{
	__vo __ui CR1;
	__vo __ui CR2;
	__vo __ui SR;
	__vo __ui DR;
	__vo __ui CRCPR;
	__vo __ui RXCRCR;
	__vo __ui TXCRCR;
	__vo __ui I2SCFGR;
	__vo __ui I2SPR;

}SPI_RegDef_t;

typedef struct
{
	__vo __ui CR1;
	__vo __ui CR2;
	__vo __ui OAR1;
	__vo __ui OAR2;
	__vo __ui DR;
	__vo __ui SR1;
	__vo __ui SR2;
	__vo __ui CCR;
	__vo __ui TRISE;
	__vo __ui FLTR;

}I2C_RegDef_t;


 /*
 * create pointers for every GPIO
 */

#define  GPIOA              (GPIO_RegDef_t*)GPIOA_BASE_ADDR
#define  GPIOB              (GPIO_RegDef_t*)GPIOB_BASE_ADDR
#define  GPIOC              (GPIO_RegDef_t*)GPIOC_BASE_ADDR
#define  GPIOD              (GPIO_RegDef_t*)GPIOD_BASE_ADDR
#define  GPIOE              (GPIO_RegDef_t*)GPIOE_BASE_ADDR
#define  GPIOF              (GPIO_RegDef_t*)GPIOF_BASE_ADDR
#define  GPIOG              (GPIO_RegDef_t*)GPIOG_BASE_ADDR
#define  GPIOH              (GPIO_RegDef_t*)GPIOH_BASE_ADDR
#define  GPIOI              (GPIO_RegDef_t*)GPIOI_BASE_ADDR


#define RCC                 ((Rcc_RegDef_t*)RCC_BASE_ADDR)

#define EXTI                ((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG              ((SYSCFG_RefDef_t*)SYSCFG_BASE_ADDR)

/*
 * Create Pointers for SPI Base addresses
 */
#define SPI1                ((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2                ((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3                ((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4                ((SPI_RegDef_t*)SPI4_BASE_ADDR)
#define SPI5                ((SPI_RegDef_t*)SPI5_BASE_ADDR)
#define SPI6                ((SPI_RegDef_t*)SPI6_BASE_ADDR)


/*
 * Pointers to I2C base addresses
 */

#define I2C1              ((I2C_RegDef_t*) I2C1_BASE_ADDR)
#define I2C2              ((I2C_RegDef_t*) I2C2_BASE_ADDR)
#define I2C3              ((I2C_RegDef_t*) I2C3_BASE_ADDR)


/*
 * Clock enable macros for GPIO ports
 */

#define GPIOA_CLK_EN()  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()  (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()  (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()  (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()  (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_EN()  (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Disable macros for GPIO Ports
 */
#define GPIOA_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DI()  (RCC->AHB1ENR &= ~(1 << 8))


/*
 * GPIO Reset
 */

#define GPIOA_RST()          (RCC->AHB1RSTR |= ~(1<<0))
#define GPIOB_RST()          (RCC->AHB1RSTR |= ~(1<<1))
#define GPIOC_RST()          (RCC->AHB1RSTR |= ~(1<<2))
#define GPIOD_RST()          (RCC->AHB1RSTR |= ~(1<<3))
#define GPIOE_RST()          (RCC->AHB1RSTR |= ~(1<<4))
#define GPIOF_RST()          (RCC->AHB1RSTR |= ~(1<<5))
#define GPIOG_RST()          (RCC->AHB1RSTR |= ~(1<<6))
#define GPIOH_RST()          (RCC->AHB1RSTR |= ~(1<<7))
#define GPIOI_RST()          (RCC->AHB1RSTR |= ~(1<<8))

/*
 * Clock enable macros for SYSCFG
 */

#define SYSCFG_CLK_EN()      (RCC->APB2ENR |=(1<<14))

/*
 * Clock Disable macros for SYSCFG
 */
#define SYSCFG_CLK_DI()      (RCC->APB2RSTR &= ~(1<<13))


/*
 * GPIO codes
 */

#define GPIO_BASE_ADDR_TO_CODE(x)           ((x==GPIOA)?0:\
		                                    (x==GPIOB)?1:\
		                                    (x==GPIOC)?2:\
		                                    (x==GPIOD)?3:\
		                                    (x==GPIOE)?4:\
		                                    (x==GPIOF)?5:\
		                                    (x==GPIOG)?6:\
		                                    (x==GPIOH)?7:\
		                                    (x==GPIOI)?8:0)


/*
 * SPI Clock enable and disable macros
 */

#define SPI1_CLK_EN()                  (RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()                  (RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()                  (RCC->APB1ENR |= (1<<15))
#define SPI4_CLK_EN()                  (RCC->APB2ENR |= (1<<13))
#define SPI5_CLK_EN()                  (RCC->APB2ENR |= (1<<20))
#define SPI6_CLK_EN()                  (RCC->APB2ENR |= (1<<21))



#define SPI1_CLK_DI()                  (RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()                  (RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()                  (RCC->APB1ENR &= ~(1<<15))
#define SPI4_CLK_DI()                  (RCC->APB2ENR &= ~(1<<13))
#define SPI5_CLK_DI()                  (RCC->APB2ENR &= ~(1<<20))
#define SPI6_CLK_DI()                  (RCC->APB2ENR &= ~(1<<21))


/*
 * SPI De-Initialization
 */

#define SPI1_RST()                  (RCC->APB2RSTR |= (1<<12))
#define SPI2_RST()                  (RCC->APB1RSTR |= (1<<14))
#define SPI3_RST()                  (RCC->APB1RSTR |= (1<<15))
#define SPI4_RST()                  (RCC->APB2RSTR |= (1<<13))
#define SPI5_RST()                  (RCC->APB2RSTR |= (1<<20))
#define SPI6_RST()                  (RCC->APB2RSTR |= (1<<21))

/*============================================================================================================
   Bit Position Definitions for SPI Peripheral
 ===============================================================================================================*/

#define SPI_CR1_CPHA             0
#define SPI_CR1_CPOL             1
#define SPI_CR1_MSTR             2
#define SPI_CR1_BR               3
#define SPI_CR1_SPE              6
#define SPI_CR1_LSBFIRST         7
#define SPI_CR1_SSI              8
#define SPI_CR1_SSM              9
#define SPI_CR1_RXONLY           10
#define SPI_CR1_DFF              11
#define SPI_CR1_CRCNEXT          12
#define SPI_CR1_CRCEN            13
#define SPI_CR1_BIDIOE           14
#define SPI_CR1_BIDIMODE         15



#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"

#endif /* INC_STM32F4XX_H_ */
