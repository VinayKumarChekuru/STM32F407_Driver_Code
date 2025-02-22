/*
 * ARM_CortexM4.h
 *
 *  Created on: Feb 4, 2025
 *      Author: JIIN17350723
 */

#ifndef ARM_CORTEXM4_H_
#define ARM_CORTEXM4_H_

#define FPU_CPACR                ((volatile uint32_t*)0xE000ED88U)
#define FPU_FPCCR                ((volatile uint32_t*)0xE000EF34U)
#define FPU_FPCAR                ((volatile uint32_t*)0xE000EF38U)
#define FPU_FPDSCR               ((volatile uint32_t*)0xE000EF3CU)


//typdef struct
//{
//	volatile uint32_t CPACR;
//	volatile uint32_t FPCCR;
//	volatile uint32_t FPCAR;
//	volatile uint32_t FPDSCR;
//
//}FPU_RegDef_t;


//Enable Floating point unit in ARM Processor
void floating_Point_ENorDi(uint8_t EnorDi);


#endif /* ARM_CORTEXM4_H_ */
