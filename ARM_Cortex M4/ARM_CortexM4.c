/*
 * ARM_CortexM4.c
 *
 *  Created on: Feb 4, 2025
 *      Author: JIIN17350723
 */

#include<stdint.h>
#include"ARM_CortexM4.h"


void floating_Point_ENorDi(uint8_t EnorDi)
{
    if (EnorDi)
    {
        // Enable FPU: Set bits 20-23 to '1' (Full Access for CP10 and CP11)
        *FPU_CPACR |= (0xF << 20);
    }
    else
    {
        // Disable FPU: Clear bits 20-23 (Access Denied for CP10 and CP11)
        *FPU_CPACR &= ~(0xF << 20);
    }

}
