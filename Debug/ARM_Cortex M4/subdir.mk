################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ARM_Cortex\ M4/ARM_CortexM4.c 

OBJS += \
./ARM_Cortex\ M4/ARM_CortexM4.o 

C_DEPS += \
./ARM_Cortex\ M4/ARM_CortexM4.d 


# Each subdirectory must supply rules for building sources it contributes
ARM_Cortex\ M4/ARM_CortexM4.o: ../ARM_Cortex\ M4/ARM_CortexM4.c ARM_Cortex\ M4/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/STM32_Code/STM32_Driver_Code_Version001/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"ARM_Cortex M4/ARM_CortexM4.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ARM_Cortex-20-M4

clean-ARM_Cortex-20-M4:
	-$(RM) ./ARM_Cortex\ M4/ARM_CortexM4.cyclo ./ARM_Cortex\ M4/ARM_CortexM4.d ./ARM_Cortex\ M4/ARM_CortexM4.o ./ARM_Cortex\ M4/ARM_CortexM4.su

.PHONY: clean-ARM_Cortex-20-M4

