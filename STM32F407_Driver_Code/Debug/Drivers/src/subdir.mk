################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/GPIO_Driver.c 

OBJS += \
./Drivers/src/GPIO_Driver.o 

C_DEPS += \
./Drivers/src/GPIO_Driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o Drivers/src/%.su Drivers/src/%.cyclo: ../Drivers/src/%.c Drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/vinaykumar/Desktop/Youtube_Workspace/STM32F407_Driver_Code/Inc" -I"C:/Users/vinaykumar/Desktop/Youtube_Workspace/STM32F407_Driver_Code/Inc" -I"C:/Users/vinaykumar/Desktop/Youtube_Workspace/STM32F407_Driver_Code/Inc" -include"C:/Users/vinaykumar/Desktop/Youtube_Workspace/STM32F407_Driver_Code/Drivers/inc/stm32f407xx.h" -include"C:/Users/vinaykumar/Desktop/Youtube_Workspace/STM32F407_Driver_Code/Drivers/inc/GPIO_Driver.h" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-src

clean-Drivers-2f-src:
	-$(RM) ./Drivers/src/GPIO_Driver.cyclo ./Drivers/src/GPIO_Driver.d ./Drivers/src/GPIO_Driver.o ./Drivers/src/GPIO_Driver.su

.PHONY: clean-Drivers-2f-src

