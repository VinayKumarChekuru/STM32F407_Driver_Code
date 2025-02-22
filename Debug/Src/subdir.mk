################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SPI_Aurdino_slave.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/usart_main.c 

OBJS += \
./Src/SPI_Aurdino_slave.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/usart_main.o 

C_DEPS += \
./Src/SPI_Aurdino_slave.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/usart_main.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"D:/STM32_Code/STM32_Driver_Code_Version001/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/SPI_Aurdino_slave.cyclo ./Src/SPI_Aurdino_slave.d ./Src/SPI_Aurdino_slave.o ./Src/SPI_Aurdino_slave.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/usart_main.cyclo ./Src/usart_main.d ./Src/usart_main.o ./Src/usart_main.su

.PHONY: clean-Src

