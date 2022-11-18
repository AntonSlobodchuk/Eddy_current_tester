################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f37x.c 

OBJS += \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f37x.o 

C_DEPS += \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f37x.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F3 -DSTM32F37 -DSTM32F373C8Tx -DDEBUG -DUSE_STDPERIPH_DRIVER -D__FPU_PRESENT -D__FPU_USED -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/StdPeriph_Driver/inc" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/utilites" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/inc" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/CMSIS/device" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


