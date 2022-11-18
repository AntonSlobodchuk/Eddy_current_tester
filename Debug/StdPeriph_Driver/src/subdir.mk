################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../StdPeriph_Driver/src/stm32f37x_adc.c \
../StdPeriph_Driver/src/stm32f37x_can.c \
../StdPeriph_Driver/src/stm32f37x_cec.c \
../StdPeriph_Driver/src/stm32f37x_comp.c \
../StdPeriph_Driver/src/stm32f37x_crc.c \
../StdPeriph_Driver/src/stm32f37x_dac.c \
../StdPeriph_Driver/src/stm32f37x_dbgmcu.c \
../StdPeriph_Driver/src/stm32f37x_dma.c \
../StdPeriph_Driver/src/stm32f37x_exti.c \
../StdPeriph_Driver/src/stm32f37x_flash.c \
../StdPeriph_Driver/src/stm32f37x_gpio.c \
../StdPeriph_Driver/src/stm32f37x_i2c.c \
../StdPeriph_Driver/src/stm32f37x_iwdg.c \
../StdPeriph_Driver/src/stm32f37x_misc.c \
../StdPeriph_Driver/src/stm32f37x_pwr.c \
../StdPeriph_Driver/src/stm32f37x_rcc.c \
../StdPeriph_Driver/src/stm32f37x_rtc.c \
../StdPeriph_Driver/src/stm32f37x_sdadc.c \
../StdPeriph_Driver/src/stm32f37x_spi.c \
../StdPeriph_Driver/src/stm32f37x_syscfg.c \
../StdPeriph_Driver/src/stm32f37x_tim.c \
../StdPeriph_Driver/src/stm32f37x_usart.c \
../StdPeriph_Driver/src/stm32f37x_wwdg.c 

OBJS += \
./StdPeriph_Driver/src/stm32f37x_adc.o \
./StdPeriph_Driver/src/stm32f37x_can.o \
./StdPeriph_Driver/src/stm32f37x_cec.o \
./StdPeriph_Driver/src/stm32f37x_comp.o \
./StdPeriph_Driver/src/stm32f37x_crc.o \
./StdPeriph_Driver/src/stm32f37x_dac.o \
./StdPeriph_Driver/src/stm32f37x_dbgmcu.o \
./StdPeriph_Driver/src/stm32f37x_dma.o \
./StdPeriph_Driver/src/stm32f37x_exti.o \
./StdPeriph_Driver/src/stm32f37x_flash.o \
./StdPeriph_Driver/src/stm32f37x_gpio.o \
./StdPeriph_Driver/src/stm32f37x_i2c.o \
./StdPeriph_Driver/src/stm32f37x_iwdg.o \
./StdPeriph_Driver/src/stm32f37x_misc.o \
./StdPeriph_Driver/src/stm32f37x_pwr.o \
./StdPeriph_Driver/src/stm32f37x_rcc.o \
./StdPeriph_Driver/src/stm32f37x_rtc.o \
./StdPeriph_Driver/src/stm32f37x_sdadc.o \
./StdPeriph_Driver/src/stm32f37x_spi.o \
./StdPeriph_Driver/src/stm32f37x_syscfg.o \
./StdPeriph_Driver/src/stm32f37x_tim.o \
./StdPeriph_Driver/src/stm32f37x_usart.o \
./StdPeriph_Driver/src/stm32f37x_wwdg.o 

C_DEPS += \
./StdPeriph_Driver/src/stm32f37x_adc.d \
./StdPeriph_Driver/src/stm32f37x_can.d \
./StdPeriph_Driver/src/stm32f37x_cec.d \
./StdPeriph_Driver/src/stm32f37x_comp.d \
./StdPeriph_Driver/src/stm32f37x_crc.d \
./StdPeriph_Driver/src/stm32f37x_dac.d \
./StdPeriph_Driver/src/stm32f37x_dbgmcu.d \
./StdPeriph_Driver/src/stm32f37x_dma.d \
./StdPeriph_Driver/src/stm32f37x_exti.d \
./StdPeriph_Driver/src/stm32f37x_flash.d \
./StdPeriph_Driver/src/stm32f37x_gpio.d \
./StdPeriph_Driver/src/stm32f37x_i2c.d \
./StdPeriph_Driver/src/stm32f37x_iwdg.d \
./StdPeriph_Driver/src/stm32f37x_misc.d \
./StdPeriph_Driver/src/stm32f37x_pwr.d \
./StdPeriph_Driver/src/stm32f37x_rcc.d \
./StdPeriph_Driver/src/stm32f37x_rtc.d \
./StdPeriph_Driver/src/stm32f37x_sdadc.d \
./StdPeriph_Driver/src/stm32f37x_spi.d \
./StdPeriph_Driver/src/stm32f37x_syscfg.d \
./StdPeriph_Driver/src/stm32f37x_tim.d \
./StdPeriph_Driver/src/stm32f37x_usart.d \
./StdPeriph_Driver/src/stm32f37x_wwdg.d 


# Each subdirectory must supply rules for building sources it contributes
StdPeriph_Driver/src/%.o: ../StdPeriph_Driver/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F3 -DSTM32F37 -DSTM32F373C8Tx -DDEBUG -DUSE_STDPERIPH_DRIVER -D__FPU_PRESENT -D__FPU_USED -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/StdPeriph_Driver/inc" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/utilites" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/inc" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/CMSIS/device" -I"D:/STM32_TEMP/stm32f373c8t6_test_filtr_03.12.21/led/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


