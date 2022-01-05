################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/I2C_tx_String_arduino.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/I2C_tx_String_arduino.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/I2C_tx_String_arduino.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/pro/Documents/stm32drivers/stm32f4 i2c driver/drivers/Inc" -I"C:/Users/pro/Documents/stm32drivers/stm32f4 i2c driver/drivers/Src" -I"C:/Users/pro/Documents/stm32drivers/stm32f4 i2c driver/drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/I2C_tx_String_arduino.d ./Src/I2C_tx_String_arduino.o ./Src/syscalls.d ./Src/syscalls.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src

