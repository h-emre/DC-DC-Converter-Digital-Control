################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ProjectDriver/Src/I2C.c \
../Drivers/ProjectDriver/Src/INA228.c 

OBJS += \
./Drivers/ProjectDriver/Src/I2C.o \
./Drivers/ProjectDriver/Src/INA228.o 

C_DEPS += \
./Drivers/ProjectDriver/Src/I2C.d \
./Drivers/ProjectDriver/Src/INA228.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ProjectDriver/Src/%.o Drivers/ProjectDriver/Src/%.su Drivers/ProjectDriver/Src/%.cyclo: ../Drivers/ProjectDriver/Src/%.c Drivers/ProjectDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I/Drivers/ProjectDriver/Src -I"C:/Users/akguc/STM32CubeIDE/BitirmeTezi/Digital_Control/Drivers/ProjectDriver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ProjectDriver-2f-Src

clean-Drivers-2f-ProjectDriver-2f-Src:
	-$(RM) ./Drivers/ProjectDriver/Src/I2C.cyclo ./Drivers/ProjectDriver/Src/I2C.d ./Drivers/ProjectDriver/Src/I2C.o ./Drivers/ProjectDriver/Src/I2C.su ./Drivers/ProjectDriver/Src/INA228.cyclo ./Drivers/ProjectDriver/Src/INA228.d ./Drivers/ProjectDriver/Src/INA228.o ./Drivers/ProjectDriver/Src/INA228.su

.PHONY: clean-Drivers-2f-ProjectDriver-2f-Src

