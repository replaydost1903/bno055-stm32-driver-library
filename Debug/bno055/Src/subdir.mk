################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bno055/Src/BNO055.c 

OBJS += \
./bno055/Src/BNO055.o 

C_DEPS += \
./bno055/Src/BNO055.d 


# Each subdirectory must supply rules for building sources it contributes
bno055/Src/%.o bno055/Src/%.su bno055/Src/%.cyclo: ../bno055/Src/%.c bno055/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/KIZAGAN/bno055_driver_development/bno055/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bno055-2f-Src

clean-bno055-2f-Src:
	-$(RM) ./bno055/Src/BNO055.cyclo ./bno055/Src/BNO055.d ./bno055/Src/BNO055.o ./bno055/Src/BNO055.su

.PHONY: clean-bno055-2f-Src

