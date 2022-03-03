################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.c \
../Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.c 

OBJS += \
./Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.o \
./Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.o 

C_DEPS += \
./Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.d \
./Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.o: ../Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Middlewares/Third_Party/MODBUS-LIB/Inc -I../Middlewares/Third_Party/MODBUS-LIB/Src -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/MODBUS-LIB/Src/Modbus.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.o: ../Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F411xE -DDEBUG -c -I../Middlewares/Third_Party/MODBUS-LIB/Inc -I../Middlewares/Third_Party/MODBUS-LIB/Src -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/MODBUS-LIB/Src/UARTCallback.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

