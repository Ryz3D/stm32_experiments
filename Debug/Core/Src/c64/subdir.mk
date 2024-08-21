################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/c64/basic.c \
../Core/Src/c64/characters.c \
../Core/Src/c64/cia.c \
../Core/Src/c64/cpu.c \
../Core/Src/c64/display.c \
../Core/Src/c64/kernal.c \
../Core/Src/c64/memory.c \
../Core/Src/c64/vic.c 

OBJS += \
./Core/Src/c64/basic.o \
./Core/Src/c64/characters.o \
./Core/Src/c64/cia.o \
./Core/Src/c64/cpu.o \
./Core/Src/c64/display.o \
./Core/Src/c64/kernal.o \
./Core/Src/c64/memory.o \
./Core/Src/c64/vic.o 

C_DEPS += \
./Core/Src/c64/basic.d \
./Core/Src/c64/characters.d \
./Core/Src/c64/cia.d \
./Core/Src/c64/cpu.d \
./Core/Src/c64/display.d \
./Core/Src/c64/kernal.d \
./Core/Src/c64/memory.d \
./Core/Src/c64/vic.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/c64/%.o Core/Src/c64/%.su Core/Src/c64/%.cyclo: ../Core/Src/c64/%.c Core/Src/c64/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Core/Inc/c64 -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-c64

clean-Core-2f-Src-2f-c64:
	-$(RM) ./Core/Src/c64/basic.cyclo ./Core/Src/c64/basic.d ./Core/Src/c64/basic.o ./Core/Src/c64/basic.su ./Core/Src/c64/characters.cyclo ./Core/Src/c64/characters.d ./Core/Src/c64/characters.o ./Core/Src/c64/characters.su ./Core/Src/c64/cia.cyclo ./Core/Src/c64/cia.d ./Core/Src/c64/cia.o ./Core/Src/c64/cia.su ./Core/Src/c64/cpu.cyclo ./Core/Src/c64/cpu.d ./Core/Src/c64/cpu.o ./Core/Src/c64/cpu.su ./Core/Src/c64/display.cyclo ./Core/Src/c64/display.d ./Core/Src/c64/display.o ./Core/Src/c64/display.su ./Core/Src/c64/kernal.cyclo ./Core/Src/c64/kernal.d ./Core/Src/c64/kernal.o ./Core/Src/c64/kernal.su ./Core/Src/c64/memory.cyclo ./Core/Src/c64/memory.d ./Core/Src/c64/memory.o ./Core/Src/c64/memory.su ./Core/Src/c64/vic.cyclo ./Core/Src/c64/vic.d ./Core/Src/c64/vic.o ./Core/Src/c64/vic.su

.PHONY: clean-Core-2f-Src-2f-c64

