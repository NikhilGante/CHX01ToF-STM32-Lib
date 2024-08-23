################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/ForusDrivers/PAL_STM32CubeL1.cpp \
../Core/Src/ForusDrivers/Utilities3n.cpp 

OBJS += \
./Core/Src/ForusDrivers/PAL_STM32CubeL1.o \
./Core/Src/ForusDrivers/Utilities3n.o 

CPP_DEPS += \
./Core/Src/ForusDrivers/PAL_STM32CubeL1.d \
./Core/Src/ForusDrivers/Utilities3n.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ForusDrivers/%.o: ../Core/Src/ForusDrivers/%.cpp Core/Src/ForusDrivers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/nikhil/STM32CubeIDE/workspace_1.7.0/ToF_Driver/Core/Src/Chirp" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

