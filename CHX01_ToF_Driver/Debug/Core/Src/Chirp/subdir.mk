################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Chirp/ch101_gpr_fw.cpp \
../Core/Src/Chirp/ch201_gprmt_fw.cpp \
../Core/Src/Chirp/ch_com.cpp \
../Core/Src/Chirp/ch_drv_ch101.cpp \
../Core/Src/Chirp/ch_drv_ch201.cpp \
../Core/Src/Chirp/ch_lib.cpp \
../Core/Src/Chirp/usr_bsp.cpp \
../Core/Src/Chirp/usr_def.cpp \
../Core/Src/Chirp/usr_i2c.cpp 

OBJS += \
./Core/Src/Chirp/ch101_gpr_fw.o \
./Core/Src/Chirp/ch201_gprmt_fw.o \
./Core/Src/Chirp/ch_com.o \
./Core/Src/Chirp/ch_drv_ch101.o \
./Core/Src/Chirp/ch_drv_ch201.o \
./Core/Src/Chirp/ch_lib.o \
./Core/Src/Chirp/usr_bsp.o \
./Core/Src/Chirp/usr_def.o \
./Core/Src/Chirp/usr_i2c.o 

CPP_DEPS += \
./Core/Src/Chirp/ch101_gpr_fw.d \
./Core/Src/Chirp/ch201_gprmt_fw.d \
./Core/Src/Chirp/ch_com.d \
./Core/Src/Chirp/ch_drv_ch101.d \
./Core/Src/Chirp/ch_drv_ch201.d \
./Core/Src/Chirp/ch_lib.d \
./Core/Src/Chirp/usr_bsp.d \
./Core/Src/Chirp/usr_def.d \
./Core/Src/Chirp/usr_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Chirp/%.o: ../Core/Src/Chirp/%.cpp Core/Src/Chirp/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xC -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/nikhil/STM32CubeIDE/workspace_1.7.0/ToF_Driver/Core/Src/Chirp" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

