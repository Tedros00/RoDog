################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CurveGen.c \
../Core/Src/HMC5883L.c \
../Core/Src/Madwick_AHRS.c \
../Core/Src/MahonyAHRS.c \
../Core/Src/kinematics.c \
../Core/Src/lsm6dso32x_reg.c \
../Core/Src/main.c \
../Core/Src/ov2640.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/CurveGen.o \
./Core/Src/HMC5883L.o \
./Core/Src/Madwick_AHRS.o \
./Core/Src/MahonyAHRS.o \
./Core/Src/kinematics.o \
./Core/Src/lsm6dso32x_reg.o \
./Core/Src/main.o \
./Core/Src/ov2640.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/CurveGen.d \
./Core/Src/HMC5883L.d \
./Core/Src/Madwick_AHRS.d \
./Core/Src/MahonyAHRS.d \
./Core/Src/kinematics.d \
./Core/Src/lsm6dso32x_reg.d \
./Core/Src/main.d \
./Core/Src/ov2640.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../FATFS/Target -I"C:/Users/User/Projects/Rodog/RoDog/CubeIDE_ws/RoDog/CMSIS/CMSIS/Include" -I../FATFS/App -I../LIBJPEG/App -I../LIBJPEG/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/LibJPEG/include -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/User/Projects/Rodog/RoDog/CubeIDE_ws/RoDog/USB_DEVICE" -I"C:/Users/User/Projects/Rodog/RoDog/CubeIDE_ws/RoDog/Middlewares/ST/STM32_USB_Device_Library" -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/wissam/STM32CubeIDE/workspace_1.10.1/RoDog/Drivers/CMSIS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CurveGen.cyclo ./Core/Src/CurveGen.d ./Core/Src/CurveGen.o ./Core/Src/CurveGen.su ./Core/Src/HMC5883L.cyclo ./Core/Src/HMC5883L.d ./Core/Src/HMC5883L.o ./Core/Src/HMC5883L.su ./Core/Src/Madwick_AHRS.cyclo ./Core/Src/Madwick_AHRS.d ./Core/Src/Madwick_AHRS.o ./Core/Src/Madwick_AHRS.su ./Core/Src/MahonyAHRS.cyclo ./Core/Src/MahonyAHRS.d ./Core/Src/MahonyAHRS.o ./Core/Src/MahonyAHRS.su ./Core/Src/kinematics.cyclo ./Core/Src/kinematics.d ./Core/Src/kinematics.o ./Core/Src/kinematics.su ./Core/Src/lsm6dso32x_reg.cyclo ./Core/Src/lsm6dso32x_reg.d ./Core/Src/lsm6dso32x_reg.o ./Core/Src/lsm6dso32x_reg.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ov2640.cyclo ./Core/Src/ov2640.d ./Core/Src/ov2640.o ./Core/Src/ov2640.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

