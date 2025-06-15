################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/lpbam/stm32_adv_lpbam_common.c \
../Utilities/lpbam/stm32_lpbam_common.c 

OBJS += \
./Utilities/lpbam/stm32_adv_lpbam_common.o \
./Utilities/lpbam/stm32_lpbam_common.o 

C_DEPS += \
./Utilities/lpbam/stm32_adv_lpbam_common.d \
./Utilities/lpbam/stm32_lpbam_common.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/lpbam/%.o Utilities/lpbam/%.su Utilities/lpbam/%.cyclo: ../Utilities/lpbam/%.c Utilities/lpbam/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../LPBAM/LpbamAp1 -I../Core/Inc -I../Utilities/lpbam -I../Utilities/lpbam/STM32U5 -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32U5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utilities-2f-lpbam

clean-Utilities-2f-lpbam:
	-$(RM) ./Utilities/lpbam/stm32_adv_lpbam_common.cyclo ./Utilities/lpbam/stm32_adv_lpbam_common.d ./Utilities/lpbam/stm32_adv_lpbam_common.o ./Utilities/lpbam/stm32_adv_lpbam_common.su ./Utilities/lpbam/stm32_lpbam_common.cyclo ./Utilities/lpbam/stm32_lpbam_common.d ./Utilities/lpbam/stm32_lpbam_common.o ./Utilities/lpbam/stm32_lpbam_common.su

.PHONY: clean-Utilities-2f-lpbam

