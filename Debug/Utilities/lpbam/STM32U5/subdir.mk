################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/lpbam/STM32U5/stm32_ll_lpbam.c 

OBJS += \
./Utilities/lpbam/STM32U5/stm32_ll_lpbam.o 

C_DEPS += \
./Utilities/lpbam/STM32U5/stm32_ll_lpbam.d 


# Each subdirectory must supply rules for building sources it contributes
Utilities/lpbam/STM32U5/%.o Utilities/lpbam/STM32U5/%.su Utilities/lpbam/STM32U5/%.cyclo: ../Utilities/lpbam/STM32U5/%.c Utilities/lpbam/STM32U5/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32U575xx -DUX_INCLUDE_USER_DEFINE_FILE -c -I../LPBAM/LpbamAp1 -I../Core/Inc -I../Utilities/lpbam -I../Utilities/lpbam/STM32U5 -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32U5xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../USBX/App -I../USBX/Target -I../Middlewares/ST/usbx/common/core/inc -I../Middlewares/ST/usbx/ports/generic/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Utilities-2f-lpbam-2f-STM32U5

clean-Utilities-2f-lpbam-2f-STM32U5:
	-$(RM) ./Utilities/lpbam/STM32U5/stm32_ll_lpbam.cyclo ./Utilities/lpbam/STM32U5/stm32_ll_lpbam.d ./Utilities/lpbam/STM32U5/stm32_ll_lpbam.o ./Utilities/lpbam/STM32U5/stm32_ll_lpbam.su

.PHONY: clean-Utilities-2f-lpbam-2f-STM32U5

