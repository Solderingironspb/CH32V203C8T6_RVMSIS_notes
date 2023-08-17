################################################################################
# MRS Version: {"version":"1.8.5","date":"2023/05/22"}
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/ch32v20x_adc.c \
../Drivers/src/ch32v20x_bkp.c \
../Drivers/src/ch32v20x_can.c \
../Drivers/src/ch32v20x_crc.c \
../Drivers/src/ch32v20x_dbgmcu.c \
../Drivers/src/ch32v20x_dma.c \
../Drivers/src/ch32v20x_exti.c \
../Drivers/src/ch32v20x_flash.c \
../Drivers/src/ch32v20x_gpio.c \
../Drivers/src/ch32v20x_i2c.c \
../Drivers/src/ch32v20x_iwdg.c \
../Drivers/src/ch32v20x_misc.c \
../Drivers/src/ch32v20x_opa.c \
../Drivers/src/ch32v20x_pwr.c \
../Drivers/src/ch32v20x_rcc.c \
../Drivers/src/ch32v20x_rtc.c \
../Drivers/src/ch32v20x_spi.c \
../Drivers/src/ch32v20x_tim.c \
../Drivers/src/ch32v20x_usart.c \
../Drivers/src/ch32v20x_wwdg.c 

OBJS += \
./Drivers/src/ch32v20x_adc.o \
./Drivers/src/ch32v20x_bkp.o \
./Drivers/src/ch32v20x_can.o \
./Drivers/src/ch32v20x_crc.o \
./Drivers/src/ch32v20x_dbgmcu.o \
./Drivers/src/ch32v20x_dma.o \
./Drivers/src/ch32v20x_exti.o \
./Drivers/src/ch32v20x_flash.o \
./Drivers/src/ch32v20x_gpio.o \
./Drivers/src/ch32v20x_i2c.o \
./Drivers/src/ch32v20x_iwdg.o \
./Drivers/src/ch32v20x_misc.o \
./Drivers/src/ch32v20x_opa.o \
./Drivers/src/ch32v20x_pwr.o \
./Drivers/src/ch32v20x_rcc.o \
./Drivers/src/ch32v20x_rtc.o \
./Drivers/src/ch32v20x_spi.o \
./Drivers/src/ch32v20x_tim.o \
./Drivers/src/ch32v20x_usart.o \
./Drivers/src/ch32v20x_wwdg.o 

C_DEPS += \
./Drivers/src/ch32v20x_adc.d \
./Drivers/src/ch32v20x_bkp.d \
./Drivers/src/ch32v20x_can.d \
./Drivers/src/ch32v20x_crc.d \
./Drivers/src/ch32v20x_dbgmcu.d \
./Drivers/src/ch32v20x_dma.d \
./Drivers/src/ch32v20x_exti.d \
./Drivers/src/ch32v20x_flash.d \
./Drivers/src/ch32v20x_gpio.d \
./Drivers/src/ch32v20x_i2c.d \
./Drivers/src/ch32v20x_iwdg.d \
./Drivers/src/ch32v20x_misc.d \
./Drivers/src/ch32v20x_opa.d \
./Drivers/src/ch32v20x_pwr.d \
./Drivers/src/ch32v20x_rcc.d \
./Drivers/src/ch32v20x_rtc.d \
./Drivers/src/ch32v20x_spi.d \
./Drivers/src/ch32v20x_tim.d \
./Drivers/src/ch32v20x_usart.d \
./Drivers/src/ch32v20x_wwdg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o: ../Drivers/src/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I"Z:\CH32V203C8T6\obj" -I"Z:\CH32V203C8T6\Core\Inc" -I"Z:\CH32V203C8T6\Core\Src" -I"Z:\CH32V203C8T6\Drivers\inc" -I"Z:\CH32V203C8T6\Drivers\src" -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

