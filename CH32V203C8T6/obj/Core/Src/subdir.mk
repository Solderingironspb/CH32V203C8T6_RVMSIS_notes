################################################################################
# MRS Version: {"version":"1.8.4","date":"2023/02/015"}
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ch32v20x_RVMSIS.c \
../Core/Src/core_riscv.c \
../Core/Src/main.c \
../Core/Src/system_ch32v20x.c 

OBJS += \
./Core/Src/ch32v20x_RVMSIS.o \
./Core/Src/core_riscv.o \
./Core/Src/main.o \
./Core/Src/system_ch32v20x.o 

C_DEPS += \
./Core/Src/ch32v20x_RVMSIS.d \
./Core/Src/core_riscv.d \
./Core/Src/main.d \
./Core/Src/system_ch32v20x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I"C:\dev\CH32V203C8T6\obj" -I"C:\dev\CH32V203C8T6\Core\Inc" -I"C:\dev\CH32V203C8T6\Core\Src" -I"C:\dev\CH32V203C8T6\Drivers\inc" -I"C:\dev\CH32V203C8T6\Drivers\src" -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

