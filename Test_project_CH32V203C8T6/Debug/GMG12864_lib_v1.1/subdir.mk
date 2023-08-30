################################################################################
# MRS Version: 1.9.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../GMG12864_lib_v1.1/GMG12864_lib.c 

OBJS += \
./GMG12864_lib_v1.1/GMG12864_lib.o 

C_DEPS += \
./GMG12864_lib_v1.1/GMG12864_lib.d 


# Each subdirectory must supply rules for building sources it contributes
GMG12864_lib_v1.1/%.o: ../GMG12864_lib_v1.1/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I../Core/Inc -I../Buttons -I../Float_transform -I../GMG12864_lib_v1.1 -I../Drivers/inc -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

