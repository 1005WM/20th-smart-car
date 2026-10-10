################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/IMU.c \
../code/buzzer.c \
../code/connect.c \
../code/image.c \
../code/justfloat.c \
../code/motor.c \
../code/vofa.c 

OBJS += \
./code/IMU.o \
./code/buzzer.o \
./code/connect.o \
./code/image.o \
./code/justfloat.o \
./code/motor.o \
./code/vofa.o 

COMPILED_SRCS += \
./code/IMU.src \
./code/buzzer.src \
./code/connect.src \
./code/image.src \
./code/justfloat.src \
./code/motor.src \
./code/vofa.src 

C_DEPS += \
./code/IMU.d \
./code/buzzer.d \
./code/connect.d \
./code/image.d \
./code/justfloat.d \
./code/motor.d \
./code/vofa.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/workspace002/Seekfree_TC264_DualCam_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


