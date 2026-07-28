################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../libraries/zf_components/printf_redirect.c \
../libraries/zf_components/seekfree_assistant.c \
../libraries/zf_components/seekfree_assistant_interface.c 

OBJS += \
./libraries/zf_components/printf_redirect.o \
./libraries/zf_components/seekfree_assistant.o \
./libraries/zf_components/seekfree_assistant_interface.o 

COMPILED_SRCS += \
./libraries/zf_components/printf_redirect.src \
./libraries/zf_components/seekfree_assistant.src \
./libraries/zf_components/seekfree_assistant_interface.src 

C_DEPS += \
./libraries/zf_components/printf_redirect.d \
./libraries/zf_components/seekfree_assistant.d \
./libraries/zf_components/seekfree_assistant_interface.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/zf_components/%.src: ../libraries/zf_components/%.c libraries/zf_components/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/workspace002/Seekfree_TC264_DualCam_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

libraries/zf_components/%.o: ./libraries/zf_components/%.src libraries/zf_components/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


