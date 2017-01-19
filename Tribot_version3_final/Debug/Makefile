################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../main.cpp \
../Messages.cpp \
../Robogami.cpp \
../SoftwareIrDAINT.cpp \
../TinyVCNL4010.cpp \
../Tiny_USI_TWI_Master.cpp \
../wiring.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
main.o \
Messages.o \
Robogami.o \
SoftwareIrDAINT.o \
TinyVCNL4010.o \
Tiny_USI_TWI_Master.o \
wiring.o

OBJS_AS_ARGS +=  \
main.o \
Messages.o \
Robogami.o \
SoftwareIrDAINT.o \
TinyVCNL4010.o \
Tiny_USI_TWI_Master.o \
wiring.o

C_DEPS +=  \
main.d \
Messages.d \
Robogami.d \
SoftwareIrDAINT.d \
TinyVCNL4010.d \
Tiny_USI_TWI_Master.d \
wiring.d

C_DEPS_AS_ARGS +=  \
main.d \
Messages.d \
Robogami.d \
SoftwareIrDAINT.d \
TinyVCNL4010.d \
Tiny_USI_TWI_Master.d \
wiring.d

OUTPUT_FILE_PATH +=Tribot_version3_final.elf

OUTPUT_FILE_PATH_AS_ARGS +=Tribot_version3_final.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler















./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -DDEBUG -DF_CPU=8000000 -DID=2  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATtiny_DFP\1.0.79\include"  -Os -fno-threadsafe-statics -fno-exceptions -MMD -ffunction-sections -fdata-sections -g2 -Wall -mmcu=attiny4313 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATtiny_DFP\1.0.79\gcc\dev\attiny4313" -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Tribot_version3_final.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=attiny4313 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATtiny_DFP\1.0.79\gcc\dev\attiny4313"  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Tribot_version3_final.elf" "Tribot_version3_final.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Tribot_version3_final.elf" "Tribot_version3_final.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "Tribot_version3_final.elf" > "Tribot_version3_final.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Tribot_version3_final.elf" "Tribot_version3_final.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "Tribot_version3_final.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Tribot_version3_final.elf" "Tribot_version3_final.a" "Tribot_version3_final.hex" "Tribot_version3_final.lss" "Tribot_version3_final.eep" "Tribot_version3_final.map" "Tribot_version3_final.srec" "Tribot_version3_final.usersignatures"
	