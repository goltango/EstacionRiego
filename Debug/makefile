################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: EstacionRiego.axf

# Tool invocations
EstacionRiego.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/goltango/workspace/workspaceLPCXpresso/CMSIS_CORE_LPC17xx/Debug" -L"/home/goltango/workspace/workspaceLPCXpresso/CMSIS_DSPLIB_CM3/lib" -Xlinker -Map="EstacionRiego.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "EstacionRiego_Debug.ld" -o "EstacionRiego.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) EstacionRiego.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "EstacionRiego.axf"; # arm-none-eabi-objcopy -v -O binary "EstacionRiego.axf" "EstacionRiego.bin" ; # checksum -p LPC1769 -d "EstacionRiego.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
