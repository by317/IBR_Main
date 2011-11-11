################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/Texas_Instruments/ccsv5/tools/compiler/c2000

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./source/easy2802x_sci_v7.3.obj" \
"./source/RingBuff.obj" \
"./source/Piccolo_PWM.obj" \
"./source/IBI_main.obj" \
"./source/DSP2802x_usDelay.obj" \
"./source/DSP2802x_SysCtrl.obj" \
"./source/DSP2802x_PieVect.obj" \
"./source/DSP2802x_PieCtrl.obj" \
"./source/DSP2802x_MemCopy.obj" \
"./source/DSP2802x_GlobalVariableDefs.obj" \
"./source/DSP2802x_DefaultIsr.obj" \
"./source/DSP2802x_CodeStartBranch.obj" \
-l"rts2800_ml.lib" \
"../DSP2802x_Headers_nonBIOS.cmd" \
"../F28026.cmd" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include source/subdir_vars.mk
-include subdir_rules.mk
-include source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: IBI.out

# Tool invocations
IBI.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/Texas_Instruments/ccsv5/tools/compiler/c2000/bin/cl2000" -v28 -mt -ml --symdebug:coff --diag_warning=225 --display_error_number --remove_hooks_when_inlining -z -m"IBI.map" --stack_size=0x300 --warn_sections -i"C:/Texas_Instruments/ccsv5/tools/compiler/c2000/lib" -i"C:/Texas_Instruments/ccsv5/tools/compiler/c2000/include" --reread_libs --rom_model -o "IBI.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C2000_EXECUTABLE_OUTPUTS__QUOTED) "IBI.out"
	-$(RM) "source\DSP2802x_DefaultIsr.pp" "source\DSP2802x_GlobalVariableDefs.pp" "source\DSP2802x_MemCopy.pp" "source\DSP2802x_PieCtrl.pp" "source\DSP2802x_PieVect.pp" "source\DSP2802x_SysCtrl.pp" "source\IBI_main.pp" "source\Piccolo_PWM.pp" "source\RingBuff.pp" "source\easy2802x_sci_v7.3.pp" 
	-$(RM) "source\DSP2802x_CodeStartBranch.obj" "source\DSP2802x_DefaultIsr.obj" "source\DSP2802x_GlobalVariableDefs.obj" "source\DSP2802x_MemCopy.obj" "source\DSP2802x_PieCtrl.obj" "source\DSP2802x_PieVect.obj" "source\DSP2802x_SysCtrl.obj" "source\DSP2802x_usDelay.obj" "source\IBI_main.obj" "source\Piccolo_PWM.obj" "source\RingBuff.obj" "source\easy2802x_sci_v7.3.obj" 
	-$(RM) "source\DSP2802x_CodeStartBranch.pp" "source\DSP2802x_usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
