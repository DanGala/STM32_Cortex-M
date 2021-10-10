#******************************************************************************
# @file      makefile
# @author    Daniel Gala Montes
# @version   V1.0
# @date      08/09/2021
# @copy
# ******************************************************************************

# Project name
PROJECT_NAME = STM32L476RG

# Directory definition
ST_LIB_DIR= STM32L4xx_HAL_Driver
RTOS_SOURCE_DIR = ../FreeRTOS/Source

# Directory for output files (lst, obj, dep, elf, sym, map, hex, bin etc.).
OUTDIR = Debug/obj
BINDIR = Debug/bin

# Toolchain definition.
TOOLCHAIN_SOURCE_DIR = C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.07/bin
CC = $(TOOLCHAIN_SOURCE_DIR)/arm-none-eabi-gcc
CXX = $(TOOLCHAIN_SOURCE_DIR)/arm-none-eabi-g++
OBJCOPY	= $(TOOLCHAIN_SOURCE_DIR)/arm-none-eabi-objcopy
OBJDUMP	= $(TOOLCHAIN_SOURCE_DIR)/arm-none-eabi-size

# Compiler flags definition.
BASE_FLAGS = -g2 \
		-O0 \
		-DSTM32L476xx \
		-MMD \
		-mthumb \
		-mcpu=cortex-m4 \
		-ffunction-sections \
		-fdata-sections \
		-fno-exceptions \
		-fno-strict-aliasing \
		-mfpu=fpv4-sp-d16 \
		-mfloat-abi=hard \
		-fsingle-precision-constant \
		-Wall

CFLAGS = $(BASE_FLAGS) -std=gnu11
CPPFLAGS = $(BASE_FLAGS) -std=gnu++11 -Wno-reorder

INCLUDES = -I ../Configuration \
		-I ../Common/Garden \
		-I ../Common/Hardware \
   		-I ./BSP/STM32L4xx_Nucleo \
   		-I $(ST_LIB_DIR)/Inc \
   		-I ./CMSIS/Include \
   		-I ./CMSIS/Device/ST/STM32L4xx/Include \
		-I ./Hardware \
		-I ./FreeRTOS \
		-I $(RTOS_SOURCE_DIR)/include \
		-I $(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM4F

LINKER_SCRIPT = Hardware/STM32L476RGTx_FLASH.ld
LINKER_FLAGS = -T $(LINKER_SCRIPT) $(BASE_FLAGS) -o $(BINDIR)/$(PROJECT_NAME).hex -Wl,--gc-sections,-Map=$(BINDIR)/$(PROJECT_NAME).map -lstdc++ --specs=nosys.specs

STARTUP = Hardware/startup_stm32l476xx.s

# Source files
CSOURCE = $(wildcard ../Common/**/*.c)
CSOURCE += $(wildcard ./Hardware/*.c)
		
# ST Library source files.
ST_LIB_SOURCE = $(wildcard $(ST_LIB_DIR)/Src/*.c)

# FreeRTOS source files.
FREERTOS_SOURCE =	$(wildcard $(RTOS_SOURCE_DIR)/*.c) \
					$(RTOS_SOURCE_DIR)/portable/GCC/ARM_CM4F/port.c \
					$(RTOS_SOURCE_DIR)/portable/MemMang/heap_1.c
	
#Add all sources
CSOURCE+=$(ST_LIB_SOURCE)
CSOURCE+=$(FREERTOS_SOURCE)
CSOURCE+=$(STARTUP)

CPPSOURCE = $(wildcard ../Common/**/*.cpp)
CPPSOURCE += $(wildcard ./Hardware/*.cpp)

# List of all source files without directory and file-extension.
ALLSRCBASE = $(notdir $(basename $(CSOURCE)))
ALLSRCBASE += $(notdir $(basename $(CPPSOURCE)))

# List of all objects files.
OBJS = $(addprefix $(OUTDIR)/, $(addsuffix .o, $(ALLSRCBASE)))

# Rules definition. ***********************************************************

# Add .d to Make's recognized suffixes.
SUFFIXES += .d

all: post-build

post-build: build
	@echo ====== Estimating size ======
	$(OBJDUMP) $(BINDIR)/$(PROJECT_NAME).elf

build: $(BINDIR)/$(PROJECT_NAME).bin

$(BINDIR)/$(PROJECT_NAME).bin : $(BINDIR)/$(PROJECT_NAME).hex
	@echo ========== Objcopy ==========
	$(OBJCOPY) $(BINDIR)/$(PROJECT_NAME).hex -O binary $(BINDIR)/$(PROJECT_NAME).bin
	$(OBJCOPY) $(BINDIR)/$(PROJECT_NAME).hex -O elf32-littlearm $(BINDIR)/$(PROJECT_NAME).elf

$(BINDIR)/$(PROJECT_NAME).hex : $(OBJS)
	@echo ========== Linking ==========
	$(CXX) $(INCLUDES) $(LINKER_FLAGS) $(OBJS)

# Compile: create object files from C source files.
define COMPILE_C_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo Building $$<...
	@$(CC) -c $$< -o $$@ $$(CFLAGS) $$(INCLUDES)
endef
$(foreach src, $(CSOURCE), $(eval $(call COMPILE_C_TEMPLATE, $(src))))

# Compile: create object files from C++ source files.
define COMPILE_CPP_TEMPLATE
$(OUTDIR)/$(notdir $(basename $(1))).o : $(1)
	@echo Building $$<...
	@$(CXX) -c $$< -o $$@ $$(CPPFLAGS) $$(INCLUDES)
endef
$(foreach src, $(CPPSOURCE), $(eval $(call COMPILE_CPP_TEMPLATE, $(src))))

clean:
	@echo ======== Cleaning up ========
	@del Debug\obj\* /S /Q
	@del Debug\bin\* /S /Q