# Usage:
#
# make = make IHEX file
# make clean = Clean object files.
# make program = Download the hex file to the device
# make read = Read program from device. Reads only a specified part of the flash and is thus not compatible with the standard nano_programmer.
# make all = program and read
# make erase = Erase flash on device
# make restore = program restore.hex into device. Having a partial image to overwrite changes in flash helps developing boot loader

APP=bootloader

#Directories
#SOURCE_DIR = ../../FreeRTOS/Source
#PLATFORM_DIR=../../Platform/nano
#PORT_DIR = $(PLATFORM_DIR)
#COMMON_DIR =../../Common

# Application options
#include app.rules
# Platform options
#include $(PLATFORM_DIR)/platform.rules

# Target file name (without extension).
TARGET = $(APP)

# Number of kilobytes (not pages) to read
PAGES = 6

#Programming port
PGM_PORT = /dev/ttyUSB1

# Optimization level
OPT =
#OPT = s

# C sources
#SRC += $(PLATFORM_DIR)/adc.c 
# $(PLATFORM_DIR)/uart.c 

# Additional build options
#CFLAGS += -DCC2430_EM

MODEL=large
PARM_MODEL=--stack-auto
CFLAGS += --model-$(MODEL) $(PARM_MODEL) -DSDCC_CC2430
LDFLAGS += --model-$(MODEL) $(PARM_MODEL) --code-loc 0x7000 --code-size 65500
LDFLAGS+=--xram-loc 57344 --xram-size 8192
CFLAGS += --std-sdcc99
CC = sdcc
FORMAT = hex

#Application section
SRC = bootloader.c

# Assembler source files
ASRC += 

# Header files
HSRC += redirectint.h 

# Extra include directories
EXTRAINCDIRS = 

CFLAGS +=

# ---------------------------------------------------------------------------

# Object files.
OBJS = $(notdir $(SRC) ) $(notdir $(ASRC) )
#OBJS = $(addprefix .rels/build/$(APP)/,$(SRC:.c=.rel) ) $(addprefix .rels/build/$(APP)/,$(ASRC:.S=.rel) )
#DEPS = $(addprefix .deps/deps/$(APP)/,$(SRC:.c=.d) ) 
# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Finish command line options
ALL_CFLAGS = $(CFLAGS) -I.
ALL_ASFLAGS = $(ASFLAGS)  -I. -x assembler-with-cpp 

# Build targets:
binary: link $(TARGET).$(FORMAT)
	@echo ""
	@echo "--- Build complete ---"

obj: $(OBJS) 
	@echo ""
	@echo "--- Build done ---"

$(TARGET).hex: $(OBJS) $(notdir $(HSRC) )
	$(CC) $(LDFLAGS) -o $(TARGET).hex $(OBJS) 
	
link: obj $(TARGET).$(FORMAT)
	@echo ""
	@echo "--- Link done ---"

clean:
	rm -rf .rels $(TARGET).hex *.hex *.asm *.ihx *.lnk *.lst *.map *.mem *.rel *.rst *.sym

%.sym: %.elf
	@echo
	@echo "Symbols:" $@
	$(NM) -n $< > $@

%.lss: %.elf
	@echo
	@echo "Listing:" $@
	$(OBJDUMP) -h -S $< > $@

program: binary
	/home/ssp/Projects/Sensinode/NanoStack-v1.0.2/Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --program $(TARGET).$(FORMAT)

read:
	/home/ssp/Projects/Sensinode/NanoStack-v1.0.2/Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --read /media/disk/kood/out.$(FORMAT) -n $(PAGES)
all: program read

erase: 
	/home/ssp/Projects/Sensinode/NanoStack-v1.0.2/Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --erase

restore: binary
	/home/ssp/Projects/Sensinode/NanoStack-v1.0.2/Tools/nano_programmer/nano_programmer -d $(PGM_PORT) --program restore.hex

