# Usage:
#
# make binary = Make binary file.
#
# make eeprom = Make eeprom file.
#
# make clean = Clean object files.
#
# make clean-deps = Clean object files and dependencies.
#
# make program = Download the hex file to the device

APP=rfid

#Directories
SOURCE_DIR = ../../FreeRTOS/Source
PLATFORM_DIR=../../Platform/nano
PORT_DIR = $(PLATFORM_DIR)
COMMON_DIR =../../Common

#BFLINTFSPATH=../intfs
#BFLSRCPATH=../comps
#INCLUDE = -I . -I $ ../types -I $(BFLINTFSPATH)/IphcsBflBal/inc -I $(BFLINTFSPATH)/IphcsBflRegCtl/inc -I $(BFLINTFSPATH)/IphcsBflIo/inc -I $(BFLINTFSPATH)/IphcsBflAux/inc -I $(BFLINTFSPATH)/IphcsBflOpCtl/inc -I $(BFLINTFSPATH)/IphcsBflMfRd/inc -I $(BFLINTFSPATH)/IphcsBflI3P3A/inc -I $(BFLINTFSPATH)/IphcsBflPolAct/inc -I $(BFLINTFSPATH)/IphcsBflNfc/inc/ -I $(BFLINTFSPATH)/IphcsBflI3P4/inc -I $(BFLINTFSPATH)/IphcsBflI3P4AAct/inc
#LIBCOBJECTS = $(BFLSRCPATH)/phcsBflBal/src/phcsBflBal_Hw1SerLin.o  $(BFLSRCPATH)/phcsBflRegCtl/src/phcsBflRegCtlSer/phcsBflRegCtl_SerHw1.o $(BFLSRCPATH)/phcsBflIo/src/phcsBflIoHw1/phcsBflIo_Hw1.o $(BFLSRCPATH)/phcsBflAux/src/phcsBflAux_Hw1.o $(BFLSRCPATH)/phcsBflOpCtl/src/phcsBflOpCtl_Hw1.o $(BFLSRCPATH)/phcsBflMfRd/src/phcsBflMfRd.o $(BFLSRCPATH)/phcsBflI3P3A/src/phcsBflI3P3A_Hw1.o $(BFLSRCPATH)/phcsBflPolAct/src/phcsBflPolAct_Hw1.o $(BFLSRCPATH)/phcsBflNfc/src/phcsBflNfc_Initiator.o $(BFLSRCPATH)/phcsBflNfc/src/phcsBflNfc_Target.o $(BFLSRCPATH)/phcsBflI3P4/src/phcsBflI3P4_Pcd.o  $(BFLSRCPATH)/phcsBflI3P4AAct/src/phcsBflI3P4AAct.o
#APPCOBJECTS = ExampleProjectC.o MifareReaderC.o FelicaReaderC.o NfcInitiatorC.o NfcInitiatorInitialiseC.o NfcTargetC.o ExampleUtilsC.o Iso14443_4_ReaderC.o ExampleGlobals.o

# Application options
include app.rules
# Platform options
include $(PLATFORM_DIR)/platform.rules

# Target file name (without extension).
TARGET = $(APP)

# This is for generating the C implementation
#app: $(APPCOBJECTS)
#	$(GCC) $(APPCOBJECTS) -L . -lCBFL -o CExample
#lib: $(LIBCOBJECTS)
#	$(LD) -shared -soname libCBFL.so.1 -o libCBFL.so.1.0 -lc $(LIBCOBJECTS)
#	$(GCC) -shared -o  libCBFL.dll $(LIBCOBJECTS)
#	/sbin/ldconfig -v -n .										
#	ln -sf libCBFL.so.1 libCBFL.so

# Optimization level
OPT =
#OPT = s

# C sources
SRC +=mifare.c
# $(PLATFORM_DIR)/uart.c 

# Additional build options
#CFLAGS += -DCC2430_EM

# Assembler source files
ASRC += 


# Extra include directories
EXTRAINCDIRS = 

CFLAGS +=

# ---------------------------------------------------------------------------

# Object files.
#OBJS = $(notdir $(SRC) ) $(notdir $(ASRC) )
OBJS = $(addprefix .rels/build/$(APP)/,$(SRC:.c=.rel) ) $(addprefix .rels/build/$(APP)/,$(ASRC:.S=.rel) )
DEPS = $(addprefix .deps/deps/$(APP)/,$(SRC:.c=.d) ) 
# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Finish command line options
ALL_CFLAGS = $(CFLAGS) -I.
ALL_ASFLAGS = $(ASFLAGS)  -I. -x assembler-with-cpp 

# Build targets:
binary: link $(TARGET).$(FORMAT)
	@echo ""
	@echo "--- Build complete ---"

config_start: 
	@echo "--- Config ---"

config: config_start module_id.h modules_conf.h
	@echo "--- Config done ---"

depend_start:
	@echo "--- Depend ---"
	
.deps/depend: 
	@mkdir -p .deps
	@touch .deps/depend

dep:	 module_id.h modules_conf.h
	@echo "--- Depend done ---"

obj: dep $(OBJS) 
	@echo ""
	@echo "--- Build done ---"

$(TARGET).hex: $(OBJS)
	$(CC) $(LDFLAGS) -o $(TARGET).hex $(OBJS) 
	
link: obj $(TARGET).$(FORMAT)
	@echo ""
	@echo "--- Link done ---"

clean:
	rm -rf .rels $(TARGET).hex *.hex *.asm *.ihx *.lnk *.lst *.map *.mem *.rel *.rst *.sym modules_conf.h module_id.h

clean-deps: 
	rm -rf .deps 
	
.deps/deps/$(APP):
	mkdir -p .deps/deps/$(APP)

.rels/build/$(APP):
	mkdir -p .rels/build/$(APP)
	
.rels/build/$(APP)/%.rel: %.c
	@echo
	@echo "Compile:" $<
	@mkdir -p $(dir  $(addprefix .rels/build/$(APP)/, $(<:.c=.rel)) ) 
	$(CC) -c $(ALL_CFLAGS) $< -o $(addprefix .rels/build/$(APP)/, $(<:.c=.rel)) 

.rels/build/$(APP)/%.s : %.c
	@echo
	@echo "Compile to asm:" $<
	$(CC) -S $(ALL_CFLAGS) $< -o $(addprefix .rels/build/$(APP)/, $(<:.c=.s)) 

.rels/build/$(APP)/%.rel : %.S
	@echo
	@echo "Assemble:" $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $(addprefix .rels/build/$(APP)/, $(<:.S=.rel)) 

.deps/deps/$(APP)/%.d: %.c
	@echo "Depend:" $<
	@mkdir -p $(dir  $(addprefix .deps/deps/$(APP)/, $(<:.c=.d)) ) 
	@$(CC) -MM $(ALL_CFLAGS) -o .deps/tmp $<
	@echo -n .rels/build/$(APP)/$(dir $<) > $@
	@cat .deps/tmp >> $@

%.sym: %.elf
	@echo
	@echo "Symbols:" $@
	$(NM) -n $< > $@

%.lss: %.elf
	@echo
	@echo "Listing:" $@
	$(OBJDUMP) -h -S $< > $@
	
program: binary
	../../Tools/programmer/programmer -d $(BSL_PORT) -c --program $(TARGET).$(FORMAT)
		 
include .deps/depend
