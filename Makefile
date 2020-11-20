NAME=stm32eforth
# tools
PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
AS=$(PREFIX)as 
LD=$(PREFIX)ld
OBJDUMP=$(PREFIX)objdump
OBJCOPY=$(PREFIX)objcopy

#build directory
BUILD_DIR=build/
#Link file
LD_FILE=stm32g431.ld 
LD_FLAGS=-mmcu=stm32g431
#sources
SRC=stm32eforth.s

.PHONY: all 

all: clean build dasm

build:  *.s 
	$(AS) $(SRC) -o$(BUILD_DIR)$(NAME).o
	$(LD) -T $(LD_FILE) -g $(BUILD_DIR)$(NAME).o -o $(BUILD_DIR)$(NAME).elf
	$(OBJCOPY) -O binary $(BUILD_DIR)$(NAME).elf $(BUILD_DIR)$(NAME).bin 

flash: $(BUILD_DIR)$(NAME).bin  
	st-flash write $(BUILD_DIR)$(NAME).bin 0x8000000

dasm:
	$(OBJDUMP) -D $(BUILD_DIR)$(NAME).elf > $(BUILD_DIR)$(NAME).dasm

.PHONY: clean 

clean:
	$(RM) build/*

	

