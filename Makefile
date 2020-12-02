NAME=stm32eforth
# tools
PREFIX=arm-none-eabi-
CC=$(PREFIX)gcc
AS=$(PREFIX)as 
LD=$(PREFIX)ld
DBG=gdb-multiarch
OBJDUMP=$(PREFIX)objdump
OBJCOPY=$(PREFIX)objcopy

#build directory
BUILD_DIR=build/
#Link file
LD_FILE=board/blue-pill/stm32f103c8t6.ld 
LD_FLAGS=-mmcu=stm32f103
#sources
SRC=stm32eforth.s 

.PHONY: all 

all: clean build dasm

build:  *.s Makefile
	$(AS) -a=$(BUILD_DIR)$(NAME).lst $(SRC) -g -o$(BUILD_DIR)$(NAME).o
	$(LD) -T $(LD_FILE) -g $(BUILD_DIR)$(NAME).o -o $(BUILD_DIR)$(NAME).elf
	$(OBJCOPY) -O binary $(BUILD_DIR)$(NAME).elf $(BUILD_DIR)$(NAME).bin 
#	$(OBJCOPY) -O ihex $(BUILD_DIR)$(NAME).elf $(BUILD_DIR)$(NAME).hex  
	$(OBJDUMP) -D $(BUILD_DIR)$(NAME).elf > $(BUILD_DIR)$(NAME).dasm

flash: $(BUILD_DIR)$(NAME).bin 
	st-flash write $(BUILD_DIR)$(NAME).bin 0x8000000

dasm:
	$(OBJDUMP) -D $(BUILD_DIR)$(NAME).elf > $(BUILD_DIR)$(NAME).dasm

debug: 
	cd $(BUILD_DIR) &&\
	$(DBG) -tui --eval-command="target remote localhost:4242" $(NAME).elf

.PHONY: clean 

clean:
	$(RM) build/*


	

