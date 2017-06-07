PROGRAM = openlight
MCU = attiny13
CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS += -Wall -g -Os -mmcu=$(MCU) -std=gnu99
LDFLAGS +=
OBJS = $(PROGRAM).o
# comment to print raw commands from build output
Q := @

all: $(PROGRAM).hex

$(PROGRAM).elf: $(PROGRAM).o
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(PROGRAM).hex: $(PROGRAM).elf
	@printf "  OBJCOPY $(subst $(shell pwd)/,,$(@))\n\n\n"
	$(Q)$(OBJCOPY) --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex $< $@
	$(Q)avr-size -C --mcu=$(MCU) $(PROGRAM).elf

%.o: %.c
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

flash: $(PROGRAM).hex
	@printf "  FLASH   $(PROGRAM).hex\n"
	$(Q)avrdude -c usbasp -p t13 -u -Uflash:w:$(PROGRAM).hex -Ulfuse:w:0x75:m -Uhfuse:w:0xFF:m

flash-example: precompiled.hex
	@printf "  FLASH   precompiled.hex\n"
	$(Q)avrdude -c usbasp -p t13 -u -Uflash:w:precompiled.hex -Ulfuse:w:0x75:m -Uhfuse:w:0xFF:m

clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f $(OBJS)
	@printf "  CLEAN   $(PROGRAM).elf\n"
	$(Q)rm -f *.elf
	@printf "  CLEAN   $(PROGRAM).hex\n"
	$(Q)rm -f $(PROGRAM).hex
