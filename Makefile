#  Makefile for Grbl-Mega-5X on the MC5S3DP-XHAT-GRBL-5X board (STM32F411RC)
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Targets:
#    make            build build/grbl.elf, .hex, .bin and .map
#    make flash      flash with OpenOCD (ST-Link, connect under reset)
#    make stflash    flash with st-flash (stlink tools)
#    make clean      remove build output
#
#  NOTE: flash the .hex/.elf, NOT the .bin. The .bin spans 0x08000000..end and
#  pads over the EEPROM-emulation sectors (0x08008000-0x0800FFFF), which would
#  wipe the stored $ settings.

TARGET   = grbl
BUILDDIR = build

PREFIX  ?= arm-none-eabi-
CC       = $(PREFIX)gcc
OBJCOPY  = $(PREFIX)objcopy
SIZE     = $(PREFIX)size

CPU      = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS     = -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000
INCLUDES = -IInc -IGrbl \
           -IDrivers/CMSIS/Include \
           -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
           -IDrivers/STM32F4xx_HAL_Driver/Inc

OPT     ?= -Os -g
CFLAGS   = $(CPU) $(DEFS) $(INCLUDES) $(OPT) -std=gnu11 -Wall \
           -ffunction-sections -fdata-sections -MMD -MP
ASFLAGS  = $(CPU)
LDSCRIPT = STM32F411RCTx_FLASH.ld
LDFLAGS  = $(CPU) -T$(LDSCRIPT) -specs=nano.specs -specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILDDIR)/$(TARGET).map
LIBS     = -lm

C_SOURCES   = $(wildcard Src/*.c) $(wildcard Grbl/*.c) \
              $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)
ASM_SOURCES = startup/startup_stm32f411xe.s

OBJECTS = $(addprefix $(BUILDDIR)/,$(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o))

OPENOCD_CFG = "MC5S3DP-XHAT-GRBL-5X Debug.cfg"

.PHONY: all clean flash stflash size

all: $(BUILDDIR)/$(TARGET).elf $(BUILDDIR)/$(TARGET).hex $(BUILDDIR)/$(TARGET).bin size

$(BUILDDIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILDDIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT)
	$(CC) $(LDFLAGS) -o $@ $(OBJECTS) $(LIBS)

$(BUILDDIR)/$(TARGET).hex: $(BUILDDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILDDIR)/$(TARGET).bin: $(BUILDDIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

size: $(BUILDDIR)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILDDIR)/$(TARGET).elf
	openocd -f $(OPENOCD_CFG) -c "program $< verify reset exit"

stflash: $(BUILDDIR)/$(TARGET).hex
	st-flash --reset --format ihex write $<

clean:
	rm -rf $(BUILDDIR)

-include $(OBJECTS:.o=.d)
