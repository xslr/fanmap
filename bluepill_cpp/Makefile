PREFIX?=arm-none-eabi-
CC=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
OD=bin

PC_CFLAGS += `pkg-config --cflags libusb-1.0` -std=c++17 -g
PC_LDFLAGS += `pkg-config --libs libusb-1.0` -lstdc++fs -pthread

all: realall.really fanmap

SFLAGS= --static -nostartfiles -std=c11 -g3 -Os
SFLAGS+= -fno-common -ffunction-sections -fdata-sections
SFLAGS+= -I./libopencm3/include -L./libopencm3/lib
LFLAGS+=-Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

M0_FLAGS= $(SFLAGS) -mcpu=cortex-m0 -mthumb -msoft-float
M0P_FLAGS= $(SFLAGS) -mcpu=cortex-m0plus -mthumb -msoft-float
M3_FLAGS= $(SFLAGS) -mcpu=cortex-m3 -mthumb -msoft-float
M4FH_FLAGS= $(SFLAGS) -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
M7SP_FLAGS= $(SFLAGS) -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16

include boards.stm32.mk

realall.really: outdir $(BOARDS_ELF) $(BOARDS_BIN) $(BOARDS_HEX)

libopencm3/Makefile:
	@echo "Initializing libopencm3 submodule"
	git submodule update --init

libopencm3/lib/libopencm3_%.a: libopencm3/Makefile
	$(MAKE) -C libopencm3

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(OBJCOPY) -Oihex $(*).elf $(*).hex

fanmap: fanmap.cc
	g++ $< $(PC_CFLAGS) $(PC_LDFLAGS) -o $@

outdir:
	mkdir -p $(OD)/stm32

clean:
	$(RM) $(BOARDS_ELF) $(BOARDS_BIN) $(BOARDS_HEX)

.PHONY: realall.really outdir clean all
$(V).SILENT:
