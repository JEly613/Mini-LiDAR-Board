# ---------------------------------------------------------------------------
# Mini-LiDAR-Board firmware -- plain GNU make + arm-none-eabi-gcc.
#
#   make            build build/lidar-imu.elf, .bin, .hex and print the size
#   make flash      flash build/lidar-imu.bin over SWD with st-flash
#   make erase      full chip erase (use if the flash gets into a bad state)
#   make clean      remove build/
#
# Deliberately no CMake, no Cube, no HAL: the whole tree is this Makefile,
# the vendored CMSIS headers under cmsis/, and the C in src/.  It builds
# offline.
# ---------------------------------------------------------------------------

TARGET    := lidar-imu
BUILD_DIR := build

# ---- Toolchain -------------------------------------------------------------
PREFIX  := arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
OBJDUMP := $(PREFIX)objdump
SIZE    := $(PREFIX)size
ST_FLASH := st-flash

# ---- Sources ---------------------------------------------------------------
SRCS := \
	src/startup_stm32f411xe.c \
	src/clock.c \
	src/tim_us.c \
	src/debug_pins.c \
	src/iwdg.c \
	src/util.c \
	src/spi1_dma.c \
	src/icm42688.c \
	src/madgwick.c \
	src/motor.c \
	src/usart1_dma.c \
	src/rplidar.c \
	src/packet.c \
	src/pkt_ring.c \
	src/usb_cdc.c \
	src/main.c

OBJS := $(addprefix $(BUILD_DIR)/,$(SRCS:.c=.o))
DEPS := $(OBJS:.o=.d)

# ---- Flags -----------------------------------------------------------------
# The Cortex-M4F on the F411 has a single-precision FPU; -mfloat-abi=hard puts
# floats in VFP registers, which is what makes a 1 kHz Madgwick update cheap.
CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# -ffreestanding / -nostdlib: the Homebrew arm-none-eabi-gcc here ships no
# newlib, so there is no libc to link against.  util.c supplies the four mem*
# functions GCC may emit calls to, and -fno-math-errno lets __builtin_sqrtf
# compile to a bare VSQRT.F32 with no libm reference.
CFLAGS := $(CPU_FLAGS) \
	-std=c11 \
	-Os -g3 \
	-Wall -Wextra -Wshadow -Wundef -Wdouble-promotion \
	-ffreestanding -fno-math-errno \
	-ffunction-sections -fdata-sections \
	-fno-common \
	-DSTM32F411xE \
	-Iinclude \
	-isystem cmsis/core \
	-isystem cmsis/device \
	-MMD -MP

LDFLAGS := $(CPU_FLAGS) \
	-T stm32f411ce.ld \
	-nostdlib \
	-Wl,--gc-sections \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
	-Wl,--print-memory-usage

# libgcc supplies the compiler helper routines (64-bit shifts, etc.).  It is
# the only library this firmware links.
LDLIBS := -lgcc

# ---- Rules -----------------------------------------------------------------
.PHONY: all clean flash erase size disasm

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex size

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) stm32f411ce.ld
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

size: $(BUILD_DIR)/$(TARGET).elf
	@$(SIZE) $<

disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -d -S $< > $(BUILD_DIR)/$(TARGET).lst
	@echo "wrote $(BUILD_DIR)/$(TARGET).lst"

# Flash over SWD.  Wiring, from header J5 on the board to an ST-Link V2:
#   J5.1 3V3   -> ST-Link 3.3 V   (reference voltage; the board is normally
#                                  powered over USB-C, in which case leave
#                                  this pin unconnected)
#   J5.2 GND   -> ST-Link GND
#   J5.3 SWDIO -> ST-Link SWDIO
#   J5.4 SWCLK -> ST-Link SWCLK
# Neither attach mode is reliable on its own with this ST-Link clone:
# a plain attach intermittently loses the race against a board that is busy
# the instant it boots (spindle PWM, UART DMA, USB), while
# --connect-under-reset intermittently trips a libusb kernel-driver detach
# error on macOS.  Try the reset-halt path first and fall back to the plain
# one, which between them has not failed yet.
flash: $(BUILD_DIR)/$(TARGET).bin
	@$(ST_FLASH) --connect-under-reset --reset write $< 0x08000000 \
	  || { echo "--> retrying without --connect-under-reset"; \
	       $(ST_FLASH) --reset write $< 0x08000000; }

erase:
	$(ST_FLASH) erase

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
