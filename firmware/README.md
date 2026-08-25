# Mini-LiDAR-Board firmware — IMU orientation + LiDAR rings

Register-level bare-metal C for the STM32F411CEU6 on the custom Mini-LiDAR
scanner board. It reads the on-board ICM-42688-P at 1 kHz, runs a 6-DOF
Madgwick filter, and streams the resulting quaternion to a host over USB
CDC-ACM at ~100 Hz.

It also drives the RPLIDAR A1: TIM2_CH3 PWM on PB10 sets the spindle speed,
USART1 with circular DMA carries the standard `SCAN` stream, and each complete
revolution is forwarded to the host paired with the quaternion at ring-close
time.

**There is no 3D reconstruction in this milestone.** Rings ship in the
sensor's own 2D polar frame. The quaternion travels alongside them, but nothing
rotates anything yet. See [HARDWARE.md](HARDWARE.md).

Host-adjustable at runtime: spindle speed (PWM duty), point decimation, and
scan on/off — see the command table in [PROTOCOL.md](PROTOCOL.md).

Related documents:

* [HARDWARE.md](HARDWARE.md) — verified pin map, IMU quirks, and where the old
  project guide is wrong (**start here**)
* [PROTOCOL.md](PROTOCOL.md) — the binary wire format
* [`../host/`](../host) — the Python visualiser

---

## Requirements

Nothing but the GNU Arm toolchain and `stlink`. No CMake, no STM32Cube, no HAL.
CMSIS core and device headers are vendored into `cmsis/`, so the tree builds
offline.

```
brew install arm-none-eabi-gcc stlink   # already present on this machine
```

The Homebrew `arm-none-eabi-gcc` ships **without newlib**: there is no `libc.a`,
no `<string.h>` and no `<math.h>`. The build is therefore `-ffreestanding
-nostdlib`, `src/util.c` supplies the four `mem*` functions GCC may emit calls
to, and `__builtin_sqrtf` compiles straight to a `VSQRT.F32` instruction, so no
libm is needed.

---

## Build

```
cd firmware
make
```

Produces `build/lidar-imu.elf`, `.bin`, `.hex` and a linker map. The build is
clean under `-Wall -Wextra -Wshadow -Wundef -Wdouble-promotion`.

Other targets:

| target | effect |
|---|---|
| `make flash` | write `build/lidar-imu.bin` to `0x08000000` via `st-flash` and reset |
| `make erase` | full chip erase, if the flash gets into a bad state |
| `make disasm` | annotated disassembly to `build/lidar-imu.lst` |
| `make clean` | remove `build/` |

---

## Wiring the ST-Link to J5

Header **J5** is the SWD port. Pinout, in order:

| J5 pin | signal | ST-Link V2 pin |
|---|---|---|
| 1 | 3V3 | 3.3 V (see note) |
| 2 | GND | GND |
| 3 | SWDIO | SWDIO |
| 4 | SWCLK | SWCLK |

**Note on pin 1.** If the board is powered over its USB-C connector, leave J5.1
disconnected — do not back-feed 3.3 V from the ST-Link into a board that is
already powered. If the board is *not* on USB, connect J5.1 to the ST-Link's
3.3 V output to power it. Never connect J5.1 to the ST-Link's 5 V pin.

Check the probe sees the target before flashing:

```
st-info --probe
```

You should see an `F4` device with 512 KiB of flash. Then:

```
make flash
```

---

## What to expect after flashing

**This board has no GPIO status LEDs.** D3 is a power indicator wired across the
3.3 V rail; the MCU cannot drive it. Nothing blinks. All status is reported over
USB, or on the J4 scope pins.

1. Plug the USB-C cable into a Mac. Within a second or two a serial device
   appears:

   ```
   ls /dev/cu.usbmodem*
   ```

   The device name embeds the STM32's 96-bit unique ID, so two boards on the
   same host get distinct nodes.

2. The firmware immediately sends a `STATUS`/`BOOT` frame, then a
   `STATUS`/`IMU_OK` frame (or an error frame if `WHO_AM_I` did not read
   `0x47`), then `CALIB_START`.

3. **Hold the board still for about two seconds.** It averages 2000 gyro
   samples to measure the zero-rate bias. If it detects motion above 5 °/s it
   retries, up to three times. Then `CALIB_DONE`.

4. From then on it streams `ORIENTATION` frames at ~100 Hz. Run the host app:

   ```
   cd ../host && python3 -m lidarscan
   ```

If you want to see raw bytes rather than run the visualiser, note that the
stream is **binary**, so `cat /dev/cu.usbmodem*` will fill the terminal with
garbage. Use `xxd` or the host app.

### J4 timing pins

Put a logic analyser on J4 to see the firmware's real timing without the USB
link:

| pin | signal | expected |
|---|---|---|
| PB3 | DEBUG_A | toggles on each IMU data-ready → 500 Hz square wave |
| PB4 | DEBUG_B | toggles on each Madgwick update → 500 Hz, phase-shifted from A by the filter latency |
| PB5 | DEBUG_C | toggles on each packet queued → 50 Hz |

If DEBUG_A is flat, the IMU is not asserting INT1. If DEBUG_A runs but DEBUG_B
does not, the main loop is stuck.

### Recovery behaviour

* An independent watchdog (IWDG, ~500 ms nominal) resets the board if the main
  loop stalls. The symptom is the USB serial device disappearing and coming
  back.
* If no IMU data-ready edge arrives for 200 ms, the firmware re-runs
  `icm_init()` and reports `IMU_STALL` then `IMU_RECOVERED`.
* If the 8 MHz crystal fails to start there is no 48 MHz for USB and no way to
  report anything, so the firmware resets. A genuinely dead crystal shows up as
  a boot loop, visible with a scope on PH0.

---

## Source layout

```
firmware/
  Makefile                    plain make, no generator
  stm32f411ce.ld              linker script (512K flash / 128K RAM)
  cmsis/core/                 vendored ARM CMSIS-Core headers
  cmsis/device/               vendored ST device headers (stm32f411xe.h etc.)
  include/, src/
    startup_stm32f411xe.c     vector table + reset entry, written in C
    clock.c                   96 MHz PLL with exact 48 MHz PLLQ for USB
    tim_us.c                  TIM5 as a 32-bit 1 MHz free-running clock
    debug_pins.c              J4 scope pins (the board's only indicators)
    iwdg.c                    independent watchdog
    util.c                    freestanding mem* replacements
    spi1_dma.c                SPI1 + DMA2 Str0/Str3; polled path for config
    icm42688.c                ICM-42688-P driver, EXTI0 data-ready path
    madgwick.c                6-DOF Madgwick filter, runtime-tunable beta
    packet.c                  wire-format serialisers (see PROTOCOL.md)
    pkt_ring.c                frame-granular TX ring, drop-oldest on overflow
    usb_cdc.c                 bare-metal OTG_FS device stack + CDC-ACM class
    main.c                    bring-up, gyro bias calibration, main loop
```

### Interrupt priorities

| priority | source | why |
|---|---|---|
| 1 | EXTI0 (IMU data-ready) | must not be delayed; it only timestamps and starts DMA |
| 1 | DMA2_Stream0 (SPI RX complete) | decodes the sample; same tier as the trigger |
| 3 | OTG_FS | USB can always wait for the IMU, never the other way round |

---

## Verification status

Honest accounting of what has and has not been exercised:

| item | status |
|---|---|
| Compiles and links cleanly, zero warnings under `-Wall -Wextra` | **verified by execution** |
| Flash/RAM footprint fits the part with enormous margin | **verified** (8.6 KiB flash, 5.4 KiB RAM) |
| Register values checked against RM0383 and the ICM-42688-P datasheet | **verified by reading the documents**, not by running |
| Clock tree, SPI/DMA path, IMU configuration, USB CDC enumeration | **written but never run on hardware** |
| Wire format round-trip (firmware layout ↔ host parser) | **verified by execution** — the host test decodes frames byte-identical to the firmware's layout |

The USB CDC stack and the SPI/IMU path are the two places to expect trouble on
first bring-up, in that order.
