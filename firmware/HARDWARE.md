# Verified hardware notes — Mini-LiDAR-Board

Everything in this file was read out of the project's KiCad schematic and PCB
(`LiDAR Scanner.kicad_sch`, `LiDAR Scanner.kicad_pcb`) rather than out of
`Lidar Project Guide.md`. **Where the two disagree, this file is right and the
guide is out of date.** The list of disagreements is in the last section.

---

## MCU

| item | value |
|---|---|
| Part | STM32F411CEU6 |
| Package | **UFQFPN-48** (4 mm × 4 mm QFN) |
| Flash / SRAM | 512 KiB / 128 KiB |
| Core | Cortex-M4F, single-precision FPU |
| HSE | 8 MHz crystal Y1 on PH0/PH1, 22 pF load caps |
| LSE | **none fitted** — no 32.768 kHz crystal on this board |
| VCAP1 | pin 22 |

Clock configuration used by this firmware (see `src/clock.c`):

```
HSE 8 MHz -> PLL(M=4, N=192, P=4, Q=8)
  VCO in   2 MHz      (RM0383: must be 1-2 MHz)
  VCO out  384 MHz    (RM0383: must be 100-432 MHz)
  SYSCLK    96 MHz    (F411 limit 100 MHz)
  PLLQ      48 MHz    exact, for USB OTG_FS
AHB /1  ->  HCLK  96 MHz
APB1 /2 ->  PCLK1 48 MHz   (F411 limit 50 MHz), APB1 timers 96 MHz
APB2 /1 ->  PCLK2 96 MHz   (F411 limit 100 MHz)
Flash: 3 wait states, prefetch + I-cache + D-cache on
Power: regulator voltage Scale 1 (required above 84 MHz)
```

---

## Pin map (authoritative)

| Function | Pin | Notes |
|---|---|---|
| IMU SPI1_CS | PA4 | software-controlled GPIO chip select, not hardware NSS |
| IMU SPI1_SCK | PA5 | AF5 |
| IMU SPI1_MISO | PA6 | AF5 |
| IMU SPI1_MOSI | PA7 | AF5 |
| IMU INT1 | **PB0** | EXTI0, data-ready interrupt |
| USB D− | PA11 | USB_OTG_FS, AF10 |
| USB D+ | PA12 | USB_OTG_FS, AF10 |
| SWDIO / SWCLK | PA13 / PA14 | header J5 (1 = 3V3, 2 = GND, 3 = SWDIO, 4 = SWCLK) |
| Debug UART2 TX / RX | PA2 / PA3 | header J3 (1 = TX, 2 = RX, 3 = GND), AF7 |
| DEBUG_A / B / C | PB3 / PB4 / PB5 | header J4, scope/logic-analyser timing pins |
| LiDAR UART1 TX / RX | PA9 / PA10 | **out of scope this milestone — left unconfigured** |
| LiDAR motor PWM | PB10 | TIM2_CH3 — **out of scope, left unconfigured (input, no drive)** |

### PB3 / PB4 are JTAG pins at reset

They come out of reset as JTDO-TRACESWO and NJTRST. Driving them as the J4
timing outputs disables the JTAG-DP. That is harmless here because J5 exposes
**SWD only**, and SWD (PA13/PA14) keeps working. The cost is that SWO trace is
unavailable.

---

## IMU — U2, TDK InvenSense ICM-42688-P, LGA-14

| item | value |
|---|---|
| VDD / VDDIO | both 3.3 V |
| Bus | **SPI only** — there is no I²C to the IMU on this board |
| WHO_AM_I (reg 0x75) | must read `0x47` |
| INT1 | pin 14 → PB0 |
| INT2 / FSYNC / CLKIN (pin 9) | **hard-tied to GND** |
| RESV pin 7 | tied to GND |
| RESV pins 2, 3, 10, 11 | unconnected |

### Consequences of pin 9 being grounded

1. **INT2 and FSYNC are unavailable.** INT1 on PB0 is the only interrupt path.
2. INT2 must never be configured as a push-pull output — it would drive into a
   ground short. `src/icm42688.c` leaves the INT2 half of `INT_CONFIG` (bits
   5:3) at its reset value: open drain, active low.
3. `TMST_CONFIG.TMST_FSYNC_EN` comes out of reset **set**, which makes the part
   interpret the grounded pin as an FSYNC input and overwrite the low byte of
   gyro X with an FSYNC timestamp. The firmware explicitly clears it during
   init. This is easy to miss and produces a subtly wrong gyro X axis.

### Other datasheet details the driver depends on

* `INT_CONFIG1` bit 4 `INT_ASYNC_RESET` comes out of reset **set**, and the
  datasheet instructs the user to clear it "for proper INT1 and INT2 pin
  operation". Left at 1, interrupt pulses go missing. The firmware writes
  `INT_CONFIG1 = 0x00`.
* After writing `PWR_MGMT0`, no register write may be issued for 200 µs; the
  gyro additionally needs up to ~30 ms to start.
* Sensitivity at the configured ranges: gyro ±500 dps → 65.5 LSB/dps;
  accel ±8 g → 4096 LSB/g; temperature → `°C = raw/132.48 + 25`.
* Maximum SPI clock is 24 MHz. The firmware uses ~1.5 MHz for configuration
  writes and 12 MHz for the data burst.

---

## THERE ARE NO GPIO STATUS LEDs

This is the single most important thing to know before writing firmware for
this board.

* **D3 is a green power LED wired directly across the 3.3 V rail through R4.**
  It has no MCU connection whatsoever. It is on whenever the board has power.
* There is no IMU LED, no LIDAR LED and no STATUS LED, despite what the project
  guide describes. They were never built.

Every piece of firmware state must therefore be reported either:

1. over the USB data link, as a `STATUS` packet (see `PROTOCOL.md`), or
2. as an edge on DEBUG_A/B/C (PB3/PB4/PB5, header J4) for a scope or logic
   analyser.

There is no blink code anywhere in this project, and there cannot be. Any
future contributor who "adds an LED heartbeat" is writing to a pin that does
not exist.

---

## DMA allocation and the reservation for LiDAR

On the STM32F411 (RM0383 Table 28, DMA2 request mapping):

```
SPI1_RX   : DMA2 Stream 0 ch 3   or   DMA2 Stream 2 ch 3
SPI1_TX   : DMA2 Stream 3 ch 3   or   DMA2 Stream 5 ch 3
USART1_RX : DMA2 Stream 2 ch 4   or   DMA2 Stream 5 ch 4
```

USART1_RX — which the LiDAR will need, because an RPLIDAR pushes a continuous
scan stream that cannot be serviced byte-by-byte alongside a 1 kHz IMU — can
only live on Stream 2 or Stream 5. This firmware therefore takes **Stream 0 for
SPI1_RX and Stream 3 for SPI1_TX**, leaving both USART1_RX options free.
Choosing Stream 2 or Stream 5 for SPI would work fine today and force a rework
later.

---

## Deviations from `Lidar Project Guide.md`

| The guide says | Reality |
|---|---|
| MCU is in an **LQFP48** package | It is **UFQFPN-48** |
| IMU INT2 / FSYNC usable | Pin 9 is **hard-tied to GND**; INT2 and FSYNC do not exist on this board |
| IMU, LIDAR and STATUS **LEDs** on GPIOs | **No GPIO LEDs exist.** D3 is a power-rail-only indicator with no MCU connection |
| — | The guide does not mention that `TMST_CONFIG.TMST_FSYNC_EN` must be cleared because of the grounded FSYNC pin |

Do not "fix" the firmware to match the guide.

---

## LiDAR wiring (added in the LiDAR milestone)

Connector **U1** (Molex 22035075) carries the RPLIDAR A1:

| U1 pin | net | goes to | note |
|---|---|---|---|
| 1 | `Net-(U1-VMOTO)` | 5 V via ferrite bead **FB2**, 47 µF bulk (C18) | motor supply |
| 2 | `PWM` | **PB10**, TIM2_CH3, AF1 | spindle speed |
| 3 | `MGND` | GND | motor ground |
| 4 | `VCC` | 5 V via polyfuse F1 | logic supply |
| 5 | `UART1_RX` | **PA9** = USART1_**TX** | MCU → LiDAR |
| 6 | `UART1_TX` | **PA10** = USART1_**RX** | LiDAR → MCU |
| 7 | `DGND` | GND | |

Three things here differ from the project guide and matter for firmware:

1. **There is no motor enable GPIO and no LiDAR enable jumper.** VMOTO is
   hard-wired to 5 V through FB2, so the spindle is powered whenever the board
   is. PWM duty on PB10 is the *only* control over the motor; 0 % is the only
   way to stop it. The guide describes a solder jumper on a "motor enable GPIO
   line" — it was never built.

2. **The UART net names are from the LiDAR's point of view.** The net called
   `UART1_RX` lands on PA9, which is USART1_**TX**. The wiring is correct; only
   the labels invite a mis-read.

3. **PA9 is also the OTG_FS VBUS sense pin**, and it is used for the LiDAR
   UART here. `GCCFG.NOVBUSSENS` must stay set or USB will not enumerate. This
   was already true before the LiDAR existed, but it is now load-bearing for
   two subsystems rather than one.

### Link budget

The A1 runs at 115200 baud and standard `SCAN` mode spends 5 bytes per
measurement, so the wire caps out at ~2300 samples/s no matter what the sensor
can do internally. Measured on this board: **1954 samples/s**. Spindle speed
therefore trades rings/s against points/ring:

| PWM duty | measured spin | points/ring |
|---|---|---|
| 350 | ~2 rev/s | ~920 |
| 600 | 3.5 rev/s | ~550 |
| 820 (default) | 5.6 rev/s | ~350 |
| 900 | 6.2 rev/s | ~313 |

The default was originally 600. That runs the spindle well below its rated
5.5 rev/s, and during a long session it stalled outright — hence 820.

### Two different LiDAR stalls

Worth knowing because they need different watchdogs, and only one of them is
obvious:

1. **The device goes quiet.** No measurement nodes arrive. Caught by watching
   `rplidar_last_node_us()`.
2. **The spindle stops but the device keeps talking.** The A1 goes on streaming
   measurement nodes at a frozen angle, so the node watchdog stays happy
   forever while no revolution ever closes. This is the one that actually bit
   us. Caught separately by watching `rplidar_last_ring_us()`.

Recovery for (2) kicks the motor to full duty for 400 ms before restoring the
commanded duty — static friction takes more torque to break than rotation takes
to sustain — then re-arms the UART DMA and restarts the scan.

Below about 2 rev/s a revolution exceeds the firmware's 1024-point cap and the
ring is truncated (flagged, not silent).

### DMA map

| request | stream | why |
|---|---|---|
| SPI1_RX | DMA2 Stream 0 ch 3 | leaves both USART1_RX options free |
| SPI1_TX | DMA2 Stream 3 ch 3 | |
| USART1_RX | DMA2 Stream 5 ch 4 | circular, never serviced on a deadline |

USART1_RX can only live on Stream 2 or Stream 5, and SPI1_RX only on Stream 0
or Stream 2. Putting SPI1 on Stream 0 during the IMU milestone is what made
this conflict-free; Stream 2 is still spare.
