/* ---------------------------------------------------------------------------
 * board.h -- Mini-LiDAR-Board hardware description.
 *
 * Every pin assignment in this file was read out of the project's KiCad
 * netlist ("LiDAR Scanner.kicad_sch" / ".kicad_pcb"), not out of the project
 * write-up.  Where the two disagree, the KiCad files win.  See HARDWARE.md for
 * the list of places where the older project guide is wrong.
 *
 * MCU : STM32F411CEU6, UFQFPN-48, 512 KiB flash / 128 KiB SRAM
 * XTAL: 8 MHz HSE on PH0/PH1 (Y1, 22 pF load caps).  There is NO 32.768 kHz
 *       LSE crystal on this board.
 * ------------------------------------------------------------------------- */

#ifndef BOARD_H
#define BOARD_H

#include "stm32f411xe.h"

/* ---------------------------------------------------------------------------
 * Clock tree (see clock.c for the derivation and the RM0383 constraints)
 * ------------------------------------------------------------------------- */
#define BOARD_HSE_HZ      8000000UL   /* Y1                                    */
#define BOARD_SYSCLK_HZ   96000000UL  /* PLL P output                          */
#define BOARD_AHB_HZ      96000000UL  /* AHB prescaler /1                      */
#define BOARD_APB1_HZ     48000000UL  /* APB1 /2  -- must stay <= 50 MHz       */
#define BOARD_APB2_HZ     96000000UL  /* APB2 /1  -- limit is 100 MHz          */
#define BOARD_APB1_TIM_HZ 96000000UL  /* APB1 timers run at 2x PCLK1 (/2 pre)  */
#define BOARD_USB48_HZ    48000000UL  /* PLL Q output, exact 48 MHz for OTG_FS */

/* ---------------------------------------------------------------------------
 * IMU: U2, InvenSense/TDK ICM-42688-P, LGA-14, SPI only.
 *
 *   VDD and VDDIO are both on the 3.3 V rail.
 *   Pin 9 (INT2/FSYNC/CLKIN) is HARD-TIED TO GND on this board, so INT2 and
 *   FSYNC are unavailable -- INT1 on PB0 is the only interrupt path.  The
 *   driver must therefore never configure INT2 as a push-pull output.
 * ------------------------------------------------------------------------- */
#define IMU_CS_PORT     GPIOA
#define IMU_CS_PIN      4U            /* PA4, software-driven chip select      */
#define IMU_SCK_PIN     5U            /* PA5, SPI1_SCK,  AF5                   */
#define IMU_MISO_PIN    6U            /* PA6, SPI1_MISO, AF5                   */
#define IMU_MOSI_PIN    7U            /* PA7, SPI1_MOSI, AF5                   */
#define IMU_INT_PORT    GPIOB
#define IMU_INT_PIN     0U            /* PB0, ICM-42688-P INT1 -> EXTI0        */

/* ---------------------------------------------------------------------------
 * USB: USB-C connector on the OTG_FS full-speed PHY.
 *
 *   PA9 (the OTG_FS VBUS sense pin) is wired to the LiDAR UART instead, so
 *   VBUS sensing MUST be disabled in GCCFG (NOVBUSSENS = 1).  See usb_cdc.c.
 * ------------------------------------------------------------------------- */
#define USB_DM_PIN      11U           /* PA11, USB_OTG_FS_DM, AF10             */
#define USB_DP_PIN      12U           /* PA12, USB_OTG_FS_DP, AF10             */

/* ---------------------------------------------------------------------------
 * Debug UART: header J3 (1 = TX, 2 = RX, 3 = GND), USART2, 921600 8N1.
 *
 * NOT IMPLEMENTED IN THIS MILESTONE.  These defines record the wiring only --
 * no USART2 driver exists yet, and the firmware leaves PA2/PA3 in their reset
 * state.  All telemetry and status reporting goes over USB CDC instead.  Kept
 * here so a printf-style side channel can be added without re-reading the
 * netlist.
 * ------------------------------------------------------------------------- */
#define DBGUART_TX_PIN  2U            /* PA2, USART2_TX, AF7                   */
#define DBGUART_RX_PIN  3U            /* PA3, USART2_RX, AF7                   */
#define DBGUART_BAUD    921600UL

/* ---------------------------------------------------------------------------
 * Scope/logic-analyser header J4.  These are the ONLY visual indicators this
 * board has -- there are NO GPIO status LEDs (D3 is a bare power LED wired
 * across 3V3 through R4, with no MCU connection at all).
 * ------------------------------------------------------------------------- */
#define DEBUG_PORT      GPIOB
#define DEBUG_A_PIN     3U            /* PB3 -- toggled on each IMU sample     */
#define DEBUG_B_PIN     4U            /* PB4 -- toggled on each Madgwick step  */
#define DEBUG_C_PIN     5U            /* PB5 -- toggled on each packet TX      */

/* ---------------------------------------------------------------------------
 * SWD programming header J5: 1 = 3V3, 2 = GND, 3 = SWDIO (PA13), 4 = SWCLK
 * (PA14).  Left in their reset state; the firmware never touches PA13/PA14.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * LiDAR: Slamtec RPLIDAR A1 on connector U1 (Molex 22035075).
 *
 * U1 pinout, from the netlist:
 *   1 VMOTO  -- 5 V through ferrite bead FB2, with 47 uF bulk (C18).
 *               The motor supply is PERMANENTLY ON; there is no enable GPIO
 *               and no solder jumper, so rotation speed is set ONLY by the
 *               PWM pin below.  Driving PWM to 0 % is the only way to stop
 *               the motor.
 *   2 PWM    -- PB10, TIM2_CH3, AF1.  Motor speed control.
 *   3 MGND   -- motor ground
 *   4 VCC_5  -- 5 V logic supply (through polyfuse F1)
 *   5 RX     -- the LiDAR's receive pin, driven by the MCU's USART1_TX (PA9)
 *   6 TX     -- the LiDAR's transmit pin, read by the MCU's USART1_RX (PA10)
 *   7 DGND   -- digital ground
 *
 * NOTE ON THE NET NAMES: the schematic nets are called UART1_RX and UART1_TX
 * from the LiDAR's point of view, so "UART1_RX" lands on the MCU's PA9, which
 * is USART1_TX.  The wiring is correct; only the labels are confusing.
 *
 * The link runs at 115200 baud.  At 5 bytes per sample that is a hard ceiling
 * of ~2300 samples/s, which at the A1's default ~5.5 rev/s gives ~360-400
 * points per revolution.
 * ------------------------------------------------------------------------- */
#define LIDAR_UART_TX_PIN   9U        /* PA9,  USART1_TX -> LiDAR RX, AF7     */
#define LIDAR_UART_RX_PIN   10U       /* PA10, USART1_RX <- LiDAR TX, AF7     */
#define LIDAR_UART_BAUD     115200UL

#define LIDAR_PWM_PORT      GPIOB
#define LIDAR_PWM_PIN       10U       /* PB10, TIM2_CH3, AF1                  */

/* ---------------------------------------------------------------------------
 * DMA assignments (RM0383 Table 28).  Written down in one place because the
 * two subsystems constrain each other:
 *
 *   SPI1_RX   : DMA2 Stream 0 ch 3   <- taken (spi1_dma.c)
 *   SPI1_TX   : DMA2 Stream 3 ch 3   <- taken (spi1_dma.c)
 *   USART1_RX : DMA2 Stream 5 ch 4   <- taken (usart1_dma.c)
 *
 * USART1_RX can only live on Stream 2 or Stream 5, and SPI1_RX can only live
 * on Stream 0 or Stream 2.  Putting SPI1 on Stream 0 kept both USART1 options
 * open; we take Stream 5 and leave Stream 2 spare.
 * ------------------------------------------------------------------------- */

/* Convenience: 96-bit factory unique device ID, used to build the USB serial
 * number string so several boards can be told apart on one host. */
#define BOARD_UID_BASE  0x1FFF7A10UL

#endif /* BOARD_H */
