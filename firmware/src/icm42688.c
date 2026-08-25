/* ---------------------------------------------------------------------------
 * icm42688.c -- ICM-42688-P register-level driver.
 *
 * Register addresses, bit fields and scale factors below come from the TDK
 * InvenSense ICM-42688-P datasheet (DS-000347).  Nothing here is guessed;
 * where a value is non-obvious the datasheet section is named in a comment.
 *
 * ---------------------------------------------------------------------------
 * BOARD-SPECIFIC CONSTRAINT
 * ---------------------------------------------------------------------------
 * On this PCB the IMU's pin 9 (INT2 / FSYNC / CLKIN) is HARD-TIED TO GND.
 * Two consequences that the code below enforces:
 *
 *   1. INT2 must never be configured as a push-pull output.  If it were, and
 *      any interrupt source were routed to it, the part would drive a
 *      push-pull high into a ground short.  We therefore leave the INT2 half
 *      of INT_CONFIG at its reset value (open-drain, active low) and never
 *      write INT_SOURCE1/INT_SOURCE3.
 *
 *   2. FSYNC cannot be used and, more importantly, must be explicitly
 *      disabled: TMST_CONFIG.TMST_FSYNC_EN comes out of reset SET, which
 *      makes the driver interpret the grounded pin as an FSYNC input and
 *      replaces the low byte of the gyro X data with an FSYNC timestamp
 *      (datasheet section 8.2, "FSYNC").  We clear it during init.
 *
 * INT1 on PB0 is therefore the only interrupt path, which is exactly what
 * this firmware uses.
 * ------------------------------------------------------------------------- */

#include "icm42688.h"
#include "board.h"
#include "debug_pins.h"
#include "spi1_dma.h"
#include "tim_us.h"
#include "util.h"

/* ---------------------------------------------------------------------------
 * Register map (bank 0 only -- this driver never leaves bank 0)
 * ------------------------------------------------------------------------- */
#define REG_DEVICE_CONFIG        0x11U
#define REG_INT_CONFIG           0x14U
#define REG_TEMP_DATA1           0x1DU  /* first byte of the burst window    */
#define REG_INT_STATUS           0x2DU
#define REG_INTF_CONFIG0         0x4CU
#define REG_INTF_CONFIG1         0x4DU
#define REG_PWR_MGMT0            0x4EU
#define REG_GYRO_CONFIG0         0x4FU
#define REG_ACCEL_CONFIG0        0x50U
#define REG_GYRO_ACCEL_CONFIG0   0x52U
#define REG_TMST_CONFIG          0x54U
#define REG_FIFO_CONFIG1         0x5FU
#define REG_FSYNC_CONFIG         0x62U
#define REG_INT_CONFIG1          0x64U
#define REG_INT_SOURCE0          0x65U
#define REG_WHO_AM_I             0x75U
#define REG_BANK_SEL             0x76U

/* DEVICE_CONFIG */
#define DEVICE_CONFIG_SOFT_RESET (1U << 0)

/* INT_CONFIG (0x14).  Bits 0-2 are INT1, bits 3-5 are INT2. */
#define INT_CONFIG_INT1_ACTIVE_HIGH  (1U << 0)
#define INT_CONFIG_INT1_PUSH_PULL    (1U << 1)
#define INT_CONFIG_INT1_LATCHED      (1U << 2)   /* left clear: pulsed mode  */

/* INT_CONFIG1 (0x64) */
#define INT_CONFIG1_TPULSE_DURATION  (1U << 6)
#define INT_CONFIG1_TDEASSERT_DIS    (1U << 5)
#define INT_CONFIG1_ASYNC_RESET      (1U << 4)

/* INT_SOURCE0 (0x65) */
#define INT_SOURCE0_UI_DRDY_INT1_EN  (1U << 3)

/* INT_STATUS (0x2D) */
#define INT_STATUS_DATA_RDY          (1U << 3)
#define INT_STATUS_RESET_DONE        (1U << 4)

/* PWR_MGMT0 (0x4E): ACCEL_MODE[1:0], GYRO_MODE[3:2]; 0b11 = low-noise. */
#define PWR_MGMT0_ACCEL_LN           (3U << 0)
#define PWR_MGMT0_GYRO_LN            (3U << 2)

/* GYRO_CONFIG0 (0x4F): GYRO_FS_SEL[7:5], GYRO_ODR[3:0] */
#define GYRO_FS_SEL_500DPS           (2U << 5)
#define GYRO_ODR_1KHZ                (0x6U << 0)

/* ACCEL_CONFIG0 (0x50): ACCEL_FS_SEL[7:5], ACCEL_ODR[3:0] */
#define ACCEL_FS_SEL_8G              (1U << 5)
#define ACCEL_ODR_1KHZ               (0x6U << 0)

/* GYRO_ACCEL_CONFIG0 (0x52): UI filter bandwidth, ODR/4 for both.  That is
 * the reset value; written explicitly so the setting is visible here. */
#define UI_FILT_BW_ODR_DIV4          0x11U

/* INTF_CONFIG0 (0x4C): UI_SIFS_CFG[1:0], 0b11 disables the I2C interface.
 * This board has no I2C on the IMU at all, so we shut that half of the pad
 * logic down rather than leaving it listening. */
#define INTF_CONFIG0_DISABLE_I2C     0x03U

/* TMST_CONFIG (0x54): bit 1 is TMST_FSYNC_EN, set at reset.  See the
 * board-specific note at the top of this file for why it must be cleared. */
#define TMST_CONFIG_FSYNC_EN         (1U << 1)

/* SPI transaction framing: bit 7 of the first byte is 1 for a read. */
#define SPI_READ_BIT                 0x80U

/* ---------------------------------------------------------------------------
 * Scale factors (datasheet section 3, "Gyroscope / Accelerometer
 * specifications", sensitivity scale factor tables)
 *
 *   gyro  +/- 500 dps  -> 65.5  LSB/dps
 *   accel +/-   8 g    -> 4096  LSB/g
 *   temperature        -> degC = (raw / 132.48) + 25
 * ------------------------------------------------------------------------- */
#define GYRO_SCALE_DPS_PER_LSB    (1.0f / 65.5f)
#define ACCEL_SCALE_G_PER_LSB     (1.0f / 4096.0f)
#define TEMP_SCALE_C_PER_LSB      (1.0f / 132.48f)
#define TEMP_OFFSET_C             25.0f

/* ---------------------------------------------------------------------------
 * Burst transfer buffers.
 *
 * One address byte followed by 14 data bytes:
 *   TEMP_DATA1/0, ACCEL_X/Y/Z (6), GYRO_X/Y/Z (6)
 * The registers are contiguous from 0x1D, so a single incrementing read gets
 * all of them in one CS assertion -- which is also what guarantees the seven
 * values belong to the same ODR tick.
 * ------------------------------------------------------------------------- */
#define BURST_DATA_LEN  14U
#define BURST_LEN       (1U + BURST_DATA_LEN)

static uint8_t s_burst_tx[BURST_LEN];
static uint8_t s_burst_rx[BURST_LEN];

/* ---------------------------------------------------------------------------
 * Lock-free single-producer / single-consumer sample queue.
 *
 * Producer: the SPI DMA completion ISR.  Consumer: the main loop.
 * Power-of-two size so the index wrap is a mask.  `head` is only ever written
 * by the producer and `tail` only by the consumer, so on a single-core
 * Cortex-M with 32-bit aligned volatile accesses no locking is needed.
 * ------------------------------------------------------------------------- */
#define SAMPLE_Q_LEN   16U   /* power of two */
#define SAMPLE_Q_MASK  (SAMPLE_Q_LEN - 1U)

static icm_sample_t     s_queue[SAMPLE_Q_LEN];
static volatile uint32_t s_q_head;
static volatile uint32_t s_q_tail;

static volatile uint32_t s_sample_count;
static volatile uint32_t s_overrun_count;
static volatile uint32_t s_last_drdy_us;
static volatile uint32_t s_pending_t_us;   /* timestamp of the in-flight burst */

/* ---------------------------------------------------------------------------
 * Blocking register access (configuration only)
 * ------------------------------------------------------------------------- */
static void icm_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
    uint8_t rx[2];

    spi1_xfer_blocking(tx, rx, 2U);

    /* The datasheet requires a short gap between consecutive register writes
     * while the device is coming out of reset; 10 us is comfortably more than
     * the specified minimum and costs nothing during init. */
    delay_us(10U);
}

static uint8_t icm_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | SPI_READ_BIT), 0x00U };
    uint8_t rx[2];

    spi1_xfer_blocking(tx, rx, 2U);
    return rx[1];
}

static void icm_modify_reg(uint8_t reg, uint8_t clear_mask, uint8_t set_mask)
{
    uint8_t v = icm_read_reg(reg);

    v = (uint8_t)((v & (uint8_t)~clear_mask) | set_mask);
    icm_write_reg(reg, v);
}

/* ---------------------------------------------------------------------------
 * EXTI0 wiring for INT1 on PB0
 * ------------------------------------------------------------------------- */
static void int1_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* PB0: input, no pull.  The ICM drives INT1 push-pull (we configure it
     * that way below), so an internal pull would only fight it. */
    IMU_INT_PORT->MODER &= ~(3UL << (IMU_INT_PIN * 2U));
    IMU_INT_PORT->PUPDR &= ~(3UL << (IMU_INT_PIN * 2U));

    /* Route EXTI line 0 to port B (EXTICR[0] bits 3:0, 0b0001 = port B). */
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | 0x0001U;

    EXTI->RTSR |=  EXTI_RTSR_TR0;   /* INT1 is active high -> rising edge    */
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    EXTI->PR    = EXTI_PR_PR0;      /* discard anything latched during setup */
    EXTI->IMR  &= ~EXTI_IMR_MR0;    /* stay masked until icm_irq_enable()    */

    NVIC_SetPriority(EXTI0_IRQn, 1U);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

void icm_irq_disable(void)
{
    EXTI->IMR &= ~EXTI_IMR_MR0;
    EXTI->PR   = EXTI_PR_PR0;
}

void icm_irq_enable(void)
{
    EXTI->PR  = EXTI_PR_PR0;
    EXTI->IMR |= EXTI_IMR_MR0;
}

/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */
icm_status_t icm_init(uint8_t *who_am_i_out)
{
    uint8_t who = 0U;

    icm_irq_disable();
    int1_gpio_init();

    /* Configuration registers get the slow clock: a marginal setup/hold on a
     * config write would silently leave the device in the wrong mode, which
     * is far more expensive to debug than the microseconds it saves. */
    spi1_speed_slow();

    /* Make sure we are talking to bank 0 -- if the firmware restarted mid-
     * transaction the device could still be pointed at another bank. */
    icm_write_reg(REG_BANK_SEL, 0x00U);

    /* ---- Soft reset -------------------------------------------------------
     * Datasheet section 14.x (DEVICE_CONFIG): after SOFT_RESET_CONFIG is set
     * the device needs 1 ms before any register is accessible again, and the
     * RESET_DONE interrupt must be cleared by reading INT_STATUS. */
    icm_write_reg(REG_DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET);
    delay_ms(5U);
    (void)icm_read_reg(REG_INT_STATUS);
    icm_write_reg(REG_BANK_SEL, 0x00U);

    /* ---- Identify the part ---------------------------------------------- */
    for (int attempt = 0; attempt < 5; attempt++) {
        who = icm_read_reg(REG_WHO_AM_I);
        if (who == ICM_WHO_AM_I_EXPECTED) {
            break;
        }
        delay_ms(2U);
    }
    if (who_am_i_out != 0) {
        *who_am_i_out = who;
    }
    if (who != ICM_WHO_AM_I_EXPECTED) {
        /* All-zero or all-ones usually means the bus itself is dead (no
         * power, CS stuck, MISO not connected) rather than a wrong part. */
        return (who == 0x00U || who == 0xFFU) ? ICM_ERR_NO_RESPONSE
                                              : ICM_ERR_WHO_AM_I;
    }

    /* ---- Interface: SPI only -------------------------------------------- */
    icm_modify_reg(REG_INTF_CONFIG0, INTF_CONFIG0_DISABLE_I2C,
                   INTF_CONFIG0_DISABLE_I2C);

    /* ---- FSYNC off (pin 9 is grounded on this board) --------------------- */
    icm_write_reg(REG_FSYNC_CONFIG, 0x00U);                 /* FSYNC_UI_SEL=0 */
    icm_modify_reg(REG_TMST_CONFIG, TMST_CONFIG_FSYNC_EN, 0x00U);

    /* ---- FIFO unused: we read the data registers directly ---------------- */
    icm_write_reg(REG_FIFO_CONFIG1, 0x00U);

    /* ---- Sensor ranges and rates ----------------------------------------
     * Written while both sensors are still OFF: several of these fields are
     * documented as "should not be changed while the sensor is running". */
    icm_write_reg(REG_GYRO_CONFIG0,  GYRO_FS_SEL_500DPS | GYRO_ODR_1KHZ);
    icm_write_reg(REG_ACCEL_CONFIG0, ACCEL_FS_SEL_8G    | ACCEL_ODR_1KHZ);
    icm_write_reg(REG_GYRO_ACCEL_CONFIG0, UI_FILT_BW_ODR_DIV4);

    /* ---- INT1 electrical configuration -----------------------------------
     * Push-pull, active high, pulsed.  The INT2 field (bits 5:3) is left at
     * zero => open drain / active low / pulsed, which is safe against the
     * grounded pin 9. */
    icm_write_reg(REG_INT_CONFIG,
                  INT_CONFIG_INT1_ACTIVE_HIGH | INT_CONFIG_INT1_PUSH_PULL);

    /* ---- INT_CONFIG1: the documented INT_ASYNC_RESET erratum --------------
     * INT_CONFIG1 bit 4 (INT_ASYNC_RESET) comes out of reset SET, and the
     * datasheet explicitly instructs: "User should change setting to 0 from
     * default setting of 1, for proper INT1 and INT2 pin operation."  Leaving
     * it at 1 produces missing or stuck interrupt pulses.
     *
     * Bits 6 (INT_TPULSE_DURATION) and 5 (INT_TDEASSERT_DISABLE) only need
     * changing for ODRs of 4 kHz and above; at our 1 kHz they stay 0, giving
     * the default 100 us pulse with de-assert duration enabled. */
    icm_write_reg(REG_INT_CONFIG1, 0x00U);

    /* ---- Route data-ready to INT1 ---------------------------------------- */
    icm_write_reg(REG_INT_SOURCE0, INT_SOURCE0_UI_DRDY_INT1_EN);

    /* ---- Power the sensors up in low-noise mode --------------------------
     * Datasheet section 12.9 (PWR_MGMT0): after this write, do not issue any
     * register write for 200 us.  The gyroscope additionally needs up to
     * ~30 ms of start-up time before its output is valid, and both sensors
     * take ~1 ms after a mode change; 60 ms is a comfortable margin. */
    icm_write_reg(REG_PWR_MGMT0, PWR_MGMT0_GYRO_LN | PWR_MGMT0_ACCEL_LN);
    delay_us(200U);
    delay_ms(60U);

    /* Clear any interrupt latched while the sensors were spinning up. */
    (void)icm_read_reg(REG_INT_STATUS);

    /* Burst reads run at 12 MHz -- well under the part's 24 MHz SPI ceiling. */
    spi1_speed_fast();

    /* Prime the transmit buffer once: byte 0 is the read command for
     * TEMP_DATA1, the remaining bytes are don't-care clock filler. */
    memset(s_burst_tx, 0x00, sizeof(s_burst_tx));
    s_burst_tx[0] = (uint8_t)(REG_TEMP_DATA1 | SPI_READ_BIT);

    s_q_head        = 0U;
    s_q_tail        = 0U;
    s_sample_count  = 0U;
    s_overrun_count = 0U;
    s_last_drdy_us  = micros();

    icm_irq_enable();
    return ICM_OK;
}

/* ---------------------------------------------------------------------------
 * Sample decoding (runs in the DMA completion ISR)
 * ------------------------------------------------------------------------- */
static inline int16_t be16(const uint8_t *p)
{
    /* The ICM-42688-P delivers big-endian words (high byte first) by default;
     * INTF_CONFIG0.SENSOR_DATA_ENDIAN is left at its reset value of 1. */
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void burst_complete(void)
{
    const uint8_t *d = &s_burst_rx[1];   /* skip the echoed address byte */
    uint32_t       head;
    uint32_t       next;

    head = s_q_head;
    next = (head + 1U) & SAMPLE_Q_MASK;

    if (next == s_q_tail) {
        /* Consumer has fallen behind.  Dropping the newest sample keeps the
         * already-queued history contiguous for the filter. */
        s_overrun_count++;
        return;
    }

    icm_sample_t *s = &s_queue[head];

    s->t_us   = s_pending_t_us;
    s->temp_c = ((float)be16(&d[0]) * TEMP_SCALE_C_PER_LSB) + TEMP_OFFSET_C;
    s->ax     =  (float)be16(&d[2])  * ACCEL_SCALE_G_PER_LSB;
    s->ay     =  (float)be16(&d[4])  * ACCEL_SCALE_G_PER_LSB;
    s->az     =  (float)be16(&d[6])  * ACCEL_SCALE_G_PER_LSB;
    s->gx     =  (float)be16(&d[8])  * GYRO_SCALE_DPS_PER_LSB;
    s->gy     =  (float)be16(&d[10]) * GYRO_SCALE_DPS_PER_LSB;
    s->gz     =  (float)be16(&d[12]) * GYRO_SCALE_DPS_PER_LSB;

    s_q_head = next;
    s_sample_count++;
}

bool icm_pop_sample(icm_sample_t *out)
{
    uint32_t tail = s_q_tail;

    if (tail == s_q_head) {
        return false;
    }
    *out = s_queue[tail];
    s_q_tail = (tail + 1U) & SAMPLE_Q_MASK;
    return true;
}

uint32_t icm_sample_count(void)   { return s_sample_count;  }
uint32_t icm_overrun_count(void)  { return s_overrun_count; }
uint32_t icm_last_drdy_us(void)   { return s_last_drdy_us;  }

/* ---------------------------------------------------------------------------
 * EXTI0: ICM-42688-P INT1 data-ready
 *
 * Kept deliberately tiny -- timestamp, mark the scope pin, start DMA.  All
 * decoding happens later in the DMA completion ISR.
 * ------------------------------------------------------------------------- */
void EXTI0_IRQHandler(void);
void EXTI0_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR0) == 0U) {
        return;
    }
    EXTI->PR = EXTI_PR_PR0;             /* write 1 to clear */

    DEBUG_A_TOGGLE();

    const uint32_t now = micros();
    s_last_drdy_us = now;

    /* If the previous burst has not finished we are not keeping up; skip this
     * sample rather than corrupting the in-flight buffer. */
    if (spi1_dma_busy()) {
        s_overrun_count++;
        return;
    }

    s_pending_t_us = now;
    (void)spi1_dma_xfer(s_burst_tx, s_burst_rx, BURST_LEN, burst_complete);
}
