/*
  UNO Q - Direct STM32U5 ADC1 registers, continuous mode, max speed.
  A0 = PA4 = ADC1 channel 9. Base = 0x42028000.
  Double buffered TX thread.

  v4 — Fix phase-discontinuity at frame boundaries.
  Strategy: Stop ADC before k_sem_give(), restart immediately after.
  ADC stop on STM32U5 takes ~2 conversion cycles (~0.5µs at 4MHz ADC clock).
  We spin-wait with a hard timeout to avoid deadlock.

  Additionally: lock the scheduler briefly around the buffer swap + sem_give
  so the TX thread doesn't preempt us between the swap and the ADC restart.
  
  Alternative approach if stop/start still loses 1-2 samples:
  Record a timestamp/cycle-count at stop and restart, embed in frame header
  so Python can interpolate the gap.
*/
#include <Arduino.h>
#include <zephyr/kernel.h>

// ── ADC1 registers ────────────────────────────────────────────
#define ADC1_BASE    0x42028000UL
#define ADC1_ISR     (*(volatile uint32_t *)(ADC1_BASE + 0x00))
#define ADC1_CR      (*(volatile uint32_t *)(ADC1_BASE + 0x08))
#define ADC1_CFGR1   (*(volatile uint32_t *)(ADC1_BASE + 0x0C))
#define ADC1_SMPR1   (*(volatile uint32_t *)(ADC1_BASE + 0x14))
#define ADC1_PCSEL   (*(volatile uint32_t *)(ADC1_BASE + 0x1C))
#define ADC1_SQR1    (*(volatile uint32_t *)(ADC1_BASE + 0x30))
#define ADC1_DR      (*(volatile uint32_t *)(ADC1_BASE + 0x40))

#define RCC_AHB2ENR1 (*(volatile uint32_t *)(0x46020C00UL + 0x8C))
#define GPIOA_MODER  (*(volatile uint32_t *)(0x42020000UL + 0x00))

#define ADC_CR_DEEPPWD  (1UL << 29)
#define ADC_CR_ADVREGEN (1UL << 28)
#define ADC_CR_ADCAL    (1UL << 31)
#define ADC_CR_ADEN     (1UL << 0)
#define ADC_CR_ADDIS    (1UL << 1)
#define ADC_CR_ADSTART  (1UL << 2)
#define ADC_CR_ADSTP    (1UL << 4)
#define ADC_ISR_ADRDY   (1UL << 0)
#define ADC_ISR_EOC     (1UL << 2)
#define ADC_ISR_EOS     (1UL << 3)
#define ADC_ISR_OVR     (1UL << 4)
#define ADC_CFGR1_OVRMOD (1UL << 12)
#define ADC_CFGR1_CONT   (1UL << 13)

// ── DWT cycle counter for precise timing ──────────────────────
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004UL)
#define SCB_DEMCR  (*(volatile uint32_t *)0xE000EDFCUL)

// ── Double buffer ─────────────────────────────────────────────
static constexpr uint32_t BAUD          = 2000000;
static constexpr uint16_t FRAME_SAMPLES = 16384;

static int16_t buf[2][FRAME_SAMPLES];
static uint8_t  write_buf = 0;
static uint16_t write_idx = 0;
static uint32_t seq       = 0;

K_SEM_DEFINE(tx_sem, 0, 1);
static volatile uint8_t  tx_buf_idx = 0;
static volatile uint32_t tx_seq     = 0;
static volatile uint32_t tx_gap_cycles = 0;  // DWT cycles ADC was stopped

// ── CRC ───────────────────────────────────────────────────────
static uint16_t crc16_ccitt(const uint8_t *d, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// ── TX thread ─────────────────────────────────────────────────
//    Frame: [MAGIC 2][seq 4][nsamp 2][gap_cycles 4][samples nsamp*2][crc 2]
//    gap_cycles = DWT cycles the ADC was stopped (0 = no gap)
//    Python can use this to compute lost time: gap_cycles / CPU_FREQ
#define TX_STACK 2048
#define TX_PRIO  3
K_THREAD_STACK_DEFINE(tx_stack, TX_STACK);
static struct k_thread tx_td;

// Frame overhead: magic(2) + seq(4) + nsamp(2) + gap_cycles(4) + crc(2) = 14
#define FRAME_HDR 12   // everything before samples
#define FRAME_FTR  2   // CRC after samples

static void tx_thread(void *, void *, void *) {
    static uint8_t tx[FRAME_HDR + FRAME_SAMPLES * 2 + FRAME_FTR];
    const uint16_t MAGIC = 0xA55A;
    while (true) {
        k_sem_take(&tx_sem, K_FOREVER);
        uint8_t  bidx = tx_buf_idx;
        uint32_t sno  = tx_seq;
        uint32_t gap  = tx_gap_cycles;

        // Header: magic(2) + seq(4) + nsamp(2) + gap_cycles(4)
        tx[0]  = MAGIC & 0xFF;       tx[1]  = MAGIC >> 8;
        tx[2]  = sno & 0xFF;         tx[3]  = (sno >> 8) & 0xFF;
        tx[4]  = (sno >> 16) & 0xFF; tx[5]  = (sno >> 24) & 0xFF;
        tx[6]  = FRAME_SAMPLES & 0xFF; tx[7] = FRAME_SAMPLES >> 8;
        tx[8]  = gap & 0xFF;         tx[9]  = (gap >> 8) & 0xFF;
        tx[10] = (gap >> 16) & 0xFF; tx[11] = (gap >> 24) & 0xFF;

        memcpy(tx + FRAME_HDR, buf[bidx], FRAME_SAMPLES * 2);

        uint16_t crc = crc16_ccitt(tx, FRAME_HDR + FRAME_SAMPLES * 2);
        tx[FRAME_HDR + FRAME_SAMPLES * 2]     = crc & 0xFF;
        tx[FRAME_HDR + FRAME_SAMPLES * 2 + 1] = crc >> 8;

        Serial1.write(tx, sizeof(tx));
    }
}

// ── ADC helpers ───────────────────────────────────────────────
static inline void adc_stop_continuous(void) {
    // Request stop of ongoing conversions
    ADC1_CR |= ADC_CR_ADSTP;
    // Spin-wait for ADSTP to clear (hardware clears it when stopped)
    // STM32U5 datasheet: max 2 ADC cycles + 1 AHB cycle.
    // At 4 MHz ADC clock and 160 MHz CPU, that's ~80 CPU cycles.
    // Hard timeout of 1000 cycles (~6µs) to be safe.
    uint32_t t0 = DWT_CYCCNT;
    while ((ADC1_CR & ADC_CR_ADSTP) && ((DWT_CYCCNT - t0) < 1000)) {
        // spin
    }
    // Clear any pending flags
    ADC1_ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
}

static inline void adc_start_continuous(void) {
    ADC1_CR |= ADC_CR_ADSTART;
}

// ── ADC init ──────────────────────────────────────────────────
static void adc1_init() {
    // Enable DWT cycle counter
    SCB_DEMCR |= (1UL << 24);    // TRCENA
    DWT_CTRL  |= 1;              // CYCCNTENA

    // Enable clocks
    RCC_AHB2ENR1 |= (1UL << 0) | (1UL << 10);
    __asm__ volatile("nop"); __asm__ volatile("nop");
    __asm__ volatile("nop"); __asm__ volatile("nop");

    // PA4 → analog
    GPIOA_MODER |= (3UL << 8);

    // Exit deep power down
    ADC1_CR = 0;
    for (volatile int i = 0; i < 10000; i++);

    // Enable voltage regulator
    ADC1_CR = ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);

    // Calibrate
    ADC1_CR |= ADC_CR_ADCAL;
    for (volatile int i = 0; i < 500000; i++);

    // Enable ADC
    ADC1_ISR = ADC_ISR_ADRDY;
    ADC1_CR |= ADC_CR_ADEN;
    for (volatile int i = 0; i < 100000; i++);

    // Configure ch9, continuous, overwrite on overrun, 5-cycle sample time
    ADC1_PCSEL  = (1UL << 9);
    ADC1_SMPR1  = 0;
    ADC1_SQR1   = (9UL << 6);
    ADC1_CFGR1  = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

    // Start continuous conversions
    ADC1_CR |= ADC_CR_ADSTART;
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial1.begin(BAUD);
    delay(500);

    adc1_init();

    k_thread_create(&tx_td, tx_stack, TX_STACK,
                    tx_thread, NULL, NULL, NULL,
                    TX_PRIO, 0, K_NO_WAIT);
}

// ── Loop — sample as fast as possible ─────────────────────────
void loop() {
    if (!(ADC1_ISR & ADC_ISR_EOC)) return;

    buf[write_buf][write_idx++] = (int16_t)(ADC1_DR & 0xFFF);

    if (write_idx >= FRAME_SAMPLES) {
        write_idx = 0;

        // ── CRITICAL SECTION: stop ADC → swap → signal TX → restart ADC ──
        // Stop the ADC so no samples are lost during context switch
        adc_stop_continuous();
        uint32_t cyc_stop = DWT_CYCCNT;

        // Lock scheduler so k_sem_give doesn't immediately preempt us
        k_sched_lock();

        // Swap buffers and signal TX
        tx_buf_idx    = write_buf;
        tx_seq        = seq++;
        write_buf     = 1 - write_buf;
        tx_gap_cycles = 0;  // will be filled in after restart
        k_sem_give(&tx_sem);

        // Restart ADC *before* unlocking scheduler
        adc_start_continuous();
        uint32_t cyc_start = DWT_CYCCNT;

        // Record how long the ADC was dark
        tx_gap_cycles = cyc_start - cyc_stop;

        k_sched_unlock();
        // TX thread now runs, but ADC is already converting again
        // ── END CRITICAL SECTION ──
    }
}
