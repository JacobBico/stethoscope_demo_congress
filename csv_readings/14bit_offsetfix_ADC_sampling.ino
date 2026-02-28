/*
  UNO Q - Direct STM32U5 ADC1 registers, continuous mode, max speed.
  A0 = PA4 = ADC1 channel 9. Base = 0x42028000.
  Double buffered TX thread. Same frame format as before so Python is unchanged.
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
#define ADC_CR_ADSTART  (1UL << 2)
#define ADC_ISR_ADRDY   (1UL << 0)
#define ADC_ISR_EOC     (1UL << 2)
#define ADC_CFGR1_OVRMOD (1UL << 12)
#define ADC_CFGR1_CONT   (1UL << 13)

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
#define TX_STACK 2048
#define TX_PRIO  3
K_THREAD_STACK_DEFINE(tx_stack, TX_STACK);
static struct k_thread tx_td;

static void tx_thread(void *, void *, void *) {
    static uint8_t tx[8 + FRAME_SAMPLES * 2 + 2];
    const uint16_t MAGIC = 0xA55A;
    while (true) {
        k_sem_take(&tx_sem, K_FOREVER);
        uint8_t  bidx = tx_buf_idx;
        uint32_t sno  = tx_seq;
        tx[0]=MAGIC&0xFF; tx[1]=MAGIC>>8;
        tx[2]=sno&0xFF;   tx[3]=(sno>>8)&0xFF;
        tx[4]=(sno>>16)&0xFF; tx[5]=(sno>>24)&0xFF;
        tx[6]=FRAME_SAMPLES&0xFF; tx[7]=FRAME_SAMPLES>>8;
        memcpy(tx+8, buf[bidx], FRAME_SAMPLES*2);
        uint16_t crc = crc16_ccitt(tx, 8+FRAME_SAMPLES*2);
        tx[8+FRAME_SAMPLES*2]   = crc & 0xFF;
        tx[8+FRAME_SAMPLES*2+1] = crc >> 8;
        Serial1.write(tx, sizeof(tx));
    }
}

// ── ADC init ──────────────────────────────────────────────────
static void adc1_init() {
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

    // Configure ch9, continuous, overwrite on overrun, 5-cycle sample time <- ideal but not possible without sacarafice.
    ADC1_PCSEL  = (1UL << 9);
    ADC1_SMPR1  = (2UL << 0);   // Cycle count 2UL = 15, 3UL = 39.       
    ADC1_SQR1   = (9UL << 6);   // L=0 (1 conv), SQ1=9
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

    buf[write_buf][write_idx++] = (int16_t)(ADC1_DR & 0x3FFF);

    if (write_idx >= FRAME_SAMPLES) {
        write_idx  = 0;
        tx_buf_idx = write_buf;
        tx_seq     = seq++;
        write_buf  = 1 - write_buf;
        k_sem_give(&tx_sem);
    }
}
