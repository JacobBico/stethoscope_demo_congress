/*
  UNO Q - Direct STM32U5 ADC1 registers at 56kHz, served via RPC Bridge.
  A0 = PA4 = ADC1 channel 9.

  loop() reads ADC_DR as fast as possible into a double buffer.
  When a buffer is full, it becomes available via RPC.
  Python calls Bridge.call("adc_get_frame") to fetch each frame.

  No Serial1, no timing pressure, no context switch issues.
  Latency is fine - frames arrive whenever Python asks for them.
*/

#include <Arduino.h>
#include <Arduino_RouterBridge.h>
#include <MsgPack.h>
#include <zephyr/kernel.h>

// 컴 ADC1 registers 컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴
#define ADC1_BASE    0x42028000UL
#define ADC1_ISR     (*(volatile uint32_t *)(ADC1_BASE + 0x00))
#define ADC1_CR      (*(volatile uint32_t *)(ADC1_BASE + 0x08))
#define ADC1_CFGR1   (*(volatile uint32_t *)(ADC1_BASE + 0x0C))
#define ADC1_SMPR1   (*(volatile uint32_t *)(ADC1_BASE + 0x14))
#define ADC1_PCSEL   (*(volatile uint32_t *)(ADC1_BASE + 0x1C))
#define ADC1_SQR1    (*(volatile uint32_t *)(ADC1_BASE + 0x30))
#define ADC1_DR      (*(volatile uint32_t *)(ADC1_BASE + 0x40))

#define RCC_AHB2ENR1 (*(volatile uint32_t *)(0x46020C00UL + 0x8C))
#define GPIOA_MODER  (*(volatile uint32_t *)(0x42020000UL))

#define ADC_CR_ADVREGEN  (1UL << 28)
#define ADC_CR_ADCAL     (1UL << 31)
#define ADC_CR_ADEN      (1UL << 0)
#define ADC_CR_ADSTART   (1UL << 2)
#define ADC_ISR_ADRDY    (1UL << 0)
#define ADC_ISR_EOC      (1UL << 2)
#define ADC_CFGR1_OVRMOD (1UL << 12)
#define ADC_CFGR1_CONT   (1UL << 13)

// 컴 Buffers 컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴�
// Smaller frame size than Serial1 version - RPC has overhead per call
// 4096 samples = ~73ms per frame at 56kHz, manageable RPC payload (~8KB)
static constexpr uint16_t FRAME_SAMPLES = 16384;

static uint16_t buf[2][FRAME_SAMPLES];
static volatile uint8_t  write_buf   = 0;
static volatile uint16_t write_idx   = 0;
static volatile uint8_t  ready_buf   = 0xFF;  // 0xFF = no frame ready
static volatile uint32_t frame_seq   = 0;

// 컴 ADC init 컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴
static void adc1_init() {
    RCC_AHB2ENR1 |= (1UL << 0) | (1UL << 10);
    __asm__ volatile("nop"); __asm__ volatile("nop");
    __asm__ volatile("nop"); __asm__ volatile("nop");

    GPIOA_MODER |= (3UL << 8);  // PA4 analog

    ADC1_CR = 0;
    for (volatile int i = 0; i < 10000; i++);
    ADC1_CR = ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 10000; i++);
    ADC1_CR |= ADC_CR_ADCAL;
    for (volatile int i = 0; i < 500000; i++);
    ADC1_ISR = ADC_ISR_ADRDY;
    ADC1_CR |= ADC_CR_ADEN;
    for (volatile int i = 0; i < 100000; i++);

    ADC1_PCSEL  = (1UL << 9);
    ADC1_SMPR1  = 0;
    ADC1_SQR1   = (9UL << 6);
    ADC1_CFGR1  = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;
    ADC1_CR    |= ADC_CR_ADSTART;
}

// 컴 RPC handler 컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴컴�
// Frame layout (matches parse_frame in Python):
// [uint8 count_lo][uint8 count_hi][seq(4)][samples(count*2)][crc16(2)]
MsgPack::bin_t<uint8_t> adc_get_frame() {
    MsgPack::bin_t<uint8_t> out;

    // No frame ready yet
    if (ready_buf == 0xFF) return out;

    // Snapshot which buffer to send
    uint8_t  bidx = ready_buf;
    uint32_t sno  = frame_seq;
    ready_buf = 0xFF;  // mark as consumed

    uint16_t count = FRAME_SAMPLES;

    // Build frame
    // Header: count(2) + seq(4) = 6 bytes
    // Payload: count * 2 bytes
    // CRC: 2 bytes
    size_t frame_size = 6 + count * 2 + 2;
    static uint8_t frame[6 + FRAME_SAMPLES * 2 + 2];

    size_t idx = 0;

    // count (2 bytes LE)
    frame[idx++] = count & 0xFF;
    frame[idx++] = (count >> 8) & 0xFF;

    // seq (4 bytes LE)
    frame[idx++] = sno & 0xFF;
    frame[idx++] = (sno >> 8) & 0xFF;
    frame[idx++] = (sno >> 16) & 0xFF;
    frame[idx++] = (sno >> 24) & 0xFF;

    // samples
    for (uint16_t i = 0; i < count; i++) {
        uint16_t v = buf[bidx][i] & 0x3FFF;  // 14-bit
        frame[idx++] = v & 0xFF;
        frame[idx++] = (v >> 8) & 0xFF;
    }

    // CRC16-IBM over entire frame so far
    uint16_t crc = 0;
    for (size_t i = 0; i < idx; i++) {
        crc ^= frame[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    frame[idx++] = crc & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;

    for (size_t i = 0; i < idx; i++)
        out.push_back(frame[i]);

    return out;
}

void setup() {
    Bridge.begin();
    Bridge.provide("adc_get_frame", adc_get_frame);
    adc1_init();
}

void loop() {
    // 1. Wait for ADC to finish a conversion
    while (!(ADC1_ISR & ADC_ISR_EOC)); 

    // 2. Write to the current buffer
    buf[write_buf][write_idx++] = (uint16_t)(ADC1_DR & 0x3FFF);

    // 3. When buffer is full, SWAP IMMEDIATELY
    if (write_idx >= FRAME_SAMPLES) {
        ready_buf = write_buf;    // Mark this buffer as "Fresh Meat" for Python
        frame_seq++;              // Increment sequence
        write_buf = 1 - write_buf; // Switch to the other buffer
        write_idx = 0;             // Reset index
    }
}

