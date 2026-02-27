#include <Arduino_RouterBridge.h>
#include <SPI.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>

#define SPI3_NODE DT_NODELABEL(spi3)

uint16_t adc_data[4] = {0, 0, 0, 0xABCD}; // Last one is our "Safety" magic number
uint16_t rx_dummy[4];

struct spi_config config = {
    .frequency = 1000000,
    .operation = SPI_OP_MODE_SLAVE | SPI_MODE_CPHA 
               | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE,
    .slave = 0,
};

void setup() {
    Bridge.begin();
    analogReadResolution(12);
    // No complicated Zephyr ADC init here—let's keep it simple
}

void loop() {
    // Read the most likely internal pins for A0-A2
    adc_data[0] = analogRead(A0);
    adc_data[1] = analogRead(A1);
    adc_data[2] = analogRead(A2);

    struct spi_buf tx_buf = {.buf = adc_data, .len = 8};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf rx_buf = {.buf = rx_dummy, .len = 8};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    // This is where Python and STM32 shake hands
    spi_transceive(DEVICE_DT_GET(SPI3_NODE), &config, &tx_set, &rx_set);
}
