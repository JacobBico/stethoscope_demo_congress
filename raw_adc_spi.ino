#include <Arduino_RouterBridge.h>
#include <SPI.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>

#define SPI3_NODE DT_NODELABEL(spi3)

uint16_t adc_data[4] = {0, 0, 0, 0xABCD}; 
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
}

void loop() {
    // Read the pins.
    adc_data[0] = analogRead(A0);


    struct spi_buf tx_buf = {.buf = adc_data, .len = 8};
    struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    struct spi_buf rx_buf = {.buf = rx_dummy, .len = 8};
    struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    // Send data through SPI.
    spi_transceive(DEVICE_DT_GET(SPI3_NODE), &config, &tx_set, &rx_set);
}
