#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

class SPIClass {
public:
    SPIClass(gpio_num_t mosiPin, gpio_num_t misoPin, gpio_num_t sckPin) : mosi_pin(mosiPin), miso_pin(misoPin), sck_pin(sckPin) {}

    void begin() {
        esp_err_t ret;

        // Configuration for the SPI bus
        spi_bus_config_t bus_config = {
            .miso_io_num = miso_pin,
            .mosi_io_num = mosi_pin,
            .sclk_io_num = sck_pin,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0
        };

        ret = spi_bus_initialize(HSPI_HOST, &bus_config, 1);
        assert(ret == ESP_OK);

        // Configuration for the SPI device
        spi_device_interface_config_t dev_config = {
            .clock_speed_hz = 1000000,  // Set your desired SPI clock frequency
            .mode = 0,  // SPI mode 0 (CPOL=0, CPHA=0)
            .spics_io_num = -1,  // Set to -1 if not using CS
            .queue_size = 7  // Number of transactions that can be queued at once
        };

        ret = spi_bus_add_device(HSPI_HOST, &dev_config, &spi_device);
        assert(ret == ESP_OK);
    }

    uint8_t transfer(uint8_t data) {
        spi_transaction_t transaction;
        memset(&transaction, 0, sizeof(transaction));
        transaction.length = 8;
        transaction.tx_buffer = &data;
        transaction.rxlength = 8;
        transaction.rx_buffer = &data;

        esp_err_t ret = spi_device_transmit(spi_device, &transaction);
        assert(ret == ESP_OK);

        return data;
    }

private:
    spi_device_handle_t spi_device;
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t sck_pin;
};

SPIClass SPI(GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14);  // Create an SPI instance with MOSI, MISO, and SCK pins

/*
void setup() {
    SPI.begin();
}

void loop() {
    uint8_t data_out = 0x55;
    uint8_t data_in = SPI.transfer(data_out);
    
    // Your code using SPI functions
}
*/