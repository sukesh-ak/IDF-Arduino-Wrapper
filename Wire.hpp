#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

class Wire {
public:
    Wire(uint8_t sdaPin, uint8_t sclPin) : sda_pin(sdaPin), scl_pin(sclPin) {}

    void begin() {
        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = sda_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = scl_pin,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000  // Adjust this to your desired I2C frequency
        };

        i2c_param_config(I2C_NUM_0, &i2c_config);
        i2c_driver_install(I2C_NUM_0, i2c_config.mode, 0, 0, 0);
    }

    void beginTransmission(uint8_t address) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    }

    size_t write(uint8_t data) {
        i2c_master_write_byte(cmd, data, true);
        return 1;
    }

    size_t write(const uint8_t* data, size_t length) {
        i2c_master_write(cmd, data, length, true);
        return length;
    }

    uint8_t endTransmission() {
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);
        return (ret == ESP_OK) ? 0 : 4;
    }

    uint8_t requestFrom(uint8_t address, size_t quantity) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, buffer, quantity, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            return quantity;
        } else {
            return 0;
        }
    }

    int available() {
        return bytesAvailable;
    }

    int read() {
        if (bytesAvailable > 0) {
            bytesAvailable--;
            return buffer[readIndex++];
        }
        return -1;
    }

private:
    i2c_cmd_handle_t cmd;
    uint8_t buffer[32];  // Adjust the buffer size as needed
    size_t readIndex = 0;
    int bytesAvailable = 0;
    uint8_t sda_pin;
    uint8_t scl_pin;
};

Wire Wire(4, 5);  // Create a Wire instance with SDA_PIN=4 and SCL_PIN=5

/*
void setup() {
    Wire.begin();
}

void loop() {
    // Your code using Wire functions
}
*/