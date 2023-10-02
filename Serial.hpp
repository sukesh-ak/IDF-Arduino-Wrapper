#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

class Serial {
public:
    Serial(uart_port_t uartNum) : uart_num(uartNum) {}

    void begin(unsigned long baudrate) {
        esp_err_t ret;

        // Configuration for the UART
        uart_config_t uart_config = {
            .baud_rate = baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0
        };

        ret = uart_param_config(uart_num, &uart_config);
        assert(ret == ESP_OK);

        // Set UART pins (TX, RX)
        if (uart_num == UART_NUM_0) {
            uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        } else if (uart_num == UART_NUM_1) {
            // Modify these pins for UART1 as needed
            uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        }

        ret = uart_driver_install(uart_num, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0);
        assert(ret == ESP_OK);
    }

    void print(const char* data) {
        uart_write_bytes(uart_num, data, strlen(data));
    }

    void println(const char* data) {
        uart_write_bytes(uart_num, data, strlen(data));
        uart_write_bytes(uart_num, "\r\n", 2);
    }

    int available() {
        size_t available_bytes;
        uart_get_buffered_data_len(uart_num, &available_bytes);
        return available_bytes;
    }

    char read() {
        uint8_t data;
        uart_read_bytes(uart_num, &data, 1, portMAX_DELAY);
        return data;
    }

private:
    uart_port_t uart_num;
    static const int RX_BUF_SIZE = 256;
    static const int TX_BUF_SIZE = 256;
};

Serial Serial(UART_NUM_0);  // Create a Serial instance for UART_NUM_0

/*
void setup() {
    Serial.begin(115200); // Set the desired baud rate
}

void loop() {
    // Your code using Serial functions
}
*/
