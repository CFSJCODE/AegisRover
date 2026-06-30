// hal_esp32.c
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

static uart_port_t uart_num = UART_NUM_1;

HalSerial* hal_serial_open(const char* port, uint32_t baud) {
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(uart_num, &cfg);
    uart_set_pin(uart_num, TX_PIN, RX_PIN, -1, -1);
    uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    return (HalSerial*)1;  // dummy non-null pointer
}

int hal_serial_read(HalSerial* s, uint8_t* buf, int len, uint32_t timeout_ms) {
    return uart_read_bytes(
        uart_num, buf, len,
        pdMS_TO_TICKS(timeout_ms)
    );
}

uint32_t hal_time_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}