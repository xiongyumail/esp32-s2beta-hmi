#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

void uart_hardware_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .use_ref_tick = 0,
    };
    uart_param_config(1, &uart_config);
    uart_set_pin(1, 20, -1, -1, -1);
    uart_driver_install(1, 200, 0, 0, NULL, 0);
    ets_delay_us(1000);
}

void uart_trans(uint8_t* src, uint32_t fb_size)
{
	uint8_t ffb[2] = {0x01, ~0x01};
	uint8_t rfb[2] = {~0x01, 0x01};

    uart_write_bytes(1, (const char *)ffb, 2);
    ets_delay_us(1000);
    uart_write_bytes(1, (const char *)src, fb_size);
    ets_delay_us(1000);
    uart_write_bytes(1, (const char *)rfb, 2);
}