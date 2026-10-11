/*
 * HLK-LD2410 (LD2410 / LD2410B / LD2410C) 24 GHz radar UART driver.
 *
 * Data frame (sent ~10 times per second):
 *   F4 F3 F2 F1 | len(2, LE) | type | 0xAA | target data ... | 0x55 0x00 | F8 F7 F6 F5
 * Target data (basic and engineering mode share the same prefix):
 *   state(1) moving_dist(2) moving_energy(1) static_dist(2) static_energy(1) detect_dist(2)
 *
 * Command frame:
 *   FD FC FB FA | len(2, LE) | cmd(2, LE) | value ... | 04 03 02 01
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "ld2410.h"

static const char *TAG = "ld2410";

#define LD2410_UART         UART_NUM_1
#define UART_RX_BUF_SIZE    1024
#define FRAME_MAX_LEN       64

static const uint8_t DATA_HEADER[4] = { 0xF4, 0xF3, 0xF2, 0xF1 };
static const uint8_t DATA_FOOTER[4] = { 0xF8, 0xF7, 0xF6, 0xF5 };
static const uint8_t CMD_HEADER[4]  = { 0xFD, 0xFC, 0xFB, 0xFA };
static const uint8_t CMD_FOOTER[4]  = { 0x04, 0x03, 0x02, 0x01 };

static ld2410_report_cb_t s_cb;

/* ---------- commands ---------- */

#if CONFIG_LD2410_APPLY_CONFIG
static void send_command(uint16_t cmd, const uint8_t *value, uint16_t value_len)
{
    uint8_t frame[FRAME_MAX_LEN];
    uint16_t len = 2 + value_len;
    size_t n = 0;

    memcpy(&frame[n], CMD_HEADER, 4);  n += 4;
    frame[n++] = len & 0xFF;
    frame[n++] = len >> 8;
    frame[n++] = cmd & 0xFF;
    frame[n++] = cmd >> 8;
    if (value_len) {
        memcpy(&frame[n], value, value_len);
        n += value_len;
    }
    memcpy(&frame[n], CMD_FOOTER, 4);  n += 4;

    uart_write_bytes(LD2410_UART, frame, n);
    uart_wait_tx_done(LD2410_UART, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(150)); /* ACK is ignored by the data-frame parser */
}

static void put_param(uint8_t *p, uint16_t word, uint32_t value)
{
    p[0] = word & 0xFF;
    p[1] = word >> 8;
    p[2] = value & 0xFF;
    p[3] = (value >> 8) & 0xFF;
    p[4] = (value >> 16) & 0xFF;
    p[5] = (value >> 24) & 0xFF;
}

static void apply_config(void)
{
    const uint8_t enable_value[2] = { 0x01, 0x00 };
    uint8_t params[18];

    put_param(&params[0],  0x0000, CONFIG_LD2410_MAX_GATE);         /* max moving gate */
    put_param(&params[6],  0x0001, CONFIG_LD2410_MAX_GATE);         /* max stationary gate */
    put_param(&params[12], 0x0002, CONFIG_LD2410_NO_ONE_DURATION);  /* no-one duration (s) */

    send_command(0x00FF, enable_value, sizeof(enable_value)); /* enable configuration */
    send_command(0x0060, params, sizeof(params));             /* max gate & no-one duration */
    send_command(0x00FE, NULL, 0);                            /* end configuration */

    ESP_LOGI(TAG, "Configured: max gate %d (%.2f m), no-one duration %d s",
             CONFIG_LD2410_MAX_GATE, CONFIG_LD2410_MAX_GATE * 0.75, CONFIG_LD2410_NO_ONE_DURATION);
}
#endif

/* ---------- data frame parser ---------- */

static uint8_t s_frame[FRAME_MAX_LEN];
static size_t s_idx;
static uint16_t s_len;

static void handle_frame(const uint8_t *payload, uint16_t len)
{
    /* payload: type, 0xAA, target data..., 0x55, 0x00 */
    if (len < 13 || payload[1] != 0xAA) {
        return;
    }
    ld2410_report_t r = {
        .state                  = (ld2410_target_state_t)(payload[2] & 0x03),
        .moving_distance_cm     = payload[3] | (payload[4] << 8),
        .moving_energy          = payload[5],
        .stationary_distance_cm = payload[6] | (payload[7] << 8),
        .stationary_energy      = payload[8],
        .detection_distance_cm  = payload[9] | (payload[10] << 8),
    };
    if (s_cb) {
        s_cb(&r);
    }
}

static void parse_byte(uint8_t b)
{
    if (s_idx < 4) {
        if (b == DATA_HEADER[s_idx]) {
            s_frame[s_idx++] = b;
        } else {
            s_idx = (b == DATA_HEADER[0]) ? 1 : 0;
        }
        return;
    }

    s_frame[s_idx++] = b;

    if (s_idx == 6) {
        s_len = s_frame[4] | (s_frame[5] << 8);
        if (s_len + 10 > FRAME_MAX_LEN) {
            s_idx = 0; /* bogus length */
        }
        return;
    }

    if (s_idx > 6 && s_idx == (size_t)s_len + 10) {
        if (memcmp(&s_frame[6 + s_len], DATA_FOOTER, 4) == 0) {
            handle_frame(&s_frame[6], s_len);
        }
        s_idx = 0;
    }
}

static void ld2410_task(void *arg)
{
    uint8_t buf[128];

#if CONFIG_LD2410_APPLY_CONFIG
    vTaskDelay(pdMS_TO_TICKS(500)); /* let the radar finish booting */
    apply_config();
    uart_flush_input(LD2410_UART);
#endif

    while (true) {
        int n = uart_read_bytes(LD2410_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            parse_byte(buf[i]);
        }
    }
}

esp_err_t ld2410_start(ld2410_report_cb_t cb)
{
    s_cb = cb;

    const uart_config_t cfg = {
        .baud_rate = CONFIG_LD2410_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LD2410_UART, UART_RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LD2410_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LD2410_UART, CONFIG_LD2410_UART_TX_GPIO, CONFIG_LD2410_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d @ %d baud, TX GPIO%d -> LD2410 RX, RX GPIO%d <- LD2410 TX",
             LD2410_UART, CONFIG_LD2410_BAUD_RATE, CONFIG_LD2410_UART_TX_GPIO, CONFIG_LD2410_UART_RX_GPIO);

    return xTaskCreate(ld2410_task, "ld2410", 3072, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
