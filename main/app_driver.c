/*
 * Relay, status LED and push-button driver.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "app_priv.h"

static const char *TAG = "app_driver";

#define NVS_NAMESPACE       "relay"
#define NVS_KEY_STATE       "on"
#define BUTTON_POLL_MS      20
#define BUTTON_DEBOUNCE_MS  60

static bool s_relay_on;
static app_button_cb_t s_short_cb;
static app_button_cb_t s_long_cb;

static void save_state(bool on)
{
#if CONFIG_RESTORE_RELAY_STATE
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_STATE, on);
        nvs_commit(h);
        nvs_close(h);
    }
#endif
}

bool app_driver_load_state(void)
{
#if CONFIG_RESTORE_RELAY_STATE
    nvs_handle_t h;
    uint8_t on = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY_STATE, &on);
        nvs_close(h);
    }
    return on != 0;
#else
    return false;
#endif
}

void app_driver_set_relay(bool on)
{
    gpio_set_level(CONFIG_RELAY_GPIO, CONFIG_RELAY_ACTIVE_HIGH ? on : !on);
#if CONFIG_STATUS_LED_GPIO >= 0
    gpio_set_level(CONFIG_STATUS_LED_GPIO, CONFIG_STATUS_LED_ACTIVE_LOW ? !on : on);
#endif
    if (on != s_relay_on) {
        s_relay_on = on;
        save_state(on);
    }
    ESP_LOGI(TAG, "Relay %s", on ? "ON" : "OFF");
}

bool app_driver_get_relay(void)
{
    return s_relay_on;
}

#if CONFIG_BUTTON_GPIO >= 0
static void button_task(void *arg)
{
    const uint32_t long_ms = CONFIG_RESET_HOLD_SECONDS * 1000;
    uint32_t pressed_ms = 0;
    bool long_fired = false;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        if (gpio_get_level(CONFIG_BUTTON_GPIO) == 0) {
            pressed_ms += BUTTON_POLL_MS;
            if (!long_fired && pressed_ms >= long_ms) {
                long_fired = true;
                ESP_LOGW(TAG, "Button held %d s -> factory reset", CONFIG_RESET_HOLD_SECONDS);
                if (s_long_cb) {
                    s_long_cb();
                }
            }
        } else {
            if (!long_fired && pressed_ms >= BUTTON_DEBOUNCE_MS && s_short_cb) {
                s_short_cb();
            }
            pressed_ms = 0;
            long_fired = false;
        }
    }
}
#endif

esp_err_t app_driver_init(app_button_cb_t on_short_press, app_button_cb_t on_long_press)
{
    s_short_cb = on_short_press;
    s_long_cb = on_long_press;

    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CONFIG_RELAY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
#if CONFIG_STATUS_LED_GPIO >= 0
    out.pin_bit_mask |= 1ULL << CONFIG_STATUS_LED_GPIO;
#endif
    ESP_ERROR_CHECK(gpio_config(&out));

    s_relay_on = app_driver_load_state();
    app_driver_set_relay(s_relay_on);

#if CONFIG_BUTTON_GPIO >= 0
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CONFIG_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
#endif
    return ESP_OK;
}
