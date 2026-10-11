#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_zigbee_core.h"

/* Zigbee network configuration: router, joins an existing network */
#define MAX_CHILDREN                    10
#define INSTALLCODE_POLICY_ENABLE       false
#define RELAY_ENDPOINT                  1
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* ZCL character strings: first byte is the length */
#define MANUFACTURER_NAME               "\x06""haxeko"
#define MODEL_IDENTIFIER                "\x0D""ESP32H2-Relay"

#define ESP_ZB_ZR_CONFIG()                                  \
    {                                                       \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,   \
        .nwk_cfg.zczr_cfg = {                               \
            .max_children = MAX_CHILDREN,                   \
        },                                                  \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                       \
    {                                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                 \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                        \
    {                                                       \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

/* Driver (app_driver.c) */
typedef void (*app_button_cb_t)(void);

esp_err_t app_driver_init(app_button_cb_t on_short_press, app_button_cb_t on_long_press);
void app_driver_set_relay(bool on);
bool app_driver_get_relay(void);
bool app_driver_load_state(void);
