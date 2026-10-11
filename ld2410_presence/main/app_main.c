/*
 * Zigbee 3.0 presence sensor: ESP32-H2 + HLK-LD2410 24 GHz mmWave radar.
 *
 * Router device with one HA endpoint exposing Basic, Identify and
 * Occupancy Sensing (0x0406) server clusters. The Occupancy attribute
 * follows the radar target state (moving and/or stationary target = occupied)
 * and is reported to the coordinator on every change.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"

#include "ld2410.h"

#if !defined CONFIG_ZB_ZCZR
#error Define ZB_ZCZR in idf.py menuconfig to compile the router source code.
#endif

static const char *TAG = "zb_presence";

/* Zigbee network configuration: router, joins an existing network */
#define MAX_CHILDREN                    10
#define INSTALLCODE_POLICY_ENABLE       false
#define SENSOR_ENDPOINT                 1
#define HA_OCCUPANCY_SENSOR_DEVICE_ID   0x0107
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* ZCL character strings: first byte is the length */
#define MANUFACTURER_NAME               "\x06""haxeko"
#define MODEL_IDENTIFIER                "\x0E""ESP32H2-LD2410"

#define ESP_ZB_ZR_CONFIG()                                  \
    {                                                       \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,   \
        .nwk_cfg.zczr_cfg = {                               \
            .max_children = MAX_CHILDREN,                   \
        },                                                  \
    }

static volatile bool s_zb_started;
static volatile bool s_joined;
static int s_occupied = -1; /* -1 = unknown, forces the first update */

/* ---------- occupancy -> Zigbee ---------- */

static void report_occupancy(uint8_t occupancy)
{
    esp_zb_zcl_set_attribute_val(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy, false);
    if (!s_joined) {
        return;
    }

    /* Explicit report to the coordinator, so it works without configured reporting */
    esp_zb_zcl_report_attr_cmd_t cmd = { 0 };
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = SENSOR_ENDPOINT;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING;
    cmd.attributeID = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

static void on_radar_report(const ld2410_report_t *r)
{
    static TickType_t last_log;
    int occupied = r->state != LD2410_TARGET_NONE;

    if (xTaskGetTickCount() - last_log > pdMS_TO_TICKS(5000)) {
        last_log = xTaskGetTickCount();
        ESP_LOGI(TAG, "state %d, moving %u cm (%u), stationary %u cm (%u), detect %u cm",
                 r->state, r->moving_distance_cm, r->moving_energy,
                 r->stationary_distance_cm, r->stationary_energy, r->detection_distance_cm);
    }

    if (occupied == s_occupied || !s_zb_started) {
        return;
    }
    s_occupied = occupied;
    ESP_LOGI(TAG, "%s (state %d, distance %u cm)", occupied ? "OCCUPIED" : "UNOCCUPIED",
             r->state, r->detection_distance_cm);

    esp_zb_lock_acquire(portMAX_DELAY);
    report_occupancy(occupied ? 1 : 0);
    esp_zb_lock_release();
}

/* ---------- button: hold to factory reset ---------- */

#if CONFIG_BUTTON_GPIO >= 0
static void button_task(void *arg)
{
    const uint32_t hold_ms = CONFIG_RESET_HOLD_SECONDS * 1000;
    uint32_t pressed_ms = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(CONFIG_BUTTON_GPIO) != 0) {
            pressed_ms = 0;
            continue;
        }
        pressed_ms += 50;
        if (pressed_ms == hold_ms && s_zb_started) {
            ESP_LOGW(TAG, "Button held %d s -> Zigbee factory reset", CONFIG_RESET_HOLD_SECONDS);
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_factory_reset(); /* leaves the network, erases zb_storage, reboots */
            esp_zb_lock_release();
        }
    }
}

static void button_init(void)
{
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CONFIG_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
}
#endif

/* ---------- commissioning ---------- */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering (pairing mode)");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted, rejoined network");
                s_joined = true;
                s_occupied = -1; /* re-send current occupancy */
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying",
                     esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network (PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            s_joined = true;
            s_occupied = -1; /* re-send current occupancy */
        } else {
            ESP_LOGI(TAG, "Network steering not successful (status: %s), retrying",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGW(TAG, "Left the network");
        s_joined = false;
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* ---------- data model ---------- */

static esp_zb_ep_list_t *sensor_ep_create(void)
{
    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01, /* mains, single phase */
    };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                                  MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                                  MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(clusters, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(clusters,
                    esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_occupancy_sensing_cluster_cfg_t occupancy_cfg = {
        .occupancy = 0,
        .sensor_type = 0,        /* PIR (closest standard type for a radar) */
        .sensor_type_bitmap = 0x01,
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(clusters,
                    esp_zb_occupancy_sensing_cluster_create(&occupancy_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = HA_OCCUPANCY_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg));
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_device_register(sensor_ep_create());
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    s_zb_started = true;
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
#if CONFIG_BUTTON_GPIO >= 0
    button_init();
#endif
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(ld2410_start(on_radar_report));
}
