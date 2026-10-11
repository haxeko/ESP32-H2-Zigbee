/*
 * Zigbee 3.0 presence sensor: ESP32-H2 + HLK-LD2410 24 GHz mmWave radar.
 *
 * Router device with three HA endpoints:
 *   1: Basic, Identify, Occupancy Sensing (0x0406) - occupied when the radar
 *      sees a moving and/or stationary target
 *   2: Binary Input "Radar online" - LD2410 data frames are being received
 *      (FAULT status flag set when the radar stops sending data)
 *   3: Analog Input "Target distance" in cm
 * Zigbee2MQTT generates entities from these clusters automatically and uses
 * the Binary/Analog Input description attribute as the entity name.
 */
#include <stdlib.h>
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
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Endpoints */
#define SENSOR_ENDPOINT                 1   /* Occupancy Sensing */
#define RADAR_ONLINE_ENDPOINT           2   /* Binary Input: LD2410 is sending data */
#define DISTANCE_ENDPOINT               3   /* Analog Input: target distance (cm) */
#define HA_OCCUPANCY_SENSOR_DEVICE_ID   0x0107
#define HA_SIMPLE_SENSOR_DEVICE_ID      0x000C

/* ZCL character strings: first byte is the length */
#define MANUFACTURER_NAME               "\x06""haxeko"
#define MODEL_IDENTIFIER                "\x0E""ESP32H2-LD2410"
#define RADAR_ONLINE_DESCRIPTION        "\x0C""Radar online"
#define DISTANCE_DESCRIPTION            "\x0F""Target distance"

#define BACNET_UNITS_CENTIMETERS        118
#define STATUS_FLAG_FAULT               0x02
#define RADAR_TIMEOUT_MS                3000    /* no frame for this long = radar offline */
#define RADAR_MONITOR_PERIOD_MS         1000
#define DISTANCE_REPORT_STEP_CM         10

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

/* Latest radar data, written by the LD2410 task, read by the Zigbee task */
static volatile TickType_t s_last_frame_tick;
static volatile bool s_frame_received;
static volatile uint16_t s_distance_cm;

/* ---------- attributes -> Zigbee (call with the Zigbee lock held / in Zigbee task) ---------- */

/* Optional attribute: log instead of aborting if the stack does not support it */
#define ADD_OPTIONAL_ATTR(fn, list, id, value)                                      \
    do {                                                                            \
        esp_err_t _e = fn(list, id, value);                                         \
        if (_e != ESP_OK) {                                                         \
            ESP_LOGW(TAG, #fn "(0x%04x) failed: %s", id, esp_err_to_name(_e));      \
        }                                                                           \
    } while (0)

static void set_and_report(uint8_t endpoint, uint16_t cluster, uint16_t attr, void *value)
{
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr, value, false);
    if (!s_joined) {
        return;
    }

    /* Explicit report to the coordinator, so it works without configured reporting */
    esp_zb_zcl_report_attr_cmd_t cmd = { 0 };
    cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    cmd.zcl_basic_cmd.dst_endpoint = 1;
    cmd.zcl_basic_cmd.src_endpoint = endpoint;
    cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd.clusterID = cluster;
    cmd.attributeID = attr;
    cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    esp_zb_zcl_report_attr_cmd_req(&cmd);
}

static void report_occupancy(uint8_t occupancy)
{
    set_and_report(SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                   ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy);
}

/* Runs every second in the Zigbee task: radar online state and target distance */
static void radar_monitor_cb(uint8_t param)
{
    static int last_online = -1; /* -1 = unknown, forces the first update */
    static int last_distance = -1;

    bool online = s_frame_received &&
                  (xTaskGetTickCount() - s_last_frame_tick) < pdMS_TO_TICKS(RADAR_TIMEOUT_MS);

    if (online != last_online) {
        last_online = online;
        bool present = online;
        uint8_t flags = online ? 0 : STATUS_FLAG_FAULT;
        if (online) {
            ESP_LOGI(TAG, "Radar ONLINE (LD2410 data frames received)");
        } else {
            ESP_LOGW(TAG, "Radar OFFLINE: no LD2410 data frame for %d ms - check wiring "
                     "(LD2410 TX -> GPIO%d, RX -> GPIO%d), 5V supply and baud rate (%d)",
                     RADAR_TIMEOUT_MS, CONFIG_LD2410_UART_RX_GPIO, CONFIG_LD2410_UART_TX_GPIO,
                     CONFIG_LD2410_BAUD_RATE);
        }
        esp_zb_zcl_set_attribute_val(RADAR_ONLINE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_BINARY_INPUT_STATUS_FLAG_ID, &flags, false);
        set_and_report(RADAR_ONLINE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
                       ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &present);
    }

    int distance = online ? s_distance_cm : 0;
    if (last_distance < 0 || abs(distance - last_distance) >= DISTANCE_REPORT_STEP_CM ||
        (distance == 0) != (last_distance == 0)) {
        last_distance = distance;
        float value = distance;
        set_and_report(DISTANCE_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
                       ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &value);
    }

    esp_zb_scheduler_alarm(radar_monitor_cb, 0, RADAR_MONITOR_PERIOD_MS);
}

static void on_radar_report(const ld2410_report_t *r)
{
    static TickType_t last_log;
    int occupied = r->state != LD2410_TARGET_NONE;

    s_distance_cm = occupied ? r->detection_distance_cm : 0;
    s_last_frame_tick = xTaskGetTickCount();
    s_frame_received = true;

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

    /* Endpoint 2: "Radar online" binary input (Zigbee2MQTT uses the description as name) */
    esp_zb_binary_input_cluster_cfg_t online_cfg = {
        .out_of_service = false,
        .status_flags = STATUS_FLAG_FAULT,
        .present_value = false,
    };
    esp_zb_attribute_list_t *online = esp_zb_binary_input_cluster_create(&online_cfg);
    ADD_OPTIONAL_ATTR(esp_zb_binary_input_cluster_add_attr, online, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, RADAR_ONLINE_DESCRIPTION);
    esp_zb_cluster_list_t *online_clusters = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(online_clusters, online,
                                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ep_cfg.endpoint = RADAR_ONLINE_ENDPOINT;
    ep_cfg.app_device_id = HA_SIMPLE_SENSOR_DEVICE_ID;
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, online_clusters, ep_cfg));

    /* Endpoint 3: "Target distance" analog input in cm */
    esp_zb_analog_input_cluster_cfg_t distance_cfg = {
        .out_of_service = false,
        .present_value = 0,
        .status_flags = 0,
    };
    uint16_t units = BACNET_UNITS_CENTIMETERS;
    float min_value = 0;
    float max_value = 600; /* LD2410 max range: 8 gates x 0.75 m */
    esp_zb_attribute_list_t *distance = esp_zb_analog_input_cluster_create(&distance_cfg);
    ADD_OPTIONAL_ATTR(esp_zb_analog_input_cluster_add_attr, distance, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID, DISTANCE_DESCRIPTION);
    ADD_OPTIONAL_ATTR(esp_zb_analog_input_cluster_add_attr, distance, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units);
    ADD_OPTIONAL_ATTR(esp_zb_analog_input_cluster_add_attr, distance, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, &min_value);
    ADD_OPTIONAL_ATTR(esp_zb_analog_input_cluster_add_attr, distance, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, &max_value);
    esp_zb_cluster_list_t *distance_clusters = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(distance_clusters, distance,
                                                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ep_cfg.endpoint = DISTANCE_ENDPOINT;
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, distance_clusters, ep_cfg));

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
    esp_zb_scheduler_alarm(radar_monitor_cb, 0, RADAR_MONITOR_PERIOD_MS);
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
