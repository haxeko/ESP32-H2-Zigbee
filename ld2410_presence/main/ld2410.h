#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    LD2410_TARGET_NONE       = 0x00,
    LD2410_TARGET_MOVING     = 0x01,
    LD2410_TARGET_STATIONARY = 0x02,
    LD2410_TARGET_BOTH       = 0x03,
} ld2410_target_state_t;

typedef struct {
    ld2410_target_state_t state;
    uint16_t moving_distance_cm;
    uint8_t  moving_energy;
    uint16_t stationary_distance_cm;
    uint8_t  stationary_energy;
    uint16_t detection_distance_cm;
} ld2410_report_t;

/* Called from the LD2410 task for every valid data frame */
typedef void (*ld2410_report_cb_t)(const ld2410_report_t *report);

esp_err_t ld2410_start(ld2410_report_cb_t cb);
