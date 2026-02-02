// HX711S strain gauge sensor support for bed probing
//
// Copyright (C) 2026 Timo V
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __SENSOR_HX711S_H__
#define __SENSOR_HX711S_H__

#include <stdint.h>
#include <string.h>
#include "board/gpio.h"
#include "sched.h"

// Forward declaration for trsync
struct trsync;

/****************************************************************
 * Configuration Constants
 ****************************************************************/

// Maximum number of sensors (4 physical + 1 fusion channel)
#define HX711S_MAX_SENSOR_NUM   5
// Maximum sliding window data points
#define HX711S_MAX_DATA_NUM     30
// Window data for averaging filter
#define HX711S_WINDOW_DATA_NUM  28
// Maximum sensors per device
#define HX711S_MAX_SENSORS      4

// Math constant
#define HX711S_PI 3.14159f

/****************************************************************
 * Enumerations
 ****************************************************************/

// Sensor operating flags
enum hx711s_flags {
    HX711S_FLAG_START = 1 << 0,
};

// Strain gauge mode (chip and bridge type)
enum hx711s_sg_mode {
    SG_MODE_HX711_FULL_BRIDGE = 0,
    SG_MODE_HX711_HALF_BRIDGE = 1,
    SG_MODE_HX717_FULL_BRIDGE = 2,
    SG_MODE_HX717_HALF_BRIDGE = 3,
};

// Probe check command modes
enum hx711s_probe_mode {
    PROBE_CHECK_MODE_NONE        = 0,
    PROBE_CHECK_MODE_START       = 1,
    PROBE_CHECK_MODE_STOP        = 2,
    PROBE_CHECK_MODE_CALIBRATION = 3,
};

// Sensor status flags
enum hx711s_status {
    SG_1_OFFLINE = 1 << 0,
    SG_2_OFFLINE = 1 << 1,
    SG_3_OFFLINE = 1 << 2,
    SG_4_OFFLINE = 1 << 3,
    SG_1_REVERSE = 1 << 4,
    SG_2_REVERSE = 1 << 5,
    SG_3_REVERSE = 1 << 6,
    SG_4_REVERSE = 1 << 7,
};

/****************************************************************
 * Data Structures
 ****************************************************************/

// High-pass filter parameters
struct hx711s_hpf_params {
    int32_t vi;             // Current input
    int32_t vi_prev;        // Previous input
    int32_t vo;             // Current output
    int32_t vo_prev;        // Previous output
    float cutoff_frq_hz;    // Cutoff frequency in Hz
    float acq_frq_hz;       // Acquisition frequency in Hz
};

// Main HX711S sensor structure
struct hx711s_sensor {
    struct timer timer;
    uint32_t oid;
    uint32_t rest_ticks;
    uint32_t sample_period;         // Sampling period in microseconds
    uint32_t enable_channels;       // Enabled channel bitmask
    uint32_t enable_hpf;            // Enable high-pass filter
    uint32_t enable_shake_filter;   // Enable shake/vibration filter
    // find_index_mode bits:
    //   bit 0: apply slope compensation
    //   bits 1-2: rollback method (00=linear, 01=backward threshold, 10=forward)
    //   bit 3: slope calculation method
    uint32_t find_index_mode;
    uint32_t heartbeat_period;      // Heartbeat interval in samples

    // Probe position
    int32_t x;
    int32_t y;
    int32_t z;

    // Probe state
    uint8_t is_bottom_detection;
    int32_t probe_check_cmd;
    uint8_t is_running_check;
    uint8_t flags;
    uint8_t is_calibration;
    uint8_t is_trigger;
    uint8_t trigger_index;
    uint8_t now_trigger;
    uint32_t trigger_tick;

    // trsync for homing/probing integration
    struct trsync *ts;
    uint8_t trigger_reason;
    uint8_t error_reason;
    uint8_t is_homing;

    // Sensor configuration
    uint32_t hx711_count;
    uint32_t sg_mode;           // Chip/bridge mode
    uint8_t install_dir;        // Installation direction (0=negative, 1=positive)
    uint32_t debug_data;
    int32_t times_read;

    // GPIO pins (for HX711 bit-bang mode)
    struct gpio_out clks[HX711S_MAX_SENSORS];
    struct gpio_in sdos[HX711S_MAX_SENSORS];

    // Trace/debug flag
    uint32_t traceflag;

    // Timestamps (index 4 = fusion timestamp)
    uint32_t time_stamp[5];

    // Sensor values
    int32_t init_values[HX711S_MAX_SENSORS];         // Baseline values
    int32_t amplitude_values[HX711S_MAX_SENSORS];    // Shake amplitude
    int32_t sample_values[HX711S_MAX_SENSORS];       // Raw ADC values
    int32_t max_data_num;                            // Sliding window size
    int32_t calibration_values[HX711S_MAX_SENSORS];  // Calibrated values
    int32_t filter_values[HX711S_MAX_SENSORS];       // Filtered values
    int32_t fusion_filter_value;                     // Fused sensor value

    // Threshold and filter parameters
    int32_t kalman_q[HX711S_MAX_SENSORS];   // Used as ADC threshold
    int32_t kalman_r[HX711S_MAX_SENSORS];   // Used as shake amplitude threshold
    int32_t max_th;                          // Maximum trigger threshold
    int32_t min_th;                          // Minimum trigger threshold
    int32_t th_k;                            // Quantization step

    int32_t status;                          // Sensor status flags
};

#endif // __SENSOR_HX711S_H__
