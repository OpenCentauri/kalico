// HX711S strain gauge sensor support for bed probing
//
// Copyright (C) 2026 Timo V
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include "basecmd.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "trsync.h"  // trsync_do_trigger
#include "sensor_hx711s.h"
#include <string.h>
#include <math.h>

/****************************************************************
 * Module State
 ****************************************************************/

static struct task_wake hx711s_wake;
static uint32_t ori_rest_ticks;

static int32_t k_slope;      // Median slope
static int32_t bias_slope;   // Slope-to-rollback ratio

// Sliding window data buffers (per-sensor + fusion channel)
static int32_t data_list[HX711S_MAX_SENSOR_NUM][HX711S_MAX_DATA_NUM];
static uint32_t timestamp_list[HX711S_MAX_SENSOR_NUM][HX711S_MAX_DATA_NUM];

// High-pass filter state
static struct hx711s_hpf_params hpf_params;
static struct hx711s_hpf_params fusion_hpf_params;

/****************************************************************
 * Utility Functions
 ****************************************************************/

static inline int32_t
hx711s_abs(int32_t val)
{
    return (val < 0) ? -val : val;
}

// Bubble sort for median/window filtering
static void
bubble_sort(int32_t *array, int len)
{
    for (int i = 0; i < len - 1; i++) {
        for (int j = 0; j < len - 1 - i; j++) {
            if (array[j] > array[j + 1]) {
                int32_t temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

// Median filter for calibration
static int32_t
median_filter(int32_t *array, int len)
{
    int32_t sorted[1000];
    if (len > 1000)
        len = 1000;
    memcpy(sorted, array, len * sizeof(int32_t));
    bubble_sort(sorted, len);

    if (len & 1)
        return sorted[(len + 1) / 2];
    else
        return (sorted[len / 2] + sorted[len / 2 + 1]) / 2;
}

/****************************************************************
 * High-Pass Filter
 ****************************************************************/

static void
hpf_init(struct hx711s_hpf_params *p, float cutoff_hz, float sample_hz,
         int32_t base)
{
    p->vi = base;
    p->vi_prev = base;
    p->vo = 0;
    p->vo_prev = 0;
    p->cutoff_frq_hz = cutoff_hz;
    p->acq_frq_hz = sample_hz;
}

static int32_t
hpf_apply(struct hx711s_hpf_params *p, int32_t input)
{
    float rc = 1.0f / (2.0f * HX711S_PI * p->cutoff_frq_hz);
    float coeff = rc / (rc + 1.0f / p->acq_frq_hz);

    p->vi = input;
    p->vo = (int32_t)((float)(p->vi - p->vi_prev + p->vo_prev) * coeff);
    p->vo_prev = p->vo;
    p->vi_prev = p->vi;

    return p->vo;
}

/****************************************************************
 * Sliding Window Filter
 ****************************************************************/

static void
sliding_window_shift(int32_t *array, int len, int32_t data)
{
    for (int i = 0; i < len - 1; i++)
        array[i] = array[i + 1];
    array[len - 1] = data;
}

static void
sliding_window_shift_u32(uint32_t *array, int len, uint32_t data)
{
    for (int i = 0; i < len - 1; i++)
        array[i] = array[i + 1];
    array[len - 1] = data;
}

// Sliding window average with exception filtering
// Returns 1 on success, 0 if value was rejected as outlier
static int
sliding_window_avg_exception_filter(int index, int max_data_num,
                                    int window_data_num, int32_t *data,
                                    uint32_t timestamp, int32_t exception_th,
                                    uint8_t enable_hpf)
{
    static int data_count[HX711S_MAX_SENSOR_NUM] = {0};
    static int32_t last_out[HX711S_MAX_SENSOR_NUM] = {0};

    if (max_data_num < 2 || max_data_num > HX711S_MAX_DATA_NUM ||
        index >= HX711S_MAX_SENSOR_NUM)
        return 0;

    int32_t filter_data = *data;

    // Apply high-pass filter if enabled
    if (enable_hpf) {
        if (index == 4)
            filter_data = hpf_apply(&fusion_hpf_params, *data);
        else
            filter_data = hpf_apply(&hpf_params, *data);
    }

    // Check for outlier
    if (exception_th > 0 && last_out[index] != 0) {
        int32_t diff = last_out[index] - filter_data;
        if (hx711s_abs(diff) > exception_th) {
            *data = last_out[index];
            return 0;
        }
    }

    // Shift data into sliding window
    sliding_window_shift(data_list[index], max_data_num, filter_data);
    sliding_window_shift_u32(timestamp_list[index], max_data_num, timestamp);

    // Calculate windowed average
    int32_t sum = 0;
    int32_t out;

    if (data_count[index] < max_data_num) {
        data_count[index]++;
        for (int i = 0; i < data_count[index]; i++)
            sum += data_list[index][i];
        out = sum / data_count[index];
    } else {
        // Sort and trim extremes
        int32_t sorted[HX711S_MAX_DATA_NUM];
        memcpy(sorted, data_list[index], max_data_num * sizeof(int32_t));
        bubble_sort(sorted, max_data_num);

        int start = (max_data_num - window_data_num) / 2;
        for (int i = start; i < start + window_data_num; i++)
            sum += sorted[i];
        out = sum / window_data_num;
    }

    if (out == 0)
        out = 1;

    last_out[index] = out;
    return 1;
}

/****************************************************************
 * Low-Level ADC Reading (HX711 bit-bang)
 ****************************************************************/

#define HX711S_MIN_PULSE_NS 200

static inline void
hx711s_delay_ns(uint32_t ns)
{
    uint32_t ticks = timer_from_us(ns * 1000) / 1000000;
    if (ticks < 1) ticks = 1;
    uint32_t end = timer_read_time() + ticks;
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Read single sensor via bit-banging.
// Called from the background task after the timer ISR confirmed DOUT is low.
static void
hx711s_read_sensor(struct hx711s_sensor *h, uint8_t sensor_idx)
{
    // Capture the timestamp as early as possible to minimize timing errors.
    uint32_t capture_time = timer_read_time();

    gpio_out_write(h->clks[sensor_idx], 0);

    // Read 24 data bits
    int32_t value = 0;
    for (int i = 0; i < 24; i++) {
        irq_disable();
        gpio_out_write(h->clks[sensor_idx], 1);
        hx711s_delay_ns(HX711S_MIN_PULSE_NS);
        gpio_out_write(h->clks[sensor_idx], 0);
        if (gpio_in_read(h->sdos[sensor_idx]))
            value |= 1 << (23 - i);
        irq_enable();
        hx711s_delay_ns(HX711S_MIN_PULSE_NS);
    }

    // Extra clock pulse to set gain for next conversion
    irq_disable();
    gpio_out_write(h->clks[sensor_idx], 1);
    hx711s_delay_ns(HX711S_MIN_PULSE_NS);
    gpio_out_write(h->clks[sensor_idx], 0);
    irq_enable();

    // Sign extend 24-bit to 32-bit
    if (value & 0x00800000)
        value |= 0xFF000000;

    h->sample_values[sensor_idx] = value;
    h->time_stamp[sensor_idx] = capture_time;
}

/****************************************************************
 * Calibration
 ****************************************************************/

// Collect baseline sensor values
// Returns TRUE when calibration complete
static uint8_t
calibration_collect(struct hx711s_sensor *h, uint8_t sensor_idx)
{
    static int32_t sample_count[4] = {0};
    static int32_t samples[4][1000];
    static int32_t sensor_min[4] = {0};
    static int32_t sensor_max[4] = {0};

    // Calibration complete
    if (h->times_read == 0) {
        h->is_calibration = 0x80;

        for (int j = 0; j < (int)h->hx711_count; j++) {
            if (sample_count[j] > 0) {
                h->amplitude_values[j] = sensor_max[j] - sensor_min[j];
                h->init_values[j] = median_filter(samples[j], sample_count[j]);

                // Validate calibration
                if (hx711s_abs(h->init_values[j]) < hx711s_abs(h->kalman_q[0]))
                    h->is_calibration |= (0x01 << j);

                if (h->amplitude_values[j] < hx711s_abs(h->kalman_r[0]))
                    h->is_calibration |= (0x01 << j);
            } else {
                h->is_calibration = 0;
                return 0;
            }
            sample_count[j] = 0;
        }
        return 1;
    }

    // Read sensor (timer ISR already confirmed DOUT is low)
    hx711s_read_sensor(h, sensor_idx);

    // Store sample
    if ((sensor_idx + 1) == h->hx711_count)
        h->times_read--;

    if (sensor_min[sensor_idx] == 0)
        sensor_min[sensor_idx] = h->sample_values[sensor_idx];
    if (sensor_max[sensor_idx] == 0)
        sensor_max[sensor_idx] = h->sample_values[sensor_idx];

    samples[sensor_idx][sample_count[sensor_idx]] = h->sample_values[sensor_idx];

    if (h->sample_values[sensor_idx] < sensor_min[sensor_idx])
        sensor_min[sensor_idx] = h->sample_values[sensor_idx];
    if (h->sample_values[sensor_idx] > sensor_max[sensor_idx])
        sensor_max[sensor_idx] = h->sample_values[sensor_idx];

    sample_count[sensor_idx]++;

    return 1;
}

/****************************************************************
 * Sensor Fusion and Filtering
 ****************************************************************/

static uint8_t
hx711s_fusion_filter(struct hx711s_sensor *h, uint8_t sensor_idx,
                     uint8_t enable_hpf)
{
    if (!(h->is_calibration & 0x80))
        return 0;

    // Apply direction correction and baseline offset
    for (int i = 0; i < (int)h->hx711_count; i++) {
        if ((h->install_dir >> i) & 0x01)
            h->calibration_values[i] = -h->sample_values[i] + h->init_values[i];
        else
            h->calibration_values[i] = h->sample_values[i] - h->init_values[i];
    }

    h->time_stamp[4] = h->time_stamp[sensor_idx];

    // Per-sensor sliding window filter
    if (h->sg_mode != SG_MODE_HX717_HALF_BRIDGE) {
        if (!sliding_window_avg_exception_filter(
                sensor_idx, h->max_data_num, h->max_data_num - 2,
                &h->calibration_values[sensor_idx],
                h->time_stamp[sensor_idx], 600000, enable_hpf)) {
            return 0;
        }
    }

    // Fuse all sensors by summing
    h->fusion_filter_value = 0;
    for (int i = 0; i < (int)h->hx711_count; i++)
        h->fusion_filter_value += h->calibration_values[i];

    // Fusion channel sliding window filter
    if (!sliding_window_avg_exception_filter(
            4, h->max_data_num, h->max_data_num - 2, &h->fusion_filter_value,
            h->time_stamp[4], 600000, enable_hpf)) {
        return 0;
    }

    return 1;
}

/****************************************************************
 * Trigger Detection
 ****************************************************************/

// Advanced trigger index finder with slope compensation
static int32_t
find_trigger_index_new(int32_t *array, struct hx711s_sensor *h)
{
    double val[HX711S_MAX_DATA_NUM] = {0};
    double val_transfer[HX711S_MAX_DATA_NUM] = {0};
    int max_num = h->max_data_num;

    // Ensure data direction is ascending
    if (array[max_num - 1] - array[0] < 0) {
        for (int i = 0; i < max_num; i++)
            val[i] = -array[i];
    } else {
        for (int i = 0; i < max_num; i++)
            val[i] = array[i];
    }

    // Normalize data to [0..1]
    double val_min = val[max_num - 1], val_max = val[0];
    for (int i = 0; i < max_num; i++) {
        if (val[i] < val_min) val_min = val[i];
        if (val[i] > val_max) val_max = val[i];
    }
    double val_err = val_max - val_min;
    if (val_err < 1e-15) val_err = 1e-15; // prevent division by zero

    for (int i = 0; i < max_num; i++)
        val_transfer[i] = (val[i] - val_min) / val_err;


    // Calculate and rotate the data by a given angle to facilitate the c
    // calculation of the earliest trigger point
    double angle = atan((val_transfer[max_num - 1] - val_transfer[0]) /
                          (max_num - 1));
    double sinAngle = sin(-angle);
    double cosAngle = cos(-angle);

    // Rotate around the origin (0,0) by angle degrees clockwise.
    // Here we can ignore the X-axis coordinate values.
    for (int i = 0; i < max_num; i++) {
      val_transfer[i] =
          ((i - 0) * sinAngle) + ((val_transfer[i] - 0) * cosAngle) + 0;
    }

    // Find the index of the minimum value after rotation
    int32_t out_index = 0;
    double min_val = val_transfer[0];
    for (int i = max_num - 1; i >= 0; i--) {
        if (min_val > val_transfer[i]) {
            min_val = val_transfer[i];
            out_index = i;
        }
    }

    // Linear regression + slope fallback
    int32_t kk = (val[max_num - 1] - val[out_index]) / (max_num - out_index);

    // Calculate slope for compensation
    int32_t fix_out_index = 0;
    if (h->find_index_mode & 0x08) {
        // Fixed pattern slope calculation
        fix_out_index = kk*k_slope / 10000 - bias_slope / 10;
    } else {
        if (kk > 2300) {
          fix_out_index = 11;
        }
        else if (kk > 2200) {
          fix_out_index = 10;
        }
        else if (kk > 2100) {
          fix_out_index = 9;
        }
        else if (kk > 2000) {
          fix_out_index = 8;
        }
        else if (kk > 1900) {
          fix_out_index = 7;
        }
        else if (kk > 1700) {
          fix_out_index = 6;
        }
        else if (kk > 1600) {
          fix_out_index = 5;
        }
        else if (kk > 1500) {
          fix_out_index = 4;
        }
        else if (kk > 1300) {
          fix_out_index = 3;
        }
        else if (kk > 1000) {
          fix_out_index = 2;
        }
        else if (kk > 900) {
          fix_out_index = 1;
        }
        else if (kk > 800) {
          fix_out_index = 0;
        }
        else if (kk > 700) {
          fix_out_index = -1;
        }
        else if (kk > 600) {
          fix_out_index = -2;
        }
        else if (kk > 500) {
          fix_out_index = -3;
        }
        else if (kk > 400) {
          fix_out_index = -4;
        }
        else  {
          fix_out_index = -5;
        }
    }

    // Rollback method selection
    if ((h->find_index_mode & 0x06) == 0x02) {
        // Backward threshold search
        for (int i = max_num - 1; i >= 0; i--) {
            if (h->min_th > val[i]) {
                out_index = i;
                break;
            }
        }
    } else if ((h->find_index_mode & 0x06) == 0x04) {
        // Forward threshold search
        for (int i = 0; i < max_num; i++) {
            if (h->min_th < val[i]) {
                out_index = i;
                break;
            }
        }
    }

    // Apply slope compensation if enabled
    if (h->find_index_mode & 0x01) {
        out_index += fix_out_index;
        if (out_index < 0)
            out_index = 0;
        if (out_index > max_num - 1)
            out_index = max_num - 1;
    }

    return out_index;
}

// Check if trigger conditions are met
// Returns trigger flags or 0 if not triggered
static uint8_t
check_trigger(int32_t *data, struct hx711s_sensor *h)
{
    int max_num = h->max_data_num;

    // Calibration mode: use average threshold
    if (h->probe_check_cmd == PROBE_CHECK_MODE_CALIBRATION) {
        int32_t avg = 0;
        for (int i = 0; i < max_num; i++)
            avg += data[i];
        avg /= max_num;
        if (avg < -h->min_th)
            return 0x60;
    }

    // Maximum threshold check - immediate trigger
    int trig = (data[max_num - 1] <= -h->max_th) && (hx711s_abs(data[0]) > 0);

    if (trig) {
        // Shake filter: reject if any point is positive
        if (h->enable_shake_filter) {
            int start = (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) ?
                        max_num - 10 : 0;
            for (int i = start; i < max_num; i++) {
                if (data[i] > h->min_th)
                    return 0;
            }
        }
        return 0x40;
    }

    // Check monotonic increase of last 3 points
    if (!(hx711s_abs(data[max_num - 1]) > hx711s_abs(data[max_num - 2]) &&
          hx711s_abs(data[max_num - 2]) > hx711s_abs(data[max_num - 3])))
        return 0;


    // Check last 3 points are largest (in absolute terms)
    for (int i = 0; i < max_num - 3; i++) {
      if (hx711s_abs(data[i]) > hx711s_abs(data[max_num - 1]) ||
          hx711s_abs(data[i]) > hx711s_abs(data[max_num - 2]) ||
          hx711s_abs(data[i]) > hx711s_abs(data[max_num - 3]))
            return 0;
    }

    // Normalize data
    double val_min = +0xFFFFFFFF, val_max = -0x00FFFFFF, val_avg = 0;
    for (int i = 0; i < max_num; i++) {
        if (data[i] < val_min) val_min = data[i];
        if (data[i] > val_max) val_max = data[i];
        val_avg += data[i];
    }
    val_avg /= max_num;
    double val_err = val_max - val_min;
    if (val_err < 1e-15) val_err = 1e-15; // prevent division by zero

    double val_p[HX711S_MAX_DATA_NUM];
    for (int i = 0; i < max_num; i++)
        val_p[i] = (data[i] - val_min) / val_err;

    // Ensure that the slope of all points relative to the last point is
    // greater than 40 degrees, which can prevent false triggers caused by
    // being too sensitive.
    for (int i = 0; i < max_num - 1; i++) {
        double k = (val_p[max_num - 1] - val_p[i]) /
                   ((max_num - 1 - i) * 1.0 / max_num);
        if (hx711s_abs(k) < 0.8)
            return 0;
        if (h->enable_shake_filter && k > 0.8)
            return 0;
    }

    // Minimum threshold check
    if (hx711s_abs(data[max_num - 1]) < h->min_th)
        return 0;

    // Shake filter: reject positive values
    if (h->enable_shake_filter && data[max_num - 1] > h->min_th)
        return 0;

    return 0x20;
}

// Main trigger check function
static int32_t
trigger_check_new(struct hx711s_sensor *h, uint8_t sensor_idx)
{
    int32_t trigger = 0;
    uint8_t trigger_index = 0;
    uint8_t check_flag;

    // Check fusion channel first
    if ((h->enable_channels >> 4) & 0x01) {
        check_flag = check_trigger(data_list[4], h);
        if (check_flag) {
            trigger_index = find_trigger_index_new(data_list[4], h);
            trigger |= 0x10;
            trigger |= (check_flag & 0x60);
            trigger |= (trigger_index << 8);
            trigger |= (0x01 << sensor_idx);
            return trigger;
        }
    }

    // Check individual sensor
    if ((h->enable_channels >> sensor_idx) & 0x01) {
        check_flag = check_trigger(data_list[sensor_idx], h);
        if (check_flag) {
            trigger_index = find_trigger_index_new(data_list[sensor_idx], h);
            trigger |= (0x01 << sensor_idx);
            trigger |= (check_flag & 0x60);
            trigger |= (trigger_index << 8);
            return trigger;
        }
    }

    return 0;
}

/****************************************************************
 * Timer Event Handler
 ****************************************************************/

static uint_fast8_t
hx711s_timer_event(struct timer *t)
{
    struct hx711s_sensor *h = container_of(t, struct hx711s_sensor, timer);
    uint32_t rest_ticks = h->rest_ticks;

    if (!(h->flags & HX711S_FLAG_START))
        return SF_DONE;

    if (h->flags & HX711S_FLAG_PENDING) {
        // Previous sample not yet consumed — back off
        rest_ticks *= 4;
    } else {
        // Check each sensor in rotation for data ready (DOUT low)
        for (uint8_t i = 0; i < h->hx711_count; i++) {
            if (!gpio_in_read(h->sdos[i])) {
                h->pending_sensor = i;
                h->flags |= HX711S_FLAG_PENDING;
                sched_wake_task(&hx711s_wake);
                break;
            }
        }
    }

    h->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

/****************************************************************
 * Commands
 ****************************************************************/

void
command_config_hx711s(uint32_t *args)
{
    struct hx711s_sensor *h = oid_alloc(args[0], command_config_hx711s,
                                        sizeof(*h));
    h->oid = args[0];
    h->hx711_count = args[1] & 0x0F;
    h->install_dir = (args[1] >> 4) & 0x0F;

    if (h->hx711_count > 4)
        shutdown("HX711S: Max 4 sensors");
    if (h->hx711_count < 1)
        shutdown("HX711S: Min 1 sensor");

    h->sg_mode = (args[3] & 0x0F000000) >> 24;

    // Default to full bridge HX711 mode for vanilla klipper
    h->sg_mode = SG_MODE_HX711_FULL_BRIDGE;

    h->find_index_mode = (args[2] & 0xF000) >> 12;

    if (h->sg_mode == SG_MODE_HX717_HALF_BRIDGE) {
        h->sample_period = 1500;
        h->max_data_num = 16;
        h->find_index_mode = 0;
        h->min_th = 0;
    } else {
        h->sample_period = args[3] & 0xFFFFFF;
        h->max_data_num = 12;
    }

    h->rest_ticks = CONFIG_CLOCK_FREQ / 1000000 * h->sample_period;
    ori_rest_ticks = h->rest_ticks;

    h->enable_channels = args[2] & 0xFF;
    h->enable_hpf = (args[2] & 0x0F00) >> 8;
    h->enable_shake_filter = (args[2] & 0xF0000) >> 16;
    h->heartbeat_period = 2000000 / h->sample_period;

    h->kalman_q[0] = args[4];
    h->kalman_r[0] = args[5];
    h->max_th = args[6];
    h->min_th = args[7];
    bias_slope = (args[8] & 0xFF00) >> 8;
    k_slope = (args[8] & 0xFFFF0000) >> 16;

    h->flags = 0;
    h->timer.func = hx711s_timer_event;

    h->is_calibration = 0;
    memset(h->init_values, 0, sizeof(h->init_values));
    memset(h->sample_values, 0, sizeof(h->sample_values));

    // Initialize high-pass filters
    float sample_rate = 1000000.0f / h->sample_period / h->hx711_count;
    hpf_init(&hpf_params, 5.0f, sample_rate, 0);
    hpf_init(&fusion_hpf_params, 5.0f, 1000000.0f / h->sample_period, 0);

    sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
          (int)args[0], (int)args[0], (int)args[1], (int)args[2], (int)args[3]);
}
DECL_COMMAND(command_config_hx711s,
    "config_hx711s oid=%c hx711_count=%c channels=%u rest_ticks=%u "
    "kalman_q=%u kalman_r=%u max_th=%u min_th=%u k=%u");

void
command_add_hx711s(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t index = args[1];
    struct hx711s_sensor *h = oid_lookup(oid, command_config_hx711s);

    if (index >= h->hx711_count)
        shutdown("HX711S: sensor index out of range");

    h->clks[index] = gpio_out_setup(args[2], 0);
    h->sdos[index] = gpio_in_setup(args[3], 0);

    sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
          (int)args[0], (int)args[0], (int)args[1], (int)args[2], (int)args[3]);
}
DECL_COMMAND(command_add_hx711s, "add_hx711s oid=%c index=%c clk_pin=%u sdo_pin=%u");

/****************************************************************
 * trsync-based Homing Command
 ****************************************************************/

void
command_hx711s_home(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_sensor *h = oid_lookup(oid, command_config_hx711s);

    // Clear homing state
    h->ts = NULL;
    h->is_homing = 0;
    h->is_trigger = 0;
    h->trigger_tick = 0;
    h->trigger_index = 0;

    // If trsync_oid is 0, homing is finished
    if (args[1] == 0) {
        return;
    }

    // Setup homing with trsync
    h->ts = trsync_oid_lookup(args[1]);
    h->homing_clock = args[2];
    h->trigger_reason = args[3];
    h->error_reason = args[4];
    h->is_homing = 1;
    h->flags = HX711S_FLAG_START | HX711S_FLAG_AWAIT_HOMING;
}
DECL_COMMAND(command_hx711s_home,
    "hx711s_home oid=%c trsync_oid=%c clock=%u trigger_reason=%c error_reason=%c");

// Query homing trigger state
void
command_hx711s_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_sensor *h = oid_lookup(oid, command_config_hx711s);

    sendf("hx711s_state oid=%c is_triggered=%c trigger_ticks=%u",
          oid, h->is_trigger > 0, h->trigger_tick);
}
DECL_COMMAND(command_hx711s_query_state, "hx711s_query_state oid=%c");

void
command_calibration_sample(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_sensor *h = oid_lookup(oid, command_config_hx711s);

    h->times_read = args[1];
    if (h->times_read < 1)
        h->times_read = 50;
    else if (h->times_read > 1000)
        h->times_read = 1000;

    h->rest_ticks = ori_rest_ticks;
    h->is_calibration = 0;

    memset(h->init_values, 0, sizeof(h->init_values));
    memset(h->calibration_values, 0, sizeof(h->calibration_values));
    memset(h->sample_values, 0, sizeof(h->sample_values));
    memset(data_list, 0, sizeof(data_list));
    memset(timestamp_list, 0, sizeof(timestamp_list));

    h->is_trigger = 0;
    h->trigger_tick = 0;
    h->probe_check_cmd = 0;

    h->flags |= HX711S_FLAG_START;

    sched_del_timer(&h->timer);
    irq_disable();
    h->timer.waketime = timer_read_time() + h->rest_ticks;
    sched_add_timer(&h->timer);
    irq_enable();

    sendf("debug_hx711s oid=%c arg[0]=%u arg[1]=%u arg[2]=%u arg[3]=%u",
          oid, 1, 0, 0, 0);
}
DECL_COMMAND(command_calibration_sample, "calibration_sample oid=%c times_read=%hu");

/****************************************************************
 * Background Task
 ****************************************************************/

void
hx711s_task(void)
{
    static uint8_t sensor_idx = 0;
    static uint32_t loop = 0;
    static uint32_t last_rep_loop = 0;
    static uint8_t last_is_calibration = 0;
    static uint8_t last_is_trigger = 0;

    if (!sched_check_wake(&hx711s_wake))
        return;

    uint8_t oid;
    struct hx711s_sensor *h;

    foreach_oid(oid, h, command_config_hx711s) {
        if (!(h->flags & HX711S_FLAG_PENDING))
            return;

        // Use the sensor index that the timer ISR found ready
        sensor_idx = h->pending_sensor;

        // Clear pending flag so the timer ISR can signal again
        irq_disable();
        h->flags &= ~HX711S_FLAG_PENDING;
        irq_enable();

        loop++;

        // Calibration phase
        if (!(h->is_calibration & 0x80)) {
            last_is_calibration = h->is_calibration;
            calibration_collect(h, sensor_idx);
            continue;
        }


        uint32_t now_tick = timer_read_time();

        // Read the ADC (timer ISR already confirmed DOUT is low)
        hx711s_read_sensor(h, sensor_idx);

        // Wait for homing_clock before enabling trigger detection
        if (h->flags & HX711S_FLAG_AWAIT_HOMING) {
            if (timer_is_before(now_tick, h->homing_clock))
                continue;
            h->flags &= ~HX711S_FLAG_AWAIT_HOMING;
        }

        // Fusion filter
        if (!hx711s_fusion_filter(h, sensor_idx, h->enable_hpf))
            continue;

        // Trigger detection
        int32_t trigger = trigger_check_new(h, sensor_idx);

        uint8_t is_triggered = (trigger > 0);
        uint8_t was_triggered = (h->is_trigger > 0);


        if (!is_triggered) {
          // Not homing and no trigger - clear state
          // During homing, keep trigger latched so query_state returns correct
          // value
          if (!h->is_homing) {
            h->is_trigger = 0;
            h->trigger_tick = 0;
          }
        } else if (!was_triggered) {
            // Trigger just detected - latch state
            h->is_trigger = trigger & 0xFF;
            h->trigger_index = (trigger >> 8) & 0xFF;

            if (h->is_trigger & 0x10)
                h->trigger_tick = timestamp_list[4][h->trigger_index]; // fusion channel timestamp
            else
                h->trigger_tick = timestamp_list[sensor_idx][h->trigger_index]; // individual sensor timestamp

            // Fire trsync if necessary
            if (h->is_homing && h->ts != NULL) {
                trsync_do_trigger(h->ts, h->trigger_reason);
            }
        }

        // Report heartbeat or trigger
        if ((loop % h->heartbeat_period) == 0 ||
            was_triggered != is_triggered ||
            last_is_calibration != h->is_calibration) {

            if (hx711s_abs((int32_t)(loop - last_rep_loop)) > 80 ||
                last_is_trigger == 0) {
                last_rep_loop = loop;

                sendf("sg_resp oid=%c vd=%c it=%c nt=%u r=%u tt=%u",
                      (uint8_t)h->oid,
                      (uint8_t)h->is_calibration,
                      (uint8_t)h->trigger_index,
                      (uint32_t)h->trigger_tick,
                      (uint32_t)h->is_trigger,
                      (uint32_t)now_tick);

                h->times_read--;
            }

            last_is_trigger = h->is_trigger;
            last_is_calibration = h->is_calibration;
        }
    }
}
DECL_TASK(hx711s_task);

void
hx711s_shutdown(void)
{
    uint8_t oid;
    struct hx711s_sensor *h;

    foreach_oid(oid, h, command_config_hx711s) {
        h->times_read = 0;
        h->flags &= ~HX711S_FLAG_START;
    }
}
DECL_SHUTDOWN(hx711s_shutdown);
