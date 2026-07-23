// Support for multi-sensor HX711 and HX717 ADC chips
//
// Copyright (C) 2026 James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_add_timer
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample_at
#include <stdint.h>

#define MAX_SENSORS 4
#define BYTES_PER_VALUE 4
#define POWERDOWN_US 10000
#define STARTUP_DELAY_US 50000
#define PULSE_TIME_TICKS timer_from_us(1)
#define SETTLING_FRAMES 5
#define QUALIFY_FRAMES 2
#define HX711S_OVERFLOW (1 << 1)

// A frame carries raw values even when it is invalid.  After checking and
// removing the wire-format tag, the host must never feed a non-zero quality
// frame into a tare, filter, or probe decision.
#define HX711S_Q_SETTLING       (1 << 0)
#define HX711S_Q_NOT_READY      (1 << 1)
#define HX711S_Q_EXTRA_LOW      (1 << 2)
#define HX711S_Q_POST_READ_LOW  (1 << 3)
#define HX711S_Q_READ_OVERRUN   (1 << 4)
#define HX711S_Q_SATURATED      (1 << 5)
#define HX711S_Q_CHANNEL_SHIFT  8
// Timing misses are the polling/read racing the free-running conversion
// clock: the frame is invalid and stays out of control data, but the ADC is
// healthy and the next poll resynchronizes.  Only a persistent streak (a
// genuinely stuck ADC) escalates to the fault and reset path.
#define HX711S_Q_TIMING_MISS    (HX711S_Q_NOT_READY | HX711S_Q_EXTRA_LOW \
                                 | HX711S_Q_POST_READ_LOW)
#define HX711S_Q_GENUINE_FAULT  (HX711S_Q_READ_OVERRUN | HX711S_Q_SATURATED)
// Set on a timing-miss frame when a persistent streak proves a stuck ADC.
// Bit 6 is reserved for the host-synthetic frame-format flag.
#define HX711S_Q_TIMING_STREAK  (1 << 7)
#define HX711S_TIMING_STREAK_LIMIT 40
#define HX711S_FRAME_TAG        UINT32_C(0xa7110000)

enum hx711s_state {
    HX711S_OFF,
    HX711S_RESET,
    HX711S_WAIT_READY,
    HX711S_SETTLING,
    HX711S_QUALIFY,
    HX711S_ONLINE,
};

struct hx711s_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint8_t pending_flag, sensor_count, gain_channel, sample_bytes;
    uint8_t state, settling_frames, qualify_frames, timing_streak;
    struct gpio_in sdos[MAX_SENSORS];
    struct gpio_out clks[MAX_SENSORS];
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

static struct task_wake wake_hx711s;

static void
hx711s_delay_noirq(void)
{
    if (CONFIG_MACH_AVR) {
        asm("nop\n    nop");
        return;
    }
    uint32_t end = timer_read_time() + PULSE_TIME_TICKS;
    while (timer_is_before(timer_read_time(), end))
        ;
}

static void
hx711s_delay(void)
{
    if (CONFIG_MACH_AVR)
        return;
    uint32_t end = timer_read_time() + PULSE_TIME_TICKS;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

static uint8_t
hx711s_ready_mask(struct hx711s_adc *h)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < h->sensor_count; i++)
        if (gpio_in_read(h->sdos[i]))
            mask |= 1 << i;
    return mask;
}

static void
hx711s_set_clocks(struct hx711s_adc *h, uint8_t value)
{
    for (uint8_t i = 0; i < h->sensor_count; i++)
        gpio_out_write(h->clks[i], value);
}

// Read all devices in lockstep.  Sample DOUT after its rising-edge setup time
// while SCK is high, then check the final post-read level before servicing
// arbitrary pending IRQ work.  This keeps every decision at a defined point
// in the data-sheet timing diagram while limiting each IRQ-off phase to 1us.
static uint8_t
hx711s_raw_read(struct hx711s_adc *h, uint32_t *bits_out, uint8_t num_bits)
{
    for (uint8_t i = 0; i < h->sensor_count; i++)
        bits_out[i] = 0;
    uint8_t post_read_low = 0;
    while (num_bits--) {
        irq_disable();
        hx711s_set_clocks(h, 1);
        hx711s_delay_noirq();
        for (uint8_t i = 0; i < h->sensor_count; i++)
            bits_out[i] = (bits_out[i] << 1) | gpio_in_read(h->sdos[i]);
        hx711s_set_clocks(h, 0);
        if (!num_bits) {
            hx711s_delay_noirq();
            for (uint8_t i = 0; i < h->sensor_count; i++)
                if (!gpio_in_read(h->sdos[i]))
                    post_read_low |= 1 << i;
        }
        irq_enable();
        hx711s_delay();
    }
    return post_read_low;
}

static uint_fast8_t
hx711s_event(struct timer *timer)
{
    struct hx711s_adc *h = container_of(timer, struct hx711s_adc, timer);
    if (h->state == HX711S_OFF)
        return SF_DONE;
    if (h->state == HX711S_RESET) {
        hx711s_set_clocks(h, 0);
        h->state = HX711S_WAIT_READY;
        h->timer.waketime += timer_from_us(STARTUP_DELAY_US);
        return SF_RESCHEDULE;
    }
    if (h->pending_flag) {
        h->sb.possible_overflows++;
        h->pending_flag |= HX711S_OVERFLOW;
        h->timer.waketime += h->rest_ticks * 4;
        return SF_RESCHEDULE;
    }
    if (!hx711s_ready_mask(h)) {
        if (h->state == HX711S_WAIT_READY)
            h->state = HX711S_SETTLING;
        h->pending_flag = 1;
        sched_wake_task(&wake_hx711s);
        h->timer.waketime += h->rest_ticks * 8;
    } else {
        h->timer.waketime += h->rest_ticks;
    }
    return SF_RESCHEDULE;
}

static void
append_value(struct hx711s_adc *h, uint32_t value)
{
    h->sb.data[h->sb.data_count] = value;
    h->sb.data[h->sb.data_count + 1] = value >> 8;
    h->sb.data[h->sb.data_count + 2] = value >> 16;
    h->sb.data[h->sb.data_count + 3] = value >> 24;
    h->sb.data_count += BYTES_PER_VALUE;
}

static void
hx711s_begin_reset(struct hx711s_adc *h)
{
    // A runtime frame fault reaches here from task context while the polling
    // timer is still linked.  Re-adding that timer corrupts the scheduler
    // list, so remove it before changing its wake time and inserting it.
    sched_del_timer(&h->timer);
    h->pending_flag = 0;
    if (h->lce)
        load_cell_probe_set_sensor_ready(h->lce, 0);
    hx711s_set_clocks(h, 1);
    h->state = HX711S_RESET;
    h->settling_frames = SETTLING_FRAMES;
    h->qualify_frames = QUALIFY_FRAMES;
    h->timing_streak = 0;
    h->timer.waketime = timer_read_time() + timer_from_us(POWERDOWN_US);
    sched_add_timer(&h->timer);
}

static void
hx711s_read_adc(struct hx711s_adc *h, uint8_t oid)
{
    uint32_t adc[MAX_SENSORS], quality = 0;
    int32_t counts[MAX_SENSORS] = {0};
    uint32_t capture_ticks = timer_read_time();
    uint8_t channel_mask = hx711s_ready_mask(h);
    uint8_t extras_mask = (1 << h->gain_channel) - 1;

    // Never clock an ADC that says its conversion is not ready.  Doing so is
    // outside the HX711 protocol and can turn a readiness fault into a second,
    // misleading gain-bit fault.  Preserve a diagnostic record and recover.
    if (channel_mask) {
        quality |= HX711S_Q_NOT_READY;
    } else {
        uint8_t post_read_low = hx711s_raw_read(
            h, adc, 24 + h->gain_channel);
        for (uint8_t i = 0; i < h->sensor_count; i++) {
            uint32_t raw = adc[i] >> h->gain_channel;
            if (raw & 0x800000)
                raw |= 0xff000000;
            counts[i] = raw;
            if ((adc[i] & extras_mask) != extras_mask) {
                quality |= HX711S_Q_EXTRA_LOW;
                channel_mask |= 1 << i;
            }
            if (post_read_low & (1 << i)) {
                quality |= HX711S_Q_POST_READ_LOW;
                channel_mask |= 1 << i;
            }
            if (counts[i] == INT32_C(0x007fffff)
                || counts[i] == -INT32_C(0x00800000)) {
                quality |= HX711S_Q_SATURATED;
                channel_mask |= 1 << i;
            }
        }
    }

    irq_disable();
    uint8_t flags = h->pending_flag;
    h->pending_flag = 0;
    irq_enable();
    if (flags & HX711S_OVERFLOW)
        quality |= HX711S_Q_READ_OVERRUN;

    uint8_t was_qualifying = h->state == HX711S_SETTLING
                             || h->state == HX711S_QUALIFY;
    if (was_qualifying) {
        if (!quality && h->state == HX711S_SETTLING && !--h->settling_frames)
            h->state = HX711S_QUALIFY;
        else if (!quality && h->state == HX711S_QUALIFY
                 && !--h->qualify_frames) {
            h->state = HX711S_ONLINE;
            if (h->lce)
                load_cell_probe_set_sensor_ready(h->lce, 1);
        }
        quality |= HX711S_Q_SETTLING;
    }
    quality |= (uint32_t)channel_mask << HX711S_Q_CHANNEL_SHIFT;

    // A persistent timing streak means a genuinely stuck ADC: tag the frame
    // hard in-band so the host records it as a fault, then reset below.
    uint8_t escalate = (quality & HX711S_Q_TIMING_MISS)
                       && !(quality & HX711S_Q_GENUINE_FAULT)
                       && h->timing_streak + 1 >= HX711S_TIMING_STREAK_LIMIT;
    if (escalate)
        quality |= HX711S_Q_TIMING_STREAK;

    // A complete sample must fit before appending it.  In the four-channel
    // case each timestamped frame is 24 bytes, while the shared bulk buffer
    // is 51 bytes: appending first when it contains 48 bytes would overrun it.
    if (h->sb.data_count + h->sample_bytes > ARRAY_SIZE(h->sb.data))
        sensor_bulk_report(&h->sb, oid);

    append_value(h, capture_ticks);
    for (uint8_t i = 0; i < h->sensor_count; i++)
        append_value(h, counts[i]);
    // Tag the timestamped wire format. A host paired with an older MCU must
    // fail closed instead of interpreting shifted count words as valid data.
    append_value(h, HX711S_FRAME_TAG | quality);

    if (quality & HX711S_Q_GENUINE_FAULT) {
        h->timing_streak = 0;
        if (h->lce)
            load_cell_probe_report_fault_at(h->lce, capture_ticks);
        if (quality & HX711S_Q_READ_OVERRUN)
            hx711s_begin_reset(h);
        return;
    }
    if (quality & HX711S_Q_TIMING_MISS) {
        if (escalate) {
            // Persistent timing failure: the ADC is genuinely stuck.
            h->timing_streak = 0;
            if (h->lce)
                load_cell_probe_report_fault_at(h->lce, capture_ticks);
            hx711s_begin_reset(h);
        } else {
            h->timing_streak++;
        }
        return;
    }
    h->timing_streak = 0;
    if (!quality && h->state == HX711S_ONLINE) {
        int32_t sum = 0;
        for (uint8_t i = 0; i < h->sensor_count; i++)
            sum += counts[i];
        if (h->lce)
            load_cell_probe_report_sample_at(h->lce, sum, capture_ticks);
    }
}

void
command_config_hx711s(uint32_t *args)
{
    struct hx711s_adc *h = oid_alloc(args[0], command_config_hx711s,
                                      sizeof(*h));
    h->timer.func = hx711s_event;
    if (args[1] < 1 || args[1] > MAX_SENSORS)
        shutdown("hx711s: sensor_count must be 1-4");
    if (args[2] < 1 || args[2] > 4)
        shutdown("hx711s: gain_channel out of range 1-4");
    h->sensor_count = args[1];
    h->gain_channel = args[2];
    // Each frame carries its MCU capture clock, all channel counts, and a
    // quality word.  The largest (four-channel) frame is 24 bytes, so exactly
    // two frames fit in the shared 51-byte bulk buffer.
    h->sample_bytes = BYTES_PER_VALUE * (h->sensor_count + 2);
    if (h->sample_bytes > ARRAY_SIZE(h->sb.data))
        shutdown("hx711s: sample does not fit bulk buffer");
    h->state = HX711S_OFF;
}
DECL_COMMAND(command_config_hx711s,
    "config_hx711s oid=%c sensor_count=%c gain_channel=%c");

void
command_add_hx711s(uint32_t *args)
{
    struct hx711s_adc *h = oid_lookup(args[0], command_config_hx711s);
    if (args[1] >= h->sensor_count)
        shutdown("hx711s: sensor index out of range");
    h->sdos[args[1]] = gpio_in_setup(args[2], 1);
    h->clks[args[1]] = gpio_out_setup(args[3], 1);
}
DECL_COMMAND(command_add_hx711s,
    "add_hx711s oid=%c index=%c sdo_pin=%u sclk_pin=%u");

void
hx711s_attach_load_cell_probe(uint32_t *args)
{
    struct hx711s_adc *h = oid_lookup(args[0], command_config_hx711s);
    h->lce = load_cell_probe_oid_lookup(args[1]);
    load_cell_probe_set_sensor_ready(h->lce, h->state == HX711S_ONLINE);
}
DECL_COMMAND(hx711s_attach_load_cell_probe,
    "hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

void
command_query_hx711s(uint32_t *args)
{
    struct hx711s_adc *h = oid_lookup(args[0], command_config_hx711s);
    h->rest_ticks = args[1];
    if (!h->rest_ticks) {
        sched_del_timer(&h->timer);
        irq_disable();
        h->state = HX711S_OFF;
        h->pending_flag = 0;
        irq_enable();
        hx711s_set_clocks(h, 1);
        if (h->lce)
            load_cell_probe_set_sensor_ready(h->lce, 0);
        return;
    }
    sensor_bulk_reset(&h->sb);
    hx711s_begin_reset(h);
}
DECL_COMMAND(command_query_hx711s, "query_hx711s oid=%c rest_ticks=%u");

void
command_query_hx711s_status(const uint32_t *args)
{
    struct hx711s_adc *h = oid_lookup(args[0], command_config_hx711s);
    uint32_t start_t = timer_read_time();
    uint8_t pending_bytes = !hx711s_ready_mask(h) ? h->sample_bytes : 0;
    sensor_bulk_status(&h->sb, args[0], start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_hx711s_status, "query_hx711s_status oid=%c");

void
hx711s_capture_task(void)
{
    if (!sched_check_wake(&wake_hx711s))
        return;
    uint8_t oid;
    struct hx711s_adc *h;
    foreach_oid(oid, h, command_config_hx711s) {
        if (h->state != HX711S_OFF && h->pending_flag)
            hx711s_read_adc(h, oid);
    }
}
DECL_TASK(hx711s_capture_task);
