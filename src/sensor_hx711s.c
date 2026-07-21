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
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include <stdint.h>

#define MAX_SENSORS 4
#define BYTES_PER_SAMPLE 4
#define SAMPLE_ERROR_DESYNC (1L << 31)
#define SAMPLE_ERROR_READ_TOO_LONG (1L << 30)
#define SAMPLE_ERROR_TORN_READ (1L << 29)
#define HX711S_OVERFLOW (1 << 1)

// A corrupt chip frame can pass the framing checks yet carry a value
// hundreds of thousands of counts from the truth. Real contact force moves
// each channel by well under this between samples (a 2mm/s approach at 80
// SPS), so a larger single-sample jump is a glitch, not a load change.
#define SPIKE_CHANNEL_THRESHOLD 100000

struct hx711s_adc {
    struct timer timer;
    uint32_t rest_ticks;
    uint32_t last_error;
    int32_t last_good_counts[MAX_SENSORS];
    int32_t pending_counts[MAX_SENSORS];
    uint8_t have_last_counts;
    uint8_t have_pending_counts;
    uint8_t pending_flag;
    uint8_t sensor_count;
    uint8_t gain_channel;   // extra clock pulses: chip type + gain selection
    uint8_t sample_bytes;   // sensor_count * BYTES_PER_SAMPLE
    struct gpio_in  sdos[MAX_SENSORS];
    struct gpio_out clks[MAX_SENSORS];
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

static struct task_wake wake_hx711s;


/****************************************************************
 * Low-level bit-banging
 ****************************************************************/

#define MIN_PULSE_TIME nsecs_to_ticks(200)

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static void
hx711s_delay_noirq(void)
{
    if (CONFIG_MACH_AVR) {
        asm("nop\n    nop");
        return;
    }
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        ;
}

static void
hx711s_delay(void)
{
    if (CONFIG_MACH_AVR)
        return;
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Read num_bits from all configured chips in lockstep. All SCKs toggle
// together so each chip's post-read analog conversion starts at the same
// instant and sees no further SCK activity during its sample window.
static void
hx711s_raw_read(struct hx711s_adc *h, uint32_t *bits_out, int num_bits)
{
    uint8_t n = h->sensor_count;
    for (uint8_t i = 0; i < n; i++)
        bits_out[i] = 0;
    while (num_bits--) {
        irq_disable();
        for (uint8_t i = 0; i < n; i++)
            gpio_out_toggle_noirq(h->clks[i]);
        hx711s_delay_noirq();
        for (uint8_t i = 0; i < n; i++)
            gpio_out_toggle_noirq(h->clks[i]);
        for (uint8_t i = 0; i < n; i++)
            bits_out[i] = (bits_out[i] << 1) | gpio_in_read(h->sdos[i]);
        irq_enable();
        hx711s_delay();
    }
}


/****************************************************************
 * HX711S sensor support
 ****************************************************************/

static uint_fast8_t
hx711s_is_data_ready(struct hx711s_adc *h)
{
    // All chips must have DOUT low before reading; they share a RATE pin but
    // may not be perfectly phase-aligned, so check every chip.
    for (uint8_t i = 0; i < h->sensor_count; i++) {
        if (gpio_in_read(h->sdos[i]))
            return 0;
    }
    return 1;
}

static uint_fast8_t
hx711s_event(struct timer *timer)
{
    struct hx711s_adc *h = container_of(timer, struct hx711s_adc, timer);
    uint32_t rest_ticks = h->rest_ticks;
    if (h->pending_flag) {
        h->sb.possible_overflows++;
        h->pending_flag |= HX711S_OVERFLOW;
        rest_ticks *= 4;
    } else if (hx711s_is_data_ready(h)) {
        h->pending_flag = 1;
        sched_wake_task(&wake_hx711s);
        rest_ticks *= 8;
    }
    h->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
append_sample(struct hx711s_adc *h, int32_t val)
{
    h->sb.data[h->sb.data_count]     = val;
    h->sb.data[h->sb.data_count + 1] = val >> 8;
    h->sb.data[h->sb.data_count + 2] = val >> 16;
    h->sb.data[h->sb.data_count + 3] = val >> 24;
    h->sb.data_count += BYTES_PER_SAMPLE;
}

static void
hx711s_read_adc(struct hx711s_adc *h, uint8_t oid)
{
    uint_fast8_t gain_channel = h->gain_channel;
    uint_fast8_t extras_mask = (1 << gain_channel) - 1;
    int32_t counts_buf[MAX_SENSORS];
    uint32_t adc[MAX_SENSORS];
    uint32_t sample_error = 0;

    // The chips free-run on independent oscillators, so a chip may latch a
    // new conversion into its output register between the data-ready poll
    // and this read, or during the read itself. Bits clocked out across a
    // latch are torn (often all-ones) with no other error signature.
    // Detect this by verifying every DOUT is still low immediately before
    // the read and back high immediately after; a low DOUT after the read
    // means that chip latched new data mid-read.
    uint32_t torn_read = !hx711s_is_data_ready(h);

    hx711s_raw_read(h, adc, 24 + gain_channel);

    hx711s_delay();
    for (uint8_t i = 0; i < h->sensor_count; i++) {
        if (!gpio_in_read(h->sdos[i]))
            torn_read = 1;
    }

    for (uint8_t i = 0; i < h->sensor_count; i++) {
        uint32_t raw = adc[i] >> gain_channel;
        if (raw & 0x800000)
            raw |= 0xFF000000;
        counts_buf[i] = (int32_t)raw;
        if ((adc[i] & extras_mask) != extras_mask)
            sample_error = SAMPLE_ERROR_DESYNC;
    }

    // Capture and clear pending flag after all reads
    irq_disable();
    uint8_t flags = h->pending_flag;
    h->pending_flag = 0;
    irq_enable();

    if (flags & HX711S_OVERFLOW)
        sample_error = SAMPLE_ERROR_READ_TOO_LONG;

    // Torn reads are a normal consequence of chip oscillator drift: discard
    // just that sample, the next conversion is valid. They can also corrupt
    // the extra-bit check, so they must take priority over a desync. A real
    // desync corrupts the serial protocol state, making all further values
    // unreliable until the chips are power cycled: latch it and keep
    // reporting it so the host restarts the sensor.
    if (torn_read)
        sample_error = SAMPLE_ERROR_TORN_READ;
    else if (sample_error)
        h->last_error = sample_error;

    if (h->last_error)
        sample_error = h->last_error;

    if (sample_error) {
        for (uint8_t i = 0; i < h->sensor_count; i++)
            append_sample(h, (int32_t)sample_error);
    } else {
        int32_t sum = 0;
        for (uint8_t i = 0; i < h->sensor_count; i++) {
            append_sample(h, counts_buf[i]);
            sum += counts_buf[i];
        }
        // Reject a single-sample glitch on any channel before forwarding the
        // summed sample to the MCU probe. Keep the raw value in the bulk
        // buffer so the host can identify the bad channel. A second sample
        // near the suspicious value confirms a genuine sustained force step;
        // an immediate return to the last good value confirms an isolated
        // glitch. This prevents a one-frame corrupt read from tripping the
        // drift safety range without filtering a real collision forever.
        uint8_t bad_mask = 0;
        if (!h->have_last_counts) {
            for (uint8_t i = 0; i < h->sensor_count; i++)
                h->last_good_counts[i] = counts_buf[i];
            h->have_last_counts = 1;
            if (h->lce)
                load_cell_probe_report_sample(h->lce, sum);
        } else {
            for (uint8_t i = 0; i < h->sensor_count; i++) {
                int32_t delta = counts_buf[i] - h->last_good_counts[i];
                if (delta > SPIKE_CHANNEL_THRESHOLD
                    || delta < -SPIKE_CHANNEL_THRESHOLD)
                    bad_mask |= 1 << i;
            }

            if (!bad_mask) {
                // A valid sample confirms that the pending sample was an
                // isolated glitch. Forward this valid sample normally.
                h->have_pending_counts = 0;
                for (uint8_t i = 0; i < h->sensor_count; i++)
                    h->last_good_counts[i] = counts_buf[i];
                if (h->lce)
                    load_cell_probe_report_sample(h->lce, sum);
            } else if (h->have_pending_counts) {
                uint8_t pending_match = 1;
                for (uint8_t i = 0; i < h->sensor_count; i++) {
                    int32_t delta = counts_buf[i] - h->pending_counts[i];
                    if (delta > SPIKE_CHANNEL_THRESHOLD
                        || delta < -SPIKE_CHANNEL_THRESHOLD) {
                        pending_match = 0;
                        break;
                    }
                }
                if (pending_match) {
                    // Two consecutive samples agree on the new value. Treat
                    // this as a real force change so it cannot be filtered
                    // indefinitely.
                    h->have_pending_counts = 0;
                    for (uint8_t i = 0; i < h->sensor_count; i++)
                        h->last_good_counts[i] = counts_buf[i];
                    if (h->lce)
                        load_cell_probe_report_sample(h->lce, sum);
                } else {
                    // Replace a stale pending glitch with the latest sample.
                    for (uint8_t i = 0; i < h->sensor_count; i++)
                        h->pending_counts[i] = counts_buf[i];
                }
            } else {
                for (uint8_t i = 0; i < h->sensor_count; i++)
                    h->pending_counts[i] = counts_buf[i];
                h->have_pending_counts = 1;
            }
        }
    }

    // Flush buffer if another sample would overflow it
    if (h->sb.data_count + h->sample_bytes > ARRAY_SIZE(h->sb.data))
        sensor_bulk_report(&h->sb, oid);
}

void
command_config_hx711s(uint32_t *args)
{
    struct hx711s_adc *h = oid_alloc(args[0], command_config_hx711s,
                                      sizeof(*h));
    h->timer.func = hx711s_event;
    uint8_t sensor_count = args[1];
    if (sensor_count < 1 || sensor_count > MAX_SENSORS)
        shutdown("hx711s: sensor_count must be 1-4");
    h->sensor_count = sensor_count;
    uint8_t gain_channel = args[2];
    if (gain_channel < 1 || gain_channel > 4)
        shutdown("hx711s: gain_channel out of range 1-4");
    h->gain_channel = gain_channel;
    h->sample_bytes = BYTES_PER_SAMPLE * sensor_count;
}
DECL_COMMAND(command_config_hx711s,
    "config_hx711s oid=%c sensor_count=%c gain_channel=%c");

void
command_add_hx711s(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *h = oid_lookup(oid, command_config_hx711s);
    uint8_t index = args[1];
    if (index >= h->sensor_count)
        shutdown("hx711s: sensor index out of range");
    h->sdos[index] = gpio_in_setup(args[2], 1);
    h->clks[index] = gpio_out_setup(args[3], 0);
    gpio_out_write(h->clks[index], 1); // put chip in power down state
}
DECL_COMMAND(command_add_hx711s,
    "add_hx711s oid=%c index=%c sdo_pin=%u sclk_pin=%u");

void
hx711s_attach_load_cell_probe(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *h = oid_lookup(oid, command_config_hx711s);
    h->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(hx711s_attach_load_cell_probe,
    "hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");

void
command_query_hx711s(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *h = oid_lookup(oid, command_config_hx711s);
    sched_del_timer(&h->timer);
    h->pending_flag = 0;
    h->last_error = 0;
    h->have_last_counts = 0;
    h->have_pending_counts = 0;
    h->rest_ticks = args[1];
    if (!h->rest_ticks) {
        for (uint8_t i = 0; i < h->sensor_count; i++)
            gpio_out_write(h->clks[i], 1); // power down all chips
        return;
    }
    for (uint8_t i = 0; i < h->sensor_count; i++)
        gpio_out_write(h->clks[i], 0); // wake all chips
    sensor_bulk_reset(&h->sb);
    // HX711 needs up to ~400ms (typ. 10-20ms) to settle after PD release.
    // Wait 50ms before first poll so the first DRDY isn't from a still-
    // stabilising chip (which can produce a spurious DESYNC).
    irq_disable();
    h->timer.waketime = timer_read_time() + timer_from_us(50000);
    sched_add_timer(&h->timer);
    irq_enable();
}
DECL_COMMAND(command_query_hx711s, "query_hx711s oid=%c rest_ticks=%u");

void
command_query_hx711s_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *h = oid_lookup(oid, command_config_hx711s);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    uint8_t is_ready = hx711s_is_data_ready(h);
    irq_enable();
    uint8_t pending_bytes = is_ready ? h->sample_bytes : 0;
    sensor_bulk_status(&h->sb, oid, start_t, 0, pending_bytes);
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
        if (h->pending_flag)
            hx711s_read_adc(h, oid);
    }
}
DECL_TASK(hx711s_capture_task);
