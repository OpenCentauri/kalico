// Support for multi-sensor HX711 and HX717 ADC chips
//
// Copyright (C) 2026 James Turton <james.turton@gmx.com>
// Original HX711 driver Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
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
#include <stdbool.h>
#include <stdint.h>

#define MAX_SENSORS 4

struct hx711s_adc;

// Each chip runs its own read state machine. The chips free-run on their
// own oscillators with no way to synchronise them, so a chip is polled and
// read on its own data ready edge and never waits on a sibling. Reading a
// chip only when every chip is ready would leave the earliest one holding
// its result until the drift between oscillators pushed that wait onto its
// next conversion, tearing the frame being clocked out.
struct hx711s_chip {
    struct timer timer;
    struct hx711s_adc *adc; // the sensor this chip belongs to
    struct gpio_in dout; // pin used to receive data from the hx711s
    struct gpio_out sclk; // pin used to generate clock for the hx711s
    int32_t counts; // most recent reading, held until the chip is read again
    uint8_t index;
    uint8_t flags;
    uint8_t bad_frame; // last read produced nothing usable, counts still holds
    uint8_t settle_remaining; // conversions still to discard after a wake
};

struct hx711s_adc {
    uint8_t gain_channel;   // the gain+channel selection (1-4)
    uint8_t sensor_count;   // number of chips in use (1-4)
    uint8_t sample_bytes;   // bytes in one multi-channel sample
    uint8_t chip_mask;      // bit per configured chip
    uint8_t have_counts;    // chips that have produced a reading
    uint32_t rest_ticks;
    uint32_t last_error;
    struct hx711s_chip chips[MAX_SENSORS];
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
};

enum {
    HX_PENDING = 1<<0, HX_OVERFLOW = 1<<1,
};

#define BYTES_PER_SAMPLE 4
#define SETTLE_CONVERSIONS 4
#define SAMPLE_ERROR_DESYNC 1L << 31
#define SAMPLE_ERROR_READ_TOO_LONG 1L << 30
#define SAMPLE_ERROR_BAD_FRAME 1L << 29

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

// Pause for 200ns
static void
hx711s_delay_noirq(void)
{
    if (CONFIG_MACH_AVR) {
        // Optimize avr, as calculating time takes longer than needed delay
        asm("nop\n    nop");
        return;
    }
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Pause for a minimum of 200ns
static void
hx711s_delay(void)
{
    if (CONFIG_MACH_AVR)
        // Optimize avr, as calculating time takes longer than needed delay
        return;
    uint32_t end = timer_read_time() + MIN_PULSE_TIME;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Read 'num_bits' from one chip
static uint32_t
hx711s_raw_read(struct gpio_in dout, struct gpio_out sclk, int num_bits)
{
    uint32_t bits_read = 0;
    while (num_bits--) {
        irq_disable();
        gpio_out_toggle_noirq(sclk);
        hx711s_delay_noirq();
        gpio_out_toggle_noirq(sclk);
        uint_fast8_t bit = gpio_in_read(dout);
        irq_enable();
        hx711s_delay();
        bits_read = (bits_read << 1) | bit;
    }
    return bits_read;
}


/****************************************************************
 * Multi-sensor HX711 and HX717 Support
 ****************************************************************/

// Check if data is ready
static uint_fast8_t
hx711s_is_data_ready(struct hx711s_chip *chip)
{
    return !gpio_in_read(chip->dout);
}

// Event handler that wakes wake_hx711s() periodically. One timer per chip,
// so a chip that is slow to convert cannot delay polling of the others.
static uint_fast8_t
hx711s_event(struct timer *timer)
{
    struct hx711s_chip *chip = container_of(timer, struct hx711s_chip, timer);
    struct hx711s_adc *hx711s = chip->adc;
    uint32_t rest_ticks = hx711s->rest_ticks;
    uint8_t flags = chip->flags;
    if (flags & HX_PENDING) {
        hx711s->sb.possible_overflows++;
        chip->flags = HX_PENDING | HX_OVERFLOW;
        rest_ticks *= 4;
    } else if (hx711s_is_data_ready(chip)) {
        // New sample pending
        chip->flags = HX_PENDING;
        sched_wake_task(&wake_hx711s);
        rest_ticks *= 8;
    }
    chip->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_counts(struct hx711s_adc *hx711s, uint32_t counts)
{
    hx711s->sb.data[hx711s->sb.data_count] = counts;
    hx711s->sb.data[hx711s->sb.data_count + 1] = counts >> 8;
    hx711s->sb.data[hx711s->sb.data_count + 2] = counts >> 16;
    hx711s->sb.data[hx711s->sb.data_count + 3] = counts >> 24;
    hx711s->sb.data_count += BYTES_PER_SAMPLE;
}

// The load cell is the sum of every chip, the value the probe triggers on
static int32_t
hx711s_sum(struct hx711s_adc *hx711s)
{
    int32_t sum = 0;
    for (uint8_t i = 0; i < hx711s->sensor_count; i++)
        sum += hx711s->chips[i].counts;
    return sum;
}

// Emit one sample carrying the latest reading from every chip
static void
add_sample(struct hx711s_adc *hx711s, uint8_t oid, uint8_t force_flush)
{
    // Add measurement to buffer
    for (uint8_t i = 0; i < hx711s->sensor_count; i++) {
        struct hx711s_chip *chip = &hx711s->chips[i];
        uint32_t counts = chip->bad_frame ? SAMPLE_ERROR_BAD_FRAME
                                          : (uint32_t)chip->counts;

        // forever send errors until reset
        if (hx711s->last_error != 0) {
            counts = hx711s->last_error;
        }

        add_counts(hx711s, counts);
    }

    if (hx711s->sb.data_count + hx711s->sample_bytes
        > ARRAY_SIZE(hx711s->sb.data) || force_flush)
        sensor_bulk_report(&hx711s->sb, oid);
}

// hx711s ADC query
static void
hx711s_read_adc(struct hx711s_chip *chip)
{
    struct hx711s_adc *hx711s = chip->adc;

    // Read from sensor
    uint_fast8_t gain_channel = hx711s->gain_channel;
    uint32_t adc = hx711s_raw_read(chip->dout, chip->sclk, 24 + gain_channel);

    // Clear pending flag (and note if an overflow occurred)
    irq_disable();
    uint8_t flags = chip->flags;
    chip->flags = 0;
    irq_enable();

    // Discard the first 4 conversions after a wake as it hasn't settled yet
    if (chip->settle_remaining) {
        chip->settle_remaining--;
        return;
    }

    // Extract report from raw data
    uint32_t counts = adc >> gain_channel;
    if (counts & 0x800000)
        counts |= 0xFF000000;

    // Check for errors
    uint_fast8_t extras_mask = (1 << gain_channel) - 1;
    if ((adc & extras_mask) != extras_mask) {
        // Transfer did not complete correctly
        hx711s->last_error = SAMPLE_ERROR_DESYNC;
    } else if (counts == 0xFFFFFFFF) {
        // DOUT stayed high for the whole frame, so the chip was not
        // presenting data at all: it reset, browned out or lost its ground.
        // This frame cannot be caught by the check above because the gain
        // bits are set too, and it sign extends to -1 counts, which is near
        // enough to a tared reading to pass a range check and fire a false
        // trigger. A stuck low line instead reads as zero and is caught as
        // a desync. Hold this chip's previous value rather than latch an
        // error, so a single disturbed frame does not stop the sensor.
        chip->bad_frame = 1;
    } else if (flags & HX_OVERFLOW) {
        // Transfer took too long
        hx711s->last_error = SAMPLE_ERROR_READ_TOO_LONG;
    } else {
        chip->bad_frame = 0;
        chip->counts = (int32_t)counts;
        hx711s->have_counts |= 1 << chip->index;
    }
}

// Create a hx711s sensor
void
command_config_hx711s(uint32_t *args)
{
    struct hx711s_adc *hx711s = oid_alloc(args[0]
                , command_config_hx711s, sizeof(*hx711s));
    uint8_t sensor_count = args[1];
    if (sensor_count < 1 || sensor_count > MAX_SENSORS) {
        shutdown("HX711S sensor count out of range 1-4");
    }
    hx711s->sensor_count = sensor_count;
    hx711s->sample_bytes = sensor_count * BYTES_PER_SAMPLE;
    hx711s->chip_mask = (1 << sensor_count) - 1;
    uint8_t gain_channel = args[2];
    if (gain_channel < 1 || gain_channel > 4) {
        shutdown("HX711S gain/channel out of range 1-4");
    }
    hx711s->gain_channel = gain_channel;
    for (uint8_t i = 0; i < sensor_count; i++) {
        struct hx711s_chip *chip = &hx711s->chips[i];
        chip->timer.func = hx711s_event;
        chip->adc = hx711s;
        chip->index = i;
    }
}
DECL_COMMAND(command_config_hx711s, "config_hx711s oid=%c sensor_count=%c"
             " gain_channel=%c");

// Assign the pins of one chip
void
command_add_hx711s(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *hx711s = oid_lookup(oid, command_config_hx711s);
    uint8_t index = args[1];
    if (index >= hx711s->sensor_count) {
        shutdown("HX711S sensor index out of range");
    }
    struct hx711s_chip *chip = &hx711s->chips[index];
    chip->dout = gpio_in_setup(args[2], 1);
    chip->sclk = gpio_out_setup(args[3], 0);
    gpio_out_write(chip->sclk, 1); // put chip in power down state
}
DECL_COMMAND(command_add_hx711s, "add_hx711s oid=%c index=%c"
             " sdo_pin=%u sclk_pin=%u");

void
hx711s_attach_load_cell_probe(uint32_t *args) {
    uint8_t oid = args[0];
    struct hx711s_adc *hx711s = oid_lookup(oid, command_config_hx711s);
    hx711s->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(hx711s_attach_load_cell_probe, "hx711s_attach_load_cell_probe"
    " oid=%c load_cell_probe_oid=%c");

// start/stop capturing ADC data
void
command_query_hx711s(uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *hx711s = oid_lookup(oid, command_config_hx711s);
    for (uint8_t i = 0; i < hx711s->sensor_count; i++) {
        sched_del_timer(&hx711s->chips[i].timer);
        hx711s->chips[i].flags = 0;
        hx711s->chips[i].bad_frame = 0;
    }
    hx711s->last_error = 0;
    hx711s->have_counts = 0;
    hx711s->rest_ticks = args[1];
    if (!hx711s->rest_ticks) {
        // End measurements
        for (uint8_t i = 0; i < hx711s->sensor_count; i++)
            gpio_out_write(hx711s->chips[i].sclk, 1); // power down state
        return;
    }
    // Start new measurements
    for (uint8_t i = 0; i < hx711s->sensor_count; i++)
        gpio_out_write(hx711s->chips[i].sclk, 0); // wake chip from power down
    sensor_bulk_reset(&hx711s->sb);
    irq_disable();
    uint32_t waketime = timer_read_time() + hx711s->rest_ticks;
    for (uint8_t i = 0; i < hx711s->sensor_count; i++) {
        hx711s->chips[i].settle_remaining = SETTLE_CONVERSIONS;
        hx711s->chips[i].timer.waketime = waketime;
        sched_add_timer(&hx711s->chips[i].timer);
    }
    irq_enable();
}
DECL_COMMAND(command_query_hx711s, "query_hx711s oid=%c rest_ticks=%u");

void
command_query_hx711s_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct hx711s_adc *hx711s = oid_lookup(oid, command_config_hx711s);
    irq_disable();
    const uint32_t start_t = timer_read_time();
    // The primary chip paces the sample stream, so only a conversion pending
    // on that chip is a sample the host has not been sent yet
    uint8_t is_data_ready = hx711s_is_data_ready(&hx711s->chips[0]);
    irq_enable();
    uint8_t pending_bytes = is_data_ready ? hx711s->sample_bytes : 0;
    sensor_bulk_status(&hx711s->sb, oid, start_t, 0, pending_bytes);
}
DECL_COMMAND(command_query_hx711s_status, "query_hx711s_status oid=%c");

// Background task that performs measurements
void
hx711s_capture_task(void)
{
    if (!sched_check_wake(&wake_hx711s))
        return;
    uint8_t oid;
    struct hx711s_adc *hx711s;
    foreach_oid(oid, hx711s, command_config_hx711s) {
        // Read every chip holding data before emitting, so a sample carries
        // the freshest reading available from each of them
        uint8_t read_primary = 0;
        for (uint8_t i = 0; i < hx711s->sensor_count; i++) {
            if (!hx711s->chips[i].flags)
                continue;
            hx711s_read_adc(&hx711s->chips[i]);
            if (i == 0)
                read_primary = 1;
        }
        // The primary chip sets the sample rate. Its conversions are evenly
        // spaced, which is what the host clock tracking assumes, while the
        // remaining channels are held at their most recent reading. Wait for
        // every chip to report once so no sample carries an unset channel.
        if (read_primary && (hx711s->last_error
                             || hx711s->have_counts == hx711s->chip_mask)) {
            // probe is optional, report if enabled. Reporting here rather
            // than on each chip read keeps the probe's filter running at the
            // rate its coefficients were designed for. A bad frame is still
            // reported: the held counts are at most one conversion old, far
            // too little to move the force past the trigger, and withholding
            // reports would starve the probe watchdog and abort the homing
            // move instead.
            if (hx711s->last_error == 0 && hx711s->lce) {
                load_cell_probe_report_sample(hx711s->lce,
                                              hx711s_sum(hx711s));
            }
            // Add measurement to buffer
            add_sample(hx711s, oid, false);
        }
    }
}
DECL_TASK(hx711s_capture_task);
