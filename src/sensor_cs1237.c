// CS1237 ADC sensor support.
//
// The CS1237 uses a half-duplex data-ready/data pin.  Keep the Klipper
// command and bulk-reporting interface separate from the wire protocol so
// this remains compatible with the Kalico load-cell probe code.

#include <stdint.h>

#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "basecmd.h"
#include "command.h"
#include "load_cell_probe.h"
#include "sched.h"
#include "sensor_bulk.h"

#define CS1237_BITS 24
#define CS1237_CFG_SPEED_SHIFT 4
#define CS1237_CFG_PGA_SHIFT 2
#define CS1237_SAMPLE_BYTES 4

#define SAMPLE_ERROR_DESYNC ((uint32_t)0x80000000)
#define SAMPLE_ERROR_READ_TOO_LONG ((uint32_t)0x40000000)

struct cs1237_sensor {
    struct timer timer;
    struct gpio_in drdy_pin;
    struct gpio_out dio_pin;
    struct gpio_out sclk_pin;
    uint8_t gain;
    uint8_t speed;
    uint32_t rest_ticks;
    uint32_t last_error;
    uint8_t flags;
    struct sensor_bulk sb;
    struct load_cell_probe *lcp;
};

enum {
    FLAG_PENDING = 1 << 0,
    FLAG_OVERFLOW = 1 << 1,
};

static struct task_wake cs1237_wake;

void command_config_cs1237(uint32_t *args);

static void
cs1237_delay(void)
{
    uint32_t end = timer_read_time() + timer_from_us(1);
    while (timer_is_before(timer_read_time(), end))
        ;
}

static void
cs1237_clock(struct cs1237_sensor *cs)
{
    irq_disable();
    gpio_out_toggle_noirq(cs->sclk_pin);
    cs1237_delay();
    gpio_out_toggle_noirq(cs->sclk_pin);
    irq_enable();
    cs1237_delay();
}

static void
cs1237_clock_count(struct cs1237_sensor *cs, uint8_t count)
{
    while (count--)
        cs1237_clock(cs);
}

static uint8_t
cs1237_clock_read(struct cs1237_sensor *cs)
{
    irq_disable();
    gpio_out_toggle_noirq(cs->sclk_pin);
    cs1237_delay();
    uint8_t bit = gpio_in_read(cs->drdy_pin);
    gpio_out_toggle_noirq(cs->sclk_pin);
    irq_enable();
    cs1237_delay();
    return bit;
}

static void
cs1237_clock_write(struct cs1237_sensor *cs, uint8_t bit)
{
    gpio_out_write(cs->dio_pin, bit);
    cs1237_clock(cs);
}

static int32_t
cs1237_read_sample(struct cs1237_sensor *cs)
{
    uint32_t raw = 0;
    for (int i = 0; i < CS1237_BITS; i++)
        raw = (raw << 1) | cs1237_clock_read(cs);

    // Three trailing clocks release DRDY and leave the converter idle.
    cs1237_clock_count(cs, 3);
    if (raw & 0x800000)
        raw |= 0xff000000;
    return (int32_t)raw;
}

static void
cs1237_write_config(struct cs1237_sensor *cs)
{
    uint8_t config = ((cs->speed & 0x3) << CS1237_CFG_SPEED_SHIFT)
                   | ((cs->gain & 0x3) << CS1237_CFG_PGA_SHIFT);
    const uint8_t command = 0x65;

    // Flush the current conversion and enter the 46-clock config window.
    cs1237_clock_count(cs, 27);

    // DIO is input for DRDY/data and output for this command/register write.
    gpio_out_reset(cs->dio_pin, 1);
    cs1237_clock_count(cs, 2);
    for (int i = 6; i >= 0; i--)
        cs1237_clock_write(cs, (command >> i) & 1);
    for (int i = 7; i >= 0; i--)
        cs1237_clock_write(cs, (config >> i) & 1);

    // Return the shared DIO pin to input mode before the next conversion.
    gpio_in_reset(cs->drdy_pin, 1);
    cs1237_clock(cs);
    gpio_out_write(cs->sclk_pin, 0);
}

static uint_fast8_t
cs1237_is_data_ready(struct cs1237_sensor *cs)
{
    return !gpio_in_read(cs->drdy_pin);
}

static uint_fast8_t
cs1237_event(struct timer *timer)
{
    struct cs1237_sensor *cs = container_of(timer, struct cs1237_sensor,
                                            timer);
    uint32_t rest_ticks = cs->rest_ticks;
    if (cs->flags & FLAG_PENDING) {
        cs->flags = FLAG_PENDING | FLAG_OVERFLOW;
        rest_ticks *= 4;
    } else if (cs1237_is_data_ready(cs)) {
        cs->flags = FLAG_PENDING;
        sched_wake_task(&cs1237_wake);
        rest_ticks *= 8;
    }
    cs->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
cs1237_add_sample(struct cs1237_sensor *cs, uint8_t oid, int32_t sample)
{
    if (cs->sb.data_count + CS1237_SAMPLE_BYTES > ARRAY_SIZE(cs->sb.data))
        sensor_bulk_report(&cs->sb, oid);
    cs->sb.data[cs->sb.data_count++] = sample;
    cs->sb.data[cs->sb.data_count++] = sample >> 8;
    cs->sb.data[cs->sb.data_count++] = sample >> 16;
    cs->sb.data[cs->sb.data_count++] = sample >> 24;
}

static void
cs1237_capture(struct cs1237_sensor *cs, uint8_t oid)
{
    int32_t sample;
    uint8_t flags;
    if (!cs1237_is_data_ready(cs))
        cs->last_error = SAMPLE_ERROR_DESYNC;
    sample = cs->last_error ? (int32_t)cs->last_error
                            : cs1237_read_sample(cs);

    irq_disable();
    flags = cs->flags;
    cs->flags = 0;
    irq_enable();
    if (flags & FLAG_OVERFLOW)
        cs->last_error = SAMPLE_ERROR_READ_TOO_LONG;
    if (cs->last_error)
        sample = (int32_t)cs->last_error;
    else if (cs->lcp)
        load_cell_probe_report_sample(cs->lcp, sample);
    cs1237_add_sample(cs, oid, sample);
}

void
cs1237_capture_task(void)
{
    if (!sched_check_wake(&cs1237_wake))
        return;
    uint8_t oid;
    struct cs1237_sensor *cs;
    foreach_oid(oid, cs, command_config_cs1237) {
        if (cs->flags & FLAG_PENDING)
            cs1237_capture(cs, oid);
    }
}
DECL_TASK(cs1237_capture_task);

void
command_config_cs1237(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_alloc(
        args[0], command_config_cs1237, sizeof(*cs));
    cs->gain = args[1];
    cs->speed = args[2];
    cs->dio_pin = gpio_out_setup(args[3], 1);
    cs->drdy_pin = gpio_in_setup(args[3], 1);
    cs->sclk_pin = gpio_out_setup(args[4], 0);

    uint32_t timeout = timer_read_time() + timer_from_us(500000);
    while (!cs1237_is_data_ready(cs)) {
        if (timer_is_before(timeout, timer_read_time()))
            shutdown("CS1237 not responding");
    }
    cs1237_write_config(cs);
}
DECL_COMMAND(command_config_cs1237,
             "config_cs1237 oid=%c gain=%c speed=%c drdy_pin=%u sclk_pin=%u");

void
command_query_cs1237(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    sched_del_timer(&cs->timer);
    cs->flags = 0;
    cs->last_error = 0;
    cs->rest_ticks = args[1];
    if (!cs->rest_ticks) {
        sensor_bulk_reset(&cs->sb);
        return;
    }
    sensor_bulk_reset(&cs->sb);
    cs->timer.waketime = timer_read_time() + cs->rest_ticks;
    cs->timer.func = cs1237_event;
    sched_add_timer(&cs->timer);
}
DECL_COMMAND(command_query_cs1237, "query_cs1237 oid=%c rest_ticks=%u");

void
command_query_cs1237_status(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    irq_disable();
    uint8_t pending_bytes = cs1237_is_data_ready(cs) ? CS1237_SAMPLE_BYTES : 0;
    uint32_t clock = timer_read_time();
    struct sensor_bulk sb = cs->sb;
    irq_enable();
    sensor_bulk_status(&sb, args[0], clock, 0, pending_bytes);
}
DECL_COMMAND(command_query_cs1237_status, "query_cs1237_status oid=%c");

void
command_cs1237_attach_load_cell_probe(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    cs->lcp = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(command_cs1237_attach_load_cell_probe,
             "cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c");
