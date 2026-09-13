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
#define CS1237_SAMPLE_BYTES 4
#define CS1237_CONFIG_READ 0x56
#define CS1237_CONFIG_WRITE 0x65
#define CS1237_CONFIG_TIMEOUT_US 350000

#define SAMPLE_ERROR_DESYNC ((uint32_t)0x80000000)
#define SAMPLE_ERROR_READ_TOO_LONG ((uint32_t)0x40000000)

struct cs1237_sensor {
    struct timer timer;
    struct gpio_in drdy_pin;
    struct gpio_out dio_pin;
    struct gpio_out sclk_pin;
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

    // Clock 25 is configuration-write status; clock 26 is reserved low;
    // clock 27 must release DRDY/DOUT high before the next conversion.
    (void)cs1237_clock_read(cs);
    uint8_t reserved = cs1237_clock_read(cs);
    uint8_t released = cs1237_clock_read(cs);
    if (reserved || !released)
        return (int32_t)SAMPLE_ERROR_DESYNC;
    if (raw & 0x800000)
        raw |= 0xff000000;
    return (int32_t)raw;
}

static uint_fast8_t
cs1237_is_data_ready(struct cs1237_sensor *cs)
{
    return !gpio_in_read(cs->drdy_pin);
}

static uint_fast8_t
cs1237_wait_ready(struct cs1237_sensor *cs)
{
    uint32_t timeout = timer_read_time()
                     + timer_from_us(CS1237_CONFIG_TIMEOUT_US);
    while (!cs1237_is_data_ready(cs)) {
        if (timer_is_before(timeout, timer_read_time()))
            return 0;
    }
    return 1;
}

static uint_fast8_t
cs1237_config_transfer(struct cs1237_sensor *cs, uint8_t command,
                       uint8_t value, uint8_t *readback)
{
    if (!cs1237_wait_ready(cs))
        return 0;

    // Flush the current conversion and enter the 46-clock config window.
    cs1237_clock_count(cs, 27);

    // DIO is input for DRDY/data and output for this command/register write.
    gpio_out_reset(cs->dio_pin, 1);
    cs1237_clock_count(cs, 2);
    for (int i = 6; i >= 0; i--)
        cs1237_clock_write(cs, (command >> i) & 1);
    // Clock 37 switches DIO from the command phase to register data.
    if (command == CS1237_CONFIG_READ)
        gpio_in_reset(cs->drdy_pin, 1);
    cs1237_clock(cs);

    uint8_t result = 0;
    for (int i = 7; i >= 0; i--) {
        if (command == CS1237_CONFIG_READ)
            result = (result << 1) | cs1237_clock_read(cs);
        else
            cs1237_clock_write(cs, (value >> i) & 1);
    }

    // Return the shared DIO pin to input mode before the next conversion.
    if (command != CS1237_CONFIG_READ)
        gpio_in_reset(cs->drdy_pin, 1);
    cs1237_clock(cs);
    uint8_t released = gpio_in_read(cs->drdy_pin);
    gpio_out_write(cs->sclk_pin, 0);
    if (!released)
        return 0;
    *readback = result;
    return 1;
}

static uint_fast8_t
cs1237_event(struct timer *timer)
{
    struct cs1237_sensor *cs = container_of(timer, struct cs1237_sensor,
                                            timer);
    uint32_t rest_ticks = cs->rest_ticks;
    if (cs->flags & FLAG_PENDING) {
        cs->flags = FLAG_PENDING | FLAG_OVERFLOW;
        cs->sb.possible_overflows++;
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
    if (sample == (int32_t)SAMPLE_ERROR_DESYNC)
        cs->last_error = SAMPLE_ERROR_DESYNC;

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
    cs->dio_pin = gpio_out_setup(args[1], 1);
    cs->drdy_pin = gpio_in_setup(args[1], 1);
    cs->sclk_pin = gpio_out_setup(args[2], 0);
}
DECL_COMMAND(command_config_cs1237,
             "config_cs1237 oid=%c drdy_pin=%u sclk_pin=%u");

void
command_cs1237_config(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    uint8_t command = args[1];
    uint8_t value = args[2];
    uint8_t readback = 0;
    uint8_t success = 0;
    if ((command == CS1237_CONFIG_READ)
        || (command == CS1237_CONFIG_WRITE && !(value & 0x83)))
        success = cs1237_config_transfer(cs, command, value, &readback);
    sendf("cs1237_config_response oid=%c success=%c value=%c",
          args[0], success, readback);
}
DECL_COMMAND(command_cs1237_config,
             "cs1237_config oid=%c command=%c value=%c");

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
