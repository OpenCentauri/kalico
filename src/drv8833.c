// MCU-side DRV8833 hall-based speed controller
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio_in / struct gpio_pwm
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time / timer_is_before
#include "command.h" // DECL_COMMAND / sendf
#include "sched.h" // DECL_TASK / sched_add_timer

#define DUTY_SCALE 10
#define DUTY_MAX (100 * DUTY_SCALE)
#define SPEED_SMOOTH_DIV 8
#define SPEED_SCALE 1000
#define PID_PARAM_SCALE 1000

struct drv8833 {
    struct timer timer;
    struct gpio_in hall_pin;
    struct gpio_pwm motor_fwd;
    struct gpio_pwm motor_rwd;
    uint32_t poll_ticks;
    uint32_t control_ticks;
    uint32_t report_ticks;
    uint32_t next_control_time;
    uint32_t next_report_time;
    uint32_t count;
    uint32_t stop_count;
    uint32_t last_edge_time;
    uint32_t last_interval_ticks;
    uint32_t hall_resolution;
    uint32_t speed_mm_s_x1000;
    uint32_t target_speed_mm_s_x1000;
    int64_t integral;
    int32_t previous_error;
    uint32_t pid_kp;
    uint32_t pid_ki;
    uint32_t pid_kd;
    uint16_t duty_x10;
    uint16_t default_duty_x10;
    uint16_t max_duty_step_x10;
    uint16_t pwm_max;
    uint8_t flags;
    uint8_t direction;
    uint8_t last_hall_value;
};

enum {
    DF_ACTIVE = 1 << 0,
    DF_PENDING = 1 << 1,
    DF_MANUAL = 1 << 2,
};

static struct task_wake drv8833_wake;

static uint32_t
drv8833_calc_speed(const struct drv8833 *d, uint32_t delta_count,
                   uint32_t delta_ticks)
{
    if (!delta_count || !delta_ticks)
        return 0;
    uint64_t numer =
        (uint64_t)delta_count * d->hall_resolution * CONFIG_CLOCK_FREQ;
    return numer / ((uint64_t)delta_ticks * 1000);
}

static uint32_t
drv8833_calc_measured_speed(const struct drv8833 *d, uint32_t time)
{
    if (!d->last_edge_time)
        return 0;
    uint32_t age_ticks = time - d->last_edge_time;
    uint32_t measured_interval = d->last_interval_ticks
                                 ? d->last_interval_ticks : age_ticks;
    if (age_ticks > measured_interval)
        measured_interval = age_ticks;
    return drv8833_calc_speed(d, 1, measured_interval);
}

static uint32_t
drv8833_update_speed(struct drv8833 *d, uint32_t time)
{
    uint32_t measured_speed = drv8833_calc_measured_speed(d, time);
    if (!d->speed_mm_s_x1000)
        d->speed_mm_s_x1000 = measured_speed;
    else
        d->speed_mm_s_x1000 +=
            ((int32_t)measured_speed - (int32_t)d->speed_mm_s_x1000)
            / SPEED_SMOOTH_DIV;
    return d->speed_mm_s_x1000;
}

static int64_t
drv8833_integral_limit(const struct drv8833 *d)
{
    if (!d->pid_ki)
        return 0x7fffffffffffffffLL;
    return ((int64_t)DUTY_MAX * PID_PARAM_SCALE * SPEED_SCALE
            * CONFIG_CLOCK_FREQ) / d->pid_ki;
}

static void
drv8833_apply_pwm(struct drv8833 *d)
{
    uint32_t pwm_value = 0;
    if ((d->flags & DF_ACTIVE) && d->duty_x10) {
        pwm_value = ((uint32_t)d->duty_x10 * d->pwm_max + DUTY_MAX / 2)
                    / DUTY_MAX;
    }
    if (d->direction) {
        gpio_pwm_write(d->motor_fwd, pwm_value);
        gpio_pwm_write(d->motor_rwd, 0);
    } else {
        gpio_pwm_write(d->motor_fwd, 0);
        gpio_pwm_write(d->motor_rwd, pwm_value);
    }
}

static void
drv8833_control_update(struct drv8833 *d, uint32_t time)
{
    if (!(d->flags & DF_ACTIVE) || (d->flags & DF_MANUAL))
        return;

    if (!d->target_speed_mm_s_x1000)
        return;

    int32_t error = (int32_t)d->target_speed_mm_s_x1000
                    - (int32_t)d->speed_mm_s_x1000;
    int32_t derivative = error - d->previous_error;
    int64_t next_integral = d->integral + (int64_t)error * d->control_ticks;
    int64_t integral_limit = drv8833_integral_limit(d);
    if (next_integral < -integral_limit)
        next_integral = -integral_limit;
    else if (next_integral > integral_limit)
        next_integral = integral_limit;

    int32_t p_term = (int64_t)d->pid_kp * error
                     / (PID_PARAM_SCALE * SPEED_SCALE);
    int32_t i_term = (int64_t)d->pid_ki * next_integral
                     / ((int64_t)PID_PARAM_SCALE * SPEED_SCALE
                        * CONFIG_CLOCK_FREQ);
    int32_t d_term = 0;
    if (d->control_ticks) {
        d_term = (int64_t)d->pid_kd * derivative * CONFIG_CLOCK_FREQ
                 / ((int64_t)PID_PARAM_SCALE * SPEED_SCALE
                    * d->control_ticks);
    }

    int32_t target_duty = d->default_duty_x10 + p_term + i_term + d_term;
    if (target_duty < 0)
        target_duty = 0;
    else if (target_duty > DUTY_MAX)
        target_duty = DUTY_MAX;

    if (!((target_duty == 0 && error < 0)
          || (target_duty == DUTY_MAX && error > 0)))
        d->integral = next_integral;

    int32_t delta_duty = target_duty - (int32_t)d->duty_x10;
    if (delta_duty < -(int32_t)d->max_duty_step_x10)
        delta_duty = -(int32_t)d->max_duty_step_x10;
    else if (delta_duty > (int32_t)d->max_duty_step_x10)
        delta_duty = d->max_duty_step_x10;

    d->duty_x10 += delta_duty;
    d->previous_error = error;
    drv8833_apply_pwm(d);
}

static void
drv8833_reset_state(struct drv8833 *d)
{
    d->count = 0;
    d->stop_count = 0;
    d->last_edge_time = 0;
    d->last_interval_ticks = 0;
    d->speed_mm_s_x1000 = 0;
    d->integral = 0;
    d->previous_error = 0;
}

static uint_fast8_t
drv8833_event(struct timer *timer)
{
    struct drv8833 *d = container_of(timer, struct drv8833, timer);
    uint32_t time = d->timer.waketime;

    uint8_t hall_value = gpio_in_read(d->hall_pin);
    if (hall_value != d->last_hall_value) {
        d->count++;
        if (d->last_edge_time)
            d->last_interval_ticks = time - d->last_edge_time;
        d->last_edge_time = time;
        d->last_hall_value = hall_value;
        if ((d->flags & DF_ACTIVE) && d->stop_count && d->count >= d->stop_count) {
            d->flags &= ~(DF_ACTIVE | DF_MANUAL);
            d->target_speed_mm_s_x1000 = 0;
            d->stop_count = 0;
            d->integral = 0;
            d->previous_error = 0;
            d->duty_x10 = 0;
            drv8833_apply_pwm(d);
            d->flags |= DF_PENDING;
            sched_wake_task(&drv8833_wake);
        }
    }

    if (!timer_is_before(time, d->next_control_time)) {
        d->next_control_time = time + d->control_ticks;
        drv8833_update_speed(d, time);
        drv8833_control_update(d, time);
    }
    if ((d->flags & DF_ACTIVE) && !timer_is_before(time, d->next_report_time)) {
        d->next_report_time = time + d->report_ticks;
        d->flags |= DF_PENDING;
        sched_wake_task(&drv8833_wake);
    }

    d->timer.waketime += d->poll_ticks;
    return SF_RESCHEDULE;
}

void
command_config_drv8833(uint32_t *args)
{
    struct drv8833 *d = oid_alloc(args[0], command_config_drv8833,
                                  sizeof(*d));
    d->motor_fwd = gpio_pwm_setup(args[1], args[3], 0);
    d->motor_rwd = gpio_pwm_setup(args[2], args[3], 0);
    d->hall_pin = gpio_in_setup(args[4], args[5]);
    d->poll_ticks = args[6];
    d->control_ticks = args[7];
    d->report_ticks = args[8];
    d->hall_resolution = args[9];
    d->pwm_max = args[10];
    d->default_duty_x10 = args[11];
    d->max_duty_step_x10 = args[12];
    d->pid_kp = 0;
    d->pid_ki = 0;
    d->pid_kd = 0;
    d->last_hall_value = gpio_in_read(d->hall_pin);
    d->direction = 1;
    d->timer.func = drv8833_event;
}
DECL_COMMAND(command_config_drv8833,
             "config_drv8833 oid=%c motor_fwd_pin=%u motor_rwd_pin=%u"
             " cycle_ticks=%u hall_pin=%u hall_pullup=%c poll_ticks=%u"
             " control_ticks=%u report_ticks=%u hall_resolution=%u"
             " pwm_max=%hu default_duty=%hu max_duty_step=%hu");

void
command_drv8833_set(uint32_t *args)
{
    struct drv8833 *d = oid_lookup(args[0], command_config_drv8833);
    sched_del_timer(&d->timer);
    d->flags &= ~DF_PENDING;

    if (!args[1]) {
        d->flags &= ~(DF_ACTIVE | DF_MANUAL);
        d->target_speed_mm_s_x1000 = 0;
        drv8833_reset_state(d);
        d->duty_x10 = 0;
        drv8833_apply_pwm(d);
        return;
    }

    uint32_t now = timer_read_time();
    d->flags = (d->flags & ~DF_MANUAL) | DF_ACTIVE | DF_PENDING;
    d->direction = !!args[2];
    d->target_speed_mm_s_x1000 = args[3];
    drv8833_reset_state(d);
    d->stop_count = args[4];
    d->duty_x10 = d->default_duty_x10;
    d->last_hall_value = gpio_in_read(d->hall_pin);
    d->next_control_time = now;
    d->next_report_time = now;
    drv8833_apply_pwm(d);
    d->timer.waketime = now + d->poll_ticks;
    sched_add_timer(&d->timer);
    sched_wake_task(&drv8833_wake);
}
DECL_COMMAND(command_drv8833_set,
             "drv8833_set oid=%c enable=%c direction=%c target_speed=%u"
             " stop_ticks=%u");

void
command_drv8833_manual(uint32_t *args)
{
    struct drv8833 *d = oid_lookup(args[0], command_config_drv8833);

    if (args[1] && (d->flags & DF_ACTIVE) && (d->flags & DF_MANUAL)) {
        d->direction = !!args[2];
        d->duty_x10 = args[3] > DUTY_MAX ? DUTY_MAX : args[3];
        drv8833_apply_pwm(d);
        return;
    }

    sched_del_timer(&d->timer);
    d->flags &= ~DF_PENDING;

    if (!args[1]) {
        d->flags &= ~(DF_ACTIVE | DF_MANUAL);
        d->target_speed_mm_s_x1000 = 0;
        drv8833_reset_state(d);
        d->duty_x10 = 0;
        drv8833_apply_pwm(d);
        return;
    }

    uint32_t now = timer_read_time();
    d->flags |= DF_ACTIVE | DF_PENDING | DF_MANUAL;
    d->direction = !!args[2];
    d->target_speed_mm_s_x1000 = 0;
    drv8833_reset_state(d);
    d->duty_x10 = args[3] > DUTY_MAX ? DUTY_MAX : args[3];
    d->last_hall_value = gpio_in_read(d->hall_pin);
    d->next_control_time = now;
    d->next_report_time = now;
    drv8833_apply_pwm(d);
    d->timer.waketime = now + d->poll_ticks;
    sched_add_timer(&d->timer);
    sched_wake_task(&drv8833_wake);
}
DECL_COMMAND(command_drv8833_manual,
             "drv8833_manual oid=%c enable=%c direction=%c duty=%hu");

void
command_drv8833_set_pid(uint32_t *args)
{
    struct drv8833 *d = oid_lookup(args[0], command_config_drv8833);
    irq_disable();
    d->pid_kp = args[1];
    d->pid_ki = args[2];
    d->pid_kd = args[3];
    d->integral = 0;
    d->previous_error = 0;
    irq_enable();
}
DECL_COMMAND(command_drv8833_set_pid,
             "drv8833_set_pid oid=%c kp=%u ki=%u kd=%u");

void
drv8833_task(void)
{
    if (!sched_check_wake(&drv8833_wake))
        return;

    uint8_t oid;
    struct drv8833 *d;
    foreach_oid(oid, d, command_config_drv8833) {
        irq_disable();
        uint8_t flags = d->flags;
        uint32_t count = d->count;
        uint32_t speed = d->speed_mm_s_x1000;
        uint16_t duty = d->duty_x10;
        d->flags = flags & ~DF_PENDING;
        irq_enable();
        if (!(flags & DF_PENDING))
            continue;
        sendf("drv8833_status oid=%c active=%c manual=%c count=%u speed=%u"
              " duty=%hu",
              oid, !!(flags & DF_ACTIVE), !!(flags & DF_MANUAL),
              count, speed, duty);
    }
}
DECL_TASK(drv8833_task);