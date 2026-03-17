// Generic load cell sensor fusion
//
// Averages samples pushed by N independent sensors into a single fused
// stream.  Sensor-agnostic: any driver that can read an ADC value can
// attach a slot and call load_cell_fusion_report_sample().
//
// Copyright (C) 2026 Timo V
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // shutdown
#include "board/misc.h" // timer_read_time
#include "sensor_bulk.h" // sensor_bulk_report
#include "load_cell_probe.h" // load_cell_probe_report_sample
#include "load_cell_fusion.h"
#include <stdint.h>

#define MAX_FUSION_SENSORS 4
#define BYTES_PER_SAMPLE 4

struct load_cell_fusion_sensor {
    struct load_cell_fusion *fusion;
    int8_t direction;
    int32_t last_sample;
    uint8_t updated;
    uint8_t has_error;
};

struct load_cell_fusion {
    struct sensor_bulk sb;
    struct load_cell_probe *lce;
    uint8_t oid;
    uint8_t sensor_count;
    uint8_t active;
    uint8_t dirty;
    struct load_cell_fusion_sensor sensors[MAX_FUSION_SENSORS];
};

static void
add_sample(struct sensor_bulk *sb, uint8_t oid, uint32_t counts)
{
    sb->data[sb->data_count] = counts;
    sb->data[sb->data_count + 1] = counts >> 8;
    sb->data[sb->data_count + 2] = counts >> 16;
    sb->data[sb->data_count + 3] = counts >> 24;
    sb->data_count += BYTES_PER_SAMPLE;
    if (sb->data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(sb->data))
        sensor_bulk_report(sb, oid);
}

/****************************************************************
 * MCU Commands
 ****************************************************************/

void
command_config_load_cell_fusion(uint32_t *args)
{
    struct load_cell_fusion *lcf = oid_alloc(args[0],
        command_config_load_cell_fusion, sizeof(*lcf));
    lcf->oid = args[0];
}
DECL_COMMAND(command_config_load_cell_fusion,
             "config_load_cell_fusion oid=%c");

void
command_load_cell_fusion_attach_probe(uint32_t *args)
{
    struct load_cell_fusion *lcf = oid_lookup(args[0],
        command_config_load_cell_fusion);
    lcf->lce = load_cell_probe_oid_lookup(args[1]);
}
DECL_COMMAND(command_load_cell_fusion_attach_probe,
             "load_cell_fusion_attach_probe oid=%c load_cell_probe_oid=%c");

void
command_query_load_cell_fusion(uint32_t *args)
{
    struct load_cell_fusion *lcf = oid_lookup(args[0],
        command_config_load_cell_fusion);
    if (!lcf->sensor_count)
        shutdown("load_cell_fusion: no sensors");
    sensor_bulk_reset(&lcf->sb);
    uint8_t i;
    for (i = 0; i < lcf->sensor_count; i++) {
        lcf->sensors[i].updated = 0;
        lcf->sensors[i].has_error = 0;
    }
    lcf->dirty = 0;
    lcf->active = args[1];
}
DECL_COMMAND(command_query_load_cell_fusion,
             "query_load_cell_fusion oid=%c active=%c");

void
command_query_load_cell_fusion_status(const uint32_t *args)
{
    uint8_t oid = args[0];
    struct load_cell_fusion *lcf = oid_lookup(oid,
        command_config_load_cell_fusion);
    uint32_t start_t = timer_read_time();
    sensor_bulk_status(&lcf->sb, oid, start_t, 0, 0);
}
DECL_COMMAND(command_query_load_cell_fusion_status,
             "query_load_cell_fusion_status oid=%c");


/****************************************************************
 * Public API (called by sensor drivers)
 ****************************************************************/

struct load_cell_fusion *
load_cell_fusion_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_load_cell_fusion);
}

struct load_cell_fusion_sensor *
load_cell_fusion_add_sensor(struct load_cell_fusion *lcf, uint8_t invert)
{
    if (lcf->sensor_count >= MAX_FUSION_SENSORS)
        shutdown("load_cell_fusion: too many sensors");
    struct load_cell_fusion_sensor *lcfs = &lcf->sensors[lcf->sensor_count++];
    lcfs->fusion = lcf;
    lcfs->direction = invert ? -1 : 1;
    return lcfs;
}

// Called by sensor drivers after reading their ADC
void
load_cell_fusion_report_sample(struct load_cell_fusion_sensor *lcfs,
                               int32_t sample, uint8_t is_error)
{
    struct load_cell_fusion *lcf = lcfs->fusion;
    if (!lcf->active)
        return;

    lcfs->last_sample = sample;
    lcfs->updated = 1;
    lcfs->has_error = is_error;
    lcf->dirty = 1;
}

static void
load_cell_fusion_process_group(struct load_cell_fusion *lcf)
{
    if (!lcf->active || !lcf->dirty)
        return;
    lcf->dirty = 0;

    // Wait until every sensor has reported at least once.
    uint8_t i;
    for (i = 0; i < lcf->sensor_count; i++) {
        if (!lcf->sensors[i].updated)
            return;
    }

    // Error in any sensor: report the errored sample.
    for (i = 0; i < lcf->sensor_count; i++) {
        if (lcf->sensors[i].has_error) {
            add_sample(&lcf->sb, lcf->oid, lcf->sensors[i].last_sample);
            return;
        }
    }

    // Compute signed rolling average from the latest sample of each sensor.
    int32_t sum = 0;
    for (i = 0; i < lcf->sensor_count; i++)
        sum += lcf->sensors[i].direction * lcf->sensors[i].last_sample;
    int32_t fused = sum / lcf->sensor_count;
    add_sample(&lcf->sb, lcf->oid, (uint32_t)fused);
    if (lcf->lce)
        load_cell_probe_report_sample(lcf->lce, fused);
}

void
load_cell_fusion_process_updates(void)
{
    uint8_t oid;
    struct load_cell_fusion *lcf;
    foreach_oid(oid, lcf, command_config_load_cell_fusion)
        load_cell_fusion_process_group(lcf);
}
