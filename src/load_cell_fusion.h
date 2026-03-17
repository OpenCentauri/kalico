#ifndef __LOAD_CELL_FUSION_H
#define __LOAD_CELL_FUSION_H

#include <stdint.h>

struct load_cell_fusion;
struct load_cell_fusion_sensor;

struct load_cell_fusion *load_cell_fusion_oid_lookup(uint8_t oid);
struct load_cell_fusion_sensor *load_cell_fusion_add_sensor(
    struct load_cell_fusion *lcf, uint8_t invert);
void load_cell_fusion_report_sample(struct load_cell_fusion_sensor *lcfs,
                                    int32_t sample,
                                    uint8_t is_error);
void load_cell_fusion_process_updates(void);

#endif // load_cell_fusion.h
