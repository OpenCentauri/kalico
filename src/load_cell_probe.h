#ifndef __LOAD_CELL_PROBE_H
#define __LOAD_CELL_PROBE_H

#include <stdint.h> // uint8_t

struct load_cell_probe *load_cell_probe_oid_lookup(uint8_t oid);
void load_cell_probe_report_sample(struct load_cell_probe *lce
                        , int32_t sample);
void load_cell_probe_report_sample_at(struct load_cell_probe *lce
                        , int32_t sample, uint32_t sample_ticks);
void load_cell_probe_report_fault_at(struct load_cell_probe *lce
                        , uint32_t sample_ticks);
void load_cell_probe_set_sensor_ready(struct load_cell_probe *lce
                        , uint8_t is_ready);

#endif // load_cell_probe.h
