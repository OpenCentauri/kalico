#ifndef __HIFI4_LOG_H
#define __HIFI4_LOG_H

#include <stdint.h>

void log_init(void);
void log_fake_init(void);
void log_clear(void);
void lprintf(const char *str);
void lprint_hex(uint32_t value);

#endif // log.h
