#ifndef __HIFI4_LOG_H
#define __HIFI4_LOG_H

#include <stdint.h>

void log_init(void);
void log_fake_init(void);
void log_clear(void);
int lprintf(const char *fmt, ...);

#endif // log.h
