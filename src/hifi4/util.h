#ifndef __HIFI4_UTIL_H
#define __HIFI4_UTIL_H

#include <stdint.h>
#include <stddef.h>

void *memcpy(void *restrict dest, const void *restrict src, size_t n);
void *memset(void *restrict dest, int c, size_t n);
void *memmove(void *restrict dest, const void *restrict src, size_t n);
void *memchr(void *__s, int __c, size_t __n);

inline void write_reg(uint32_t addr, uint32_t val){
  *((volatile unsigned long *)(addr)) = val;
}

inline uint32_t read_reg(uint32_t addr){
  return *((volatile unsigned long *)(addr));
}

void set_bit(uint32_t addr, uint8_t bit);

void clear_bit(uint32_t addr, uint8_t bit);

void *malloc(size_t size);

int strcmp(const char *s1, const char *s2);

int xt_sprintf(char * str, const char * format, ...);
int xt_printf(const char *format, ...);

#endif // util.h
