// Helper functions for hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "util.h"

#define UART_BASE 0x02500000
#define UART_THR  (*(volatile uint32_t*)(UART_BASE + 0x00))
#define UART_LSR  (*(volatile uint32_t*)(UART_BASE + 0x14))

void *memcpy(void *restrict dest, const void *restrict src, size_t n){
    volatile uint8_t *d = (volatile uint8_t*)dest;
    volatile const uint8_t *s = (volatile const uint8_t*)src;
    
    while (n--) {
        *d++ = *s++;
    }

    /* Memory barrier to ensure writes complete */
    __asm__ __volatile__ ("" ::: "memory");

    return dest;
}

void *memset(void *dest, int c, size_t n){
    volatile uint8_t *d = (volatile uint8_t*)dest;
    uint8_t val = (uint8_t)c;
    
    /* Prevent compiler from optimizing this into a builtin memset call */
    while (n--) {
        *d++ = val;
    }
    
    /* Memory barrier to ensure writes complete */
    __asm__ __volatile__ ("" ::: "memory");
    
    return dest;
}

void *memmove (void *dest, const void *src, size_t n){
    volatile uint8_t *d = (volatile uint8_t*)dest;
    volatile const uint8_t *s = (volatile const uint8_t*)src;
    if (d < s)
    {
        while (n--)
            *d++ = *s++;
    }
    else
    {
        volatile const uint8_t *lasts = s + (n-1);
        volatile uint8_t *lastd = d + (n-1);
        while (n--)
            *lastd-- = *lasts--;
    }

    /* Memory barrier to ensure writes complete */
    __asm__ __volatile__ ("" ::: "memory");

    return dest;
}

void *memchr(void *src, int c, size_t n){
    volatile uint8_t *s = (volatile uint8_t*)src;
    uint8_t val = (uint8_t)c;

  while (n--)
    {
      if (*s == val)
        return (void *) s;
      s++;
    }

  return NULL;
}

void set_bit(uint32_t addr, uint8_t bit){
  write_reg(addr, read_reg(addr) | (1<<bit));
}

void clear_bit(uint32_t addr, uint8_t bit){
  write_reg(addr, read_reg(addr) & ~(1<<bit));
}

int strcmp(const char *s1, const char *s2)
{
    while (*s1 == *s2++)
        if (*s1++ == 0)
            return (0);
    return (*(const unsigned char *)s1 - *(const unsigned char *)(s2 - 1));
}
