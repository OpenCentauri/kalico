#include "log.h"
#include <hal.h>
#include <stdio.h>
#include <stdarg.h>
#include "sched.h" // DECL_INIT
#include "util.h" // memcpy

char __attribute__((section(".trace_buf"))) trace_buffer[TRACE_BUF_SIZE];

static uint32_t trace_pos = 0;

int lprintf(const char *fmt, ...) {
    typedef __builtin_va_list va_list;

    va_list args;
    va_start(args, fmt);
    
    char temp_buffer[256];
    char *out = temp_buffer;
    
    while (*fmt) {
        if (*fmt != '%') {
            *out++ = *fmt++;
            continue;
        }
        
        fmt++; // skip '%'
        
        // Parse flags
        int left_align = 0, zero_pad = 0, show_sign = 0, space_sign = 0; //, alt_form = 0;
        while (1) {
            if (*fmt == '-')      { left_align = 1; fmt++; }
            else if (*fmt == '0') { zero_pad = 1; fmt++; }
            else if (*fmt == '+') { show_sign = 1; fmt++; }
            else if (*fmt == ' ') { space_sign = 1; fmt++; }
            // else if (*fmt == '#') { alt_form = 1; fmt++; }
            else break;
        }
        
        // Parse width
        int width = 0;
        while (*fmt >= '0' && *fmt <= '9') {
            width = width * 10 + (*fmt - '0');
            fmt++;
        }
        
        // Parse precision
        int precision = -1;
        if (*fmt == '.') {
            fmt++;
            precision = 0;
            while (*fmt >= '0' && *fmt <= '9') {
                precision = precision * 10 + (*fmt - '0');
                fmt++;
            }
        }
        
        // Parse length modifier
        int is_long = 0;
        if (*fmt == 'l') {
            fmt++;
            is_long = 1;
        }
        
        // Handle specifier
        char specifier = *fmt++;
        char temp[65]; // enough for 64-bit binary + null
        char *str = temp;
        int len = 0;
        int is_negative = 0;
        
        switch (specifier) {
        case '%':
            *out++ = '%';
            continue;
            
        case 'c':
            temp[0] = (char)va_arg(args, int);
            temp[1] = '\0';
            len = 1;
            break;
            
        case 's': {
            str = va_arg(args, char*);
            if (!str) str = "(null)";
            char *s = str;
            while (*s) { len++; s++; }
            if (precision >= 0 && len > precision) len = precision;
            break;
        }
            
        case 'd':
        case 'i': {
            long val;
            if (is_long) val = va_arg(args, long);
            else val = va_arg(args, int);
            
            if (val < 0) { is_negative = 1; val = -val; }
            
            char *p = temp + sizeof(temp) - 1;
            *p = '\0';
            if (val == 0) { *--p = '0'; }
            else {
                while (val > 0) {
                    *--p = '0' + (val % 10);
                    val /= 10;
                }
            }
            str = p;
            while (*p) { len++; p++; }
            break;
        }
            
        case 'u':
        case 'x':
        case 'X': {
            unsigned long val;
            if (is_long) val = va_arg(args, unsigned long);
            else val = va_arg(args, unsigned int);
            
            int base = (specifier == 'u') ? 10 : 16;
            const char *digits = (specifier == 'X') ? "0123456789ABCDEF" : "0123456789abcdef";
            
            char *p = temp + sizeof(temp) - 1;
            *p = '\0';
            if (val == 0) { *--p = '0'; }
            else {
                while (val > 0) {
                    *--p = digits[val % base];
                    val /= base;
                }
            }
            str = p;
            while (*p) { len++; p++; }
            break;
        }
            
        case 'p': {
            unsigned long val = (unsigned long)va_arg(args, void*);
            const char *digits = "0123456789abcdef";
            
            char *p = temp + sizeof(temp) - 1;
            *p = '\0';
            for (int i = 0; i < (int)(2 * sizeof(void*)); i++) {
                *--p = digits[val & 0xf];
                val >>= 4;
            }
            *--p = 'x';
            *--p = '0';
            str = p;
            len = 2 + 2 * sizeof(void*);
            break;
        }
            
        default:
            *out++ = specifier;
            continue;
        }
        
        // Calculate padding
        int prefix_len = 0;
        if (is_negative || show_sign || space_sign) prefix_len = 1;
        
        int total_len = len + prefix_len;
        int pad = (width > total_len) ? width - total_len : 0;
        char pad_char = (zero_pad && !left_align && precision < 0) ? '0' : ' ';
        
        // Output: [padding] [sign] [digits] or [sign] [zero-padding] [digits]
        if (!left_align && pad_char == ' ') {
            while (pad-- > 0) *out++ = ' ';
        }
        
        if (is_negative) *out++ = '-';
        else if (show_sign) *out++ = '+';
        else if (space_sign) *out++ = ' ';
        
        if (!left_align && pad_char == '0') {
            while (pad-- > 0) *out++ = '0';
        }
        
        for (int i = 0; i < len; i++) *out++ = str[i];
        
        if (left_align) {
            while (pad-- > 0) *out++ = ' ';
        }
    }
    
    *out = '\0';
    va_end(args);
    int len = out - temp_buffer;

    /*
     * Linux reads the trace buffer as a flat string up to the first
     * null byte. Append directly with no gaps. When full, stop writing
     * (the buffer is 4 KiB — plenty for boot diagnostics).
     */
    uint32_t pos = trace_pos;

    if (pos + len >= TRACE_BUF_SIZE)
        return 0;  // Buffer full, discard

    memcpy((void *)(trace_buffer + pos), temp_buffer, len);
    trace_pos = pos + len;

    // Null-terminate so Linux knows where the content ends,
    // but don't advance trace_pos past it — next write overwrites it.
    if (trace_pos < TRACE_BUF_SIZE)
        trace_buffer[trace_pos] = '\0';

    return len;
}
