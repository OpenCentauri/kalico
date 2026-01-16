// Generic implementation of dynamic memory pool
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/misc.h" // dynmem_start

extern uint32_t _heap_start, _heap_end;

// Return the start of memory available for dynamic allocations
void *
dynmem_start(void)
{
    return &_heap_start;
}

// Return the end of memory available for dynamic allocations
void *
dynmem_end(void)
{
    return &_heap_end;
}
