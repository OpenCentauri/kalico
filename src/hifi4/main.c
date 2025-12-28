// Main entry point for hifi4.
//
// Copyright (C) 2025  James Turton <james.turton@gmx.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <hal.h> // hal_init
#include "sched.h" // sched_main

int
main(void)
{
    hal_init();

    sched_main();
}
