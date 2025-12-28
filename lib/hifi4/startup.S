/*
 * startup.S - Startup code for R528/T113 HiFi4 DSP (Call0 ABI)
 *
 * Copyright (C) 2025  James Turton <james.turton@gmx.com>
 * This file may be distributed under the terms of the GNU GPLv3 license.
 *
 * Call0 ABI: No register windows, uses a0 as return address
 * - a0: return address
 * - a1: stack pointer
 * - a2-a7: function arguments / return values
 * - a8-a15: callee-saved (must be preserved across calls)
 */

/*
 * Reset vector - entry point after reset
 * Literals MUST be placed before the code that uses them for l32r
 */
    .section .vectors.literal, "a"
    .align 4
    .literal_position

_lit_stack_top:
    .word   __stack_top
_lit_ps_init:
    .word   0x00000020
_lit_sidata:
    .word   _sidata
_lit_sdata:
    .word   _sdata
_lit_edata:
    .word   _edata
_lit_sbss:
    .word   _sbss
_lit_ebss:
    .word   _ebss

    .section .vectors, "ax"
    .global _start
    .global _reset
    .align 4

_start:
_reset:
    /* Disable interrupts - set INTLEVEL to max */
    rsil    a2, 15
    
    /* Set up the stack pointer (must be 16-byte aligned) */
    l32r    a1, _lit_stack_top
    
    /* Set processor state for Call0 ABI
     * PS.WOE = 0 (no window overflow)
     * PS.CALLINC = 0
     * PS.UM = 0 (kernel mode)
     * PS.EXCM = 0
     * PS.INTLEVEL = 0 (will enable interrupts later)
     */
    l32r    a2, _lit_ps_init
    wsr     a2, ps
    rsync

    /* Initialize CCOUNT (cycle counter) */
    movi    a2, 0
    wsr     a2, ccount
    
    /* Clear a0 (return address) - we never return from here */
    movi    a0, 0
    
    /* Initialize .data section (copy from ROM/IRAM to DRAM) */
    l32r    a2, _lit_sidata         /* Source (load address) */
    l32r    a3, _lit_sdata          /* Destination start */
    l32r    a4, _lit_edata          /* Destination end */
    
    bgeu    a3, a4, .Lcopy_data_done
.Lcopy_data_loop:
    l32i    a5, a2, 0
    s32i    a5, a3, 0
    addi    a2, a2, 4
    addi    a3, a3, 4
    bltu    a3, a4, .Lcopy_data_loop
.Lcopy_data_done:

    /* Clear .bss section */
    l32r    a2, _lit_sbss           /* BSS start */
    l32r    a3, _lit_ebss           /* BSS end */
    movi    a4, 0                   /* Value to write (zero) */

    bgeu    a2, a3, .Lclear_bss_done
.Lclear_bss_loop:
    s32i    a4, a2, 0
    addi    a2, a2, 4
    bltu    a2, a3, .Lclear_bss_loop
.Lclear_bss_done:

    /* Call main() - Call0 ABI uses call0 instruction */
    call0   main

    /* If main() returns, halt */
.Lhalt:
    waiti   0                   /* Wait for interrupt (low power) */
    j       .Lhalt


/*
 * Interrupt/Exception vectors
 * 
 * For HiFi4 DSP, interrupts typically come through external interrupt pins
 * which are then dispatched. The exact vector addresses depend on VECBASE.
 */
    .section .init.literal, "a"
    .align 4
    .literal_position

    .section .init, "ax"
    .align 4

/*
 * User exception handler (EXCCAUSE determines the cause)
 */
    .global _user_exception
    .type   _user_exception, @function
_user_exception:
    /* For now, just infinite loop on exception */
    j       _user_exception
    .size   _user_exception, . - _user_exception


/*
 * Interrupt handler for external interrupts (Call0 ABI version)
 * 
 * In Call0 ABI, we need to save/restore a0 (return address) and
 * any registers we use. a2-a7 are caller-saved, a8-a15 are callee-saved.
 */
    .global _interrupt_handler
    .type   _interrupt_handler, @function
_interrupt_handler:
    /* Allocate stack frame - save all registers we might clobber
     * Frame size: 80 bytes (16-byte aligned)
     */
    addi    a1, a1, -80
    
    /* Save return address and special registers */
    s32i    a0, a1, 0           /* Return address */
    
    /* Read and save EPC (return PC for interrupt) */
    rsr     a0, epc1
    s32i    a0, a1, 4
    
    /* Read and save PS */
    rsr     a0, ps
    s32i    a0, a1, 8
    
    /* Save caller-saved registers (a2-a7) */
    s32i    a2, a1, 16
    s32i    a3, a1, 20
    s32i    a4, a1, 24
    s32i    a5, a1, 28
    s32i    a6, a1, 32
    s32i    a7, a1, 36
    
    /* Save callee-saved registers we'll use (a8-a15) */
    s32i    a8, a1, 40
    s32i    a9, a1, 44
    s32i    a10, a1, 48
    s32i    a11, a1, 52
    s32i    a12, a1, 56
    s32i    a13, a1, 60
    s32i    a14, a1, 64
    s32i    a15, a1, 68
    
    /* Call C interrupt dispatcher 
     * In Call0 ABI, call0 puts return address in a0
     */
    call0   hal_irq_dispatch
    
    /* Restore callee-saved registers */
    l32i    a15, a1, 68
    l32i    a14, a1, 64
    l32i    a13, a1, 60
    l32i    a12, a1, 56
    l32i    a11, a1, 52
    l32i    a10, a1, 48
    l32i    a9, a1, 44
    l32i    a8, a1, 40
    
    /* Restore caller-saved registers */
    l32i    a7, a1, 36
    l32i    a6, a1, 32
    l32i    a5, a1, 28
    l32i    a4, a1, 24
    l32i    a3, a1, 20
    l32i    a2, a1, 16
    
    /* Restore PS */
    l32i    a0, a1, 8
    wsr     a0, ps
    
    /* Restore EPC */
    l32i    a0, a1, 4
    wsr     a0, epc1
    
    /* Restore return address */
    l32i    a0, a1, 0
    
    /* Deallocate stack frame */
    addi    a1, a1, 80
    
    /* Return from interrupt level 1 */
    rsync
    rfe
    
    .size   _interrupt_handler, . - _interrupt_handler


/*
 * Level 2/3/4/5 interrupt handlers (stubs - customize as needed)
 */
    .global _interrupt_level2
    .type   _interrupt_level2, @function
_interrupt_level2:
    rfi     2
    .size   _interrupt_level2, . - _interrupt_level2

    .global _interrupt_level3
    .type   _interrupt_level3, @function
_interrupt_level3:
    rfi     3
    .size   _interrupt_level3, . - _interrupt_level3


/*
 * NMI handler
 */
    .global _nmi_handler
    .type   _nmi_handler, @function
_nmi_handler:
    j       _nmi_handler        /* Infinite loop on NMI */
    .size   _nmi_handler, . - _nmi_handler


/*
 * Debug exception handler
 */
    .global _debug_exception
    .type   _debug_exception, @function
_debug_exception:
    j       _debug_exception
    .size   _debug_exception, . - _debug_exception


/*
 * Weak symbol for hal_irq_dispatch - allows linking without HAL
 */
    .weak   hal_irq_dispatch


/*
 * Default IRQ handler (does nothing, returns)
 */
    .global _default_irq_handler
    .type   _default_irq_handler, @function
_default_irq_handler:
    ret
    .size   _default_irq_handler, . - _default_irq_handler
