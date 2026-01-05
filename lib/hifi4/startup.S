.section .ResetVector.literal, "a"
    .align 4
.Lstart_addr:
    .word   _start

    .section .ResetVector.text, "ax"
    .global _ResetVector
    .align 4

_ResetVector:
    /* Load address of _start from literal pool and jump */
    l32r    a0, .Lstart_addr
    jx      a0

    .text
    .align  4
    .global _start
    .type   _start, @function

_start:
    /* Initialize PS register - DISABLE windowed calls */
    movi    a2, 0x00000000  /* PS.WOE=0, PS.INTLEVEL=0, PS.EXCM=0 */
    wsr.ps  a2
    rsync
    
    /* Set interrupt vector base */
    movi    a2, _memmap_vecbase_reset
    wsr.vecbase a2
    rsync
    
    /* Initialize stack pointer */
    movi    a1, __stack
    
    /* Clear BSS section BEFORE calling main */
    movi    a2, _bss_start
    movi    a3, _bss_end
    movi    a4, 0           /* Value to write (zero) */
.Lbss_loop:
    bgeu    a2, a3, .Lbss_done
    s32i    a4, a2, 0
    addi    a2, a2, 4
    j       .Lbss_loop
.Lbss_done:
    
    /* Set return address to hang loop in case main returns */
    movi    a0, .Lhang
    
    /* Jump to main (using call0 - non-windowed call) */
    j       main

    /* If main returns, loop forever */
.Lhang:
    j       .Lhang

/* ============================================================
   INTERRUPT AND EXCEPTION VECTOR HANDLERS
   ============================================================ */

/* Level 2 Interrupt Vector (Medium priority interrupts) */
    .section .Level2InterruptVector.literal, "a"
    .align 4
.Llevel2_handler_addr:
    .word   main

    .section .Level2InterruptVector.text, "ax"
    .global _Level2InterruptVector
    .align  4
_Level2InterruptVector:
    l32r    a0, .Llevel2_handler_addr
    callx0  a0
    
    /* Return from interrupt level 2 */
    rfi     2

/* Level 3 Interrupt Vector (High priority interrupts) */
    .section .Level3InterruptVector.literal, "a"
    .align 4
.Llevel3_handler_addr:
    .word   main

    .section .Level3InterruptVector.text, "ax"
    .global _Level3InterruptVector
    .align  4
_Level3InterruptVector:
    l32r    a0, .Llevel3_handler_addr
    callx0  a0
    
    /* Return from interrupt level 3 */
    rfi     3

/* Debug Exception Vector */
    .section .DebugExceptionVector.literal, "a"
    .align 4
.Ldebug_handler_addr:
    .word   main

    .section .DebugExceptionVector.text, "ax"
    .global _DebugExceptionVector
    .align  4
_DebugExceptionVector:
    l32r    a0, .Ldebug_handler_addr
    callx0  a0
    
    /* Return from debug exception */
    rfdd

/* Kernel Exception Vector */
    .section .KernelExceptionVector.literal, "a"
    .align 4
.Lkernel_handler_addr:
    .word   hal_irq_dispatch

    .section .KernelExceptionVector.text, "ax"
    .global _KernelExceptionVector
    .align  4
_KernelExceptionVector:
    wsr     a0, 209
    l32r    a0, .Lkernel_handler_addr
    callx0  a0
    
    /* Return from exception */
    rfe

/* User Exception Vector */
    .section .UserExceptionVector.literal, "a"
    .align 4
.Luser_handler_addr:
    .word   main

    .section .UserExceptionVector.text, "ax"
    .global _UserExceptionVector
    .align  4
_UserExceptionVector:
    l32r    a0, .Luser_handler_addr
    callx0  a0
    
    /* Return from exception */
    rfe

/* Double Exception Vector (Critical - handles exceptions during exception handling) */
    .section .DoubleExceptionVector.literal, "a"
    .align 4

    .section .DoubleExceptionVector.text, "ax"
    .global _DoubleExceptionVector
    .align  4
_DoubleExceptionVector:
    /* Double exception is critical - minimal handling */
    /* Try to save some diagnostic info */
    rsr.exccause a2
    rsr.excvaddr a3
    
    /* If we return, hang forever */
.Ldouble_hang:
    j       .Ldouble_hang
