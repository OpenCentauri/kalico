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

    .section .Level2InterruptVector.text, "ax"
    .global _Level2InterruptVector
    .align  4
_Level2InterruptVector:
    /* Save minimal context on stack */
    addi    a1, a1, -64
    s32i    a0, a1, 0
    s32i    a2, a1, 4
    s32i    a3, a1, 8
    s32i    a4, a1, 12
    s32i    a5, a1, 16
    s32i    a6, a1, 20
    s32i    a7, a1, 24
    s32i    a8, a1, 28
    
    /* Save special registers */
    rsr.epc2 a2
    s32i    a2, a1, 32
    rsr.excsave2 a2
    s32i    a2, a1, 36
    
    /* Restore special registers */
    l32i    a2, a1, 32
    wsr.epc2 a2
    l32i    a2, a1, 36
    wsr.excsave2 a2
    
    /* Restore context */
    l32i    a0, a1, 0
    l32i    a2, a1, 4
    l32i    a3, a1, 8
    l32i    a4, a1, 12
    l32i    a5, a1, 16
    l32i    a6, a1, 20
    l32i    a7, a1, 24
    l32i    a8, a1, 28
    addi    a1, a1, 64
    
    /* Return from interrupt level 2 */
    rfi     2

/* Level 3 Interrupt Vector (High priority interrupts) */
    .section .Level3InterruptVector.literal, "a"
    .align 4

    .section .Level3InterruptVector.text, "ax"
    .global _Level3InterruptVector
    .align  4
_Level3InterruptVector:
    /* Save minimal context */
    addi    a1, a1, -64
    s32i    a0, a1, 0
    s32i    a2, a1, 4
    s32i    a3, a1, 8
    s32i    a4, a1, 12
    s32i    a5, a1, 16
    s32i    a6, a1, 20
    s32i    a7, a1, 24
    s32i    a8, a1, 28
    
    /* Save special registers */
    rsr.epc3 a2
    s32i    a2, a1, 32
    rsr.excsave3 a2
    s32i    a2, a1, 36
    
    /* Restore special registers */
    l32i    a2, a1, 32
    wsr.epc3 a2
    l32i    a2, a1, 36
    wsr.excsave3 a2
    
    /* Restore context */
    l32i    a0, a1, 0
    l32i    a2, a1, 4
    l32i    a3, a1, 8
    l32i    a4, a1, 12
    l32i    a5, a1, 16
    l32i    a6, a1, 20
    l32i    a7, a1, 24
    l32i    a8, a1, 28
    addi    a1, a1, 64
    
    /* Return from interrupt level 3 */
    rfi     3

/* Debug Exception Vector */
    .section .DebugExceptionVector.literal, "a"
    .align 4

    .section .DebugExceptionVector.text, "ax"
    .global _DebugExceptionVector
    .align  4
_DebugExceptionVector:
    /* Save minimal context */
    addi    a1, a1, -32
    s32i    a0, a1, 0
    s32i    a2, a1, 4
    s32i    a3, a1, 8
    
    /* Restore context */
    l32i    a0, a1, 0
    l32i    a2, a1, 4
    l32i    a3, a1, 8
    addi    a1, a1, 32
    
    /* Return from debug exception */
    rfdd

/* Kernel Exception Vector */
    .section .KernelExceptionVector.literal, "a"
    .align 4

    .section .KernelExceptionVector.text, "ax"
    .global _KernelExceptionVector
    .align  4
_KernelExceptionVector:
    /* Save context */
    addi    a1, a1, -64
    s32i    a0, a1, 0
    s32i    a2, a1, 4
    s32i    a3, a1, 8
    s32i    a4, a1, 12
    
    /* Save exception cause and address */
    rsr.exccause a2
    s32i    a2, a1, 16
    rsr.excvaddr a2
    s32i    a2, a1, 20
    
    /* Restore context */
    l32i    a0, a1, 0
    l32i    a2, a1, 4
    l32i    a3, a1, 8
    l32i    a4, a1, 12
    addi    a1, a1, 64
    
    /* Return from exception */
    rfe

/* User Exception Vector */
    .section .UserExceptionVector.literal, "a"
    .align 4

    .section .UserExceptionVector.text, "ax"
    .global _UserExceptionVector
    .align  4
_UserExceptionVector:
    /* Save context */
    addi    a1, a1, -64
    s32i    a0, a1, 0
    s32i    a2, a1, 4
    s32i    a3, a1, 8
    s32i    a4, a1, 12
    
    /* Save exception cause and address */
    rsr.exccause a2
    s32i    a2, a1, 16
    rsr.excvaddr a2
    s32i    a2, a1, 20
    
    /* Restore context */
    l32i    a0, a1, 0
    l32i    a2, a1, 4
    l32i    a3, a1, 8
    l32i    a4, a1, 12
    addi    a1, a1, 64
    
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
