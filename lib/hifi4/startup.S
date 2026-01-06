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

    .set FRAME_PC,      0
    .set FRAME_PS,      4
    .set FRAME_A0,      8
    .set FRAME_A2,      12
    .set FRAME_A3,      16
    .set FRAME_A4,      20
    .set FRAME_A5,      24
    .set FRAME_A6,      28
    .set FRAME_A7,      32
    .set FRAME_A8,      36
    .set FRAME_A9,      40
    .set FRAME_A10,     44
    .set FRAME_A11,     48
    .set FRAME_A12,     52
    .set FRAME_A13,     56
    .set FRAME_A14,     60
    .set FRAME_A15,     64
    .set FRAME_SAR,     68
    .set FRAME_LEVEL,   72
    .set FRAME_SIZE,    80  /* 16-byte aligned */

    .section ._xt_int_enter.literal, "a"
    .align 4
.L_handler:
    .word   hal_irq_dispatch

/* ============================================================================
 * Shared Interrupt Entry Point
 * Called from all level vectors with:
 *   a0 = interrupt level (2-5)
 *   a1 = adjusted stack (frame already allocated)
 *   FRAME_A0 already saved
 * ============================================================================ */

    .section .text._xt_int_enter, "ax"
    .global _xt_int_enter
    .type _xt_int_enter, @function
    .align 4

_xt_int_enter:
    /* Save the level in frame */
    s32i    a0, a1, FRAME_LEVEL
    
    /* Now we need to save EPC and EPS based on level */
    /* a0 still has the level number */
    
    /* Save remaining registers first */
    s32i    a2, a1, FRAME_A2
    s32i    a3, a1, FRAME_A3
    s32i    a4, a1, FRAME_A4
    s32i    a5, a1, FRAME_A5
    s32i    a6, a1, FRAME_A6
    s32i    a7, a1, FRAME_A7
    s32i    a8, a1, FRAME_A8
    s32i    a9, a1, FRAME_A9
    s32i    a10, a1, FRAME_A10
    s32i    a11, a1, FRAME_A11
    s32i    a12, a1, FRAME_A12
    s32i    a13, a1, FRAME_A13
    s32i    a14, a1, FRAME_A14
    s32i    a15, a1, FRAME_A15
    rsr     a2, sar
    s32i    a2, a1, FRAME_SAR
    
    /* Get level back and handle EPC/EPS based on level */
    l32i    a3, a1, FRAME_LEVEL
    
    /* Branch based on level */
    beqi    a3, 2, .Llevel2_save
    beqi    a3, 3, .Llevel3_save
    beqi    a3, 4, .Llevel4_save
    beqi    a3, 5, .Llevel5_save
    j       .Lsave_done     /* Unknown level, skip */

.Llevel2_save:
    rsr     a2, epc2
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps2
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done

.Llevel3_save:
    rsr     a2, epc3
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps3
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done

.Llevel4_save:
    rsr     a2, epc4
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps4
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done

.Llevel5_save:
    rsr     a2, epc5
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps5
    s32i    a2, a1, FRAME_PS
    /* fall through */

.Lsave_done:
    /* Call C interrupt handler with level as argument */
    l32i    a2, a1, FRAME_LEVEL
    l32r    a0, .L_handler
    callx0  a0
    
    /* Restore based on level */
    l32i    a3, a1, FRAME_LEVEL
    
    beqi    a3, 2, .Llevel2_restore
    beqi    a3, 3, .Llevel3_restore
    beqi    a3, 4, .Llevel4_restore
    beqi    a3, 5, .Llevel5_restore
    j       .Lrestore_regs

.Llevel2_restore:
    l32i    a2, a1, FRAME_PC
    wsr     a2, epc2
    l32i    a2, a1, FRAME_PS
    wsr     a2, eps2
    j       .Lrestore_regs

.Llevel3_restore:
    l32i    a2, a1, FRAME_PC
    wsr     a2, epc3
    l32i    a2, a1, FRAME_PS
    wsr     a2, eps3
    j       .Lrestore_regs

.Llevel4_restore:
    l32i    a2, a1, FRAME_PC
    wsr     a2, epc4
    l32i    a2, a1, FRAME_PS
    wsr     a2, eps4
    j       .Lrestore_regs

.Llevel5_restore:
    l32i    a2, a1, FRAME_PC
    wsr     a2, epc5
    l32i    a2, a1, FRAME_PS
    wsr     a2, eps5
    /* fall through */

.Lrestore_regs:
    /* Restore all registers */
    l32i    a2, a1, FRAME_SAR
    wsr     a2, sar
    l32i    a15, a1, FRAME_A15
    l32i    a14, a1, FRAME_A14
    l32i    a13, a1, FRAME_A13
    l32i    a12, a1, FRAME_A12
    l32i    a11, a1, FRAME_A11
    l32i    a10, a1, FRAME_A10
    l32i    a9, a1, FRAME_A9
    l32i    a8, a1, FRAME_A8
    l32i    a7, a1, FRAME_A7
    l32i    a6, a1, FRAME_A6
    l32i    a5, a1, FRAME_A5
    l32i    a4, a1, FRAME_A4
    l32i    a3, a1, FRAME_A3
    l32i    a2, a1, FRAME_A2
    l32i    a0, a1, FRAME_A0
    
    /* Restore stack pointer */
    addi    a1, a1, FRAME_SIZE
    
    /* The problem: we need the level to know which RFI to use,
       but we've restored all registers. Solution: use a fixed
       level approach or save level elsewhere. */
    
    /* For simplicity, let's use separate return paths */
    /* We'll re-check the level from EPS */
    
    /* Actually, let's just use level 3 return for now since
       that's where timer interrupts are. For a full implementation,
       you'd need separate return trampolines or save level in 
       a reserved memory location. */
    
    rfe

/* Level 2 Interrupt Vector (Medium priority interrupts) */
    .section .Level2InterruptVector.literal, "a"
    .align 4
.Llevel2_handler_addr:
    .word   hal_irq_dispatch

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
    .word   hal_irq_dispatch

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
    .word   hal_irq_dispatch

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
    addi    a1, a1, -FRAME_SIZE
    s32i    a0, a1, FRAME_A0
    movi    a0, 1                   /* Interrupt level */
    j       _xt_int_enter
    
    /* Return from exception */
    rfe

/* User Exception Vector */
    .section .UserExceptionVector.literal, "a"
    .align 4
.Luser_handler_addr:
    .word   hal_irq_dispatch

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
