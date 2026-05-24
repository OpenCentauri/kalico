/* ============================================================
   INTERRUPT AND EXCEPTION VECTOR HANDLERS
   ============================================================ */

    .set FRAME_PC,      0
    .set FRAME_PS,      4
    .set FRAME_A0,      8
    .set FRAME_A1,      12
    .set FRAME_A2,      16
    .set FRAME_A3,      20
    .set FRAME_A4,      24
    .set FRAME_A5,      28
    .set FRAME_A6,      32
    .set FRAME_A7,      36
    .set FRAME_A8,      40
    .set FRAME_A9,      44
    .set FRAME_A10,     48
    .set FRAME_A11,     52
    .set FRAME_A12,     56
    .set FRAME_A13,     60
    .set FRAME_A14,     64
    .set FRAME_A15,     68
    .set FRAME_SAR,     72
    .set FRAME_LEVEL,   76
    .set FRAME_LBEG,    80
    .set FRAME_LEND,    84
    .set FRAME_LCOUNT,  88
    .set FRAME_SIZE,    96  /* Keep 16-byte aligned */

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
    /* Make space on the stack for the interrupt frame */
    addi    a1, a1, -FRAME_SIZE

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
    rsr     a2, lbeg
    s32i    a2, a1, FRAME_LBEG
    rsr     a2, lend
    s32i    a2, a1, FRAME_LEND
    rsr     a2, lcount
    s32i    a2, a1, FRAME_LCOUNT
    
    /* Branch based on level */
    beqi    a0, 1, .Llevel1_save
    beqi    a0, 2, .Llevel2_save
    beqi    a0, 3, .Llevel3_save
    beqi    a0, 4, .Llevel4_save
    beqi    a0, 5, .Llevel5_save
    j       .Lsave_done     /* Unknown level, skip */

.Llevel1_save:
    rsr     a2, excsave1
    s32i    a2, a1, FRAME_A0
    rsr     a2, epc1
    s32i    a2, a1, FRAME_PC
    j       .Lsave_done

.Llevel2_save:
    rsr     a2, excsave2
    s32i    a2, a1, FRAME_A0
    rsr     a2, epc2
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps2
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done

.Llevel3_save:
    rsr     a2, excsave3
    s32i    a2, a1, FRAME_A0
    rsr     a2, epc3
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps3
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done

.Llevel4_save:
    rsr     a2, excsave4
    s32i    a2, a1, FRAME_A0
    rsr     a2, epc4
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps4
    s32i    a2, a1, FRAME_PS
    j       .Lsave_done
    /* fall through */

.Llevel5_save:
    rsr     a2, excsave5
    s32i    a2, a1, FRAME_A0
    rsr     a2, epc5
    s32i    a2, a1, FRAME_PC
    rsr     a2, eps5
    s32i    a2, a1, FRAME_PS
    /* fall through */

.Lsave_done:
    /* Synchronize after all special register reads */
    rsync
    memw
    
    /* Call C interrupt handler with level as argument */
    l32i    a2, a1, FRAME_LEVEL
    l32r    a0, .L_handler
    callx0  a0

    memw

    /* Restore all registers */
    l32i    a2, a1, FRAME_LCOUNT
    wsr     a2, lcount
    l32i    a2, a1, FRAME_LBEG
    wsr     a2, lbeg
    l32i    a2, a1, FRAME_LEND
    wsr     a2, lend
    l32i    a2, a1, FRAME_SAR
    wsr     a2, sar
    isync

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
    
    /* Restore based on level */
    l32i    a0, a1, FRAME_LEVEL
    
    beqi    a0, 1, .Llevel1_restore
    beqi    a0, 2, .Llevel2_restore
    beqi    a0, 3, .Llevel3_restore
    beqi    a0, 4, .Llevel4_restore
    beqi    a0, 5, .Llevel5_restore
    /* Unknown level, fall through */
    rfe

.Llevel1_restore:
    l32i    a0, a1, FRAME_PC
    wsr     a0, epc1
    addi    a1, a1, FRAME_SIZE
    rsr     a0, excsave1
    rfe

.Llevel2_restore:
    l32i    a0, a1, FRAME_PC
    wsr     a0, epc2
    l32i    a0, a1, FRAME_PS
    wsr     a0, eps2
    addi    a1, a1, FRAME_SIZE
    rsr     a0, excsave2
    rfi     2

.Llevel3_restore:
    l32i    a0, a1, FRAME_PC
    wsr     a0, epc3
    l32i    a0, a1, FRAME_PS
    wsr     a0, eps3
    addi    a1, a1, FRAME_SIZE
    rsr     a0, excsave3
    rfi     3

.Llevel4_restore:
    l32i    a0, a1, FRAME_PC
    wsr     a0, epc4
    l32i    a0, a1, FRAME_PS
    wsr     a0, eps4
    addi    a1, a1, FRAME_SIZE
    rsr     a0, excsave4
    rfdd

.Llevel5_restore:
    l32i    a0, a1, FRAME_PC
    wsr     a0, epc5
    l32i    a0, a1, FRAME_PS
    wsr     a0, eps5
    addi    a1, a1, FRAME_SIZE
    rsr     a0, excsave5
    rfe


/* Level 2 Interrupt Vector (Medium priority interrupts) */
    .section .Level2InterruptVector.text, "ax"
    .global _Level2InterruptVector
    .align  4
_Level2InterruptVector:
    wsr     a0, excsave2            /* preserve a0 */
    movi    a0, 2                   /* Interrupt level */
    j       _xt_int_enter

/* Level 3 Interrupt Vector (High priority interrupts) */
    .section .Level3InterruptVector.text, "ax"
    .global _Level3InterruptVector
    .align  4
_Level3InterruptVector:
    wsr     a0, excsave3            /* preserve a0 */
    movi    a0, 3                   /* Interrupt level */
    j       _xt_int_enter

/* Debug Exception Vector */
    .section .DebugExceptionVector.text, "ax"
    .global _DebugExceptionVector
    .align  4
_DebugExceptionVector:
    wsr     a0, excsave4            /* preserve a0 */
    movi    a0, 4                   /* Interrupt level */
    j       _xt_int_enter

/* Kernel Exception Vector */
    .section .KernelExceptionVector.text, "ax"
    .global _KernelExceptionVector
    .align  4
_KernelExceptionVector:
    wsr     a0, excsave1            /* preserve a0 */
    movi    a0, 1                   /* Interrupt level */
    j       _xt_int_enter

/* User Exception Vector */
    .section .UserExceptionVector.text, "ax"
    .global _UserExceptionVector
    .align  4
_UserExceptionVector:
    wsr     a0, excsave5            /* preserve a0 */
    movi    a0, 5                   /* Interrupt level */
    j       _xt_int_enter

/* Double Exception Vector (Critical - handles exceptions during exception handling) */
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
