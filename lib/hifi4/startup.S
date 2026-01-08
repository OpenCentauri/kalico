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
    call0   main

    /* If main returns, loop forever */
.Lhang:
    call0   .Lhang
