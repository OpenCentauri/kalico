ENTRY(_ResetVector)

MEMORY
{
    iram0 (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00010000
    ddr1  (rwx) : ORIGIN = 0x32000000, LENGTH = 0x00100000
}

PHDRS
{
    iram0_phdr PT_LOAD;
    ddr1_phdr  PT_LOAD;
}

PROVIDE(_memmap_vecbase_reset = ORIGIN(iram0) + 0x800);  /* Vector base with room for oemhead */

SECTIONS
{
    /* OEM header section at start of IRAM */
    .oemhead.text 0x00400000 : {
        KEEP(*(.oemhead.text))
    } > iram0 :iram0_phdr
    
    .ResetVector.literal 0x00400800 : ALIGN(4) {
        *(.ResetVector.literal)
    } > iram0 :iram0_phdr
    
    .ResetVector.text : ALIGN(4) {
        *(.ResetVector.text)
    } > iram0 :iram0_phdr
    
    /* Interrupt vector literals - MUST come before vector code */
    .Level2InterruptVector.literal 0x00401000 : ALIGN(4) {
        *(.Level2InterruptVector.literal)
    } > iram0 :iram0_phdr
    
    /* Interrupt vectors at vecbase + offsets */
    .Level2InterruptVector.text 0x00401180 : ALIGN(4) {
        KEEP(*(.Level2InterruptVector.text))
    } > iram0 :iram0_phdr
    
    .Level3InterruptVector.literal : ALIGN(4) {
        *(.Level3InterruptVector.literal)
    } > iram0 :iram0_phdr
    
    .Level3InterruptVector.text 0x00401200 : ALIGN(4) {
        KEEP(*(.Level3InterruptVector.text))
    } > iram0 :iram0_phdr
    
    .DebugExceptionVector.literal : ALIGN(4) {
        *(.DebugExceptionVector.literal)
    } > iram0 :iram0_phdr
    
    .DebugExceptionVector.text 0x00401280 : ALIGN(4) {
        KEEP(*(.DebugExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .KernelExceptionVector.literal : ALIGN(4) {
        *(.KernelExceptionVector.literal)
    } > iram0 :iram0_phdr
    
    .KernelExceptionVector.text 0x00401300 : ALIGN(4) {
        KEEP(*(.KernelExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .UserExceptionVector.literal : ALIGN(4) {
        *(.UserExceptionVector.literal)
    } > iram0 :iram0_phdr
    
    .UserExceptionVector.text 0x00401380 : ALIGN(4) {
        KEEP(*(.UserExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .DoubleExceptionVector.literal : ALIGN(4) {
        *(.DoubleExceptionVector.literal)
    } > iram0 :iram0_phdr
    
    .DoubleExceptionVector.text 0x00401400 : ALIGN(4) {
        KEEP(*(.DoubleExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .text : ALIGN(4) {
        *(.literal .literal.* .text .text.*)
        *(.stub)
        *(.gnu.linkonce.literal.* .gnu.linkonce.t.*)
        *(.fini.literal)
        *(.fini)
        *(.gnu.version)
    } > ddr1 :ddr1_phdr
    
    .rodata : ALIGN(4) {
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
        *(.rodata1)
    } > ddr1 :ddr1_phdr
    
    .data : ALIGN(4) {
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.data1)
        *(.sdata .sdata.* .sdata2.*)
        *(.gnu.linkonce.s.*)
    } > ddr1 :ddr1_phdr
    
    .bss (NOLOAD) : ALIGN(4) {
        _bss_start = ABSOLUTE(.);
        *(.dynsbss)
        *(.sbss .sbss.* .scommon)
        *(.dynbss)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = ABSOLUTE(.);
    } > ddr1 :ddr1_phdr
    
    /* Heap section - place after BSS */
    .heap (NOLOAD) : ALIGN(8) {
        _heap_start = ABSOLUTE(.);
        . = . + 0x10000;  /* 64KB heap */
        _heap_end = ABSOLUTE(.);
    } > ddr1 :ddr1_phdr

    /* Reserve space for stack at end of DDR - MOVED AFTER HEAP */
    .stack (NOLOAD) : ALIGN(16) {
        . = . + 0x8000;  /* 32KB stack */
        _stack_end = ABSOLUTE(.);
    } > ddr1 :ddr1_phdr
    
    /* Update stack pointer to point to END of stack section */
    PROVIDE(__stack = _stack_end);
    
    _end = .;
}
