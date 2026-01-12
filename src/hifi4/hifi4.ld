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

PROVIDE(_memmap_reset_vector = 0x00400660);
PROVIDE(_memmap_vecbase_reset = 0x00401000);

SECTIONS
{
    /* OEM header section at start of IRAM */
    .oemhead.text 0x00400000 : ALIGN(4) {
        _oemhead_text_start = ABSOLUTE(.);
        KEEP(*(.oemhead.text))
    } > iram0 :iram0_phdr
    
    ._xt_int_enter.literal : ALIGN(4) {
        *(._xt_int_enter.literal)
    } > iram0 :iram0_phdr

    .text._xt_int_enter : ALIGN(4) {
        KEEP(*(.text._xt_int_enter))
    } > iram0 :iram0_phdr
    
    .ResetVector.literal : ALIGN(4) {
        *(.ResetVector.literal)
    } > iram0 :iram0_phdr
    
    .ResetVector.text 0x00400660 : ALIGN(4) {
        *(.ResetVector.text)
    } > iram0 :iram0_phdr
    
    /* Interrupt vectors at vecbase + offsets */
    .Level2InterruptVector.text 0x0040117C : ALIGN(4) {
        KEEP(*(.Level2InterruptVector.text))
    } > iram0 :iram0_phdr
    
    .Level3InterruptVector.text 0x0040119C : ALIGN(4) {
        KEEP(*(.Level3InterruptVector.text))
    } > iram0 :iram0_phdr
    
    .DebugExceptionVector.text 0x004011BC : ALIGN(4) {
        KEEP(*(.DebugExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .NMIExceptionVector.text 0x004011DC : ALIGN(4) {
        KEEP(*(.NMIExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .KernelExceptionVector.text 0x004011FC : ALIGN(4) {
        KEEP(*(.KernelExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .UserExceptionVector.text 0x0040121C : ALIGN(4) {
        KEEP(*(.UserExceptionVector.text))
    } > iram0 :iram0_phdr
    
    .DoubleExceptionVector.text 0x0040123C : ALIGN(4) {
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
