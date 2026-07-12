ENTRY(_ResetVector)

MEMORY
{
    iram0 (rx)      : ORIGIN = 0x00400000, LENGTH = 0x00010000
    dram0 (rw)      : ORIGIN = 0x00420000, LENGTH = 0x00008000
    dram1 (rw)      : ORIGIN = 0x00440000, LENGTH = 0x00008000
    sram_trace (rw) : ORIGIN = 0x00027000, LENGTH = 0x00001000
}

PHDRS
{
    iram0_phdr PT_LOAD;
    rodata_phdr PT_LOAD;
    dram0_phdr PT_LOAD;
    dram1_phdr PT_LOAD;
}

PROVIDE(_memmap_reset_vector = 0x00400660);
PROVIDE(_memmap_vecbase_reset = 0x00401000);

SECTIONS
{
    .resource_table : ALIGN(4) {
        KEEP(*(.resource_table))
    } > iram0 :iram0_phdr

    .trace_buf (NOLOAD) : ALIGN(4)
    {
        _trace_buf_start = .;
        KEEP(*(.trace_buf))
        _trace_buf_end = .;
    } > sram_trace :NONE

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
    
    .text 0x00404000 : ALIGN(4) {
        *(.literal .literal.* .text .text.*)
        *(.stub)
        *(.gnu.linkonce.literal.* .gnu.linkonce.t.*)
        *(.fini.literal)
        *(.fini)
        *(.gnu.version)
    } > iram0 :iram0_phdr
    
    .rodata : ALIGN(4) {
        _rodata_start = ABSOLUTE(.);
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
        *(.rodata1)
        . = ALIGN(4);
        _rodata_end = ABSOLUTE(.);
    } > dram0 AT > iram0 :rodata_phdr

    _rodata_lma = LOADADDR(.rodata);
    
    .data : ALIGN(4) {
        _data_start = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.data1)
        *(.sdata .sdata.* .sdata2.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(4);
        _data_end = ABSOLUTE(.);
    } > dram0 AT > iram0 :rodata_phdr

    _data_lma = LOADADDR(.data);
    
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
    } > dram0 :NONE
    
    /* Heap section - place after BSS */
    .heap (NOLOAD) : ALIGN(8) {
        _heap_start = ABSOLUTE(.);
        . = . + 0x2000;  /* 8KB heap */
        _heap_end = ABSOLUTE(.);
    } > dram1 :NONE

    /* Reserve space for stack in DRAM1, after the heap */
    .stack (NOLOAD) : ALIGN(16) {
        . = . + 0x1000;  /* 4 KB stack */
        _stack_end = ABSOLUTE(.);
    } > dram1 :NONE
    
    /* Update stack pointer to point to END of stack section */
    PROVIDE(__stack = _stack_end);
    
    _end = .;
}
