/* Entry point */
ENTRY(_start)

/* Memory regions */
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

/* Stack and heap sizes */
__stack_size = 32K;
__heap_size = 64K;

SECTIONS
{
    /* OEM header section at start of IRAM */
    .oemhead.text : ALIGN(4)
    {
        KEEP(*(.oemhead.text))
    } > iram0 :iram0_phdr

    /* Code section */
    .text : ALIGN(4)
    {
        _stext = .;

        /* Literal pools for vectors - MUST come before vectors */
        KEEP(*(.vectors.literal))
        
        /* Reset and exception vectors first */
        KEEP(*(.vectors))
        KEEP(*(.vectors.*))

        /* Literal pools for init section */
        KEEP(*(.init.literal))
        
        /* Init code */
        KEEP(*(.init))
        KEEP(*(.init.*))
        
        /* General literal pools - place before/interleaved with code */
        *(.literal .literal.* .stub .gnu.warning .gnu.linkonce.literal.*)
        *(.gnu.linkonce.t.*.literal)
        
        /* Main code */
        *(.text)
        *(.text.*)
        *(.gnu.linkonce.t.*)
        
        /* More literals that might be generated */
        *(.literal.*)
        
        /* Read-only data embedded in text for l32r access */
        . = ALIGN(4);
        *(.rodata)
        *(.rodata.*)
        *(.rodata1)
        *(.gnu.linkonce.r.*)
        
        /* C++ constructors/destructors */
        . = ALIGN(4);
        __CTOR_LIST__ = .;
        KEEP(*(.ctors))
        __CTOR_END__ = .;
        
        __DTOR_LIST__ = .;
        KEEP(*(.dtors))
        __DTOR_END__ = .;
        
        . = ALIGN(4);
        _etext = .;
    } > iram0 :iram0_phdr

    /* Initialized data */
    .data : ALIGN(4)
    {
        _sdata = .;
        
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d.*)
        
        . = ALIGN(4);
        _edata = .;
    } > ddr1 :ddr1_phdr

    /* Load address for .data section (for copying at startup) */
    _sidata = LOADADDR(.data);

    /* Uninitialized data (BSS) */
    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        __bss_start__ = .;
        
        *(.bss)
        *(.bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = .;
    } > ddr1 :ddr1_phdr

    /* Heap */
    .heap (NOLOAD) : ALIGN(8)
    {
        __heap_start = .;
        . += __heap_size;
        __heap_end = .;
    } > ddr1 :ddr1_phdr

    /* Stack */
    .stack (NOLOAD) : ALIGN(16)
    {
        __stack_start = .;
        . += __stack_size;
        __stack_end = .;
        __stack_top = .;
    } > ddr1 :ddr1_phdr

    /* Debug information (optional, removed in release builds) */
    .debug_info     0 : { *(.debug_info)     }
    .debug_abbrev   0 : { *(.debug_abbrev)   }
    .debug_line     0 : { *(.debug_line)     }
    .debug_frame    0 : { *(.debug_frame)    }
    .debug_str      0 : { *(.debug_str)      }
    .debug_loc      0 : { *(.debug_loc)      }
    .debug_ranges   0 : { *(.debug_ranges)   }

    /* Discard sections */
    /DISCARD/ :
    {
        *(.note.GNU-stack)
        *(.comment)
    }
}

/* Provide symbols for startup code */
PROVIDE(_stack_top = __stack_top);
PROVIDE(_heap_start = __heap_start);
PROVIDE(_heap_end = __heap_end);
