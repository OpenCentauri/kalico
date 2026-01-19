/**
 * @file cache.c
 * @brief Cache management for HiFi4 DSP on Allwinner R528/T113
 * 
 * This HiFi4 configuration:
 *   - 32 KB I-cache, 64-byte lines, 2 ways
 *   - 32 KB D-cache, 64-byte lines, 4 ways
 *   - Writeback D-cache
 *   - NO CACHEATTR register (XCHAL_HAVE_CACHEATTR = 0)
 *   - Uses TLB for region protection with translation (XCHAL_HAVE_XLT_CACHEATTR = 1)
 *   - Has spanning way TLB (XCHAL_HAVE_SPANNING_WAY = 1)
 * 
 * Cache attributes are controlled by TLB entries, not a CACHEATTR register.
 * The initial TLB configuration determines which regions are cached.
 * 
 * To change caching behavior, you need to modify the TLB entries using
 * WITLB/WDTLB instructions (write I/D TLB).
 */

#include "hal.h"

/*============================================================================
 * Cache Configuration from core-isa.h
 *============================================================================*/

/* Use the actual values from the Xtensa config */
#define DCACHE_LINE_SIZE    64      /* 64 bytes */
#define ICACHE_LINE_SIZE    64      /* 64 bytes */
#define XCHAL_ICACHE_LINEWIDTH		6	/* log2(I line size in bytes) */
#define XCHAL_DCACHE_LINEWIDTH		6	/* log2(D line size in bytes) */
#define DCACHE_SIZE         32768   /* 32768 bytes */
#define ICACHE_SIZE         32768   /* 32768 bytes */
#define DCACHE_WAYS         4       /* 4 */
#define ICACHE_WAYS         2       /* 2 */

/*============================================================================
 * TLB Cache Attribute Values (for XLT_CACHEATTR configs)
 * 
 * These are the attribute values used in TLB entries.
 * The exact encoding depends on the Xtensa configuration.
 * Common values for region protection:
 *   0x0 = No access (generates exception)
 *   0x1 = Bypass cache (device/uncached memory)
 *   0x2 = Write-through cached
 *   0x4 = Write-back cached
 *   0xE = Isolate (for cache testing)
 *   0xF = Illegal
 *============================================================================*/

#define CA_BYPASS       1   /* Bypass cache (uncached) */
#define CA_WRITETHRU    2   /* Write-through cache */
#define CA_WRITEBACK    4   /* Write-back cache */
#define CA_ILLEGAL      15  /* Illegal access */

/*============================================================================
 * Data Cache Operations
 *============================================================================*/

void dcache_region_invalidate(void *addr, uint32_t size)
{
    uint32_t start = (uint32_t)addr & ~(DCACHE_LINE_SIZE - 1);
    uint32_t end = (uint32_t)addr + size;
    
    for (uint32_t a = start; a < end; a += DCACHE_LINE_SIZE) {
        __asm__ volatile("dhi %0, 0" :: "a"(a));
    }
    __asm__ volatile("dsync");
}

void dcache_region_writeback(void *addr, uint32_t size)
{
    uint32_t start = (uint32_t)addr & ~(DCACHE_LINE_SIZE - 1);
    uint32_t end = (uint32_t)addr + size;
    
    for (uint32_t a = start; a < end; a += DCACHE_LINE_SIZE) {
        __asm__ volatile("dhwb %0, 0" :: "a"(a));
    }
    __asm__ volatile("dsync");
}

void dcache_region_writeback_invalidate(void *addr, uint32_t size)
{
    uint32_t start = (uint32_t)addr & ~(DCACHE_LINE_SIZE - 1);
    uint32_t end = (uint32_t)addr + size;
    
    for (uint32_t a = start; a < end; a += DCACHE_LINE_SIZE) {
        __asm__ volatile("dhwbi %0, 0" :: "a"(a));
    }
    __asm__ volatile("dsync");
}

/*============================================================================
 * Instruction Cache Operations
 *============================================================================*/

void icache_region_invalidate(void *addr, uint32_t size)
{
    uint32_t start = (uint32_t)addr & ~(ICACHE_LINE_SIZE - 1);
    uint32_t end = (uint32_t)addr + size;
    
    for (uint32_t a = start; a < end; a += ICACHE_LINE_SIZE) {
        __asm__ volatile("ihi %0, 0" :: "a"(a));
    }
    __asm__ volatile("isync");
}

/*============================================================================
 * Whole Cache Operations
 *============================================================================*/

void dcache_invalidate_all(void)
{
    /* 
     * Index-based invalidate iterates through all cache indices.
     * For a 32KB, 4-way, 64-byte line cache:
     *   Sets = 32768 / (64 * 4) = 128 sets
     */
    uint32_t sets = DCACHE_SIZE / (DCACHE_LINE_SIZE * DCACHE_WAYS);
    
    for (uint32_t way = 0; way < DCACHE_WAYS; way++) {
        for (uint32_t set = 0; set < sets; set++) {
            /* Index format: set in upper bits, way in lower bits */
            uint32_t index = (set << XCHAL_DCACHE_LINEWIDTH) | (way << 1);
            __asm__ volatile("dii %0, 0" :: "a"(index));
        }
    }
    __asm__ volatile("dsync");
}

void dcache_writeback_all(void)
{
    uint32_t sets = DCACHE_SIZE / (DCACHE_LINE_SIZE * DCACHE_WAYS);
    
    for (uint32_t way = 0; way < DCACHE_WAYS; way++) {
        for (uint32_t set = 0; set < sets; set++) {
            uint32_t index = (set << XCHAL_DCACHE_LINEWIDTH) | (way << 1);
            __asm__ volatile("diwb %0, 0" :: "a"(index));
        }
    }
    __asm__ volatile("dsync");
}

void dcache_writeback_invalidate_all(void)
{
    uint32_t sets = DCACHE_SIZE / (DCACHE_LINE_SIZE * DCACHE_WAYS);
    
    for (uint32_t way = 0; way < DCACHE_WAYS; way++) {
        for (uint32_t set = 0; set < sets; set++) {
            uint32_t index = (set << XCHAL_DCACHE_LINEWIDTH) | (way << 1);
            __asm__ volatile("diwbi %0, 0" :: "a"(index));
        }
    }
    __asm__ volatile("dsync");
}

void icache_invalidate_all(void)
{
    uint32_t sets = ICACHE_SIZE / (ICACHE_LINE_SIZE * ICACHE_WAYS);
    
    for (uint32_t way = 0; way < ICACHE_WAYS; way++) {
        for (uint32_t set = 0; set < sets; set++) {
            uint32_t index = (set << XCHAL_ICACHE_LINEWIDTH) | (way << 1);
            __asm__ volatile("iii %0, 0" :: "a"(index));
        }
    }
    __asm__ volatile("isync");
}

void cache_sync(void)
{
    __asm__ volatile("dsync" ::: "memory");
    __asm__ volatile("isync");
}

/*============================================================================
 * TLB-based Cache Attribute Control
 * 
 * This configuration uses a "spanning way" TLB that covers the entire
 * 4GB address space with 512MB regions. Each region has a cache attribute.
 * 
 * The DTLB and ITLB are separate, so we need to configure both.
 *============================================================================*/

/**
 * @brief Read DTLB entry for a given virtual address
 * @param vaddr Virtual address to look up
 * @return TLB entry value (contains PPN and attributes)
 */
static uint32_t read_dtlb(uint32_t vaddr)
{
    uint32_t result;
    __asm__ volatile("rdtlb1 %0, %1" : "=a"(result) : "a"(vaddr));
    return result;
}

/**
 * @brief Read ITLB entry for a given virtual address
 */
static uint32_t read_itlb(uint32_t vaddr)
{
    uint32_t result;
    __asm__ volatile("ritlb1 %0, %1" : "=a"(result) : "a"(vaddr));
    return result;
}

/**
 * @brief Write DTLB entry
 * @param vaddr Virtual address (determines which TLB entry)
 * @param pte Page table entry value (PPN + attributes)
 * 
 * For spanning way, vaddr selects the 512MB region.
 * The entry value format depends on the config, typically:
 *   [31:12] = Physical page number (PPN) 
 *   [3:0] = Cache attribute
 */
static void write_dtlb(uint32_t vaddr, uint32_t pte)
{
    __asm__ volatile(
        "wdtlb %0, %1\n"
        "dsync\n"
        :: "a"(pte), "a"(vaddr)
    );
}

/**
 * @brief Write ITLB entry
 */
static void write_itlb(uint32_t vaddr, uint32_t pte)
{
    __asm__ volatile(
        "witlb %0, %1\n"
        "isync\n"
        :: "a"(pte), "a"(vaddr)
    );
}

/**
 * @brief Set cache attribute for a memory region
 * @param vaddr Start of 512MB region (0x00000000, 0x20000000, etc.)
 * @param attr Cache attribute (CA_BYPASS, CA_WRITEBACK, etc.)
 * 
 * For identity mapping (vaddr == paddr), we just need to set the attribute.
 */
void cache_set_region_attr(uint32_t vaddr, uint32_t attr)
{
    /* Read current entry to get PPN */
    uint32_t dtlb = read_dtlb(vaddr);
    uint32_t itlb = read_itlb(vaddr);
    
    /* Clear old attribute, set new one */
    dtlb = (dtlb & ~0xF) | (attr & 0xF);
    itlb = (itlb & ~0xF) | (attr & 0xF);
    
    /* Write back */
    write_dtlb(vaddr, dtlb);
    write_itlb(vaddr, itlb);
}

/*============================================================================
 * Cache Initialization
 *============================================================================*/

/**
 * @brief Initialize caches
 * 
 * On this TLB-based system, cache behavior depends on TLB entries.
 * The default TLB setup (from reset vector/LSP) should already have
 * reasonable settings. We just invalidate to start fresh.
 */
void cache_init(void)
{
    /* Invalidate both caches */
    icache_invalidate_all();
    dcache_invalidate_all();
    
    __asm__ volatile("dsync");
    __asm__ volatile("isync");
}

/**
 * @brief Enable caching for DDR region at 0x30000000
 * 
 * Sets the 0x20000000-0x3FFFFFFF region to write-back cached.
 */
void cache_enable_ddr(void)
{
    /* Flush and invalidate first */
    dcache_writeback_invalidate_all();
    icache_invalidate_all();
    
    /* Set region 1 (0x20000000-0x3FFFFFFF) to write-back */
    cache_set_region_attr(0x20000000, CA_WRITEBACK);
    
    __asm__ volatile("dsync");
    __asm__ volatile("isync");
}

/*============================================================================
 * Debug Functions
 *============================================================================*/

void cache_dump_config(void)
{
    hal_debug_print("\n=== Cache Configuration ===\n");
    
    hal_debug_print("D-Cache: ");
    hal_debug_hex(DCACHE_SIZE);
    hal_debug_print(" bytes, ");
    hal_debug_hex(DCACHE_LINE_SIZE);
    hal_debug_print("-byte lines, ");
    hal_debug_hex(DCACHE_WAYS);
    hal_debug_print(" ways\n");
    
    hal_debug_print("I-Cache: ");
    hal_debug_hex(ICACHE_SIZE);
    hal_debug_print(" bytes, ");
    hal_debug_hex(ICACHE_LINE_SIZE);
    hal_debug_print("-byte lines, ");
    hal_debug_hex(ICACHE_WAYS);
    hal_debug_print(" ways\n");
    
    hal_debug_print("\nTLB Cache Attributes by Region:\n");
    
    /* Read and display TLB entries for each 512MB region */
    for (uint32_t region = 0; region < 8; region++) {
        uint32_t vaddr = region * 0x20000000;
        uint32_t dtlb = read_dtlb(vaddr);
        uint32_t itlb = read_itlb(vaddr);
        
        hal_debug_print("  0x");
        hal_debug_hex(vaddr);
        hal_debug_print(": DTLB=");
        hal_debug_hex(dtlb);
        hal_debug_print(" ITLB=");
        hal_debug_hex(itlb);
        
        /* Decode attribute */
        uint32_t d_attr = dtlb & 0xF;
        hal_debug_print(" (");
        switch (d_attr) {
            case 0:  hal_debug_print("NO ACCESS"); break;
            case 1:  hal_debug_print("BYPASS"); break;
            case 2:  hal_debug_print("WRITE-THRU"); break;
            case 4:  hal_debug_print("WRITE-BACK"); break;
            case 14: hal_debug_print("ISOLATE"); break;
            case 15: hal_debug_print("ILLEGAL"); break;
            default: hal_debug_print("UNKNOWN"); break;
        }
        hal_debug_print(")\n");
    }
}
