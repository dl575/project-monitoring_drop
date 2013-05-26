#include "monitoring.h"

inline unsigned round_up_uint(unsigned val, unsigned align)
{
    unsigned mask = align - 1;
    return (val + mask) & ~mask;
}

inline void set_tag(unsigned addr, unsigned size, unsigned value)
{
    if (size == 0)
        return;

    register volatile unsigned int *fifo = (unsigned int *)MONITOR_ADDR;
    // printf("set taint [0x%x-0x%x]=%d\n", addr, size, value);
    unsigned bound = addr + size;

    while (addr < bound) {
        unsigned next_addr = round_up_uint(addr, ISA_PAGE_SIZE);
        if (next_addr == addr)
            next_addr += ISA_PAGE_SIZE;
        next_addr = next_addr < bound ? next_addr : bound;

        *(fifo+1) = addr;
        *(fifo+4) = next_addr - addr;
        *(fifo+5) = value;

        addr = next_addr;
    }
}

void init_section_tags()
{
    // initialize .code and .rodata
    extern void * _init , * __exidx_end;
    set_tag((unsigned)&_init, (unsigned)&__exidx_end-(unsigned)&_init+1, 1);
    // initialize .preinit_array, .init_array and .fini_array
    extern void * __preinit_array_start, * __fini_array_end;
    set_tag((unsigned)&__preinit_array_start, (unsigned)&__fini_array_end-(unsigned)&__preinit_array_start+1, 1);
    // initialize .data
    extern void * __data_start , * _edata;
    set_tag((unsigned)&__data_start, (unsigned)&_edata-(unsigned)&__data_start+1, 1);    
    // initialize .bss
    extern void * __bss_start__ , * __bss_end__;
    set_tag((unsigned)&__bss_start__, (unsigned)&__bss_end__-(unsigned)&__bss_start__+1, 1);
}

inline void set_tag_base(unsigned addr, unsigned value)
{
    register volatile unsigned int *fifo = (unsigned int *)MONITOR_ADDR;
    *(fifo+1) = addr;
    // use this field to distinguish between base and bound
    *(fifo+2) = 0;
    *(fifo+3) = value;
}

inline void set_tag_bound(unsigned addr, unsigned value)
{
    register volatile unsigned int *fifo = (unsigned int *)MONITOR_ADDR;
    *(fifo+1) = addr;
    // use this field to distinguish between base and bound
    *(fifo+2) = 1;
    *(fifo+3) = value;
}
