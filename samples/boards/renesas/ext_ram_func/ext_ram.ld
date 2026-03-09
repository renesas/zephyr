SECTION_DATA_PROLOGUE(.ext_ram,,)
{
	. = ALIGN(4);
	PLACE_SYMBOL_HERE(__ext_ram_memory_start);
	KEEP(*(.ext_ramfunc*))
	PLACE_SYMBOL_HERE(__ext_ram_memory_end);
	. = ALIGN(4);

} GROUP_DATA_LINK_IN(PSRAM, ROMABLE_REGION)

#if defined(CONFIG_RX)
___ext_ram_memory_size = ___ext_ram_memory_end - ___ext_ram_memory_start;
___ext_ram_memory_load_start = LOADADDR(.ext_ram);
___ext_ram_memory_region_load_start = LOADADDR(.ext_ram);
PROVIDE(__ext_ram_memory_size = ___ext_ram_memory_size);
PROVIDE(__ext_ram_memory_load_start = ___ext_ram_memory_load_start);
PROVIDE(__ext_ram_memory_region_load_start = ___ext_ram_memory_region_load_start);
#else
__ext_ram_memory_size = __ext_ram_memory_end - __ext_ram_memory_start;
__ext_ram_memory_load_start = LOADADDR(.ext_ram);
__ext_ram_memory_region_load_start = LOADADDR(.ext_ram);
#endif
