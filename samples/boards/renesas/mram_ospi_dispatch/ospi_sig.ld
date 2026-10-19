/*
 * Signature slot for the code placed in the external flash. It is linked
 * into the external flash and patched after link by scripts/ospi_sign.py.
 */
SECTION_PROLOGUE(.extflash_sig,,)
{
	. = ALIGN(4);
	__extflash_sig_start = .;
	KEEP(*(.extflash_sig))
	__extflash_sig_end = .;
} GROUP_LINK_IN(EXTFLASH)
