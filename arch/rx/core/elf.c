/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/llext/elf.h>
#include <zephyr/llext/llext.h>
#include <zephyr/llext/llext_internal.h>
#include <zephyr/llext/loader.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(elf, CONFIG_LLEXT_LOG_LEVEL);

/* RX Relocation Types - from binutils elf/rx.h */
#define R_RX_NONE           0x00
#define R_RX_DIR32          0x01
#define R_RX_DIR24S         0x02
#define R_RX_DIR16          0x03
#define R_RX_DIR16U         0x04
#define R_RX_DIR16S         0x05
#define R_RX_DIR8           0x06
#define R_RX_DIR8U          0x07
#define R_RX_DIR8S          0x08
#define R_RX_DIR24S_PCREL   0x09
#define R_RX_DIR16S_PCREL   0x0A
#define R_RX_DIR8S_PCREL    0x0B

/**
 * @brief Write relocation value to location based on RX endianness
 *
 * RX is little-endian for data, but we need to handle instruction encoding carefully
 */
static inline void rx_write_32(uint8_t *loc, uint32_t val)
{
	/* RX uses little-endian byte order */
	sys_put_le32(val, loc);
}

static inline void rx_write_24(uint8_t *loc, uint32_t val)
{
	/* Write 24-bit value in little-endian order */
	loc[0] = (uint8_t)(val & 0xFF);
	loc[1] = (uint8_t)((val >> 8) & 0xFF);
	loc[2] = (uint8_t)((val >> 16) & 0xFF);
}

static inline void rx_write_16(uint8_t *loc, uint16_t val)
{
	sys_put_le16(val, loc);
}

/**
 * @brief Architecture specific function for relocating partially linked (static) elf
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting extensions
 * must implement this.
 */
int arch_elf_relocate(struct llext_loader *ldr, struct llext *ext, elf_rela_t *rel,
		      const elf_shdr_t *shdr)
{
	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);
	const uintptr_t loc = llext_get_reloc_instruction_location(ldr, ext, shdr->sh_info, rel);

	elf_sym_t sym;
	uintptr_t sym_base_addr;
	const char *sym_name;
	int ret;

	/* Read symbol information */
	ret = llext_read_symbol(ldr, ext, rel, &sym);
	if (ret != 0) {
		LOG_ERR("Could not read symbol from binary!");
		return ret;
	}

	sym_name = llext_symbol_name(ldr, ext, &sym);
	if (!sym_name) {
		LOG_ERR("Symbol name is NULL!");
		return -ENOENT;
	}

	ret = llext_lookup_symbol(ldr, ext, &sym_base_addr, rel, &sym, sym_name, shdr);

	LOG_DBG("Relocation: type=%u loc=0x%lx sym=0x%lx addend=%ld name='%s'",
	        reloc_type, loc, sym_base_addr, (long)rel->r_addend, sym_name);

	/* Perform relocation based on type */
	switch (reloc_type) {
	case R_RX_NONE:
		/* No operation needed */
		LOG_DBG("R_RX_NONE - skipping");
		break;

	case R_RX_DIR32: {
		/* 32-bit absolute address relocation */
		uint32_t val32 = (uint32_t)(sym_base_addr + rel->r_addend);
		rx_write_32((uint8_t *)loc, val32);
		LOG_DBG("R_RX_DIR32: wrote 0x%08x to 0x%lx", val32, loc);
		break;
	}

	case R_RX_DIR24S: {
		/* 24-bit signed absolute address */
		int32_t val24 = (int32_t)(sym_base_addr + rel->r_addend);

		/* Check 24-bit signed range: -8MB to +8MB-1 */
		if (val24 < -0x800000 || val24 > 0x7FFFFF) {
			LOG_ERR("R_RX_DIR24S overflow: %s value=0x%x out of range",
			        sym_name, val24);
			return -ENOEXEC;
		}

		rx_write_24((uint8_t *)loc, (uint32_t)val24);
		LOG_DBG("R_RX_DIR24S: wrote 0x%06x to 0x%lx", val24 & 0xFFFFFF, loc);
		break;
	}

	case R_RX_DIR16:
	case R_RX_DIR16S: {
		/* 16-bit signed address */
		int32_t val16 = (int32_t)(sym_base_addr + rel->r_addend);

		if (val16 < -32768 || val16 > 32767) {
			LOG_ERR("R_RX_DIR16S overflow: %s value=%d out of range",
			        sym_name, val16);
			return -ENOEXEC;
		}

		rx_write_16((uint8_t *)loc, (uint16_t)val16);
		LOG_DBG("R_RX_DIR16S: wrote 0x%04x to 0x%lx", (uint16_t)val16, loc);
		break;
	}

	case R_RX_DIR16U: {
		/* 16-bit unsigned address */
		uint32_t val16 = (uint32_t)(sym_base_addr + rel->r_addend);

		if (val16 > 65535) {
			LOG_ERR("R_RX_DIR16U overflow: %s value=%u out of range",
			        sym_name, val16);
			return -ENOEXEC;
		}

		rx_write_16((uint8_t *)loc, (uint16_t)val16);
		LOG_DBG("R_RX_DIR16U: wrote 0x%04x to 0x%lx", (uint16_t)val16, loc);
		break;
	}

	case R_RX_DIR8:
	case R_RX_DIR8S: {
		/* 8-bit signed value */
		int32_t val8 = (int32_t)(sym_base_addr + rel->r_addend);

		if (val8 < -128 || val8 > 127) {
			LOG_ERR("R_RX_DIR8S overflow: %s value=%d out of range",
			        sym_name, val8);
			return -ENOEXEC;
		}

		*((uint8_t *)loc) = (uint8_t)val8;
		LOG_DBG("R_RX_DIR8S: wrote 0x%02x to 0x%lx", (uint8_t)val8, loc);
		break;
	}

	case R_RX_DIR8U: {
		/* 8-bit unsigned value */
		uint32_t val8 = (uint32_t)(sym_base_addr + rel->r_addend);

		if (val8 > 255) {
			LOG_ERR("R_RX_DIR8U overflow: %s value=%u out of range",
			        sym_name, val8);
			return -ENOEXEC;
		}

		*((uint8_t *)loc) = (uint8_t)val8;
		LOG_DBG("R_RX_DIR8U: wrote 0x%02x to 0x%lx", (uint8_t)val8, loc);
		break;
	}

	case R_RX_DIR24S_PCREL: {
		/* 24-bit signed PC-relative displacement */
		int32_t offset24 = (int32_t)(sym_base_addr + rel->r_addend - loc);

		/* Check 24-bit signed range: -8MB to +8MB-1 */
		if (offset24 < -0x800000 || offset24 > 0x7FFFFF) {
			LOG_ERR("R_RX_DIR24S_PCREL overflow: %s offset=0x%x (PC=0x%lx, target=0x%lx)",
			        sym_name, offset24, loc, sym_base_addr);
			return -ENOEXEC;
		}

		rx_write_24((uint8_t *)loc, (uint32_t)offset24);
		LOG_DBG("R_RX_DIR24S_PCREL: offset=0x%x to 0x%lx (target=0x%lx)",
		        offset24, loc, sym_base_addr);
		break;
	}

	case R_RX_DIR16S_PCREL: {
		/* 16-bit signed PC-relative displacement */
		int32_t offset16 = (int32_t)(sym_base_addr + rel->r_addend - loc);

		if (offset16 < -32768 || offset16 > 32767) {
			LOG_ERR("R_RX_DIR16S_PCREL overflow: %s offset=%d out of range",
			        sym_name, offset16);
			return -ENOEXEC;
		}

		rx_write_16((uint8_t *)loc, (uint16_t)offset16);
		LOG_DBG("R_RX_DIR16S_PCREL: offset=0x%x to 0x%lx",
		        (uint16_t)offset16, loc);
		break;
	}

	case R_RX_DIR8S_PCREL: {
		/* 8-bit signed PC-relative displacement */
		int32_t offset8 = (int32_t)(sym_base_addr + rel->r_addend - loc);

		if (offset8 < -128 || offset8 > 127) {
			LOG_ERR("R_RX_DIR8S_PCREL overflow: %s offset=%d out of range",
			        sym_name, offset8);
			return -ENOEXEC;
		}

		*((uint8_t *)loc) = (uint8_t)offset8;
		LOG_DBG("R_RX_DIR8S_PCREL: offset=0x%02x to 0x%lx",
		        (uint8_t)offset8, loc);
		break;
	}

	default:
		LOG_ERR("Unknown relocation type: %u at offset 0x%x", reloc_type, rel->r_offset);
		return -ENOTSUP;
	}

	return 0;
}
