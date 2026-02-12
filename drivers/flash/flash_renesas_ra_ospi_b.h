/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_FLASH_RENESAS_RA_OSPI_B_H_
#define ZEPHYR_DRIVERS_FLASH_RENESAS_RA_OSPI_B_H_

#include <zephyr/drivers/flash.h>
#include <zephyr/dt-bindings/flash_controller/ospi.h>
#include <zephyr/drivers/clock_control/renesas_ra_cgc.h>
#include <r_spi_flash_api.h>
#include <r_ospi_b.h>
#include "spi_nor_s28hx512t.h"
#include "spi_nor.h"
#include "jesd216.h"

/* Flash device sector size */
#define SECTOR_SIZE_128K (0x20000)
#define SECTOR_SIZE_256K (0x40000)

/* Flash device timing */
#define TIME_ERASE_256K (16000)
#define TIME_ERASE_4K   (1000U)
#define TIME_WRITE      (1000U)

/* Macros for OSPI command code */
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
#define COMMAND_WRITE_ENABLE_SPI   (0x06)
#define COMMAND_WRITE_ENABLE_OPI   (0x0606)
#define COMMAND_WRITE_REGISTER_SPI (0x71)
#define COMMAND_WRITE_REGISTER_OPI (0x7171)
#define COMMAND_READ_STATUS_SPI    (0x05)
#define COMMAND_READ_STATUS_OPI    (0x0505)
#define COMMAND_READ_REGISTER_SPI  (0x65)
#define COMMAND_READ_REGISTER_OPI  (0x6565)
#define COMMAND_READ_DEVICE_ID_SPI (0x9F)
#define COMMAND_READ_DEVICE_ID_OPI (0x9F9F)
#define COMMAND_READ_SFDP_ID_OPI   (S28HX512T_SPI_NOR_OCMD_RSFDPID)
#elif defined(CONFIG_BOARD_EK_RA8P1)
#define COMMAND_WRITE_ENABLE_SPI   (0x06)
#define COMMAND_WRITE_ENABLE_OPI   (0x06F9)
#define COMMAND_WCR2_SPI           (0x72)
#define COMMAND_WCR2_OPI           (0x728D)
#define COMMAND_WRITE_REGISTER_SPI (COMMAND_WCR2_SPI)
#define COMMAND_WRITE_REGISTER_OPI (COMMAND_WCR2_OPI)
#define COMMAND_READ_STATUS_SPI    (0x05)
#define COMMAND_READ_STATUS_OPI    (0x05FA)
#define COMMAND_RCR2_SPI           (0x71)
#define COMMAND_RCR2_OPI           (0x718E)
#define COMMAND_READ_REGISTER_SPI  (COMMAND_RCR2_SPI)
#define COMMAND_READ_REGISTER_OPI  (COMMAND_RCR2_OPI)
#define COMMAND_READ_DEVICE_ID_SPI (0x9F)
#define COMMAND_READ_DEVICE_ID_OPI (0x9F60)
#define COMMAND_READ_SFDP_ID_OPI   (0x5AA5)
#elif defined(CONFIG_BOARD_EK_RA8D2)
#define COMMAND_WRITE_ENABLE_SPI   (0x06)
#define COMMAND_WRITE_ENABLE_OPI   (0x0606)
#define COMMAND_READ_STATUS_SPI    (0x05)
#define COMMAND_READ_STATUS_OPI    (0x0505)
#define COMMAND_WCR_SPI            (0x81)
#define COMMAND_WCR_OPI            (0x8181)
#define COMMAND_WRITE_REGISTER_SPI (COMMAND_WCR_SPI)
#define COMMAND_WRITE_REGISTER_OPI (COMMAND_WCR_OPI)
#define COMMAND_RCR_SPI            (0x85)
#define COMMAND_RCR_OPI            (0x8585)
#define COMMAND_READ_REGISTER_SPI  (COMMAND_RCR_SPI)
#define COMMAND_READ_REGISTER_OPI  (COMMAND_RCR_OPI)
#define COMMAND_READ_DEVICE_ID_SPI (0x9F)
#define COMMAND_READ_DEVICE_ID_OPI (0x9F9F)
#define COMMAND_READ_SFDP_ID_OPI   (S28HX512T_SPI_NOR_OCMD_RSFDPID)
#define COMMAND_ENTER_4BYTES_ADDR  (0xB7)
#endif

/* Macros for OSPI command length */
#define COMMAND_LENGTH_SPI (1U)
#define COMMAND_LENGTH_OPI (2U)

/* Macros for OSPI transfer address */
#define ADDRESS_DUMMY        (0U)
#define ADDRESS_LENGTH_ZERO  (0U)
#define ADDRESS_LENGTH_THREE (3U)
#define ADDRESS_LENGTH_FOUR  (4U)

/* Macros for OSPI transfer data */
#define DATA_DUMMY        (0U)
#define DATA_LENGTH_ZERO  (0U)
#define DATA_LENGTH_ONE   (1U)
#define DATA_LENGTH_TWO   (2U)
#define DATA_LENGTH_THREE (3U)
#define DATA_LENGTH_FOUR  (4U)
#define DATA_LENGTH_EIGHT (8U)

/* Macros for OSPI transfer dummy cycles */
#define DUMMY_CYCLE_WRITE_SPI       (0U)
#define DUMMY_CYCLE_WRITE_OPI       (0U)
#define DUMMY_CYCLE_READ_STATUS_SPI (0U)
#if defined(CONFIG_BOARD_EK_RA8D2)
#define DUMMY_CYCLE_READ_STATUS_OPI (8U)
#elif defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1) ||                          \
	defined(CONFIG_BOARD_EK_RA8P1)
#define DUMMY_CYCLE_READ_STATUS_OPI (4U)
#endif

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
#define DUMMY_CYCLE_READ_REGISTER_SPI (1U)
#elif defined(CONFIG_BOARD_EK_RA8P1)
#define DUMMY_CYCLE_READ_REGISTER_SPI (0U)
#elif defined(CONFIG_BOARD_EK_RA8D2)
#define DUMMY_CYCLE_READ_REGISTER_SPI (16U)
#endif

#if defined(CONFIG_BOARD_EK_RA8D2)
#define DUMMY_CYCLE_READ_REGISTER_OPI (16U)
#elif defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1) ||                          \
	defined(CONFIG_BOARD_EK_RA8P1)
#define DUMMY_CYCLE_READ_REGISTER_OPI (4U)
#endif

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
#define DUMMY_CYCLE_READ_MEMORY_SPI (3U)
#define DUMMY_CYCLE_READ_MEMORY_OPI (10U)
#elif defined(CONFIG_BOARD_EK_RA8P1)
#define DUMMY_CYCLE_READ_MEMORY_SPI (8U)
#define DUMMY_CYCLE_READ_MEMORY_OPI (10U)
#elif defined(CONFIG_BOARD_EK_RA8D2)
#define DUMMY_CYCLE_READ_MEMORY_SPI (11U)
#define DUMMY_CYCLE_READ_MEMORY_OPI (11U)
#endif

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
/* Macros for flash device register address */
#define ADDRESS_STR1V_REGISTER (0x00800000)
#define ADDRESS_STR2V_REGISTER (0x00800001)
#define ADDRESS_CFR1V_REGISTER (0x00800002)
#define ADDRESS_CFR2V_REGISTER (0x00800003)
#define ADDRESS_CFR3V_REGISTER (0x00800004)
#define ADDRESS_CFR4V_REGISTER (0x00800005)
#define ADDRESS_CFR5V_REGISTER (0x00800006)

/* Macros for configure flash device */
#define DATA_CFR2V_REGISTER         (0x83)
#define DATA_CFR3V_REGISTER         (0x40)
#define DATA_SET_SPI_CFR5V_REGISTER (0x40)
#define DATA_SET_OPI_CFR5V_REGISTER (0x43)
#elif defined(CONFIG_BOARD_EK_RA8P1)
/* Macros for flash device register address */
#define ADDRESS_CR2_000H_REGISTER (0x00000000)
#define ADDRESS_CR2_300H_REGISTER (0x00000300)

/* Macros for configure flash device */
#define DATA_SET_SPI_CR2_000H     (0x00)
#define DATA_SET_OPI_CR2_000H     (0x02)
#define DATA_SET_CR2_300H         (0x05)

#elif defined(CONFIG_BOARD_EK_RA8D2)
/* Macros for flash device register address */
#define ADDRESS_CR_00H_REGISTER (0x00000000)
#define ADDRESS_CR_01H_REGISTER (0x00000001)
#define ADDRESS_CR_05H_REGISTER (0x00000005)

/* Macros for configure flash device */
#define DATA_SET_SPI_CR_00H     (0xDF)
#define DATA_SET_OPI_CR_00H     (0xE7)
#define DATA_SET_CR_01H         (0x10)
#endif

/* Flash device address space mapping */
#define SECTOR_OFFSET(sector_no) ((uint8_t *)(sector_no * SPI_NOR_SECTOR_SIZE))

/* Erase command */
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
static spi_flash_erase_command_t erase_command_list[] = {
	{.command = SPI_NOR_CMD_SE_4B, .size = SPI_NOR_SECTOR_SIZE},
	{.command = S28HX512T_SPI_NOR_CMD_SE_256KB, .size = SECTOR_SIZE_256K},
	{.command = S28HX512T_SPI_NOR_CMD_ERCHP, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};

static spi_flash_erase_command_t high_speed_erase_command_list[] = {
	{.command = S28HX512T_SPI_NOR_OCMD_SE_4KB, .size = SPI_NOR_SECTOR_SIZE},
	{.command = S28HX512T_SPI_NOR_OCMD_SE_256KB, .size = SECTOR_SIZE_256K},
	{.command = S28HX512T_SPI_NOR_OCMD_ERCHP, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};
#elif defined(CONFIG_BOARD_EK_RA8P1)
static spi_flash_erase_command_t erase_command_list[] = {
	{.command = SPI_NOR_CMD_SE_4B, .size = SPI_NOR_SECTOR_SIZE},
	{.command = SPI_NOR_CMD_BE_4B, .size = SPI_NOR_BLOCK_SIZE},
	{.command = SPI_NOR_CMD_CE, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};

static spi_flash_erase_command_t high_speed_erase_command_list[] = {
	{.command = SPI_NOR_OCMD_SE, .size = SPI_NOR_SECTOR_SIZE},
	{.command = 0xDC23, .size = SPI_NOR_BLOCK_SIZE},
	{.command = SPI_NOR_OCMD_CE, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};
#elif defined(CONFIG_BOARD_EK_RA8D2)
static spi_flash_erase_command_t erase_command_list[] = {
	{.command = SPI_NOR_CMD_SE_4B, .size = SPI_NOR_SECTOR_SIZE},
	{.command = SPI_NOR_CMD_BE_4B, .size = SPI_NOR_BLOCK_SIZE},
	{.command = SPI_NOR_CMD_CE, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};

static spi_flash_erase_command_t high_speed_erase_command_list[] = {
	{.command = S28HX512T_SPI_NOR_OCMD_SE_4KB, .size = SPI_NOR_SECTOR_SIZE},
	{.command = S28HX512T_SPI_NOR_OCMD_SE_256KB, .size = SECTOR_SIZE_256K},
	{.command = S28HX512T_SPI_NOR_OCMD_ERCHP, .size = SPI_FLASH_ERASE_SIZE_CHIP_ERASE}};
#endif

/* Erase command length */
#define ERASE_COMMAND_LENGTH(arr) (sizeof(arr) / sizeof((arr)[0]))

static ospi_b_table_t const erase_commands = {
	.p_table = &erase_command_list,
	.length = ERASE_COMMAND_LENGTH(erase_command_list),
};

static ospi_b_table_t const high_speed_erase_commands = {
	.p_table = &high_speed_erase_command_list,
	.length = ERASE_COMMAND_LENGTH(high_speed_erase_command_list),
};

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
#define COMMAND_SET_1S_1S_1S                                                                       \
	.read_command = SPI_NOR_CMD_READ_FAST, .read_dummy_cycles = 3,                             \
	.program_command = SPI_NOR_CMD_PP_4B, .program_dummy_cycles = 0,                           \
	.write_enable_command = SPI_NOR_CMD_WREN, .status_command = SPI_NOR_CMD_RDSR,              \
	.status_dummy_cycles = 0
#define COMMAND_SET_8D_8D_8D                                                                       \
	.read_command = S28HX512T_SPI_NOR_OCMD_READ,                                               \
	.read_dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_MEM_OCTAL,                                 \
	.program_command = S28HX512T_SPI_NOR_OCMD_PP_4B,                                           \
	.program_dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR_OCTAL,                                  \
	.write_enable_command = S28HX512T_SPI_NOR_OCMD_WEN,                                        \
	.status_command = S28HX512T_SPI_NOR_OCMD_RSR,                                              \
	.status_dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG_OCTAL
#elif defined(CONFIG_BOARD_EK_RA8P1)
#define COMMAND_SET_1S_1S_1S                                                                       \
	.read_command = SPI_NOR_CMD_READ_FAST_4B, .read_dummy_cycles = SPI_NOR_DUMMY_RD,           \
	.program_command = SPI_NOR_CMD_PP_4B, .program_dummy_cycles = 0,                           \
	.write_enable_command = SPI_NOR_CMD_WREN, .status_command = SPI_NOR_CMD_RDSR,              \
	.status_dummy_cycles = 0
#define COMMAND_SET_8D_8D_8D                                                                       \
	.read_command = SPI_NOR_OCMD_DTR_RD, .read_dummy_cycles = 10,                              \
	.program_command = SPI_NOR_OCMD_PAGE_PRG, .program_dummy_cycles = 0,                       \
	.write_enable_command = SPI_NOR_OCMD_WREN, .status_command = SPI_NOR_OCMD_RDSR,            \
	.status_dummy_cycles = SPI_NOR_DUMMY_REG_OCTAL
#elif defined(CONFIG_BOARD_EK_RA8D2)
#define COMMAND_SET_1S_1S_1S                                                                       \
	.read_command = SPI_NOR_CMD_READ_FAST_4B, .read_dummy_cycles = 16,                         \
	.program_command = 0x12, .program_dummy_cycles = 0,                                        \
	.write_enable_command = SPI_NOR_CMD_WREN, .status_command = SPI_NOR_CMD_RDSR,              \
	.status_dummy_cycles = 0
#define COMMAND_SET_8D_8D_8D                                                                       \
	.read_command = 0x0C0C, .read_dummy_cycles = 16, .program_command = 0x1212,                \
	.program_dummy_cycles = 0, .write_enable_command = S28HX512T_SPI_NOR_OCMD_WEN,             \
	.status_command = S28HX512T_SPI_NOR_OCMD_RSR, .status_dummy_cycles = 8
#endif
/* Reset value */
#define RESET_VALUE (0x00)

/* Transfer table */
typedef enum e_transfer {
	TRANSFER_WRITE_ENABLE_SPI = 0,
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	TRANSFER_WRITE_CFR2V_SPI,
	TRANSFER_WRITE_CFR3V_SPI,
	TRANSFER_WRITE_CFR5V_SPI,
#elif defined(CONFIG_BOARD_EK_RA8P1)
	TRANSFER_WRITE_CR2_000H_SPI,
	TRANSFER_WRITE_CR2_300H_SPI,
#elif defined(CONFIG_BOARD_EK_RA8D2)
	TRANSFER_WRITE_CR_00H_SPI,
	TRANSFER_WRITE_CR_01H_SPI,
	TRANSFER_WRITE_CR_05H_SPI,
#endif

	TRANSFER_READ_STATUS_SPI,

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	TRANSFER_READ_CFR2V_SPI,
	TRANSFER_READ_CFR3V_SPI,
	TRANSFER_READ_CFR5V_SPI,
#elif defined(CONFIG_BOARD_EK_RA8P1)
	TRANSFER_READ_CR2_000H_SPI,
	TRANSFER_READ_CR2_300H_SPI,
#elif defined(CONFIG_BOARD_EK_RA8D2)
	TRANSFER_READ_CR_00H_SPI,
	TRANSFER_READ_CR_01H_SPI,
	TRANSFER_ENTER_4BYTES_ADDRESS,
#endif
	TRANSFER_READ_DEVICE_ID_SPI,
	TRANSFER_READ_SFDP_ID_SPI,

	TRANSFER_WRITE_ENABLE_OPI,
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	TRANSFER_WRITE_CFR2V_OPI,
	TRANSFER_WRITE_CFR3V_OPI,
	TRANSFER_WRITE_CFR5V_OPI,
#elif defined(CONFIG_BOARD_EK_RA8P1)
	TRANSFER_WRITE_CR2_000H_OPI,
#elif defined(CONFIG_BOARD_EK_RA8D2)
	TRANSFER_WRITE_CR_00H_OPI,
#endif

	TRANSFER_READ_STATUS_OPI,

#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	TRANSFER_READ_CFR2V_OPI,
	TRANSFER_READ_CFR3V_OPI,
	TRANSFER_READ_CFR5V_OPI,
#elif defined(CONFIG_BOARD_EK_RA8P1)
	TRANSFER_READ_CR2_000H_OPI,
	TRANSFER_READ_CR2_300H_OPI,
#elif defined(CONFIG_BOARD_EK_RA8D2)
	TRANSFER_READ_CR_00H_OPI,
	TRANSFER_READ_CR_01H_OPI,
#endif

	TRANSFER_READ_DEVICE_ID_OPI,
	TRANSFER_READ_SFDP_ID_OPI,
	TRANSFER_MAX
} transfer_t;

spi_flash_direct_transfer_t direct_transfer[TRANSFER_MAX] = {
	/* Transfer structure for SPI mode */
	[TRANSFER_WRITE_ENABLE_SPI] = {.command = SPI_NOR_CMD_WREN,
				       .address = ADDRESS_DUMMY,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_SPI,
				       .address_length = ADDRESS_LENGTH_ZERO,
				       .data_length = DATA_LENGTH_ZERO,
				       .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR},
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	[TRANSFER_WRITE_CFR2V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_WR_WRARG,
				      .address = S28HX512T_SPI_NOR_CFR2V_ADDR,
				      .data = DATA_CFR2V_REGISTER,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR},
	[TRANSFER_WRITE_CFR3V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_WR_WRARG,
				      .address = S28HX512T_SPI_NOR_CFR3V_ADDR,
				      .data = DATA_CFR3V_REGISTER,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR},
	[TRANSFER_WRITE_CFR5V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_WR_WRARG,
				      .address = S28HX512T_SPI_NOR_CFR5V_ADDR,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR},
#elif defined(CONFIG_BOARD_EK_RA8P1)
	[TRANSFER_WRITE_CR2_000H_SPI] = {.command = COMMAND_WRITE_REGISTER_SPI,
					 .address = ADDRESS_CR2_000H_REGISTER,
					 .data = DATA_DUMMY,
					 .command_length = COMMAND_LENGTH_SPI,
					 .address_length = ADDRESS_LENGTH_FOUR,
					 .data_length = DATA_LENGTH_ONE,
					 .dummy_cycles = DUMMY_CYCLE_WRITE_SPI},
	[TRANSFER_WRITE_CR2_300H_SPI] = {.command = COMMAND_WRITE_REGISTER_SPI,
					 .address = ADDRESS_CR2_300H_REGISTER,
					 .data = DATA_DUMMY,
					 .command_length = COMMAND_LENGTH_SPI,
					 .address_length = ADDRESS_LENGTH_FOUR,
					 .data_length = DATA_LENGTH_ONE,
					 .dummy_cycles = DUMMY_CYCLE_WRITE_SPI},
#elif defined(CONFIG_BOARD_EK_RA8D2)
	[TRANSFER_WRITE_CR_00H_SPI] = {.command = COMMAND_WRITE_REGISTER_SPI,
				       .address = ADDRESS_CR_00H_REGISTER,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_SPI,
				       .address_length = ADDRESS_LENGTH_FOUR,
				       .data_length = DATA_LENGTH_ONE,
				       .dummy_cycles = DUMMY_CYCLE_WRITE_SPI},
	[TRANSFER_WRITE_CR_01H_SPI] = {.command = COMMAND_WRITE_REGISTER_SPI,
				       .address = ADDRESS_CR_01H_REGISTER,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_SPI,
				       .address_length = ADDRESS_LENGTH_FOUR,
				       .data_length = DATA_LENGTH_ONE,
				       .dummy_cycles = DUMMY_CYCLE_WRITE_SPI},
	[TRANSFER_ENTER_4BYTES_ADDRESS] = {.command = COMMAND_ENTER_4BYTES_ADDR,
					   .address = ADDRESS_DUMMY,
					   .data = DATA_DUMMY,
					   .command_length = COMMAND_LENGTH_SPI,
					   .address_length = ADDRESS_DUMMY,
					   .data_length = DATA_DUMMY,
					   .dummy_cycles = DUMMY_CYCLE_WRITE_SPI},
#endif
	[TRANSFER_READ_STATUS_SPI] = {.command = SPI_NOR_CMD_RDSR,
				      .address = ADDRESS_DUMMY,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_ZERO,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_STATUS},
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	[TRANSFER_READ_CFR2V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_RREG,
				     .address = S28HX512T_SPI_NOR_CFR2V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_SPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_ONE,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG},
	[TRANSFER_READ_CFR3V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_RREG,
				     .address = S28HX512T_SPI_NOR_CFR3V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_SPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_ONE,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG},
	[TRANSFER_READ_CFR5V_SPI] = {.command = S28HX512T_SPI_NOR_CMD_RREG,
				     .address = S28HX512T_SPI_NOR_CFR5V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_SPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_ONE,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG},
#elif defined(CONFIG_BOARD_EK_RA8P1)
	[TRANSFER_READ_CR2_000H_SPI] = {.command = COMMAND_READ_REGISTER_SPI,
					.address = ADDRESS_CR2_000H_REGISTER,
					.data = DATA_DUMMY,
					.command_length = COMMAND_LENGTH_SPI,
					.address_length = ADDRESS_LENGTH_FOUR,
					.data_length = DATA_LENGTH_ONE,
					.dummy_cycles = DUMMY_CYCLE_READ_REGISTER_SPI},
	[TRANSFER_READ_CR2_300H_SPI] = {.command = COMMAND_READ_REGISTER_SPI,
					.address = ADDRESS_CR2_300H_REGISTER,
					.data = DATA_DUMMY,
					.command_length = COMMAND_LENGTH_SPI,
					.address_length = ADDRESS_LENGTH_FOUR,
					.data_length = DATA_LENGTH_ONE,
					.dummy_cycles = DUMMY_CYCLE_READ_REGISTER_SPI},
#elif defined(CONFIG_BOARD_EK_RA8D2)
	[TRANSFER_READ_CR_00H_SPI] = {.command = COMMAND_READ_REGISTER_SPI,
				      .address = ADDRESS_CR_00H_REGISTER,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = DUMMY_CYCLE_READ_REGISTER_SPI},
	[TRANSFER_READ_CR_01H_SPI] = {.command = COMMAND_READ_REGISTER_SPI,
				      .address = ADDRESS_CR_01H_REGISTER,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_SPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_ONE,
				      .dummy_cycles = DUMMY_CYCLE_READ_REGISTER_SPI},
#endif
	[TRANSFER_READ_DEVICE_ID_SPI] = {.command = SPI_NOR_CMD_RDID,
					 .address = ADDRESS_DUMMY,
					 .data = DATA_DUMMY,
					 .command_length = COMMAND_LENGTH_SPI,
					 .address_length = ADDRESS_LENGTH_ZERO,
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
					 .data_length = DATA_LENGTH_FOUR,
#elif defined(CONFIG_BOARD_EK_RA8P1) || defined(CONFIG_BOARD_EK_RA8D2)
					 .data_length = DATA_LENGTH_THREE,
#endif
					 .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_STATUS},
	[TRANSFER_READ_SFDP_ID_SPI] = {.command = S28HX512T_SPI_NOR_CMD_RSFDPID,
				       .address = ADDRESS_DUMMY,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_SPI,
				       .address_length = ADDRESS_LENGTH_THREE,
				       .data_length = DATA_LENGTH_EIGHT,
				       .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_SFDP},
	/* Transfer structure for OPI mode */
	[TRANSFER_WRITE_ENABLE_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_WEN,
				       .address = ADDRESS_DUMMY,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_OPI,
				       .address_length = ADDRESS_LENGTH_ZERO,
				       .data_length = DATA_LENGTH_ZERO,
				       .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR_OCTAL},
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	[TRANSFER_WRITE_CFR2V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_WR_REG2,
				      .address = S28HX512T_SPI_NOR_CFR2V_ADDR,
				      .data = DATA_CFR2V_REGISTER,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR_OCTAL},
	[TRANSFER_WRITE_CFR3V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_WR_REG2,
				      .address = S28HX512T_SPI_NOR_CFR3V_ADDR,
				      .data = DATA_CFR3V_REGISTER,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR_OCTAL},
	[TRANSFER_WRITE_CFR5V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_WR_REG2,
				      .address = S28HX512T_SPI_NOR_CFR5V_ADDR,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_WR_OCTAL},
#elif defined(CONFIG_BOARD_EK_RA8P1)
	[TRANSFER_WRITE_CR2_000H_OPI] = {.command = COMMAND_WRITE_REGISTER_OPI,
					 .address = ADDRESS_CR2_000H_REGISTER,
					 .data = DATA_DUMMY,
					 .command_length = COMMAND_LENGTH_OPI,
					 .address_length = ADDRESS_LENGTH_FOUR,
					 .data_length = DATA_LENGTH_TWO,
					 .dummy_cycles = DUMMY_CYCLE_WRITE_OPI},
#elif defined(CONFIG_BOARD_EK_RA8D2)
	[TRANSFER_WRITE_CR_00H_OPI] = {.command = COMMAND_WRITE_REGISTER_OPI,
				       .address = ADDRESS_CR_00H_REGISTER,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_OPI,
				       .address_length = ADDRESS_LENGTH_FOUR,
				       .data_length = DATA_LENGTH_TWO,
				       .dummy_cycles = DUMMY_CYCLE_WRITE_OPI},
#endif
	[TRANSFER_READ_STATUS_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_RSR,
				      .address = ADDRESS_DUMMY,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_STATUS_OCTAL},
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
	[TRANSFER_READ_CFR2V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_RSR,
				     .address = S28HX512T_SPI_NOR_CFR2V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_OPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_TWO,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG_OCTAL},
	[TRANSFER_READ_CFR3V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_RSR,
				     .address = S28HX512T_SPI_NOR_CFR3V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_OPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_TWO,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG_OCTAL},
	[TRANSFER_READ_CFR5V_OPI] = {.command = S28HX512T_SPI_NOR_OCMD_RREG,
				     .address = S28HX512T_SPI_NOR_CFR5V_ADDR,
				     .data = DATA_DUMMY,
				     .command_length = COMMAND_LENGTH_OPI,
				     .address_length = ADDRESS_LENGTH_FOUR,
				     .data_length = DATA_LENGTH_TWO,
				     .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_REG_OCTAL},
#elif defined(CONFIG_BOARD_EK_RA8P1)
	[TRANSFER_READ_CR2_000H_OPI] = {.command = COMMAND_READ_REGISTER_OPI,
					.address = ADDRESS_CR2_000H_REGISTER,
					.data = DATA_DUMMY,
					.command_length = COMMAND_LENGTH_OPI,
					.address_length = ADDRESS_LENGTH_FOUR,
					.data_length = DATA_LENGTH_TWO,
					.dummy_cycles = DUMMY_CYCLE_READ_REGISTER_OPI},
	[TRANSFER_READ_CR2_300H_OPI] = {.command = COMMAND_READ_REGISTER_OPI,
					.address = ADDRESS_CR2_300H_REGISTER,
					.data = DATA_DUMMY,
					.command_length = COMMAND_LENGTH_OPI,
					.address_length = ADDRESS_LENGTH_FOUR,
					.data_length = DATA_LENGTH_TWO,
					.dummy_cycles = DUMMY_CYCLE_READ_REGISTER_OPI},

#elif defined(CONFIG_BOARD_EK_RA8D2)
	[TRANSFER_READ_CR_00H_OPI] = {.command = COMMAND_READ_REGISTER_OPI,
				      .address = ADDRESS_CR_00H_REGISTER,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = DUMMY_CYCLE_READ_REGISTER_OPI},
	[TRANSFER_READ_CR_01H_OPI] = {.command = COMMAND_READ_REGISTER_OPI,
				      .address = ADDRESS_CR_01H_REGISTER,
				      .data = DATA_DUMMY,
				      .command_length = COMMAND_LENGTH_OPI,
				      .address_length = ADDRESS_LENGTH_FOUR,
				      .data_length = DATA_LENGTH_TWO,
				      .dummy_cycles = DUMMY_CYCLE_READ_REGISTER_OPI},
#endif
	[TRANSFER_READ_DEVICE_ID_OPI] = {.command = COMMAND_READ_DEVICE_ID_OPI,
					 .address = ADDRESS_DUMMY,
					 .data = DATA_DUMMY,
					 .command_length = COMMAND_LENGTH_OPI,
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1)
					 .address_length = ADDRESS_LENGTH_FOUR,
					 .data_length = DATA_LENGTH_FOUR,
#elif defined(CONFIG_BOARD_EK_RA8P1)
					 .address_length = ADDRESS_LENGTH_FOUR,
					 .data_length = DATA_LENGTH_THREE * 2,
#elif defined(CONFIG_BOARD_EK_RA8D2)
					 .address_length = ADDRESS_LENGTH_ZERO,
					 .data_length = DATA_LENGTH_THREE,
#endif
					 .dummy_cycles = DUMMY_CYCLE_READ_STATUS_OPI},
	[TRANSFER_READ_SFDP_ID_OPI] = {.command = COMMAND_READ_SFDP_ID_OPI,
				       .address = ADDRESS_DUMMY,
				       .data = DATA_DUMMY,
				       .command_length = COMMAND_LENGTH_OPI,
				       .address_length = ADDRESS_LENGTH_FOUR,
				       .data_length = DATA_LENGTH_EIGHT,
#if defined(CONFIG_BOARD_EK_RA8D1) || defined(CONFIG_BOARD_EK_RA8M1) ||                            \
	defined(CONFIG_BOARD_EK_RA8D2)
				       .dummy_cycles = S28HX512T_SPI_NOR_DUMMY_RD_SFDP_OCTAL},
#elif defined(CONFIG_BOARD_EK_RA8P1)
				       .dummy_cycles = 20},
#endif
};

#endif /* ZEPHYR_DRIVERS_FLASH_RENESAS_RA_OSPI_B_H_ */
