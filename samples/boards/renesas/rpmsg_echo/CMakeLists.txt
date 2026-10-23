cmake_minimum_required(VERSION 3.20.0)
# Copyright (c) 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0
#

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(rpmsg_echo)

target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR})

target_sources(app PRIVATE src/main_remote.c)

if(CONFIG_SAMPLE_RPMSG_ECHO_KERNELSPACE)
  set(RPMSG_ECHO_FLASH_SREC_FILE
      ${ZEPHYR_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}_flash.srec)

  # Resolve the bintools properties now so empty optional flags are omitted
  # instead of being passed to objcopy as literal empty arguments.
  get_target_property(RPMSG_ECHO_ELFCONVERT_COMMAND bintools elfconvert_command)
  get_target_property(RPMSG_ECHO_ELFCONVERT_FLAG bintools elfconvert_flag)
  get_target_property(RPMSG_ECHO_ELFCONVERT_OUTTARGET bintools elfconvert_flag_outtarget)
  get_target_property(RPMSG_ECHO_ELFCONVERT_SREC_LEN bintools elfconvert_flag_srec_len)
  get_target_property(RPMSG_ECHO_ELFCONVERT_REMOVE_SECTION bintools elfconvert_flag_section_remove)
  get_target_property(RPMSG_ECHO_ELFCONVERT_INFILE bintools elfconvert_flag_infile)
  get_target_property(RPMSG_ECHO_ELFCONVERT_OUTFILE bintools elfconvert_flag_outfile)
  get_target_property(RPMSG_ECHO_ELFCONVERT_FINAL bintools elfconvert_flag_final)

  add_custom_command(
    OUTPUT ${RPMSG_ECHO_FLASH_SREC_FILE}
    COMMAND ${RPMSG_ECHO_ELFCONVERT_COMMAND}
            ${RPMSG_ECHO_ELFCONVERT_FLAG}
            --srec-forceS3
            ${RPMSG_ECHO_ELFCONVERT_OUTTARGET}srec
            ${RPMSG_ECHO_ELFCONVERT_SREC_LEN}1
            ${RPMSG_ECHO_ELFCONVERT_REMOVE_SECTION}.resource_table
            ${RPMSG_ECHO_ELFCONVERT_INFILE}${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}
            ${RPMSG_ECHO_ELFCONVERT_OUTFILE}${RPMSG_ECHO_FLASH_SREC_FILE}
            ${RPMSG_ECHO_ELFCONVERT_FINAL}
    DEPENDS zephyr_final
    COMMENT "Generating ROM Writer S-record without .resource_table"
    VERBATIM
  )

  add_custom_target(rpmsg_echo_flash_srec ALL
    DEPENDS ${RPMSG_ECHO_FLASH_SREC_FILE}
  )
endif()
