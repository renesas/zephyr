# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mram_ospi_dispatch)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Run ext_code from the external flash (XIP). No need to copy.
zephyr_code_relocate(FILES src/ext_code.c LOCATION EXTFLASH_TEXT NOCOPY)

# But still relocate (copy) the data to RAM
zephyr_code_relocate(FILES src/ext_code.c LOCATION RAM_DATA)

# sram_code instead runs entirely from SRAM after being copied there.
zephyr_code_relocate(FILES src/sram_code.c LOCATION RAM)

if(CONFIG_FLASH_RENESAS_RA_OSPI_B)
  # Signature slot linked right after the relocated code in external flash
  zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/ospi_sig.ld)

  set(OSPI_TEXT_SECTION .extflash_text_reloc)
  set(OSPI_SIG_SECTION  .extflash_sig)
  set(OSPI_SIGN_PY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ospi_sign.py
      --mcuboot-scripts ${ZEPHYR_MCUBOOT_MODULE_DIR}/scripts)

  # Signing key. Pass -DOSPI_SIGN_KEY=<pem> for a real key; otherwise a
  # throw-away development key is generated in the build directory.
  # SYSBUILD GLOBAL: a plain -DOSPI_SIGN_KEY=... given to `west build --sysbuild`
  # is forwarded to this image
  zephyr_get(OSPI_SIGN_KEY SYSBUILD GLOBAL)
  if(NOT OSPI_SIGN_KEY)
    set(OSPI_SIGN_KEY ${PROJECT_BINARY_DIR}/ospi_dev_key_mldsa87.pem)
    if(NOT EXISTS ${OSPI_SIGN_KEY})
      execute_process(COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} genkey ${OSPI_SIGN_KEY}
                      COMMAND_ERROR_IS_FATAL ANY)
    endif()
    message(WARNING "Using a generated DEVELOPMENT ML-DSA-87 key (${OSPI_SIGN_KEY}); "
                    "pass -DOSPI_SIGN_KEY=<pem> for production")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${OSPI_SIGN_KEY})

  # Public key embedded in the MRAM image
  execute_process(COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} header ${OSPI_SIGN_KEY}
                          ${PROJECT_BINARY_DIR}/ospi_pubkey.h
                  COMMAND_ERROR_IS_FATAL ANY)
  target_include_directories(app PRIVATE ${PROJECT_BINARY_DIR})

  # zephyr.bin/.hex (the input of imgtool when MCUboot is enabled) must only
  # contain the MRAM image: the OSPI sections live at 0x90000000 and would
  # turn the binary into a ~2 GB file. They are produced by split_flash below.
  set_property(TARGET bintools APPEND PROPERTY elfconvert_flag
               --remove-section=${OSPI_TEXT_SECTION} --remove-section=${OSPI_SIG_SECTION})

  find_program(SREC_CAT srec_cat REQUIRED)

  set(ELF             $<TARGET_FILE:zephyr_final>)
  set(SIGNED_ELF      ${PROJECT_BINARY_DIR}/zephyr_signed.elf)
  set(MERGED_HEX      ${PROJECT_BINARY_DIR}/zephyr_merged.hex)

  if(CONFIG_BOOTLOADER_MCUBOOT)
    set(INTERNAL_HEX_CMD ${CMAKE_COMMAND} -E copy
        ${PROJECT_BINARY_DIR}/zephyr/zephyr.signed.hex ${PROJECT_BINARY_DIR}/zephyr_internal.hex)
    set(INTERNAL_BIN_CMD ${CMAKE_COMMAND} -E copy
        ${PROJECT_BINARY_DIR}/zephyr/zephyr.signed.bin ${PROJECT_BINARY_DIR}/zephyr_internal.signed.bin)
  else()
    set(INTERNAL_HEX_CMD ${CMAKE_OBJCOPY} -R ${OSPI_TEXT_SECTION} -R ${OSPI_SIG_SECTION} -O ihex
        ${SIGNED_ELF} ${PROJECT_BINARY_DIR}/zephyr_internal.hex)
    set(INTERNAL_BIN_CMD ${CMAKE_OBJCOPY} -R ${OSPI_TEXT_SECTION} -R ${OSPI_SIG_SECTION} -O binary
        ${SIGNED_ELF} ${PROJECT_BINARY_DIR}/zephyr_internal.signed.bin)
  endif()

  add_custom_target(
    split_flash ALL
    DEPENDS zephyr_final
    # Sign the exact bytes the CPU will read from OSPI and patch the
    # signature into its reserved slot
    COMMAND ${CMAKE_OBJCOPY} -O binary -j ${OSPI_TEXT_SECTION} ${ELF}
            ${PROJECT_BINARY_DIR}/ospi_text.bin
    COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} sign ${OSPI_SIGN_KEY}
            ${PROJECT_BINARY_DIR}/ospi_text.bin ${PROJECT_BINARY_DIR}/ospi_sig.bin
    COMMAND ${CMAKE_OBJCOPY} --update-section ${OSPI_SIG_SECTION}=${PROJECT_BINARY_DIR}/ospi_sig.bin
            ${ELF} ${SIGNED_ELF}
    # Main MRAM image, OSPI sections removed. With MCUboot this is the
    # imgtool-signed image (header + TLVs) that MCUboot validates.
    COMMAND ${INTERNAL_HEX_CMD}
    COMMAND ${INTERNAL_BIN_CMD}
    # OSPI data only (intermediate)
    COMMAND ${CMAKE_OBJCOPY} -j ${OSPI_TEXT_SECTION} -j ${OSPI_SIG_SECTION} -O ihex
            ${SIGNED_ELF} ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex
    # Pad to an even length (the image is odd: the signature is 4627 bytes),
    # then byte-swap for octa-mode read order
    COMMAND ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex -Intel
            -fill 0xFF -within ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex -Intel -range-padding 2
            -byte_swap 2 -o ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel
    # Flash-ready OSPI binary: starts at 0x90000000 (offset 0 of the OSPI window)
    COMMAND ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel
            -offset -0x90000000 -o ${PROJECT_BINARY_DIR}/zephyr_ospi.signed.bin -Binary
    COMMAND ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_internal.hex -Intel
            ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel -o ${MERGED_HEX} -Intel
    COMMENT "Signing and splitting OSPI (${OSPI_TEXT_SECTION}) image for octa-mode read order"
    VERBATIM
  )
endif()
