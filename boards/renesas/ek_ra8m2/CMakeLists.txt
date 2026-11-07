# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

include(${ZEPHYR_BASE}/boards/renesas/common/dt_partition_helpers.cmake)

if(CONFIG_BUILD_WITH_TFM AND CONFIG_SOC_R7KA8M2JFLCAC_CM85)
  # The flash map in ek_ra8m2_r7ka8m2jflcac_ns_partition.dtsi is the BL2=ON
  # layout (BL2 + primary/secondary slots) that TF-M checks against.
  if(NOT CONFIG_TFM_BL2)
    message(FATAL_ERROR "ek_ra8m2: TF-M requires CONFIG_TFM_BL2=y; the board "
            "flash map (ek_ra8m2_r7ka8m2jflcac_ns_partition.dtsi) is the BL2 layout.")
  endif()

  # Memory layout.
  dt_reg_size_by_label(_s_code_size  slot0_partition)
  dt_reg_size_by_label(_ns_code_size slot0_ns_partition)
  dt_reg_addr_by_label(_ns_code_addr slot0_ns_partition)
  dt_flash_base_by_label(_ns_base    slot0_ns_partition)
  dt_reg_size_by_label(_s_sram_size  sram0_s)
  dt_reg_size_by_label(_ns_sram_size sram0_ns)
  dt_reg_size_by_label(_ps_size      tfm_ps_partition)
  dt_reg_size_by_label(_its_size     tfm_its_partition)
  dt_reg_size_by_label(_otp_size     tfm_otp_partition)
  dt_reg_size_by_label(_bl2_size     boot_partition)
  dt_reg_addr_by_label(_bl2_addr     boot_partition)
  dt_flash_base_by_label(_bl2_base   boot_partition)

  math(EXPR _ns_code_offset "${_ns_code_addr} - ${_ns_base}")
  math(EXPR _bl2_offset "${_bl2_addr} - ${_bl2_base}")

  set_property(TARGET zephyr_property_target
    APPEND PROPERTY TFM_CMAKE_OPTIONS
    -DRA_CM33_NS_PRESENT=OFF
    -DFLASH_AREA_BL2_OFFSET=${_bl2_offset}
    -DFLASH_AREA_BL2_SIZE=${_bl2_size}
    -DCPU0_NS_ROM_ALIAS_BASE=${_ns_base}
    -DFLASH_S_PARTITION_SIZE=${_s_code_size}
    -DFLASH_CPU0_NS_PARTITION_SIZE=${_ns_code_size}
    -DCPU0_NS_IMAGE_PRIMARY_PARTITION_OFFSET=${_ns_code_offset}
    -DS_DATA_SIZE=${_s_sram_size}
    -DCPU0_NS_RAM_DATA_SIZE=${_ns_sram_size}
    -DFLASH_PS_AREA_SIZE=${_ps_size}
    -DFLASH_ITS_AREA_SIZE=${_its_size}
    -DFLASH_OTP_NV_COUNTERS_AREA_SIZE=${_otp_size}
    -DMLDSA_NATIVE_DIR=${ZEPHYR_MLDSA_NATIVE_MODULE_DIR} # MLDSA currently enabled by default
    )

  # The RA TF-M platform generates the Renesas partition data file next to
  # the first image programmed (bl2.rpd with BL2, tfm_s.rpd otherwise), in
  # the TF-M build tree only. Copy it next to the Zephyr images for the
  # Renesas tools (e2 studio / RFP) to set the security boundaries.
  if(CONFIG_TFM_BL2)
    set(_rpd_file bl2.rpd)
  else()
    set(_rpd_file tfm_s.rpd)
  endif()

  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/bin/${_rpd_file}
      ${CMAKE_BINARY_DIR}/zephyr/${_rpd_file}
    )
  set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts
    ${CMAKE_BINARY_DIR}/zephyr/${_rpd_file}
    )
endif()
