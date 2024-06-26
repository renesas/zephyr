# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_MEC172X
  soc_i2c.c
)
zephyr_library_sources_ifdef(CONFIG_HAS_MEC5_HAL
  soc_cmn_init.c
)

if (DEFINED CONFIG_MCHP_HEADER_VERBOSE_OUTPUT)
    set(MCHP_HEADER_VERBOSE_OPTION "-v")
endif()

if (DEFINED CONFIG_MCHP_MEC_UNSIGNED_HEADER)
  set(MCHP_MEC_BIN_NAME ${CONFIG_KERNEL_BIN_NAME}.mchp.bin)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${PYTHON_EXECUTABLE} ${ZEPHYR_BASE}/soc/microchip/mec/common/spigen/mec_spi_gen.py
    -i ${KERNEL_BIN_NAME}
    -o ${MCHP_MEC_BIN_NAME}
    -c ${CONFIG_MCHP_MEC_HEADER_CHIP}
    -s ${CONFIG_MCHP_MEC_HEADER_FLASH_SIZE}
    -f ${CONFIG_MCHP_MEC_HEADER_SPI_FREQ_MHZ}
    -r ${CONFIG_MCHP_MEC_HEADER_SPI_READ_MODE}
    -m ${CONFIG_MCHP_MEC_HEADER_FLASH_SPI_MODE}
    --drvstr ${CONFIG_MCHP_MEC_HEADER_SPI_DRVSTR}
    --slewrate ${CONFIG_MCHP_MEC_HEADER_SPI_SLEW_RATE}
    ${MCHP_HEADER_VERBOSE_OPTION}
  )
endif()
