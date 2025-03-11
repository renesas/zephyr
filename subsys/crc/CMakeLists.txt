# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CRC4 OR CONFIG_CRC4_TI)
  set(CRC4_ENABLED TRUE)
endif()

if(CONFIG_CRC8 OR CONFIG_CRC8_ROHC OR CONFIG_CRC8_CCITT)
  set(CRC8_ENABLED TRUE)
endif()

if(CONFIG_CRC16 OR CONFIG_CRC16_CCITT OR CONFIG_CRC16_ITU_T OR
    CONFIG_CRC16_REFLECT OR CONFIG_CRC16_ANSI)
  set(CRC16_ENABLED TRUE)
endif()

if(CONFIG_CRC_HW_HANDLER)
  zephyr_sources(crc_hardware.c)
else()
  if(CRC4_ENABLED)
    zephyr_sources(crc4_sw.c)
  endif()

  zephyr_sources_ifdef(CONFIG_CRC7_BE crc7_sw.c)

  if(CRC8_ENABLED)
    zephyr_sources(crc8_sw.c)
  endif()

  if(CRC16_ENABLED)
    zephyr_sources(crc16_sw.c)
  endif()

  zephyr_sources_ifdef(CONFIG_CRC24_PGP crc24_sw.c)
  zephyr_sources_ifdef(CONFIG_CRC32_C crc32c_sw.c)
  zephyr_sources_ifdef(CONFIG_CRC32_IEEE crc32_sw.c)
endif()

zephyr_sources_ifdef(CONFIG_CRC_SHELL crc_shell.c)
