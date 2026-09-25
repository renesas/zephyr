# Copyright 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.24)

set(CMAKE_SKIP_INSTALL_RULES
    ON
    CACHE BOOL "" FORCE
)

set(ET_PTE_FILE_PATH
    ""
    CACHE FILEPATH "Path to the ExecuTorch .pte (or .bpte) model to embed"
)
set(ET_PTE_SECTION
    "network_model_sec"
    CACHE STRING "Section attribute used for the generated model data"
)

if(NOT ET_PTE_FILE_PATH)
  message(
    FATAL_ERROR
      "ET_PTE_FILE_PATH must point to the ExecuTorch .pte (or .bpte) model to embed."
  )
endif()

if(NOT IS_ABSOLUTE "${ET_PTE_FILE_PATH}")
  get_filename_component(
    ET_PTE_FILE_PATH "${ET_PTE_FILE_PATH}" ABSOLUTE BASE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

if(NOT EXISTS "${ET_PTE_FILE_PATH}")
  message(
    FATAL_ERROR
      "Could not find ExecuTorch model at ET_PTE_FILE_PATH: ${ET_PTE_FILE_PATH}"
  )
endif()

set(ET_PTE_FILE_PATH
    "${ET_PTE_FILE_PATH}"
    CACHE FILEPATH "Path to the ExecuTorch .pte (or .bpte) model to embed"
          FORCE
)

find_package(
  Python3
  COMPONENTS Interpreter
  REQUIRED
)

# Needed early (before project()) just for ZEPHYR_EXECUTORCH_MODULE_DIR, to
# locate gen_oplist.py without assuming this sample's own path nests inside
# the executorch checkout.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

execute_process(
  COMMAND
    ${Python3_EXECUTABLE}
    "${ZEPHYR_EXECUTORCH_MODULE_DIR}/codegen/tools/gen_oplist.py"
    --model_file_path=${ET_PTE_FILE_PATH}
    --output_path=${CMAKE_CURRENT_BINARY_DIR}/temp.yaml
  RESULT_VARIABLE GEN_OPLIST_RESULT
  OUTPUT_VARIABLE CMD_RESULT
  ERROR_VARIABLE GEN_OPLIST_ERROR
)

if(NOT GEN_OPLIST_RESULT EQUAL 0)
  message(
    FATAL_ERROR
      "gen_oplist.py failed with exit code ${GEN_OPLIST_RESULT}: ${GEN_OPLIST_ERROR}"
  )
endif()

if(CMD_RESULT MATCHES "aten::" OR CMD_RESULT MATCHES "dim_order_ops::")
  set(FOUND_OPS_IN_FILE "true")
else()
  set(FOUND_OPS_IN_FILE "false")
endif()

if(${FOUND_OPS_IN_FILE})
  set(EXECUTORCH_SELECT_OPS_LIST "")
  set(EXECUTORCH_SELECT_OPS_MODEL
      "${ET_PTE_FILE_PATH}"
      CACHE STRING "Select operators from this ExecuTorch model" FORCE
  )
  set(_EXECUTORCH_GEN_ZEPHYR_PORTABLE_OPS ON)
  message(
    "gen_oplist:  EXECUTORCH_SELECT_OPS_MODEL=${ET_PTE_FILE_PATH} is used to auto generate ops from"
  )
else()
  set(EXECUTORCH_SELECT_OPS_LIST "")
  set(EXECUTORCH_SELECT_OPS_MODEL "")
  set(_EXECUTORCH_GEN_ZEPHYR_PORTABLE_OPS OFF)
  message(
    "gen_oplist: No non-delegated ops were found in ${ET_PTE_FILE_PATH}; no ops added to build"
  )
endif()

dt_nodelabel(model_ddr_path NODELABEL "model_ddr")
if(model_ddr_path)
  configure_file(
    model_section.ld.in ${CMAKE_CURRENT_BINARY_DIR}/model_section.ld @ONLY
  )
  zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_BINARY_DIR}/model_section.ld)

  # EK-RA8P1's OSPI controller reads back bytes as b1-b0-b3-b2 when running
  # in octa mode, so the model PTE blob linked into model_ddr (ET_PTE_SECTION,
  # see model_section.ld.in) must be byte-swapped in the flashed hex image or
  # it reads back scrambled at runtime. Split the final image into an
  # internal-flash part (PTE section removed) and an OSPI-only part (PTE
  # section only), byte-swap the OSPI part, then remerge.
  find_program(SREC_CAT srec_cat REQUIRED)

  set(MERGED_HEX ${PROJECT_BINARY_DIR}/zephyr_merged.hex)

  add_custom_target(
    split_flash ALL
    DEPENDS zephyr_final
    # Main internal flash image, OSPI section removed
    COMMAND
      ${CMAKE_OBJCOPY} -R ${ET_PTE_SECTION} -O ihex
      $<TARGET_FILE:zephyr_final>
      ${PROJECT_BINARY_DIR}/zephyr_internal.hex
    # OSPI data only (intermediate)
    COMMAND
      ${CMAKE_OBJCOPY} -j ${ET_PTE_SECTION} -O ihex
      $<TARGET_FILE:zephyr_final>
      ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex
    # Byte-swap the OSPI hex
    COMMAND
      ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex -Intel
      -byte_swap 2 -o ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel
    COMMAND
      ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_internal.hex -Intel
      ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel -o ${MERGED_HEX}
      -Intel
    COMMENT "Splitting and byte-swapping OSPI (${ET_PTE_SECTION}) flash image for octa-mode read order"
  )
endif()

# Route the method/temp allocator pools to external SDRAM when the board
# provides one, since some boards' on-chip SRAM is too small to hold both
# pools for larger models (e.g. MobileNetV2 needs ~3 MiB combined).
dt_nodelabel(pool_sdram_path NODELABEL "sdram1")
if(pool_sdram_path)
  configure_file(
    pool_section.ld.in ${CMAKE_CURRENT_BINARY_DIR}/pool_section.ld @ONLY
  )
  zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_BINARY_DIR}/pool_section.ld)
endif()

project(executorch_mv2_ethosu_renesas)

set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wno-switch -Wno-float-conversion -Wno-double-promotion -Wno-psabi -ffunction-sections -fdata-sections"
)

if(NOT DEFINED EXECUTORCH_DIR)
  message(
    STATUS
      "ZEPHYR_EXECUTORCH_MODULE_DIR set to : ${ZEPHYR_EXECUTORCH_MODULE_DIR}"
  )
  if(DEFINED ZEPHYR_EXECUTORCH_MODULE_DIR)
    set(EXECUTORCH_DIR ${ZEPHYR_EXECUTORCH_MODULE_DIR})
    message(
      STATUS "Using Zephyr module discovery: EXECUTORCH_DIR=${EXECUTORCH_DIR}"
    )
  else()
    message(
      FATAL_ERROR
        "ExecuTorch module not found. Ensure it's properly configured in your Zephyr workspace."
    )
  endif()
else()
  message(STATUS "Using predefined EXECUTORCH_DIR=${EXECUTORCH_DIR}")
endif()

set(EXECUTORCH_ROOT ${EXECUTORCH_DIR})
include(${EXECUTORCH_DIR}/tools/cmake/Utils.cmake)

if(NOT TARGET portable_kernels)
  set(EXECUTORCH_PORTABLE_BUILD_KERNELS_ONLY ON)
  add_subdirectory(
    ${EXECUTORCH_DIR}/kernels/portable
    ${CMAKE_CURRENT_BINARY_DIR}/executorch/kernels/portable
  )
  unset(EXECUTORCH_PORTABLE_BUILD_KERNELS_ONLY)
endif()
set(EXECUTORCH_OPS_LIB "")
if(_EXECUTORCH_GEN_ZEPHYR_PORTABLE_OPS)
  include(${EXECUTORCH_DIR}/tools/cmake/Codegen.cmake)
  if(NOT DEFINED EXECUTORCH_ENABLE_DTYPE_SELECTIVE_BUILD)
    set(EXECUTORCH_ENABLE_DTYPE_SELECTIVE_BUILD "")
  endif()
  gen_selected_ops(
    LIB_NAME
    "cpu_portable_ops_lib"
    OPS_SCHEMA_YAML
    ""
    ROOT_OPS
    "${EXECUTORCH_SELECT_OPS_LIST}"
    INCLUDE_ALL_OPS
    ""
    OPS_FROM_MODEL
    "${EXECUTORCH_SELECT_OPS_MODEL}"
    DTYPE_SELECTIVE_BUILD
    "${EXECUTORCH_ENABLE_DTYPE_SELECTIVE_BUILD}"
  )
  generate_bindings_for_kernels(
    LIB_NAME "cpu_portable_ops_lib" FUNCTIONS_YAML
    ${EXECUTORCH_DIR}/kernels/portable/functions.yaml DTYPE_SELECTIVE_BUILD
    "${EXECUTORCH_ENABLE_DTYPE_SELECTIVE_BUILD}"
  )
  gen_operators_lib(
    LIB_NAME
    "cpu_portable_ops_lib"
    KERNEL_LIBS
    portable_kernels
    DEPS
    executorch
    DTYPE_SELECTIVE_BUILD
    "${EXECUTORCH_ENABLE_DTYPE_SELECTIVE_BUILD}"
  )
  set(EXECUTORCH_OPS_LIB "cpu_portable_ops_lib")
endif()

set(_local_flatcc_root ${CMAKE_BINARY_DIR}/flatcc_src)
if(NOT EXISTS ${_local_flatcc_root}/CMakeLists.txt)
  file(MAKE_DIRECTORY ${_local_flatcc_root})
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${EXECUTORCH_DIR}/third-party/flatcc ${_local_flatcc_root}
  )
endif()
set(EXECUTORCH_FLATCC_SOURCE_ROOT
    ${_local_flatcc_root}
    CACHE PATH "" FORCE
)
set(EXECUTORCH_FLATCC_INSTALL_ROOT
    ${_local_flatcc_root}
    CACHE PATH "" FORCE
)

set(app_sources
    src/main.c
    src/camera.c
    src/display.c
    src/ai_processing.cpp
    ${EXECUTORCH_DIR}/examples/arm/executor_runner/arm_memory_allocator.cpp
)
target_sources(app PRIVATE ${app_sources})

set(_model_pte_header ${CMAKE_CURRENT_BINARY_DIR}/model_pte.h)
add_custom_command(
  OUTPUT ${_model_pte_header}
  COMMAND
    ${Python3_EXECUTABLE}
    ${EXECUTORCH_DIR}/examples/arm/executor_runner/pte_to_header.py --pte
    ${ET_PTE_FILE_PATH} --outdir ${CMAKE_CURRENT_BINARY_DIR} --section
    ${ET_PTE_SECTION}
  DEPENDS ${ET_PTE_FILE_PATH}
          ${EXECUTORCH_DIR}/examples/arm/executor_runner/pte_to_header.py
  COMMENT "Converting ${ET_PTE_FILE_PATH} to model_pte.h"
)
add_custom_target(gen_model_header DEPENDS ${_model_pte_header})
add_dependencies(app gen_model_header)

if(DEFINED CONFIG_EXECUTORCH_METHOD_ALLOCATOR_POOL_SIZE)
  target_compile_definitions(
    app
    PRIVATE
      ET_ARM_METHOD_ALLOCATOR_POOL_SIZE=${CONFIG_EXECUTORCH_METHOD_ALLOCATOR_POOL_SIZE}
  )
endif()
if(DEFINED CONFIG_EXECUTORCH_TEMP_ALLOCATOR_POOL_SIZE)
  target_compile_definitions(
    app
    PRIVATE
      ET_ARM_BAREMETAL_SCRATCH_TEMP_ALLOCATOR_POOL_SIZE=${CONFIG_EXECUTORCH_TEMP_ALLOCATOR_POOL_SIZE}
  )
endif()
if(CONFIG_ET_ARM_MODEL_PTE_DMA_ACCESSIBLE)
  target_compile_definitions(app PRIVATE ET_ARM_MODEL_PTE_DMA_ACCESSIBLE)
endif()

target_link_libraries(app PRIVATE libexecutorch)
target_compile_options(libexecutorch PRIVATE -w)
if(EXECUTORCH_OPS_LIB)
  target_link_libraries(app PRIVATE ${EXECUTORCH_OPS_LIB})
  target_compile_options(${EXECUTORCH_OPS_LIB} PRIVATE -w)
endif()
if(CONFIG_CPU_CORTEX_M)
  if(TARGET cortex_m_ops_lib)
    target_link_libraries(app PRIVATE cortex_m_ops_lib)
    target_compile_options(cortex_m_ops_lib PRIVATE -w)
  endif()
  if(TARGET cortex_m_kernels)
    executorch_target_link_options_shared_lib(cortex_m_kernels)
    target_link_libraries(app PRIVATE cortex_m_kernels)
    target_compile_options(cortex_m_kernels PRIVATE -w)
  endif()
endif()
if(TARGET quantized_kernels)
  executorch_target_link_options_shared_lib(quantized_kernels)
  target_link_libraries(app PRIVATE quantized_kernels)
  target_compile_options(quantized_kernels PRIVATE -w)
endif()
if(TARGET portable_kernels)
  executorch_target_link_options_shared_lib(portable_kernels)
  target_link_libraries(app PRIVATE portable_kernels)
  target_compile_options(portable_kernels PRIVATE -w)
endif()
if(TARGET executorch_delegate_ethos_u)
  executorch_target_link_options_shared_lib(executorch_delegate_ethos_u)
  target_link_libraries(app PRIVATE executorch_delegate_ethos_u)
  target_compile_options(executorch_delegate_ethos_u PRIVATE -w)
endif()
if(TARGET ethosu_core_driver)
  target_link_libraries(app PRIVATE ethosu_core_driver)
  target_compile_options(ethosu_core_driver PRIVATE -w)
endif()

target_include_directories(app PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
get_target_property(OUT app LINK_LIBRARIES)
message(STATUS ${OUT})
