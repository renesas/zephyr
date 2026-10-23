# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(SB_CONFIG_RPMSG_ECHO_KERNELSPACE_IMAGE AND
   NOT TARGET rpmsg_echo_kernelspace)
  set(rpmsg_echo_kernelspace_FILE_SUFFIX kernelspace CACHE STRING
      "File suffix for the R-Car kernel-space OpenAMP image")

  ExternalZephyrProject_Add(
    APPLICATION rpmsg_echo_kernelspace
    SOURCE_DIR ${APP_DIR}
    BUILD_ONLY TRUE
  )
endif()
