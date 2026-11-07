# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

# Helpers for TF-M memory layout in board CMakeLists.txt files of Renesas RA.

# Size / address of the reg of the node with <label>.
function(dt_reg_size_by_label out label)
  dt_nodelabel(node NODELABEL ${label} REQUIRED)
  dt_reg_size(size PATH ${node})
  set(${out} ${size} PARENT_SCOPE)
endfunction()

function(dt_reg_addr_by_label out label)
  dt_nodelabel(node NODELABEL ${label} REQUIRED)
  dt_reg_addr(addr PATH ${node})
  set(${out} ${addr} PARENT_SCOPE)
endfunction()

function(dt_flash_base_by_label out label)
  dt_nodelabel(path NODELABEL ${label} REQUIRED)
  string(REGEX REPLACE "/[^/]+$" "" container "${path}")
  string(REGEX REPLACE "/[^/]+$" "" device "${container}")
  dt_reg_addr(base PATH ${device})
  set(${out} ${base} PARENT_SCOPE)
endfunction()
