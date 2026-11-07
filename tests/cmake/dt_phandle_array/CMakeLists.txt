# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dt_phandle_array)

target_sources(app PRIVATE ${ZEPHYR_BASE}/misc/empty_file.c)

# Fail unless dt_prop() of <label>'s <prop> at <index> is <expected>.
function(check_phandle_array label prop index expected)
  dt_nodelabel(node NODELABEL ${label} REQUIRED)
  dt_prop(actual PATH ${node} PROPERTY ${prop} INDEX ${index})
  if(NOT "${actual}" STREQUAL "${expected}")
    message(FATAL_ERROR
      "${label}.${prop}[${index}]: expected '${expected}', got '${actual}'")
  endif()
endfunction()

# Controller path of every entry, cells are dropped.
check_phandle_array(test_holder gpios 0 /test/gpio@1000)
check_phandle_array(test_holder gpios 2 /test/gpio@2000)
check_phandle_array(test_holder foos  0 /test/gpio@2000)

# An unspecified entry keeps its index.
check_phandle_array(test_holder gpios 1 NONE)

# The whole property is a list with one element per entry.
dt_nodelabel(holder_node NODELABEL test_holder REQUIRED)
dt_prop(gpios PATH ${holder_node} PROPERTY gpios)
list(LENGTH gpios gpios_len)
if(NOT gpios_len EQUAL 3)
  message(FATAL_ERROR "holder.gpios: expected 3 entries, got ${gpios_len}")
endif()

# A property that is not set stays undefined.
dt_nodelabel(empty_node NODELABEL test_empty_holder REQUIRED)
dt_prop(unset PATH ${empty_node} PROPERTY gpios)
if(DEFINED unset)
  message(FATAL_ERROR "empty_holder.gpios must be undefined, got '${unset}'")
endif()
