# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

# BSP clock options of the TF-M secure image, derived from devicetree.
# Options whose node is not "okay" are not passed, except *_SOURCE options,
# which are passed as DISABLED.

function(bsp_cfg_option key value)
  set_property(TARGET zephyr_property_target
    APPEND PROPERTY TFM_CMAKE_OPTIONS -DRA_BSP_CLK_${key}=${value})
endfunction()

function(bsp_dt_node label path_var ok_var)
  dt_nodelabel(node NODELABEL ${label} REQUIRED)
  dt_node_has_status(ok PATH ${node} STATUS okay)
  set(${path_var} ${node} PARENT_SCOPE)
  set(${ok_var} ${ok} PARENT_SCOPE)
endfunction()

set(bsp_src_clock_hoco HOCO)
set(bsp_src_clock_moco MOCO)
set(bsp_src_clock_loco LOCO)
set(bsp_src_clock_main_osc MAIN_OSC)
set(bsp_src_clock_subclk SUBCLOCK)
set(bsp_src_pllp PLL1P)
set(bsp_src_pllq PLL1Q)
set(bsp_src_pllr PLL1R)
set(bsp_src_pll2p PLL2P)
set(bsp_src_pll2q PLL2Q)
set(bsp_src_pll2r PLL2R)

function(bsp_dt_clk_source var path)
  dt_prop(ctlr PATH ${path} PROPERTY clocks INDEX 0 REQUIRED)
  get_filename_component(name ${ctlr} NAME)
  string(REPLACE "-" "_" name ${name})
  if(NOT DEFINED bsp_src_${name})
    message(FATAL_ERROR "${path}: unsupported clock source node \"${name}\"")
  endif()
  set(${var} ${bsp_src_${name}} PARENT_SCOPE)
endfunction()

function(bsp_cfg_prop key label prop)
  bsp_dt_node(${label} node ok)
  if(ok)
    dt_prop(val PATH ${node} PROPERTY ${prop} REQUIRED)
    bsp_cfg_option(${key} "${val}")
  endif()
endfunction()

function(bsp_cfg_src key label)
  bsp_dt_node(${label} node ok)
  if(ok)
    bsp_dt_clk_source(source ${node})
    bsp_cfg_option(${key} "${source}")
  else()
    bsp_cfg_option(${key} "DISABLED")
  endif()
endfunction()

# Emits <prefix>_SOURCE and <prefix>_DIV.
function(bsp_cfg_periph prefix label)
  bsp_cfg_src(${prefix}_SOURCE ${label})
  bsp_cfg_prop(${prefix}_DIV ${label} div)
endfunction()

# Emits <prefix>_SOURCE, _DIV, _MUL and _FREQUENCY_HZ for node <label>, and for
# each output node <label>p/q/r: <odiv_prefix><P|Q|R> and
# <out_prefix><P|Q|R>_FREQUENCY_HZ.
function(bsp_cfg_pll prefix label out_prefix odiv_prefix)
  bsp_cfg_src(${prefix}_SOURCE ${label})
  bsp_cfg_prop(${prefix}_DIV ${label} div)

  bsp_dt_node(${label} node ok)
  if(ok)
    dt_nodelabel(xtal NODELABEL xtal REQUIRED)
    dt_prop(xtal_hz PATH ${xtal} PROPERTY clock-frequency REQUIRED)
    dt_prop(mul_int PATH ${node} PROPERTY mul INDEX 0 REQUIRED)
    dt_prop(mul_frac PATH ${node} PROPERTY mul INDEX 1 REQUIRED)
    dt_prop(div PATH ${node} PROPERTY div REQUIRED)

    if(NOT mul_frac EQUAL 0)
      message(FATAL_ERROR
        "${label}: fractional PLL multiplier (${mul_int}.${mul_frac}) is not supported")
    endif()

    math(EXPR freq "${xtal_hz} * ${mul_int} / ${div}")
    bsp_cfg_option(${prefix}_MUL "${mul_int}:${mul_frac}")
    bsp_cfg_option(${prefix}_FREQUENCY_HZ "${freq}")
  endif()

  foreach(out P Q R)
    string(TOLOWER ${out} out_lc)
    bsp_cfg_prop(${odiv_prefix}${out} ${label}${out_lc} div)
    bsp_cfg_prop(${out_prefix}${out}_FREQUENCY_HZ ${label}${out_lc} freq)
  endforeach()
endfunction()

function(bsp_cfg_hoco key label)
  set(supported_hz 16000000 18000000 20000000 32000000 48000000)
  set(register_code 0 1 2 4 7)

  dt_nodelabel(hoco NODELABEL ${label} REQUIRED)
  dt_prop(hz PATH ${hoco} PROPERTY clock-frequency REQUIRED)

  list(FIND supported_hz ${hz} index)
  if(index EQUAL -1)
    message(FATAL_ERROR
      "Invalid HOCO frequency ${hz}: only 16MHz, 18MHz, 20MHz, 32MHz and 48MHz are supported")
  endif()
  list(GET register_code ${index} code)
  bsp_cfg_option(${key} "${code}")
endfunction()

# Oscillators
bsp_cfg_prop(XTAL_HZ xtal clock-frequency)
bsp_cfg_hoco(HOCO_FREQUENCY hoco)

# PLLs
bsp_cfg_pll(PLL pll PLL1 PLODIV)
bsp_cfg_pll(PLL2 pll2 PLL2 PL2ODIV)

# System clock source and dividers
bsp_cfg_src(CLOCK_SOURCE pclkblock)
bsp_cfg_prop(CPUCLK_DIV cpuclk0 div)
bsp_cfg_prop(CPUCLK1_DIV cpuclk1 div)
bsp_cfg_prop(MRICLK_DIV mriclk div)
bsp_cfg_prop(FCLK_DIV mrpclk div)
bsp_cfg_prop(ICLK_DIV iclk div)
bsp_cfg_prop(PCLKA_DIV pclka div)
bsp_cfg_prop(PCLKB_DIV pclkb div)
bsp_cfg_prop(PCLKC_DIV pclkc div)
bsp_cfg_prop(PCLKD_DIV pclkd div)
bsp_cfg_prop(PCLKE_DIV pclke div)
bsp_cfg_prop(BCLK_DIV bclk div)

# External bus clock output
bsp_cfg_prop(SDCLK_OUTPUT bclkout sdclk)
bsp_cfg_prop(BCLK_OUTPUT bclkout clk-out-div) # 0 off, 1 BCLK, 2 BCLK/2

# Peripheral clocks (<prefix>_SOURCE + <prefix>_DIV)
bsp_cfg_periph(SCICLK sciclk)
bsp_cfg_periph(GPTCLK gptclk)
bsp_cfg_periph(UCLK uclk)
bsp_cfg_periph(OCTACLK octaspiclk)
bsp_cfg_periph(ESWCLK eswclk)
bsp_cfg_periph(ESWPHYCLK eswphyclk)
bsp_cfg_periph(ETHPHYCLK ethphyclk)
bsp_cfg_periph(BCLKA bclka)
bsp_cfg_periph(CLKOUT clkout)
bsp_cfg_periph(SPICLK spiclk)
bsp_cfg_periph(CANFDCLK canfdclk)
bsp_cfg_periph(I3CCLK i3cclk)
bsp_cfg_periph(ADCCLK adcclk)
bsp_cfg_periph(USB60CLK usb60clk)
