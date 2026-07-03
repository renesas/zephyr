# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RENESAS_RCAR_MP_PHY renesas_rcar_mp_phy.c)
