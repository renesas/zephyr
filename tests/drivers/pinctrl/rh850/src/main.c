/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/ztest.h>
#include <bsp_api.h>

/* pin configuration for test device */
#define TEST_DEVICE DT_NODELABEL(test_device)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static const struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

ZTEST(pinctrl_rh850, test_dt_extract)
{
	const struct pinctrl_state *scfg;

	zassert_equal(pcfg->state_cnt, 1U);

	scfg = &pcfg->states[0];

	zassert_equal(scfg->id, PINCTRL_STATE_DEFAULT);
	zassert_equal(scfg->pin_cnt, 6U);

	zassert_equal(RH850_GET_MODE(scfg->pins[0].cfg), RH850_ALT_OUT8);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[0].cfg), 0U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[0].cfg), 0U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[0].cfg), 0U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[0].cfg), RH850_DRIVE_STRENGTH_LOW);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[0].cfg), 0U);
	zassert_equal(scfg->pins[0].port_id, BSP_IO_PORT_P02);
	zassert_equal(scfg->pins[0].pin_num, 7U);

	zassert_equal(RH850_GET_MODE(scfg->pins[1].cfg), RH850_ALT_IN8);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[1].cfg), 0U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[1].cfg), 0U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[1].cfg), 0U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[1].cfg), RH850_DRIVE_STRENGTH_LOW);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[1].cfg), 0U);
	zassert_equal(scfg->pins[1].port_id, BSP_IO_PORT_P02);
	zassert_equal(scfg->pins[1].pin_num, 8);

	zassert_equal(RH850_GET_MODE(scfg->pins[2].cfg), RH850_ALT_IN13);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[2].cfg), 1U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[2].cfg), 0U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[2].cfg), 0U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[2].cfg), RH850_DRIVE_STRENGTH_LOW);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[2].cfg), 0U);
	zassert_equal(scfg->pins[2].port_id, BSP_IO_PORT_P20);
	zassert_equal(scfg->pins[2].pin_num, 3);

	zassert_equal(RH850_GET_MODE(scfg->pins[3].cfg), RH850_ALT_IN13);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[3].cfg), 0U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[3].cfg), 1U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[3].cfg), 0U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[3].cfg), RH850_DRIVE_STRENGTH_MEDIUM);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[3].cfg), 0U);
	zassert_equal(scfg->pins[3].port_id, BSP_IO_PORT_P21);
	zassert_equal(scfg->pins[3].pin_num, 3);

	zassert_equal(RH850_GET_MODE(scfg->pins[4].cfg), RH850_ALT_OUT4);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[4].cfg), 0U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[4].cfg), 0U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[4].cfg), 2U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[4].cfg), RH850_DRIVE_STRENGTH_HIGH);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[4].cfg), 0U);
	zassert_equal(scfg->pins[4].port_id, BSP_IO_PORT_P22);
	zassert_equal(scfg->pins[4].pin_num, 1);

	zassert_equal(RH850_GET_MODE(scfg->pins[5].cfg), RH850_ALT_OUT10);
	zassert_equal(RH850_GET_PULL_UP(scfg->pins[5].cfg), 0U);
	zassert_equal(RH850_GET_PULL_DOWN(scfg->pins[5].cfg), 0U);
	zassert_equal(RH850_GET_OPEN_DRAIN(scfg->pins[5].cfg), 0U);
	zassert_equal(RH850_GET_DRIVE_STRENGTH(scfg->pins[5].cfg), RH850_DRIVE_STRENGTH_LOW);
	zassert_equal(RH850_GET_DIRECT_CONTROL(scfg->pins[5].cfg), 1U);
	zassert_equal(scfg->pins[5].port_id, BSP_IO_PORT_P10);
	zassert_equal(scfg->pins[5].pin_num, 10);
}

ZTEST_SUITE(pinctrl_rh850, NULL, NULL, NULL, NULL, NULL);
