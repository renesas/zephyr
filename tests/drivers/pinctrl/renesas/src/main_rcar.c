/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/ztest.h>

#define GET_BANK(pin_val)      ((pin_val) / 32U)
#define GET_PIN_IN_BANK(pin_val) ((pin_val) % 32U)

#define GET_FUNC(f)       ((f).func)
#define GET_GROUP(f)      ((f).group)
#define GET_ALTSEL_BIT(f) ((f).pin)

#define GET_PULL_UP(flags) \
	(((flags) & RCAR_PIN_PULL_UP) == RCAR_PIN_PULL_UP)

#define GET_PULL_DOWN(flags) \
	(((flags) & RCAR_PIN_PULL_DOWN) == RCAR_PIN_PULL_DOWN && \
	!((flags) & RCAR_PIN_FLAGS_PUD))

#define GET_BIAS_DISABLE(flags) \
	(((flags) & RCAR_PIN_PULL_DISABLE) == RCAR_PIN_PULL_DISABLE && \
	!((flags) & RCAR_PIN_FLAGS_PUEN))

#define GET_FUNC_SET(flags) \
	(((flags) & RCAR_PIN_FLAGS_FUNC_SET) != 0U)

#define TEST_DEVICE DT_NODELABEL(test_device)
PINCTRL_DT_DEV_CONFIG_DECLARE(TEST_DEVICE);
static const struct pinctrl_dev_config *pcfg = PINCTRL_DT_DEV_CONFIG_GET(TEST_DEVICE);

ZTEST(pinctrl_rcar_x5h, test_dt_extract)
{
	const struct pinctrl_state *scfg;

	zassert_equal(pcfg->state_cnt, 1U);

	scfg = &pcfg->states[0];
	zassert_equal(scfg->id, PINCTRL_STATE_DEFAULT);

	zassert_equal(scfg->pin_cnt, 6U);

	zassert_equal(scfg->pins[0].pin,            RCAR_GP_PIN(5, 6));
	zassert_equal(GET_BANK(scfg->pins[0].pin),              5U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[0].pin),       6U);
	zassert_equal(GET_FUNC(scfg->pins[0].func),             0U);
	zassert_equal(GET_GROUP(scfg->pins[0].func),            5U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[0].func),       6U);
	zassert_true(GET_FUNC_SET(scfg->pins[0].flags));
	zassert_false(GET_PULL_UP(scfg->pins[0].flags));
	zassert_false(GET_PULL_DOWN(scfg->pins[0].flags));
	zassert_false(GET_BIAS_DISABLE(scfg->pins[0].flags));
	zassert_equal(scfg->pins[0].drive_strength,             0U);

	zassert_equal(scfg->pins[1].pin,            RCAR_GP_PIN(5, 7));
	zassert_equal(GET_BANK(scfg->pins[1].pin),              5U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[1].pin),       7U);
	zassert_equal(GET_FUNC(scfg->pins[1].func),             0U);
	zassert_equal(GET_GROUP(scfg->pins[1].func),            5U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[1].func),       7U);
	zassert_true(GET_FUNC_SET(scfg->pins[1].flags));
	zassert_false(GET_PULL_UP(scfg->pins[1].flags));
	zassert_false(GET_PULL_DOWN(scfg->pins[1].flags));
	zassert_false(GET_BIAS_DISABLE(scfg->pins[1].flags));
	zassert_equal(scfg->pins[1].drive_strength,             0U);

	zassert_equal(scfg->pins[2].pin,            RCAR_GP_PIN(1, 1));
	zassert_equal(GET_BANK(scfg->pins[2].pin),              1U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[2].pin),       1U);
	zassert_equal(GET_FUNC(scfg->pins[2].func),             0U);
	zassert_equal(GET_GROUP(scfg->pins[2].func),            1U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[2].func),       1U);
	zassert_true(GET_FUNC_SET(scfg->pins[2].flags));
	zassert_true(GET_PULL_UP(scfg->pins[2].flags));
	zassert_false(GET_PULL_DOWN(scfg->pins[2].flags));
	zassert_false(GET_BIAS_DISABLE(scfg->pins[2].flags));
	zassert_equal(scfg->pins[2].drive_strength,             0U);

	zassert_equal(scfg->pins[3].pin,            RCAR_GP_PIN(1, 0));
	zassert_equal(GET_BANK(scfg->pins[3].pin),              1U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[3].pin),       0U);
	zassert_equal(GET_FUNC(scfg->pins[3].func),             0U);
	zassert_equal(GET_GROUP(scfg->pins[3].func),            1U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[3].func),       0U);
	zassert_true(GET_FUNC_SET(scfg->pins[3].flags));
	zassert_false(GET_PULL_UP(scfg->pins[3].flags));
	zassert_true(GET_PULL_DOWN(scfg->pins[3].flags));
	zassert_false(GET_BIAS_DISABLE(scfg->pins[3].flags));
	zassert_equal(scfg->pins[3].drive_strength,             12U);

	zassert_equal(scfg->pins[4].pin,            RCAR_GP_PIN(5, 6));
	zassert_equal(GET_BANK(scfg->pins[4].pin),              5U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[4].pin),       6U);
	zassert_equal(GET_FUNC(scfg->pins[4].func),             1U);
	zassert_equal(GET_GROUP(scfg->pins[4].func),            5U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[4].func),       6U);
	zassert_true(GET_FUNC_SET(scfg->pins[4].flags));
	zassert_false(GET_PULL_UP(scfg->pins[4].flags));
	zassert_false(GET_PULL_DOWN(scfg->pins[4].flags));
	zassert_true(GET_BIAS_DISABLE(scfg->pins[4].flags));
	zassert_equal(scfg->pins[4].drive_strength,             24U);

	zassert_equal(scfg->pins[5].pin,            RCAR_GP_PIN(8, 0));
	zassert_equal(GET_BANK(scfg->pins[5].pin),              8U);
	zassert_equal(GET_PIN_IN_BANK(scfg->pins[5].pin),       0U);
	zassert_equal(GET_FUNC(scfg->pins[5].func),             0U);
	zassert_equal(GET_GROUP(scfg->pins[5].func),            8U);
	zassert_equal(GET_ALTSEL_BIT(scfg->pins[5].func),       0U);
	zassert_true(GET_FUNC_SET(scfg->pins[5].flags));
	zassert_false(GET_PULL_UP(scfg->pins[5].flags));
	zassert_false(GET_PULL_DOWN(scfg->pins[5].flags));
	zassert_false(GET_BIAS_DISABLE(scfg->pins[5].flags));
	zassert_equal(scfg->pins[5].drive_strength,             6U);
}

ZTEST_SUITE(pinctrl_rcar_x5h, NULL, NULL, NULL, NULL, NULL);
