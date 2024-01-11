/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
#if BSP_CFG_MCU_PART_SERIES == 8
	R_PMISC->PWPRS = 0;
	R_PMISC->PWPRS = BIT(R_PMISC_PWPR_PFSWE_Pos);
#else
	R_PMISC->PWPR = 0;
	R_PMISC->PWPR = BIT(R_PMISC_PWPR_PFSWE_Pos);
#endif

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		const pinctrl_soc_pin_t *pin = &pins[i];

		/* PMR bits should be cleared before specifying PSEL. Reference section "20.7 Notes
		 * on the PmnPFS Register Setting" in the RA6M3 manual R01UH0886EJ0100.
		 */
		if ((pin->cfg & BIT(R_PFS_PORT_PIN_PmnPFS_PMR_Pos)) > 0) {
			/* Clear PMR */
			R_PFS->PORT[pin->port_num].PIN[pin->pin_num].PmnPFS_b.PMR = 0;
			/* Apply new config with PMR = 0 */
			R_PFS->PORT[pin->port_num].PIN[pin->pin_num].PmnPFS =
				(pin->cfg & ~(BIT(R_PFS_PORT_PIN_PmnPFS_PMR_Pos)));
		}

		/* Apply new config */
		R_PFS->PORT[pin->port_num].PIN[pin->pin_num].PmnPFS = pin->cfg;
	}

#if BSP_CFG_MCU_PART_SERIES == 8
	R_PMISC->PWPRS = 0;
	R_PMISC->PWPRS = BIT(R_PMISC_PWPR_B0WI_Pos);
#else
	R_PMISC->PWPR = 0;
	R_PMISC->PWPR = BIT(R_PMISC_PWPR_B0WI_Pos);
#endif
	return 0;
}
