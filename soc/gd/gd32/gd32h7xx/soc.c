/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

void soc_early_init_hook(void)
{
	/*
	 * The vendor SystemInit() configures the power supply (SMPS), the
	 * flash wait states and the PLL0 clock tree from the 25MHz HXTAL to
	 * 600MHz (firmware defined via __SYSTEM_CLOCK_600M_PLL0_HXTAL).
	 */
	SystemInit();
}
