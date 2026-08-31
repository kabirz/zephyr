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

	/*
	 * Enable the instruction cache as early as possible: the core
	 * executes from flash whose wait states are severe at 600MHz, and
	 * with every fetch stalled the whole system crawls (measured on
	 * this board without the I-cache: 368us for one net_pkt pool
	 * allocation, 4.7ms ping RTT; with it: 26us / 0.6ms - an ~8x
	 * difference on code-dense paths).  The data cache stays off for
	 * now: it needs the DMA descriptor SRAM mapped non-cacheable via
	 * a static MPU region first (see the ethernet driver and the
	 * bring-up notes for the failed experiments).
	 */
	SCB_EnableICache();
}
