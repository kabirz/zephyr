/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_DW_GD32_H
#define ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_DW_GD32_H

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/dt-bindings/clock/gd32h7xx-clocks.h>

#include <usb_dwc2_hw.h>

/*
 * GD32H7xx USBHS bring-up, following the GigaDevice V1.6.0 firmware
 * library (Examples/USBHS/usb_device):
 *
 *  1. PMU: enable the 3.3V USB supply regulator and its voltage detector,
 *     wait for the supply-ready flag (PMU_CTL2, APB4 0x58000000 + 0x5800).
 *  2. RCU: select IRC48M as the 48MHz PHY reference (USBCLKCTL @ +0xD4,
 *     USBHS048MSEL bits 5-6 = 3) and enable the IRC48M oscillator
 *     (ADDCTL0 @ +0xC0).
 *  3. GUSBCS: select the embedded full-speed PHY (bit 6, GigaDevice
 *     EMBPHY_FS) BEFORE the core soft reset performed by usb_dw_init() -
 *     the reset handshake needs the core clock running.
 *  4. pwr_on(): no VBUS sensing is assumed; force B-session valid via the
 *     GOTGCS override (bits 6/7 on GigaDevice - NOT the STM32 GOTGCTL
 *     positions) and power the transceiver with GCCFG.PWRON (bit 16, the
 *     register at offset 0x38 is GCCFG on this core, not GGPIO).
 *
 * The GD32 fabric does not implement the read-only GHWCFG identity
 * registers (all read as 0); endpoint capabilities come from devicetree.
 */

#define USB_DW_GD32_PMU_BASE      0x58005800UL /* APB4 + 0x5800 */
#define USB_DW_GD32_PMU_CTL2_OFF  0x10

#define USB_DW_GD32_RCU_BASE      0x58024400UL /* AHB4 + 0x4400 */
#define USB_DW_GD32_RCU_ADDCTL0   0xC0
#define USB_DW_GD32_RCU_USBCLKCTL 0xD4

/* PMU_CTL2 */
#define GD32_PMU_VUSB33DEN BIT(24)
#define GD32_PMU_USBSEN    BIT(25)
#define GD32_PMU_USB33RF   BIT(26)

/* RCU_ADDCTL0 */
#define GD32_RCU_IRC48MEN  BIT(16)
#define GD32_RCU_IRC48MSTB BIT(17)

/* GUSBCS */
#define GD32_GUSBCS_EMBPHY_FS BIT(6)

/* GOTGCS (offset 0x00): B-session valid override, GigaDevice positions */
#define GD32_GOTGCS_BVOE BIT(6)
#define GD32_GOTGCS_BVOV BIT(7)

/* GCCFG (offset 0x38): transceiver power on */
#define GD32_GCCFG_PWRON BIT(16)

static inline int usb_dw_gd32_wait(volatile uint32_t *reg, uint32_t mask)
{
	int t;

	for (t = 0; !(*reg & mask) && t < 100000; t++) {
		k_busy_wait(1);
	}

	return (*reg & mask) ? 0 : -ETIMEDOUT;
}

static inline int pwr_on_gd32_usbhs(struct usb_dwc2_reg *const base)
{
	/* runs after the core reset and initial DCFG programming in
	 * usb_dw_init(): finish the vendor bring-up sequence */

	/* clear any stale OTG interrupt flags (ID pin change etc.) */
	base->gotgint = 0xFFFFFFFFU;

	/* restart the PHY clock: clear power-down clock gating
	 * (PWRCLKCTL @ 0xE00, vendor usb_devcore_init) */
	*(volatile uint32_t *)((uintptr_t)base + 0xE00U) = 0U;

	/* DevSpd: this is an HS core with the embedded FS PHY selected,
	 * which per the GigaDevice library wants DS=1 (not the FS-core
	 * value 3 the generic driver writes) */
	base->dcfg = (base->dcfg & ~0x3U) | 0x1U;

	/* power the FS transceiver and force B-session valid (no VBUS
	 * sensing) */
	base->gotgctl |= (GD32_GOTGCS_BVOE | GD32_GOTGCS_BVOV);
	base->ggpio |= GD32_GCCFG_PWRON;

	return 0;
}

#define QUIRK_GD32_USBHS_DEFINE(n)                                            \
	static int clk_enable_gd32_usbhs_##n(void)                            \
	{                                                                     \
		static const uint16_t clk_pmu = GD32_CLOCK_PMU;               \
		static const uint16_t clk_usbhs0 = GD32_CLOCK_USBHS0;         \
		volatile uint32_t *rcu =                                      \
			(volatile uint32_t *)USB_DW_GD32_RCU_BASE;            \
		volatile uint32_t *pmu =                                      \
			(volatile uint32_t *)USB_DW_GD32_PMU_BASE;            \
		volatile uint32_t *gusbcfs =                                  \
			(volatile uint32_t *)(DT_INST_REG_ADDR(n) +           \
					      0x0C);                          \
		int ret;                                                      \
                                                                              \
		if (!device_is_ready(GD32_CLOCK_CONTROLLER)) {                \
			return -ENODEV;                                       \
		}                                                             \
                                                                              \
		ret = clock_control_on(GD32_CLOCK_CONTROLLER,                 \
				       (clock_control_subsys_t)&clk_pmu);     \
		if (ret) {                                                    \
			return ret;                                           \
		}                                                             \
                                                                              \
		ret = clock_control_on(GD32_CLOCK_CONTROLLER,                 \
				       (clock_control_subsys_t)&clk_usbhs0);  \
		if (ret) {                                                    \
			return ret;                                           \
		}                                                             \
                                                                              \
		/* 3.3V USB supply must be up before touching the PHY */      \
		pmu[USB_DW_GD32_PMU_CTL2_OFF / 4U] |=                         \
			(GD32_PMU_USBSEN | GD32_PMU_VUSB33DEN);               \
		ret = usb_dw_gd32_wait(                                       \
			&pmu[USB_DW_GD32_PMU_CTL2_OFF / 4U],                  \
			GD32_PMU_USB33RF);                                    \
		if (ret) {                                                    \
			return ret;                                           \
		}                                                             \
                                                                              \
		/* 48MHz PHY reference from IRC48M */                         \
		rcu[USB_DW_GD32_RCU_ADDCTL0 / 4U] |= GD32_RCU_IRC48MEN;       \
		ret = usb_dw_gd32_wait(                                       \
			&rcu[USB_DW_GD32_RCU_ADDCTL0 / 4U],                   \
			GD32_RCU_IRC48MSTB);                                  \
		if (ret) {                                                    \
			return ret;                                           \
		}                                                             \
		rcu[USB_DW_GD32_RCU_USBCLKCTL / 4U] =                         \
			(rcu[USB_DW_GD32_RCU_USBCLKCTL / 4U] &                \
			 ~(3U << 5)) | (3U << 5);                             \
                                                                              \
		/* embedded FS PHY before the core soft reset */              \
		*gusbcfs |= GD32_GUSBCS_EMBPHY_FS;                            \
		k_busy_wait(10);                                              \
                                                                              \
		return 0;                                                     \
	}

#define USB_DW_QUIRK_GD32_USBHS_DEFINE(n)                                     \
	COND_CODE_1(DT_INST_NODE_HAS_COMPAT(n, gd_gd32_usbhs),                \
		    (QUIRK_GD32_USBHS_DEFINE(n)), ())

#endif /* ZEPHYR_DRIVERS_USB_DEVICE_USB_DC_DW_GD32_H */
