/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 *
 * GigaDevice GD32H7xx TRNG driver (polling mode).
 *
 * Unlike the GD32F4xx, the H7 TRNG hangs on AHB2 and needs no 48MHz
 * auxiliary clock, so the CK48M plumbing of the F4 driver does not apply.
 */

#define DT_DRV_COMPAT gd_gd32_trng

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <gd32_trng.h>

LOG_MODULE_REGISTER(entropy_gd32_h7, CONFIG_ENTROPY_LOG_LEVEL);

/* Prevent infinite wait in case TRNG never asserts DRDY */
#define GD32_TRNG_DRDY_TIMEOUT_MS 100U

struct entropy_gd32_h7_config {
	uint16_t clkid;
};

static void entropy_gd32_h7_recover(void)
{
	trng_disable();
	trng_deinit();
	trng_interrupt_flag_clear(TRNG_INT_FLAG_CEIF);
	trng_interrupt_flag_clear(TRNG_INT_FLAG_SEIF);
	trng_enable();
}

static int entropy_gd32_h7_wait_drdy(void)
{
	uint32_t start = k_cycle_get_32();
	uint32_t timeout_cycles =
		(uint32_t)((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC *
			   (uint64_t)GD32_TRNG_DRDY_TIMEOUT_MS / 1000ULL);

	while (SET != trng_flag_get(TRNG_FLAG_DRDY)) {
		if (SET == trng_flag_get(TRNG_FLAG_CECS)) {
			/* Clock error: TRNG kernel clock misconfigured */
			return -EIO;
		}

		if (SET == trng_flag_get(TRNG_FLAG_SECS)) {
			entropy_gd32_h7_recover();
		}

		if ((k_cycle_get_32() - start) > timeout_cycles) {
			return -ETIMEDOUT;
		}

		/* Never yield/sleep in ISR paths */
		if (!k_is_in_isr() && !k_is_pre_kernel()) {
			k_yield();
		}
	}

	return 0;
}

static int entropy_gd32_h7_fetch_busywait(uint8_t *dst, uint16_t len)
{
	while (len > 0U) {
		int ret = entropy_gd32_h7_wait_drdy();

		if (ret < 0) {
			return ret;
		}

		uint32_t word = trng_get_true_random_data();

		if (len >= sizeof(word)) {
			memcpy(dst, &word, sizeof(word));
			dst += sizeof(word);
			len -= sizeof(word);
		} else {
			memcpy(dst, &word, len);
			len = 0U;
		}
	}

	return 0;
}

static int entropy_gd32_h7_init(const struct device *dev)
{
	const struct entropy_gd32_h7_config *cfg = dev->config;
	int ret = clock_control_on(GD32_CLOCK_CONTROLLER,
				   (clock_control_subsys_t)&cfg->clkid);

	if (ret < 0) {
		return ret;
	}

	trng_deinit();
	trng_interrupt_flag_clear(TRNG_INT_FLAG_CEIF);
	trng_interrupt_flag_clear(TRNG_INT_FLAG_SEIF);
	trng_enable();

	return 0;
}

static int entropy_gd32_h7_get_entropy(const struct device *dev, uint8_t *buffer,
				       uint16_t length)
{
	ARG_UNUSED(dev);
	if ((buffer == NULL) || (length == 0U)) {
		return -EINVAL;
	}

	return entropy_gd32_h7_fetch_busywait(buffer, length);
}

static int entropy_gd32_h7_get_entropy_isr(const struct device *dev, uint8_t *buffer,
					   uint16_t length, uint32_t flags)
{
	ARG_UNUSED(dev);

	if ((buffer == NULL) || (length == 0U)) {
		return -EINVAL;
	}

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		uint16_t written = 0U;

		while ((written < length) &&
		       (SET == trng_flag_get(TRNG_FLAG_DRDY))) {
			uint32_t word = trng_get_true_random_data();
			uint16_t chunk = MIN((uint16_t)sizeof(word),
					     (uint16_t)(length - written));

			memcpy(&buffer[written], &word, chunk);
			written += chunk;
		}

		return written;
	}

	/* Busy-wait (ISR-safe): fill the whole buffer, return bytes written */
	int ret = entropy_gd32_h7_fetch_busywait(buffer, length);

	if (ret < 0) {
		return ret;
	}

	return length;
}

static DEVICE_API(entropy, entropy_gd32_h7_api) = {
	.get_entropy = entropy_gd32_h7_get_entropy,
	.get_entropy_isr = entropy_gd32_h7_get_entropy_isr,
};

static const struct entropy_gd32_h7_config entropy_gd32_h7_cfg = {
	.clkid = DT_INST_CLOCKS_CELL(0, id),
};

DEVICE_DT_INST_DEFINE(0, entropy_gd32_h7_init, NULL, NULL, &entropy_gd32_h7_cfg,
		      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
		      &entropy_gd32_h7_api);
