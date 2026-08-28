/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flash_gd32.h"

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <cmsis_core.h>

#include <gd32_fmc.h>

LOG_MODULE_DECLARE(flash_gd32);

#define GD32_NV_FLASH_V4_NODE		DT_INST(0, gd_gd32_nv_flash_v4)
#define GD32_NV_FLASH_V4_TIMEOUT	DT_PROP(GD32_NV_FLASH_V4_NODE, max_erase_time_ms)
#define GD32_NV_FLASH_V4_SECTOR_SIZE	KB(4)

/**
 * @brief GD32 FMC v4 flash memory layout for GD32H7xx series: uniform
 * 4KB sectors over the whole main flash array.
 */
#if defined(CONFIG_FLASH_PAGE_LAYOUT) && defined(CONFIG_SOC_SERIES_GD32H7XX)
static const struct flash_pages_layout gd32_fmc_v4_layout[] = {
	{.pages_count = SOC_NV_FLASH_SIZE / GD32_NV_FLASH_V4_SECTOR_SIZE,
	 .pages_size = GD32_NV_FLASH_V4_SECTOR_SIZE},
};
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#define GD32_FMC_V4_WRITE_ERR (FMC_STAT_WPERR | FMC_STAT_PGSERR)
#define GD32_FMC_V4_ERASE_ERR (FMC_STAT_WPERR | FMC_STAT_PGSERR)

static inline void gd32_fmc_v4_unlock(void)
{
	if ((FMC_CTL & FMC_CTL_LK) != 0U) {
		FMC_KEY = UNLOCK_KEY0;
		FMC_KEY = UNLOCK_KEY1;
	}
}

static inline void gd32_fmc_v4_lock(void)
{
	FMC_CTL |= FMC_CTL_LK;
}

static int gd32_fmc_v4_wait_idle(void)
{
	const int64_t expired_time = k_uptime_get() + GD32_NV_FLASH_V4_TIMEOUT;

	while (FMC_STAT & FMC_STAT_BUSY) {
		if (k_uptime_get() > expired_time) {
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static inline void gd32_fmc_v4_clear_flags(void)
{
	/* STAT flags are cleared by writing 1 */
	FMC_STAT = FMC_STAT_ENDF | GD32_FMC_V4_WRITE_ERR;
}

bool flash_gd32_valid_range(off_t offset, uint32_t len, bool write)
{
	if ((offset < 0) || (offset > SOC_NV_FLASH_SIZE) ||
	    ((offset + len) > SOC_NV_FLASH_SIZE)) {
		return false;
	}

	if (write) {
		/* Check offset and len aligned to write-block-size. */
		return ((offset % sizeof(flash_prg_t)) == 0U) &&
		       ((len % sizeof(flash_prg_t)) == 0U);
	}

	/* Erases must cover whole sectors. */
	return ((offset % GD32_NV_FLASH_V4_SECTOR_SIZE) == 0U) &&
	       ((len % GD32_NV_FLASH_V4_SECTOR_SIZE) == 0U);
}

int flash_gd32_write_range(off_t offset, const void *data, size_t len)
{
	uint32_t addr = SOC_NV_FLASH_ADDR + offset;
	const uint32_t *prg_data = data;
	size_t words = len / sizeof(flash_prg_t);
	int ret = 0;

	gd32_fmc_v4_unlock();

	for (size_t i = 0U; i < words; i++) {
		/* Same sequence as the vendor fmc_word_program() */
		FMC_CTL |= FMC_CTL_PG;
		__ISB();
		__DSB();
		REG32(addr) = prg_data[i];
		__ISB();
		__DSB();

		ret = gd32_fmc_v4_wait_idle();
		if (ret < 0) {
			goto expired_out;
		}

		if ((FMC_STAT & GD32_FMC_V4_WRITE_ERR) != 0U) {
			ret = -EIO;
			LOG_ERR("FMC programming failed @0x%08x", addr);
			goto expired_out;
		}

		addr += sizeof(flash_prg_t);
	}

expired_out:
	FMC_CTL &= ~FMC_CTL_PG;
	gd32_fmc_v4_clear_flags();
	gd32_fmc_v4_lock();

	return ret;
}

int flash_gd32_erase_block(off_t offset, size_t size)
{
	uint32_t addr = SOC_NV_FLASH_ADDR + offset;
	int ret = 0;

	gd32_fmc_v4_unlock();

	while (size > 0U) {
		ret = gd32_fmc_v4_wait_idle();
		if (ret < 0) {
			goto expired_out;
		}

		FMC_CTL |= FMC_CTL_SER;
		FMC_ADDR = addr;
		FMC_CTL |= FMC_CTL_START;

		ret = gd32_fmc_v4_wait_idle();
		if (ret < 0) {
			goto expired_out;
		}

		if ((FMC_STAT & GD32_FMC_V4_ERASE_ERR) != 0U) {
			ret = -EIO;
			LOG_ERR("FMC sector erase failed @0x%08x", addr);
			goto expired_out;
		}

		gd32_fmc_v4_clear_flags();
		FMC_CTL &= ~FMC_CTL_SER;

		addr += GD32_NV_FLASH_V4_SECTOR_SIZE;
		size -= MIN(size, GD32_NV_FLASH_V4_SECTOR_SIZE);
	}

expired_out:
	FMC_CTL &= ~FMC_CTL_SER;
	gd32_fmc_v4_clear_flags();
	gd32_fmc_v4_lock();

	return ret;
}

#ifdef CONFIG_FLASH_PAGE_LAYOUT
void flash_gd32_pages_layout(const struct device *dev,
			     const struct flash_pages_layout **layout,
			     size_t *layout_size)
{
	ARG_UNUSED(dev);

	*layout = gd32_fmc_v4_layout;
	*layout_size = ARRAY_SIZE(gd32_fmc_v4_layout);
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
