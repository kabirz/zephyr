/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FLASH_GD32_OSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_FLASH_GD32_OSPI_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Switch the OSPI controller to memory-mapped read mode.
 *
 * Configures quad fast read (0x6B) as the memory-mapped read command
 * (setting the flash QE bit if needed) and switches the controller to
 * memory-mapped mode. While enabled the flash is directly readable
 * through the returned base address with plain pointer accesses;
 * erase/program are not possible via memory writes and every flash
 * API operation switches the controller back to indirect mode
 * automatically.
 *
 * @param dev OSPI flash device.
 * @param base Mapped base address output (0x90000000 for OSPI0,
 *             0x70000000 for OSPI1).
 *
 * @retval 0 on success.
 */
int flash_gd32_ospi_mm_enable(const struct device *dev, uintptr_t *base);

/**
 * @brief Switch the OSPI controller back to indirect mode.
 * @param dev OSPI flash device.
 * @retval 0 on success.
 */
int flash_gd32_ospi_mm_disable(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_FLASH_GD32_OSPI_H_ */
