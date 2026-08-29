/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32H7xx OSPI (OCTOSPI-style) NOR flash driver, indirect mode with
 * standard SPI (1-1-1) commands. One attached flash per OSPI
 * controller, described by size/jedec-id on the controller node.
 */

#define DT_DRV_COMPAT gd_gd32_ospi

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/gd32_ospi.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <gd32_ospi.h>
#include <gd32_rcu.h>

LOG_MODULE_REGISTER(flash_gd32_ospi, CONFIG_FLASH_LOG_LEVEL);

/* GD25Qxx command set (standard SPI) */
#define GD25_CMD_WREN          0x06U
#define GD25_CMD_READ          0x03U
#define GD25_CMD_PAGE_PROGRAM  0x02U
#define GD25_CMD_SECTOR_ERASE  0x20U
#define GD25_CMD_READ_STATUS   0x05U
#define GD25_CMD_READ_ID       0x9FU
#define GD25_CMD_READ_SR2      0x35U
#define GD25_CMD_WRITE_SR2     0x31U
/* quad fast read: 1-line cmd/addr, 8 dummy cycles, 4-line data */
#define GD25_CMD_QUAD_READ     0x6BU

#define GD25_PAGE_SIZE         256U
#define GD25_SECTOR_SIZE       4096U
#define GD25_SR_WIP            BIT(0)
/* status register-2: quad enable (GD25Qxx, non-QPI) */
#define GD25_SR2_QE            BIT(1)

/* memory-mapped window bases */
#define GD32_OSPI0_MM_BASE     0x90000000UL
#define GD32_OSPI1_MM_BASE     0x70000000UL

/* generous timeouts (sector erase max ~3s on GD25Q64) */
#define GD25_READY_TIMEOUT_MS  5000U

/* OSPI manager port assignment: this driver always uses port 0 */
#define GD32_OSPIM_PORT        OSPIM_PORT0

struct flash_gd32_ospi_config {
	uint32_t reg;
	uint16_t clkid;
	uint16_t ospim_clkid;
	uint32_t size;
	uint8_t jedec_id[3];
	const struct pinctrl_dev_config *pcfg;
};

struct flash_gd32_ospi_data {
	struct k_mutex lock;
	bool mm;         /* controller currently in memory-mapped mode */
};

static const struct flash_gd32_ospi_config *flash_gd32_ospi_cfg(
	const struct device *dev)
{
	return dev->config;
}

/*
 * Issue one standard-SPI command. address phase optional, data phase
 * optional; for write commands data comes from tx (nbdata bytes), for
 * read commands into rx. Mirrors the vendor EVAL flash driver flow.
 */
static void flash_gd32_ospi_cmd(const struct device *dev, uint8_t ins,
				bool with_addr, uint32_t addr,
				bool with_data, uint32_t nbdata,
				const uint8_t *tx, uint8_t *rx)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	ospi_parameter_struct ospi_cfg = {0};
	ospi_regular_cmd_struct cmd = {0};

	cmd.operation_type = OSPI_OPTYPE_COMMON_CFG;
	cmd.instruction = ins;
	cmd.ins_mode = OSPI_INSTRUCTION_1_LINE;
	cmd.ins_size = OSPI_INSTRUCTION_8_BITS;
	cmd.addr_mode = with_addr ? OSPI_ADDRESS_1_LINE : OSPI_ADDRESS_NONE;
	cmd.addr_size = OSPI_ADDRESS_24_BITS;
	cmd.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
	cmd.address = addr;
	cmd.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
	cmd.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
	cmd.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
	cmd.data_mode = with_data ? OSPI_DATA_1_LINE : OSPI_DATA_NONE;
	cmd.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
	cmd.dummy_cycles = OSPI_DUMYC_CYCLES_0;
	cmd.nbdata = nbdata;

	ospi_command_config(cfg->reg, &ospi_cfg, &cmd);

	if (with_data) {
		if (tx != NULL) {
			ospi_transmit(cfg->reg, (uint8_t *)tx);
		} else {
			ospi_receive(cfg->reg, rx);
		}
	}
}

static int flash_gd32_ospi_read_status(const struct device *dev, uint8_t *sr)
{
	flash_gd32_ospi_cmd(dev, GD25_CMD_READ_STATUS, false, 0, true, 1, NULL, sr);

	return 0;
}

static int flash_gd32_ospi_wait_ready(const struct device *dev)
{
	uint32_t deadline = k_uptime_get_32() + GD25_READY_TIMEOUT_MS;
	uint8_t sr = GD25_SR_WIP;

	while ((sr & GD25_SR_WIP) != 0U) {
		if (k_uptime_get_32() > deadline) {
			LOG_ERR("flash busy timeout");
			return -ETIMEDOUT;
		}
		k_msleep(2);
		flash_gd32_ospi_read_status(dev, &sr);
	}

	return 0;
}

static int flash_gd32_ospi_wren(const struct device *dev)
{
	flash_gd32_ospi_cmd(dev, GD25_CMD_WREN, false, 0, false, 0, NULL, NULL);

	return 0;
}

/* switch the controller from memory-mapped back to indirect mode */
static void flash_gd32_ospi_ensure_indirect(const struct device *dev)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *data = dev->data;

	if (!data->mm) {
		return;
	}

	/* wait for any in-flight mapped access, then leave mapped mode */
	while (ospi_flag_get(cfg->reg, OSPI_FLAG_BUSY) != RESET) {
	}
	ospi_functional_mode_config(cfg->reg, OSPI_INDIRECT_WRITE);
	data->mm = false;
}

int flash_gd32_ospi_mm_enable(const struct device *dev, uintptr_t *base)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *data = dev->data;
	uint8_t sr2 = 0;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->mm) {
		goto out;
	}

	/* enable quad mode on the flash once: QE (SR2 bit1) */
	flash_gd32_ospi_cmd(dev, GD25_CMD_READ_SR2, false, 0, true, 1, NULL, &sr2);
	if ((sr2 & GD25_SR2_QE) == 0U) {
		uint8_t val = sr2 | GD25_SR2_QE;

		ret = flash_gd32_ospi_wren(dev);
		if (ret != 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
		flash_gd32_ospi_cmd(dev, GD25_CMD_WRITE_SR2, false, 0,
				    true, 1, &val, NULL);
		ret = flash_gd32_ospi_wait_ready(dev);
		if (ret != 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
		LOG_INF("flash quad mode enabled (sr2 0x%02x -> 0x%02x)",
			sr2, val);
	}

	/* memory-mapped read command: quad fast read, 8 dummy cycles */
	ospi_parameter_struct ospi_cfg = {0};
	ospi_regular_cmd_struct cmd = {0};

	cmd.operation_type = OSPI_OPTYPE_READ_CFG;
	cmd.instruction = GD25_CMD_QUAD_READ;
	cmd.ins_mode = OSPI_INSTRUCTION_1_LINE;
	cmd.ins_size = OSPI_INSTRUCTION_8_BITS;
	cmd.addr_mode = OSPI_ADDRESS_1_LINE;
	cmd.addr_size = OSPI_ADDRESS_24_BITS;
	cmd.addr_dtr_mode = OSPI_ADDRDTR_MODE_DISABLE;
	cmd.alter_bytes_mode = OSPI_ALTERNATE_BYTES_NONE;
	cmd.alter_bytes_size = OSPI_ALTERNATE_BYTES_24_BITS;
	cmd.alter_bytes_dtr_mode = OSPI_ABDTR_MODE_DISABLE;
	cmd.data_mode = OSPI_DATA_4_LINES;
	cmd.data_dtr_mode = OSPI_DADTR_MODE_DISABLE;
	cmd.dummy_cycles = 8U;
	cmd.nbdata = 0;

	ospi_command_config(cfg->reg, &ospi_cfg, &cmd);

	while (ospi_flag_get(cfg->reg, OSPI_FLAG_BUSY) != RESET) {
	}
	ospi_functional_mode_config(cfg->reg, OSPI_MEMORY_MAPPED);
	data->mm = true;

out:
	if (base != NULL) {
		*base = (cfg->reg == OSPI0) ? GD32_OSPI0_MM_BASE
					    : GD32_OSPI1_MM_BASE;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

int flash_gd32_ospi_mm_disable(const struct device *dev)
{
	struct flash_gd32_ospi_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	flash_gd32_ospi_ensure_indirect(dev);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int flash_gd32_ospi_read(const struct device *dev, off_t offset,
				void *data, size_t len)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *dev_data = dev->data;

	if (offset < 0 || len > cfg->size || offset > cfg->size - len) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	flash_gd32_ospi_ensure_indirect(dev);
	flash_gd32_ospi_cmd(dev, GD25_CMD_READ, true, (uint32_t)offset,
			    true, len, NULL, data);
	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int flash_gd32_ospi_write(const struct device *dev, off_t offset,
				 const void *data, size_t len)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *dev_data = dev->data;
	const uint8_t *src = data;
	int ret;

	if (offset < 0 || len > cfg->size || offset > cfg->size - len) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	flash_gd32_ospi_ensure_indirect(dev);

	while (len > 0U) {
		/* never cross a page boundary within one program command */
		size_t page_left = GD25_PAGE_SIZE - (offset % GD25_PAGE_SIZE);
		size_t chunk = MIN(len, page_left);

		ret = flash_gd32_ospi_wren(dev);
		if (ret != 0) {
			return ret;
		}

		flash_gd32_ospi_cmd(dev, GD25_CMD_PAGE_PROGRAM, true,
				    (uint32_t)offset, true, chunk, src, NULL);

		ret = flash_gd32_ospi_wait_ready(dev);
		if (ret != 0) {
			return ret;
		}

		src += chunk;
		offset += chunk;
		len -= chunk;
	}

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int flash_gd32_ospi_erase(const struct device *dev, off_t offset,
				 size_t len)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *dev_data = dev->data;
	int ret;

	if (offset < 0 || offset % GD25_SECTOR_SIZE != 0U ||
	    len % GD25_SECTOR_SIZE != 0U ||
	    len > cfg->size || offset > cfg->size - len) {
		return -EINVAL;
	}
	if (len == 0U) {
		return 0;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	flash_gd32_ospi_ensure_indirect(dev);

	while (len > 0U) {
		ret = flash_gd32_ospi_wren(dev);
		if (ret != 0) {
			return ret;
		}

		flash_gd32_ospi_cmd(dev, GD25_CMD_SECTOR_ERASE, true,
				    (uint32_t)offset, false, 0, NULL, NULL);

		ret = flash_gd32_ospi_wait_ready(dev);
		if (ret != 0) {
			return ret;
		}

		offset += GD25_SECTOR_SIZE;
		len -= GD25_SECTOR_SIZE;
	}

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
void flash_gd32_ospi_pages_layout(const struct device *dev,
				  const struct flash_pages_layout **layout,
				  size_t *layout_size)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	static struct flash_pages_layout dev_layout;

	dev_layout.pages_count = cfg->size / GD25_SECTOR_SIZE;
	dev_layout.pages_size = GD25_SECTOR_SIZE;

	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_parameters *
flash_gd32_ospi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters params = {
		.write_block_size = 1,
		.erase_value = 0xff,
	};

	return &params;
}

static DEVICE_API(flash, flash_gd32_ospi_driver_api) = {
	.erase = flash_gd32_ospi_erase,
	.write = flash_gd32_ospi_write,
	.read = flash_gd32_ospi_read,
	.get_parameters = flash_gd32_ospi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_gd32_ospi_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

static int flash_gd32_ospi_init(const struct device *dev)
{
	const struct flash_gd32_ospi_config *cfg = flash_gd32_ospi_cfg(dev);
	struct flash_gd32_ospi_data *data = dev->data;
	ospi_parameter_struct ospi_cfg;
	uint8_t id[3] = {0};
	int ret;

	k_mutex_init(&data->lock);
	data->mm = false;

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	(void)clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);
	(void)clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->ospim_clkid);

	/* route the OSPIx SCK/CSN/IO[3:0] signals through OSPIM port 0 */
	ospim_port_sck_config(GD32_OSPIM_PORT, OSPIM_PORT_SCK_ENABLE);
	ospim_port_csn_config(GD32_OSPIM_PORT, OSPIM_PORT_CSN_ENABLE);
	ospim_port_io3_0_config(GD32_OSPIM_PORT, OSPIM_IO_LOW_ENABLE);

	if (cfg->reg == OSPI0) {
		ospim_port_sck_source_select(GD32_OSPIM_PORT, OSPIM_SCK_SOURCE_OSPI0_SCK);
		ospim_port_csn_source_select(GD32_OSPIM_PORT, OSPIM_CSN_SOURCE_OSPI0_CSN);
		ospim_port_io3_0_source_select(GD32_OSPIM_PORT, OSPIM_SRCPLIO_OSPI0_IO_LOW);
	} else {
		ospim_port_sck_source_select(GD32_OSPIM_PORT, OSPIM_SCK_SOURCE_OSPI1_SCK);
		ospim_port_csn_source_select(GD32_OSPIM_PORT, OSPIM_CSN_SOURCE_OSPI1_CSN);
		ospim_port_io3_0_source_select(GD32_OSPIM_PORT, OSPIM_SRCPLIO_OSPI1_IO_LOW);
	}

	ospi_struct_init(&ospi_cfg);
	/* prescaler 9 -> SCK = OSPI kernel clock / 10 (conservative value
	 * proven on the vendor EVAL board; GD25Q64 allows far more) */
	ospi_cfg.prescaler = 9U;
	ospi_cfg.fifo_threshold = OSPI_FIFO_THRESHOLD_4;
	ospi_cfg.sample_shift = OSPI_SAMPLE_SHIFTING_NONE;
	ospi_cfg.device_size = OSPI_MESZ_8_MBS;
	ospi_cfg.wrap_size = OSPI_DIRECT;
	ospi_cfg.cs_hightime = OSPI_CS_HIGH_TIME_3_CYCLE;
	ospi_cfg.memory_type = OSPI_STANDARD_MODE;
	ospi_cfg.delay_hold_cycle = OSPI_DELAY_HOLD_NONE;
	ospi_init(cfg->reg, &ospi_cfg);
	ospi_enable(cfg->reg);

	flash_gd32_ospi_cmd(dev, GD25_CMD_READ_ID, false, 0, true, 3, NULL, id);

	if (id[0] == 0U || id[0] == 0xffU) {
		LOG_ERR("no flash response (jedec id %02x %02x %02x)",
			id[0], id[1], id[2]);
		return -ENODEV;
	}

	if (id[0] != cfg->jedec_id[0] || id[1] != cfg->jedec_id[1] ||
	    id[2] != cfg->jedec_id[2]) {
		LOG_WRN("unexpected jedec id %02x %02x %02x (dt expects "
			"%02x %02x %02x), continuing",
			id[0], id[1], id[2],
			cfg->jedec_id[0], cfg->jedec_id[1], cfg->jedec_id[2]);
	} else {
		LOG_INF("gd25 flash ready, jedec id %02x %02x %02x, %u KB",
			id[0], id[1], id[2], cfg->size / 1024U);
	}

	return 0;
}

#define FLASH_GD32_OSPI_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	static const struct flash_gd32_ospi_config				\
		flash_gd32_ospi_config_##n = {					\
		.reg = DT_INST_REG_ADDR(n),					\
		.clkid = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id),			\
		.ospim_clkid = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, id),		\
		.size = DT_INST_PROP(n, size),					\
		.jedec_id = DT_INST_PROP(n, jedec_id),				\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
	};									\
	static struct flash_gd32_ospi_data flash_gd32_ospi_data_##n;		\
	DEVICE_DT_INST_DEFINE(n, flash_gd32_ospi_init, NULL,			\
			      &flash_gd32_ospi_data_##n,			\
			      &flash_gd32_ospi_config_##n, POST_KERNEL,	\
			      CONFIG_FLASH_INIT_PRIORITY,			\
			      &flash_gd32_ospi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_GD32_OSPI_INIT)
