/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 *
 * GigaDevice GD32H7xx Ethernet MAC driver (ENET0/ENET1).
 *
 * The GD32H7xx ENET is a DesignWare-style MAC + DMA operating against an
 * MII/RMII PHY (RMII selected through SYSCFG).  This driver manages the
 * DMA descriptors directly (4-word, ring mode) and the PHY over MDIO,
 * independent of the vendor library's global descriptor state.
 *
 * RX frames are handed to the network stack without the trailing FCS: the
 * MAC strips pad/FCS when ENET_MAC_CFG_APCD is set, and the descriptor
 * frame length then reflects the stripped length.  TX is serialized
 * through a single descriptor with a completion semaphore (the same model
 * as the STM32 HAL v1 driver); the RX path uses a descriptor ring fed by
 * the ENET interrupt through a dedicated thread.
 */

#define DT_DRV_COMPAT gd_gd32_eth

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/cache.h>
#include <ethernet/eth_stats.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <errno.h>
#include <string.h>

#include <gd32_enet.h>
#include <gd32_rcu.h>
#include <gd32_syscfg.h>

LOG_MODULE_REGISTER(eth_gd32, CONFIG_ETHERNET_LOG_LEVEL);

/* Buffer size covers MTU (1500) + 14 bytes of ethernet header + optional
 * VLAN tag (4) with room to spare; the MAC pads short frames itself. */
#define ETH_GD32_BUF_SIZE	1536U

/* Minimal MII definitions (this fork has no zephyr/drivers/mii.h). */
#define MII_BMCR		0x00U
#define MII_BMSR		0x01U
#define MII_PHYID1		0x02U
#define MII_PHYID2		0x03U
#define MII_ANAR		0x04U
#define MII_ANLPAR		0x05U

#define MII_BMCR_RESET		BIT(15)
#define MII_BMCR_POWERDOWN	BIT(11)
#define MII_BMCR_AUTONEG_EN	BIT(12)
#define MII_BMCR_RESTART	BIT(9)

#define MII_BMSR_LINK		BIT(2)
#define MII_BMSR_AUTONEG_CP	BIT(5)

#define MII_ABILITY_100FD	BIT(8)
#define MII_ABILITY_100HD	BIT(7)
#define MII_ABILITY_10FD	BIT(6)
#define MII_ABILITY_10HD	BIT(5)
#define MII_ANAR_DEFAULT	(0x01U | MII_ABILITY_100FD | MII_ABILITY_100HD | \
				 MII_ABILITY_10FD | MII_ABILITY_10HD | BIT(10))

/* MDIO clock (MDC) derived from CK_AHB (300MHz on the H7): /142 -> ~2.1MHz,
 * inside the PHY's 2.5MHz spec and the DIV142 range of the vendor table. */
#define ETH_GD32_MDC_CLK_RANGE	ENET_MDC_HCLK_DIV142

/* TX descriptor arm value: single-segment frame with completion interrupt.
 * CHM (second address chained) must be part of every status write: this IP
 * follows the explicit next-descriptor pointers in chained mode, and the
 * vendor BSP ships chained (not ring) descriptor lists. */
#define TX_ARM	(ENET_TDES0_FSG | ENET_TDES0_LSG | ENET_TDES0_INTC | \
		 ENET_TDES0_TCHM)

/* RX interrupt sources, masked NAPI-style for the whole ring drain and
 * re-armed by the RX thread once the ring is empty. */
#define ETH_GD32_DMA_INTEN_RX	(ENET_DMA_INTEN_RIE | ENET_DMA_INTEN_RBUIE)

/* Consecutive RBU interrupts tolerated before RBUIE is masked as a
 * backoff (reset on every successful receive). */
#define ETH_GD32_RBU_RUN_MAX	16U

struct eth_gd32_desc {
	volatile uint32_t status;
	volatile uint32_t control_buffer_size;
	volatile uint32_t buffer1_addr;
	volatile uint32_t buffer2_next_desc_addr;
};

struct eth_gd32_config {
	uint32_t base;
	uint16_t clk_enet;
	uint16_t clk_enet_tx;
	uint16_t clk_enet_rx;
	uint16_t clk_syscfg;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec phy_reset;
	void (*irq_config_func)(void);
};

struct eth_gd32_data {
	uint32_t base;
	const struct device *dev;
	struct net_if *iface;
	uint8_t mac_addr[6];
	uint8_t phy_addr;
	bool link_up;
	bool started;

	/* volatile probe counters for bring-up debugging */
#ifdef CONFIG_ETH_GD32_IRQ_TEST
	volatile uint32_t t_entries;
	volatile uint32_t t_bits[32];
	volatile uint32_t t_residual;
	volatile uint32_t t_last_stat;
#endif

	struct k_mutex tx_lock;
	struct k_sem tx_done;
	struct k_sem rx_sem;
	struct k_work_delayable link_work;
	struct k_thread rx_thread;

	struct eth_gd32_desc *txdesc;
	struct eth_gd32_desc *rxdesc;
	uint8_t *txbuf;
	uint8_t *rxbuf;
	uint32_t rx_idx;
	uint32_t rx_desc_num;
	volatile uint32_t rbu_run;
#ifdef CONFIG_ETH_GD32_IRQ_TEST
	volatile uint32_t t_rs_cyc;
	volatile uint32_t t_wake_cyc;
	volatile uint32_t t_copy_cyc;
	volatile uint32_t t_frame_len;
#endif
};

/* MII access ------------------------------------------------------------- */

static int eth_gd32_mdio_op(uint32_t base, uint8_t phy_addr, uint8_t reg,
			    bool write, uint16_t *value)
{
	uint32_t ctl;

	ENET_MAC_PHY_DATA(base) = write ? *value : 0U;

	ctl = ETH_GD32_MDC_CLK_RANGE |
	      MAC_PHY_CTL_PR(reg) |
	      MAC_PHY_CTL_PA(phy_addr) |
	      ENET_MAC_PHY_CTL_PB;
	if (write) {
		ctl |= ENET_MAC_PHY_CTL_PW;
	}
	ENET_MAC_PHY_CTL(base) = ctl;

	/* A single MII transaction takes ~30us at 2.1MHz; allow far more. */
	for (int i = 0; i < 200; i++) {
		if (!(ENET_MAC_PHY_CTL(base) & ENET_MAC_PHY_CTL_PB)) {
			if (!write) {
				*value = ENET_MAC_PHY_DATA(base) & 0xFFFFU;
			}
			return 0;
		}
		k_busy_wait(10U);
	}

	return -ETIMEDOUT;
}

static int eth_gd32_phy_read(const struct device *dev, uint8_t reg,
			     uint16_t *value)
{
	const struct eth_gd32_data *data = dev->data;

	return eth_gd32_mdio_op(data->base, data->phy_addr, reg, false, value);
}

static int eth_gd32_phy_write(const struct device *dev, uint8_t reg,
			      uint16_t value)
{
	const struct eth_gd32_data *data = dev->data;

	return eth_gd32_mdio_op(data->base, data->phy_addr, reg, true, &value);
}

/* MAC address ------------------------------------------------------------ */

static void eth_gd32_mac_addr_set(uint32_t base, const uint8_t *mac)
{
	ENET_MAC_ADDR0H(base) = ((uint32_t)mac[5] << 8) | mac[4];
	ENET_MAC_ADDR0L(base) = ((uint32_t)mac[3] << 24) |
				((uint32_t)mac[2] << 16) |
				((uint32_t)mac[1] << 8) | mac[0];
}

/* PHY bring-up ----------------------------------------------------------- */

static int eth_gd32_phy_reset_hw(const struct device *dev)
{
	const struct eth_gd32_config *cfg = dev->config;
	int ret;

	if (cfg->phy_reset.port == NULL) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&cfg->phy_reset, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return ret;
	}

	k_msleep(10U);
	gpio_pin_set_dt(&cfg->phy_reset, 0);
	k_msleep(50U);

	return 0;
}

static int eth_gd32_phy_scan(struct eth_gd32_data *data)
{
	for (uint8_t addr = 0U; addr < 32U; addr++) {
		uint16_t id1 = 0U, id2 = 0U;

		data->phy_addr = addr;
		if (eth_gd32_mdio_op(data->base, addr, MII_PHYID1, false, &id1) < 0 ||
		    eth_gd32_mdio_op(data->base, addr, MII_PHYID2, false, &id2) < 0) {
			continue;
		}
		if (id1 == 0x0000U || id1 == 0xFFFFU ||
		    id2 == 0x0000U || id2 == 0xFFFFU) {
			continue;
		}

		LOG_INF("PHY at MDIO address %u: ID 0x%04x%04x",
			addr, id1, id2);
		return 0;
	}

	return -ENODEV;
}

static int eth_gd32_phy_init(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	int ret;
	uint16_t bmsr;
	int timeout;

	/* BMCR reset, then start autonegotiation with all 10/100 abilities.
	 * The result (speed/duplex) is picked up by the link monitor work. */
	ret = eth_gd32_phy_write(dev, MII_BMCR, MII_BMCR_RESET);
	if (ret < 0) {
		return ret;
	}

	timeout = 100;
	do {
		k_msleep(5U);
		ret = eth_gd32_phy_read(dev, MII_BMCR, &bmsr);
		if (ret < 0) {
			return ret;
		}
	} while ((bmsr & MII_BMCR_RESET) != 0U && --timeout > 0);
	if ((bmsr & MII_BMCR_RESET) != 0U) {
		return -ETIMEDOUT;
	}

	ret = eth_gd32_phy_write(dev, MII_ANAR, MII_ANAR_DEFAULT);
	if (ret < 0) {
		return ret;
	}
	ret = eth_gd32_phy_write(dev, MII_BMCR,
				 MII_BMCR_AUTONEG_EN | MII_BMCR_RESTART);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* Link monitoring -------------------------------------------------------- */

static void eth_gd32_link_work(struct k_work *work)
{
	struct k_work_delayable *dwork =
		k_work_delayable_from_work(work);
	struct eth_gd32_data *data =
		CONTAINER_OF(dwork, struct eth_gd32_data, link_work);
	const struct device *dev = data->dev;
	uint16_t bmsr;
	uint16_t lpa;
	int ret;

	/* BMSR link status is latched-low: read twice. */
	(void)eth_gd32_phy_read(dev, MII_BMSR, &bmsr);
	ret = eth_gd32_phy_read(dev, MII_BMSR, &bmsr);
	if (ret < 0) {
		/* MDIO timeouts can occur while the PHY is reset; retry later */
		goto resched;
	}

	if ((bmsr & MII_BMSR_LINK) != 0U && !data->link_up) {
		uint32_t cfg;
		bool speed100 = true;
		bool fulldup = true;

		ret = eth_gd32_phy_read(dev, MII_ANLPAR, &lpa);
		if (ret == 0) {
			uint16_t abilities = lpa & (MII_ABILITY_100FD |
						    MII_ABILITY_100HD |
						    MII_ABILITY_10FD |
						    MII_ABILITY_10HD);

			if (abilities != 0U) {
				speed100 = (lpa & (MII_ABILITY_100FD |
						   MII_ABILITY_100HD)) != 0U;
				fulldup = (lpa & (MII_ABILITY_100FD |
						  MII_ABILITY_10FD)) != 0U;
			}
		}

		/* Program the resolved speed/duplex into the MAC. */
		cfg = ENET_MAC_CFG(data->base);
		cfg &= ~(ENET_MAC_CFG_REN | ENET_MAC_CFG_TEN |
			 ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM);
		if (speed100) {
			cfg |= ENET_MAC_CFG_SPD;
		}
		if (fulldup) {
			cfg |= ENET_MAC_CFG_DPM;
		}
		cfg |= ENET_MAC_CFG_REN | ENET_MAC_CFG_TEN;
		ENET_MAC_CFG(data->base) = cfg;

		data->link_up = true;
		net_if_carrier_on(data->iface);
		LOG_INF("Link up: %s Mbps, %s duplex",
			speed100 ? "100" : "10",
			fulldup ? "full" : "half");
	} else if ((bmsr & MII_BMSR_LINK) == 0U && data->link_up) {
		data->link_up = false;
		net_if_carrier_off(data->iface);
		LOG_INF("Link down");
	}

resched:
	k_work_reschedule(&data->link_work, K_MSEC(5000));
}

/* RX path ---------------------------------------------------------------- */

static void eth_gd32_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&data->rx_sem, K_MSEC(2));
#ifdef CONFIG_ETH_GD32_IRQ_TEST
		data->t_wake_cyc = k_cycle_get_32();
#endif

		/* Drain a bounded batch per wakeup.  The batch limit plus
		 * the blocking semaphore wait keep the IP stack (lower
		 * thread priority) running: an unbounded drain loop would
		 * starve it, and k_yield() would not help - it only hands
		 * the CPU to threads of the SAME priority. */
		for (uint32_t batch = 0U;
		     batch < CONFIG_ETH_GD32_RX_BATCH &&
		     !(data->rxdesc[data->rx_idx].status &
		       ENET_RDES0_DAV);
		     batch++) {
			struct eth_gd32_desc *desc =
				&data->rxdesc[data->rx_idx];
			uint32_t status = desc->status;
			uint32_t frame_len =
				(status & ENET_RDES0_FRML) >> 16;
			struct net_pkt *pkt = NULL;

			if ((status & (ENET_RDES0_ERRS | ENET_RDES0_LERR)) !=
				    0U ||
			    frame_len < (14U + 1U)) {
				eth_stats_update_errors_rx(data->iface);
			} else {
				uint8_t *rxptr =
					&data->rxbuf[data->rx_idx *
							ETH_GD32_BUF_SIZE];

				/* The MAC does not strip the trailing FCS
				 * (APCD has no effect on RX here), drop the
				 * last 4 bytes before handing the frame to
				 * the stack. */
				if (frame_len >= 4U) {
					frame_len -= 4U;
				}
				pkt = net_pkt_rx_alloc_with_buffer(
					data->iface, frame_len,
					NET_AF_UNSPEC, 0, K_NO_WAIT);
				if (pkt == NULL) {
					eth_stats_update_errors_rx(
						data->iface);
				} else {
#ifdef CONFIG_ETH_GD32_IRQ_TEST
					uint32_t c0 = k_cycle_get_32();
#endif
#if defined(CONFIG_CACHE_MANAGEMENT)
					sys_cache_data_invd_range(rxptr,
						frame_len);
#endif
					if (net_pkt_write(pkt, rxptr,
						frame_len) != 0) {
						net_pkt_unref(pkt);
						pkt = NULL;
						eth_stats_update_errors_rx(
							data->iface);
					}
#ifdef CONFIG_ETH_GD32_IRQ_TEST
					data->t_copy_cyc =
						k_cycle_get_32() - c0;
					data->t_frame_len = frame_len;
#endif
				}
			}

			/* Give the descriptor back to the DMA. */
			desc->status = 0U;
			barrier_dmem_fence_full();
			desc->status = ENET_RDES0_DAV;
			barrier_dmem_fence_full();
			ENET_DMA_RPEN(base) = 0U;

			data->rx_idx = (data->rx_idx + 1U) %
				       data->rx_desc_num;

			if (pkt != NULL) {
				if (net_recv_data(data->iface, pkt) != 0) {
					net_pkt_unref(pkt);
				}
			}
		}

		/* Reception may have stalled while all descriptors were held
		 * by the CPU (RBU); a poll demand restarts the fetch. */
		if (ENET_DMA_STAT(base) & ENET_DMA_STAT_RBU) {
			ENET_DMA_STAT(base) = ENET_DMA_STAT_RBU;
			ENET_DMA_RPEN(base) = 0U;
		}

		/* Re-arm RBUIE if the RBU backoff masked it: the ring has
		 * been drained now, so the RBU condition is gone. */
		if (data->started &&
		    (ENET_DMA_INTEN(base) & ENET_DMA_INTEN_RBUIE) == 0U) {
			data->rbu_run = 0U;
			ENET_DMA_INTEN(base) |= ENET_DMA_INTEN_RBUIE;
		}
	}
}

/* Interrupt handling ------------------------------------------------------ */

static void eth_gd32_mac_flags_clear(uint32_t base);

static void eth_gd32_isr(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;
	uint32_t stat = ENET_DMA_STAT(base);
	uint32_t clear = 0U;
	uint32_t rest;

#ifdef CONFIG_ETH_GD32_IRQ_TEST
	data->t_entries++;
	data->t_last_stat = stat;
	for (uint32_t bit = 0U; bit < 32U; bit++) {
		if (stat & BIT(bit)) {
			data->t_bits[bit]++;
		}
	}
#endif
	/* The interrupt line asserts whenever any status bit with its INTEN
	 * enable set is pending - the normal/abnormal summary bits are
	 * conveniences, not gates.  Every asserted source must be cleared
	 * here or the ISR re-enters immediately: the historic "interrupt
	 * storm" was the TBU bit surviving the ISR (it was only cleared in
	 * the abnormal-summary branch, which it never takes).  With a
	 * single-descriptor TX list the DMA reports TBU after every
	 * transmitted frame - a normal event on this IP. */

	if (stat & ENET_DMA_STAT_TS) {
		clear |= ENET_DMA_STAT_TS;
		k_sem_give(&data->tx_done);
	}
	if (stat & ENET_DMA_STAT_RS) {
		clear |= ENET_DMA_STAT_RS;
		data->rbu_run = 0U;
#ifdef CONFIG_ETH_GD32_IRQ_TEST
		data->t_rs_cyc = k_cycle_get_32();
#endif
		k_sem_give(&data->rx_sem);
	}
	if (stat & ENET_DMA_STAT_TBU) {
		/* Normal with an idle TX list; send() issues the next transmit
		 * poll demand itself (a poll demand here would re-assert TBU
		 * right away). */
		clear |= ENET_DMA_STAT_TBU;
	}
	if (stat & ENET_DMA_STAT_RBU) {
		/* The RX ring drained while the thread was busy; wake it so it
		 * returns the descriptors and reception resumes.  While the
		 * ring stays exhausted the RBU condition re-asserts the
		 * moment it is cleared; after a run of such interrupts with
		 * no received frame in between, mask RBUIE so the line can
		 * not livelock the ISR.  The RX thread re-arms it after the
		 * next drain. */
		clear |= ENET_DMA_STAT_RBU;
		k_sem_give(&data->rx_sem);
		if (++data->rbu_run >= ETH_GD32_RBU_RUN_MAX) {
			data->rbu_run = 0U;
			ENET_DMA_INTEN(base) &= ~ENET_DMA_INTEN_RBUIE;
			ENET_DMA_STAT(base) = ENET_DMA_STAT_RBU;
		}
	}

	if (stat & (ENET_DMA_STAT_TPS | ENET_DMA_STAT_TJT |
			    ENET_DMA_STAT_RO | ENET_DMA_STAT_TU |
			    ENET_DMA_STAT_RPS | ENET_DMA_STAT_RWT |
			    ENET_DMA_STAT_ER | ENET_DMA_STAT_ET |
			    ENET_DMA_STAT_FBE)) {
		clear |= stat & (ENET_DMA_STAT_TPS | ENET_DMA_STAT_TJT |
				 ENET_DMA_STAT_RO | ENET_DMA_STAT_TU |
				 ENET_DMA_STAT_RPS | ENET_DMA_STAT_RWT |
				 ENET_DMA_STAT_ER | ENET_DMA_STAT_ET |
				 ENET_DMA_STAT_FBE);
		if (stat & ENET_DMA_STAT_FBE) {
			LOG_ERR("fatal bus error, status 0x%08x", stat);
			eth_stats_update_errors_tx(data->iface);
		}
	}

	if (stat & (ENET_DMA_STAT_MSC | ENET_DMA_STAT_WUM |
		    ENET_DMA_STAT_TST)) {
		/* Mirrors of MAC-level events (the MSC statistics sources are
		 * individually maskable and their flags need single-bit
		 * write-1-to-clear accesses; the mirrors follow the flags). */
		eth_gd32_mac_flags_clear(base);
		rest = stat & (ENET_DMA_STAT_MSC | ENET_DMA_STAT_WUM |
			       ENET_DMA_STAT_TST);
		while (rest) {
			uint32_t bit = rest & ~(rest - 1U);

			ENET_DMA_STAT(base) = bit;
			rest &= ~bit;
		}
	}

	if (stat & (ENET_DMA_STAT_NI | ENET_DMA_STAT_AI)) {
		clear |= stat & (ENET_DMA_STAT_NI | ENET_DMA_STAT_AI);
	}

	if (clear != 0U) {
		ENET_DMA_STAT(base) = clear;
#ifdef CONFIG_ETH_GD32_IRQ_TEST
		data->t_residual |= ENET_DMA_STAT(base) & clear;
#endif
	}
}

/* TX path ----------------------------------------------------------------- */

/* Wait until the single TX descriptor has been released by the DMA.  The
 * completion interrupt is the fast path; poll the descriptor ownership as
 * a fallback.  No transmit poll demands are issued here: repeated poll
 * demands can restart an in-progress descriptor fetch on this IP. */
static int eth_gd32_tx_wait_free(struct eth_gd32_data *data)
{
	if (k_sem_take(&data->tx_done, K_MSEC(20)) == 0) {
		return 0;
	}

	for (int waited = 0; waited < 100; waited++) {
		if (!(data->txdesc->status & ENET_TDES0_DAV)) {
			/* Released without the completion interrupt:
			 * restore the semaphore token. */
			k_sem_give(&data->tx_done);
			return 0;
		}
		k_msleep(10);
	}

	return -ETIMEDOUT;
}

static int eth_gd32_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;
	size_t len = net_pkt_get_len(pkt);
	int ret;

	if (!data->started) {
		return -ENETDOWN;
	}
	if (!data->link_up) {
		return -ENETDOWN;
	}
	if (len > ETH_GD32_BUF_SIZE) {
		return -EIO;
	}

	k_mutex_lock(&data->tx_lock, K_FOREVER);

	if (eth_gd32_tx_wait_free(data) != 0) {
		k_mutex_unlock(&data->tx_lock);
		LOG_ERR("TX descriptor release timeout");
		return -EIO;
	}

#ifdef CONFIG_ETH_GD32_IRQ_TEST
	uint32_t c0 = k_cycle_get_32();
#endif
	ret = net_pkt_read(pkt, data->txbuf, len);
	if (ret != 0) {
		k_mutex_unlock(&data->tx_lock);
		return ret;
	}

#if defined(CONFIG_CACHE_MANAGEMENT)
	sys_cache_data_flush_range(data->txbuf, len);
#endif
#ifdef CONFIG_ETH_GD32_IRQ_TEST
	data->t_copy_cyc = k_cycle_get_32() - c0;
	data->t_frame_len = len;
#endif

	data->txdesc->status = 0U;
	data->txdesc->control_buffer_size = len;
	data->txdesc->buffer1_addr = (uint32_t)data->txbuf;
	barrier_dmem_fence_full();
	data->txdesc->status = TX_ARM | ENET_TDES0_DAV;
	barrier_dmem_fence_full();

	ENET_DMA_TPEN(base) = 0U;

	k_mutex_unlock(&data->tx_lock);

	return 0;
}

/* Start/stop -------------------------------------------------------------- */

/* MSC (MAC statistics) flag and mask bits.  The MSC interrupt sources are
 * enabled out of reset: every received unicast frame (and a few error
 * classes) latches a flag in MSC_RINTF / MSC_TINTF, which summarizes into
 * MAC_INTF.MSC and mirrors into DMA_STAT bit 27.  That mirror follows the
 * source flags instead of being plain W1C status, so an unhandled MSC flag
 * holds the ENET interrupt line asserted forever - observed as an ISR
 * livelock starving the whole system once any DMA interrupt was enabled.
 *
 * On this IP the MSC/MAC flag registers do not accept multi-bit write-1-
 * to-clear accesses (the vendor library also clears them one bit at a
 * time), so every clear below is a single-bit write.  The MSC sources are
 * additionally masked so the flags never latch in the first place. */

#define MSC_RX_INT_FLAGS	(ENET_MSC_RINTF_RFCE | ENET_MSC_RINTF_RFAE | \
				 ENET_MSC_RINTF_RGUF)
#define MSC_TX_INT_FLAGS	(ENET_MSC_TINTF_TGFSC | ENET_MSC_TINTF_TGFMSC | \
				 ENET_MSC_TINTF_TGF)
#define MAC_INT_FLAGS		(ENET_MAC_INTF_WUM | ENET_MAC_INTF_MSC | \
				 ENET_MAC_INTF_MSCR | ENET_MAC_INTF_MSCT | \
				 ENET_MAC_INTF_TMST)

/* Clear the latched MAC-level interrupt flags, one bit per write. */
static void eth_gd32_mac_flags_clear(uint32_t base)
{
	uint32_t rest = ENET_MAC_INTF(base) & MAC_INT_FLAGS;

	/* The MSC summary mirrors the per-source flags; kill the sources
	 * first so the summary can not re-latch mid-clear. */
	ENET_MSC_RINTF(base) = ENET_MSC_RINTF_RFCE;
	ENET_MSC_RINTF(base) = ENET_MSC_RINTF_RFAE;
	ENET_MSC_RINTF(base) = ENET_MSC_RINTF_RGUF;
	ENET_MSC_TINTF(base) = ENET_MSC_TINTF_TGFSC;
	ENET_MSC_TINTF(base) = ENET_MSC_TINTF_TGFMSC;
	ENET_MSC_TINTF(base) = ENET_MSC_TINTF_TGF;

	while (rest) {
		uint32_t bit = rest & ~(rest - 1U);

		ENET_MAC_INTF(base) = bit;
		rest &= ~bit;
	}
}

/* Mask every MAC-level interrupt source and drop the latched flags, so
 * only the DMA descriptor-completion bits in DMA_STAT remain as interrupt
 * sources (those behave like ordinary W1C bits). */
static void eth_gd32_irq_srcs_quiesce(uint32_t base)
{
	uint32_t rest;

	/* Mask the MSC interrupt sources (1 = masked) and the MAC wakeup /
	 * timestamp sources.  Read-modify-write: a plain write replaces the
	 * whole register, it does not accumulate. */
	ENET_MSC_RINTMSK(base) |= MSC_RX_INT_FLAGS;
	ENET_MSC_TINTMSK(base) |= MSC_TX_INT_FLAGS;
	ENET_MAC_INTMSK(base) |= ENET_MAC_INTMSK_WUMIM |
				 ENET_MAC_INTMSK_TMSTIM;

	eth_gd32_mac_flags_clear(base);

	/* Clear any DMA status left over from before the software reset;
	 * only the defined event bits (0..10, 13..16), multi-bit W1C is
	 * fine for this register. */
	rest = 0x1E7FFU;
	while (rest) {
		uint32_t bit = rest & ~(rest - 1U);

		ENET_DMA_STAT(base) = bit;
		rest &= ~bit;
	}
}

static int eth_gd32_start(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;

	if (data->started) {
		return -EALREADY;
	}

	/* TX: single chained descriptor, idle until a frame is submitted.
	 * The next-descriptor pointer wraps to itself. */
	data->txdesc->status = ENET_TDES0_TCHM;
	data->txdesc->control_buffer_size = 0U;
	data->txdesc->buffer1_addr = (uint32_t)data->txbuf;
	data->txdesc->buffer2_next_desc_addr = (uint32_t)data->txdesc;

	/* RX ring in chained mode: buffers owned by the DMA, explicit
	 * next-descriptor pointers wrapping back to the first entry. */
	for (uint32_t i = 0U; i < data->rx_desc_num; i++) {
		struct eth_gd32_desc *desc = &data->rxdesc[i];

		desc->status = 0U;
		desc->control_buffer_size = ENET_RDES1_RCHM |
					    ETH_GD32_BUF_SIZE;
		desc->buffer1_addr =
			(uint32_t)&data->rxbuf[i * ETH_GD32_BUF_SIZE];
		if (i == (data->rx_desc_num - 1U)) {
			desc->buffer2_next_desc_addr = (uint32_t)data->rxdesc;
		} else {
			desc->buffer2_next_desc_addr =
				(uint32_t)&data->rxdesc[i + 1U];
		}

		barrier_dmem_fence_full();
		desc->status = ENET_RDES0_DAV;
	}
	data->rx_idx = 0U;

	ENET_DMA_TDTADDR(base) = (uint32_t)data->txdesc;
	ENET_DMA_RDTADDR(base) = (uint32_t)data->rxdesc;

	/* DMA bus mode: address-aligned, fixed burst, 32-beat bursts,
	 * RX/TX arbitration 2:1 with an independent RX burst length
	 * (matches the vendor library defaults). */
	ENET_DMA_BCTL(base) = ENET_DMA_BCTL_AA | ENET_DMA_BCTL_FB |
			      ENET_PGBL_32BEAT |
			      ENET_ARBITRATION_RXTX_2_1 |
			      ENET_RXDP_32BEAT |
			      ENET_RXTX_DIFFERENT_PGBL;

	/* Store-and-forward on both directions, matching the vendor BSP. */
	ENET_DMA_CTL(base) = ENET_DMA_CTL_TSFD | ENET_DMA_CTL_RSFD;

	/* Receive everything (promiscuous); the stack filters. */
	ENET_MAC_FRMF(base) = ENET_MAC_FRMF_PM;

	/* 100M/full-duplex until the link monitor resolves the actual rate.
	 * CSD: ignore the carrier sense input, valid in full duplex and
	 * needed because the CRS_DV strap state of the LAN8720 can leave
	 * the MAC deferring transmissions otherwise. */
	ENET_MAC_CFG(base) = ENET_MAC_CFG_REN | ENET_MAC_CFG_TEN |
			     ENET_MAC_CFG_SPD | ENET_MAC_CFG_DPM |
			     ENET_MAC_CFG_APCD | ENET_MAC_CFG_CSD;

	k_sem_reset(&data->tx_done);
	k_sem_give(&data->tx_done);

	eth_gd32_irq_srcs_quiesce(base);

	/* Normal and abnormal summary plus the per-source enables; the
	 * early-transmit / early-receive interrupts serve no purpose here. */
	ENET_DMA_INTEN(base) = ENET_DMA_INTEN_NIE | ENET_DMA_INTEN_AIE |
			       ENET_DMA_INTEN_RIE | ENET_DMA_INTEN_TIE |
			       ENET_DMA_INTEN_TBUIE | ENET_DMA_INTEN_RBUIE |
			       ENET_DMA_INTEN_TPSIE | ENET_DMA_INTEN_RPSIE |
			       ENET_DMA_INTEN_TJTIE | ENET_DMA_INTEN_TUIE |
			       ENET_DMA_INTEN_ROIE | ENET_DMA_INTEN_RWTIE |
			       ENET_DMA_INTEN_FBEIE;

	ENET_DMA_CTL(base) |= ENET_DMA_CTL_STE | ENET_DMA_CTL_SRE;
	ENET_DMA_RPEN(base) = 0U;

	data->started = true;
	data->link_up = false;
	net_if_carrier_off(data->iface);
	k_work_reschedule(&data->link_work, K_NO_WAIT);

	return 0;
}

static int eth_gd32_stop(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;

	if (!data->started) {
		return -EALREADY;
	}
	data->started = false;

	k_work_cancel_delayable(&data->link_work);

	ENET_DMA_INTEN(base) = 0U;
	ENET_MAC_CFG(base) &= ~(ENET_MAC_CFG_REN | ENET_MAC_CFG_TEN);
	ENET_DMA_CTL(base) &= ~(ENET_DMA_CTL_STE | ENET_DMA_CTL_SRE);

	data->link_up = false;
	net_if_carrier_off(data->iface);

	k_sem_reset(&data->tx_done);
	k_sem_give(&data->tx_done);

	return 0;
}

/* Network interface glue --------------------------------------------------- */

static enum ethernet_hw_caps eth_gd32_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static void eth_gd32_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_gd32_data *data = dev->data;

	data->iface = iface;
	data->rx_idx = 0U;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr),
			     NET_LINK_ETHERNET);
	ethernet_init(iface);

	/* No carrier before the first successful PHY link poll. */
	net_if_carrier_off(iface);
}

static const struct ethernet_api eth_gd32_api = {
	.iface_api.init = eth_gd32_iface_init,
	.get_capabilities = eth_gd32_get_capabilities,
	.send = eth_gd32_send,
	.start = eth_gd32_start,
	.stop = eth_gd32_stop,
};

/* Instance construction ---------------------------------------------------- */

#define ETH_GD32_IRQ_CONFIG(n)						\
static void eth_gd32_irq_config_##n(void)				\
{									\
	IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),		\
		    eth_gd32_isr, DEVICE_DT_INST_GET(n), 0);		\
	irq_enable(DT_INST_IRQN(n));					\
}

#define ETH_GD32_INIT(n)						\
	PINCTRL_DT_INST_DEFINE(n);					\
	ETH_GD32_IRQ_CONFIG(n)						\
									\
	/* The ENET DMA requires its descriptors in the peripheral SRAM0	\
	 * (0x30000000); the DT "SRAM0" memory region keeps *(SRAM0)		\
	 * content.  Frame buffers live in the default SRAM: with D-cache	\
	 * enabled this needs CONFIG_CACHE_MANAGEMENT (the driver then		\
	 * invalidates RX and flushes TX buffers around each DMA access);	\
	 * with the cache maintenance in place the buffers may leave the	\
	 * 16 KB SRAM0, which alone could not hold rings larger than 8		\
	 * descriptors.								\
	 */								\
	BUILD_ASSERT((CONFIG_ETH_GD32_RX_DESC_NUM + 2U) * 16U <=	\
		     CONFIG_ETH_GD32_SRAM0_BUDGET,			\
		     "ENET descriptors must fit the SRAM0 budget");	\
	K_KERNEL_STACK_DEFINE(eth_gd32_rx_stack_##n,			\
			      CONFIG_ETH_GD32_RX_THREAD_STACK_SIZE);	\
	static struct eth_gd32_desc					\
		eth_gd32_txdesc_##n[1U] __aligned(4)			\
		__attribute__((__section__("SRAM0")));			\
	static struct eth_gd32_desc					\
		eth_gd32_rxdesc_##n[CONFIG_ETH_GD32_RX_DESC_NUM]	\
		__aligned(4)						\
		__attribute__((__section__("SRAM0")));			\
	static uint8_t							\
		eth_gd32_txbuf_##n[ETH_GD32_BUF_SIZE] __aligned(4);	\
	static uint8_t eth_gd32_rxbuf_##n				\
		[CONFIG_ETH_GD32_RX_DESC_NUM][ETH_GD32_BUF_SIZE]	\
		__aligned(4);						\
	static struct eth_gd32_data eth_gd32_data_##n = {		\
		.txdesc = eth_gd32_txdesc_##n,				\
		.rxdesc = eth_gd32_rxdesc_##n,				\
		.txbuf = eth_gd32_txbuf_##n,				\
		.rxbuf = (uint8_t *)eth_gd32_rxbuf_##n,			\
		.rx_desc_num = CONFIG_ETH_GD32_RX_DESC_NUM,		\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, local_mac_address),	\
			(.mac_addr = DT_INST_PROP(n, local_mac_address),)) \
	};								\
	static const struct eth_gd32_config eth_gd32_config_##n = {	\
		.base = DT_INST_REG_ADDR(n),				\
		.clk_enet = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, id),	\
		.clk_enet_tx = DT_INST_CLOCKS_CELL_BY_IDX(n, 1, id),	\
		.clk_enet_rx = DT_INST_CLOCKS_CELL_BY_IDX(n, 2, id),	\
		.clk_syscfg = DT_INST_CLOCKS_CELL_BY_IDX(n, 3, id),	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.phy_reset = GPIO_DT_SPEC_INST_GET_OR(n, phy_reset_gpios, {}), \
		.irq_config_func = eth_gd32_irq_config_##n,		\
	};								\
									\
	static int eth_gd32_init_##n(const struct device *dev)		\
	{								\
		struct eth_gd32_data *data = dev->data;			\
		const struct eth_gd32_config *cfg = dev->config;	\
		uint32_t cfg2;						\
		int ret;						\
		int timeout;						\
									\
		data->base = cfg->base;					\
		data->dev = dev;					\
		k_mutex_init(&data->tx_lock);				\
		k_sem_init(&data->tx_done, 0, 1);			\
		k_sem_init(&data->rx_sem, 0, 1);			\
		k_work_init_delayable(&data->link_work,			\
				      eth_gd32_link_work);		\
									\
		ret = pinctrl_apply_state(cfg->pcfg,			\
					  PINCTRL_STATE_DEFAULT);	\
		if (ret < 0) {						\
			return ret;					\
		}							\
									\
		(void)clock_control_on(GD32_CLOCK_CONTROLLER,		\
			(clock_control_subsys_t)&cfg->clk_enet);	\
		(void)clock_control_on(GD32_CLOCK_CONTROLLER,		\
			(clock_control_subsys_t)&cfg->clk_enet_tx);	\
		(void)clock_control_on(GD32_CLOCK_CONTROLLER,		\
			(clock_control_subsys_t)&cfg->clk_enet_rx);	\
		(void)clock_control_on(GD32_CLOCK_CONTROLLER,		\
			(clock_control_subsys_t)&cfg->clk_syscfg);	\
									\
		/* Feed the PHY its 50MHz reference: CK_OUT0 = PLL0P/12	\
		 * (600MHz / 12) on PA8.				\
		 */							\
		cfg2 = RCU_CFG2;					\
		cfg2 &= ~(RCU_CFG2_CKOUT0SEL | RCU_CFG2_CKOUT0DIV);	\
		cfg2 |= RCU_CKOUT0SRC_PLL0P | RCU_CKOUT0_DIV12;		\
		RCU_CFG2 = cfg2;					\
									\
		/* Select the RMII PHY interface in SYSCFG. */		\
		if (cfg->base == ENET_BASE) {				\
			SYSCFG_PMCFG |= SYSCFG_PMCFG_ENET0_PHY_SEL;	\
		} else {						\
			SYSCFG_PMCFG |= SYSCFG_PMCFG_ENET1_PHY_SEL;	\
		}							\
									\
		/* Reset the PHY with the reference clock already	\
		 * running so its straps latch correctly.		\
		 */							\
		ret = eth_gd32_phy_reset_hw(dev);			\
		if (ret < 0) {						\
			return ret;					\
		}							\
									\
		/* Software reset of MAC + DMA. */			\
		ENET_DMA_BCTL(cfg->base) = ENET_DMA_BCTL_SWR;		\
		timeout = 1000;						\
		while ((ENET_DMA_BCTL(cfg->base) & ENET_DMA_BCTL_SWR) != 0U) { \
			if (--timeout <= 0) {				\
				LOG_ERR("DMA software reset timeout");	\
				return -ETIMEDOUT;			\
			}						\
			k_busy_wait(10U);				\
		}							\
									\
		eth_gd32_mac_addr_set(cfg->base, data->mac_addr);	\
		eth_gd32_irq_srcs_quiesce(cfg->base);			\
									\
		ret = eth_gd32_phy_scan(data);				\
		if (ret < 0) {						\
			LOG_ERR("no PHY responding on MDIO");		\
			return ret;					\
		}							\
		ret = eth_gd32_phy_init(dev);				\
		if (ret < 0) {						\
			LOG_ERR("PHY init failed: %d", ret);		\
			return ret;					\
		}							\
									\
		cfg->irq_config_func();					\
									\
		k_thread_create(&data->rx_thread, eth_gd32_rx_stack_##n,\
			K_KERNEL_STACK_SIZEOF(eth_gd32_rx_stack_##n),	\
			eth_gd32_rx_thread, (void *)dev, NULL, NULL,	\
			CONFIG_ETH_GD32_RX_THREAD_PRIO, 0, K_NO_WAIT);	\
		k_thread_name_set(&data->rx_thread, "eth_gd32_rx");	\
									\
		LOG_INF("ENET MAC %02x:%02x:%02x:%02x:%02x:%02x",	\
			data->mac_addr[0], data->mac_addr[1],		\
			data->mac_addr[2], data->mac_addr[3],		\
			data->mac_addr[4], data->mac_addr[5]);		\
									\
		return 0;						\
	}								\
									\
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_gd32_init_##n, NULL,	\
		&eth_gd32_data_##n, &eth_gd32_config_##n,		\
		CONFIG_ETH_INIT_PRIORITY, &eth_gd32_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_GD32_INIT)

#ifdef CONFIG_ETH_GD32_IRQ_TEST
/* Bring-up diagnosis: runtime INTEN control and interrupt-source counters,
 * so an interrupt misbehaviour can be bisected over the shell without
 * reflashing.  Compiled only with CONFIG_ETH_GD32_IRQ_TEST=y. */
#include <stdlib.h>
#include <zephyr/shell/shell.h>

static uint32_t eth_gd32_test_base(void)
{
	struct eth_gd32_data *data = DEVICE_DT_INST_GET(0)->data;

	return data->base;
}

static struct eth_gd32_data *eth_gd32_test_data(void)
{
	return DEVICE_DT_INST_GET(0)->data;
}

static int cmd_gdeth_int(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t base = eth_gd32_test_base();
	uint32_t mask = strtoul(argv[1], NULL, 16);

	eth_gd32_irq_srcs_quiesce(base);
	ENET_DMA_INTEN(base) = mask;
	shell_print(sh, "INTEN = 0x%08x", mask);
	return 0;
}

static int cmd_gdeth_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ENET_DMA_INTEN(eth_gd32_test_base()) = 0U;
	shell_print(sh, "INTEN = 0");
	return 0;
}

static int cmd_gdeth_stat(const struct shell *sh, size_t argc, char **argv)
{
	struct eth_gd32_data *data = eth_gd32_test_data();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "entries=%u last_stat=0x%08x residual=0x%08x",
		    data->t_entries, data->t_last_stat, data->t_residual);
	shell_print(sh, "rs_cyc=%u wake_cyc=%u isr->thread=%u",
		    data->t_rs_cyc, data->t_wake_cyc,
		    data->t_wake_cyc - data->t_rs_cyc);
	shell_print(sh, "frame=%u copy_cyc=%u (%u cyc/byte)",
		    data->t_frame_len, data->t_copy_cyc,
		    data->t_frame_len ? data->t_copy_cyc /
					data->t_frame_len : 0U);
	for (uint32_t bit = 0U; bit < 32U; bit++) {
		if (data->t_bits[bit] != 0U) {
			shell_print(sh, "  bit %2u (%s): %u", bit,
				    (bit == 0)  ? "TS" :
				    (bit == 2)  ? "TBU" :
				    (bit == 6)  ? "RS" :
				    (bit == 7)  ? "RBU" :
				    (bit == 15) ? "AI" :
				    (bit == 16) ? "NI" :
				    (bit == 27) ? "MSC" :
				    (bit == 28) ? "WUM" :
				    (bit == 29) ? "TST" : "?",
				    data->t_bits[bit]);
		}
	}
	return 0;
}

static int cmd_gdeth_regs(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t base = eth_gd32_test_base();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "DMA_STAT  = 0x%08x", ENET_DMA_STAT(base));
	shell_print(sh, "DMA_INTEN = 0x%08x", ENET_DMA_INTEN(base));
	shell_print(sh, "MAC_INTF  = 0x%08x", ENET_MAC_INTF(base));
	shell_print(sh, "MAC_INTMSK= 0x%08x", ENET_MAC_INTMSK(base));
	shell_print(sh, "MSC_RINTF = 0x%08x", ENET_MSC_RINTF(base));
	shell_print(sh, "MSC_TINTF = 0x%08x", ENET_MSC_TINTF(base));
	shell_print(sh, "MSC_RINTMSK=0x%08x", ENET_MSC_RINTMSK(base));
	shell_print(sh, "MSC_TINTMSK=0x%08x", ENET_MSC_TINTMSK(base));
	return 0;
}

static int cmd_gdeth_zero(const struct shell *sh, size_t argc, char **argv)
{
	struct eth_gd32_data *data = eth_gd32_test_data();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	data->t_entries = 0U;
	data->t_residual = 0U;
	data->t_last_stat = 0U;
	memset((void *)data->t_bits, 0, sizeof(data->t_bits));
	shell_print(sh, "counters cleared");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(gdeth_cmds,
	SHELL_CMD(int, NULL, "gdeth int <hexmask>: quiesce then set INTEN",
		  cmd_gdeth_int),
	SHELL_CMD(off, NULL, "gdeth off: INTEN = 0", cmd_gdeth_off),
	SHELL_CMD(stat, NULL, "gdeth stat: ISR counters", cmd_gdeth_stat),
	SHELL_CMD(regs, NULL, "gdeth regs: interrupt register dump",
		  cmd_gdeth_regs),
	SHELL_CMD(zero, NULL, "gdeth zero: clear counters", cmd_gdeth_zero),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(gdeth, &gdeth_cmds, "GD32 ENET IRQ diagnosis", NULL);
#endif /* CONFIG_ETH_GD32_IRQ_TEST */
