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
	volatile uint32_t dbg_rx_frames;
	volatile uint32_t dbg_tx_done;
	volatile uint32_t dbg_rbu;

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

static bool eth_gd32_drv_echo(struct eth_gd32_data *data, uint8_t *frame,
			      uint16_t len);

static void eth_gd32_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		k_sem_take(&data->rx_sem, K_MSEC(2));

		while (!(data->rxdesc[data->rx_idx].status &
			 ENET_RDES0_DAV)) {
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
#if defined(ETH_GD32_DRV_ECHO) && ETH_GD32_DRV_ECHO
				if (eth_gd32_drv_echo(data, rxptr,
						      frame_len)) {
					pkt = NULL;
					goto release;
				}
#endif
				pkt = net_pkt_rx_alloc_with_buffer(
					data->iface, frame_len,
					NET_AF_UNSPEC, 0, K_NO_WAIT);
				if (pkt == NULL) {
					eth_stats_update_errors_rx(
						data->iface);
				} else {
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
				}
			}

release:
			/* Give the descriptor back to the DMA. */
			desc->status = 0U;
			barrier_dmem_fence_full();
			desc->status = ENET_RDES0_DAV;
			barrier_dmem_fence_full();
			ENET_DMA_RPEN(base) = 0U;

			data->rx_idx = (data->rx_idx + 1U) %
				       data->rx_desc_num;

			if (pkt != NULL) {
				data->dbg_rx_frames++;
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
	}
}

/* Interrupt handling ------------------------------------------------------ */

static void eth_gd32_isr(const struct device *dev)
{
	struct eth_gd32_data *data = dev->data;
	uint32_t base = data->base;
	uint32_t stat = ENET_DMA_STAT(base);
	uint32_t clear = 0U;

	if (stat & ENET_DMA_STAT_TS) {
		clear |= ENET_DMA_STAT_TS;
		data->dbg_tx_done++;
		k_sem_give(&data->tx_done);
	}
	if (stat & ENET_DMA_STAT_RS) {
		clear |= ENET_DMA_STAT_RS;
		k_sem_give(&data->rx_sem);
	}

	if (stat & ENET_DMA_STAT_AI) {
		if (stat & ENET_DMA_STAT_TBU) {
			clear |= ENET_DMA_STAT_TBU;
			/* No poll demand here: with an empty TX list the
			 * poll would re-assert TBU right away and livelock
			 * the ISR.  send() issues the poll itself. */
		}
		if (stat & ENET_DMA_STAT_RBU) {
			clear |= ENET_DMA_STAT_RBU;
			k_sem_give(&data->rx_sem);
		}
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

	if (stat & (ENET_DMA_STAT_NI | ENET_DMA_STAT_AI)) {
		clear |= stat & (ENET_DMA_STAT_NI | ENET_DMA_STAT_AI);
	}

	if (clear != 0U) {
		ENET_DMA_STAT(base) = clear;
	}
}

/* TX path ----------------------------------------------------------------- */

/* Wait until the single TX descriptor has been released by the DMA by
 * polling its ownership; the completion interrupt is not used on this IP
 * (the interrupt line re-asserts and storms once completion interrupts are
 * enabled, so the whole driver runs polled like the vendor BSP). */
static int eth_gd32_tx_wait_free(struct eth_gd32_data *data)
{
	for (int waited = 0; waited < 100; waited++) {
		if (!(data->txdesc->status & ENET_TDES0_DAV)) {
			return 0;
		}
		k_msleep(1);
	}

	return -ETIMEDOUT;
}

/* Temporary driver-level echo for latency debugging: converts an ICMP echo
 * request straight out of the RX buffer into a reply (swap MACs and IPs,
 * fix both checksums) and transmits it with no network stack involvement.
 * Disabled by default; the network stack owns ICMP in normal operation. */
#define ETH_GD32_DRV_ECHO 1
static bool eth_gd32_drv_echo(struct eth_gd32_data *data, uint8_t *frame,
			      uint16_t len)
{
	uint16_t ethertype, iplen, icmplen, sum16;
	uint32_t sum;
	uint8_t *ip, *icmp;
	uint8_t i;

	if (len < 42U) {
		return false;
	}

	ethertype = ((uint16_t)frame[12] << 8) | frame[13];
	if (ethertype != 0x0800U) {
		return false;
	}

	ip = frame + 14U;
	if (((ip[0] >> 4) != 4U) || ((ip[0] & 0xFU) < 5U) || (ip[9] != 1U)) {
		return false;
	}

	iplen = ((uint16_t)ip[2] << 8) | ip[3];
	if (iplen < 20U || (14U + iplen) > len) {
		return false;
	}

	icmp = ip + ((uint16_t)(ip[0] & 0xFU) << 2);
	if (icmp[0] != 8U) {
		return false;
	}

	icmplen = iplen - ((uint16_t)(ip[0] & 0xFU) << 2);
	icmp[0] = 0U;
	icmp[2] = 0U;
	icmp[3] = 0U;
	sum = 0U;
	for (uint16_t j = 0U; j < icmplen; j += 2U) {
		sum += ((uint16_t)icmp[j] << 8) |
		       (j + 1U < icmplen ? icmp[j + 1] : 0U);
	}
	while (sum >> 16) {
		sum = (sum & 0xFFFFU) + (sum >> 16);
	}
	sum16 = ~((uint16_t)sum);
	icmp[2] = (uint8_t)(sum16 >> 8);
	icmp[3] = (uint8_t)sum16;

	/* swap source and destination IP addresses, rebuild the header
	 * checksum */
	for (i = 0U; i < 4U; i++) {
		uint8_t tmp = ip[12U + i];

		ip[12U + i] = ip[16U + i];
		ip[16U + i] = tmp;
	}
	ip[10] = 0U;
	ip[11] = 0U;
	sum = 0U;
	for (i = 0U; i < 20U; i += 2U) {
		sum += ((uint16_t)ip[i] << 8) | ip[i + 1U];
	}
	while (sum >> 16) {
		sum = (sum & 0xFFFFU) + (sum >> 16);
	}
	sum16 = ~((uint16_t)sum);
	ip[10] = (uint8_t)(sum16 >> 8);
	ip[11] = (uint8_t)sum16;

	/* swap source and destination MACs */
	for (i = 0U; i < 6U; i++) {
		uint8_t tmp = frame[i];

		frame[i] = frame[i + 6U];
		frame[i + 6U] = tmp;
	}

	/* raw transmit, same descriptor discipline as eth_gd32_send() */
	k_mutex_lock(&data->tx_lock, K_FOREVER);
	if (eth_gd32_tx_wait_free(data) != 0) {
		k_mutex_unlock(&data->tx_lock);
		return true;
	}
	memcpy(data->txbuf, frame, len);
#if defined(CONFIG_CACHE_MANAGEMENT)
	sys_cache_data_flush_range(data->txbuf, len);
#endif
	data->txdesc->status = 0U;
	data->txdesc->control_buffer_size = len;
	data->txdesc->buffer1_addr = (uint32_t)data->txbuf;
	barrier_dmem_fence_full();
	data->txdesc->status = TX_ARM | ENET_TDES0_DAV;
	barrier_dmem_fence_full();
	ENET_DMA_TPEN(data->base) = 0U;
	k_mutex_unlock(&data->tx_lock);

	return true;
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

	ret = net_pkt_read(pkt, data->txbuf, len);
	if (ret != 0) {
		k_mutex_unlock(&data->tx_lock);
		return ret;
	}

#if defined(CONFIG_CACHE_MANAGEMENT)
	sys_cache_data_flush_range(data->txbuf, len);
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

	/* No DMA interrupts: this IP storms the interrupt line once
	 * completion interrupts are enabled (the RX thread walks the ring
	 * every couple of ms instead, like the vendor polled BSP). */
	ENET_DMA_INTEN(base) = 0U;

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
	/* The ENET DMA masters only perform reliably on the peripheral	\
	 * SRAM0 (0x30000000): descriptors and frame buffers in the	\
	 * default AXI SRAM make the DMA service them in batches with	\
	 * seconds of latency (the vendor BSP links them to SRAM0	\
	 * through .ARM.__at_0x30000000 scatter sections and marks the	\
	 * region non-cacheable for the same reason).  The DT "SRAM0"	\
	 * memory region keeps *(SRAM0) content.			\
	 */								\
	BUILD_ASSERT(CONFIG_ETH_GD32_RX_DESC_NUM * ETH_GD32_BUF_SIZE +	\
		     ETH_GD32_BUF_SIZE +				\
		     (CONFIG_ETH_GD32_RX_DESC_NUM + 2U) * 16U <=	\
		     CONFIG_ETH_GD32_SRAM0_BUDGET,			\
		     "ENET RX/TX data must fit the SRAM0 budget");	\
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
		eth_gd32_txbuf_##n[ETH_GD32_BUF_SIZE] __aligned(4)	\
		__attribute__((__section__("SRAM0")));			\
	static uint8_t eth_gd32_rxbuf_##n				\
		[CONFIG_ETH_GD32_RX_DESC_NUM][ETH_GD32_BUF_SIZE]	\
		__aligned(4)						\
		__attribute__((__section__("SRAM0")));			\
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
