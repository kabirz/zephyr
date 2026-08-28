/*
 * Copyright (c) 2026 GD32H7xx Zephyr bring-up
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal GD32H7xx CAN-FD controller driver: classic and FD frames
 * (up to 64 data bytes), one transmit mailbox (MB0), one receive-all
 * mailbox (MB1) with software filtering, message-buffer interrupt driven.
 */

#define DT_DRV_COMPAT gd_gd32_can

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/gd32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <string.h>

#include <gd32_can.h>
#include <gd32_rcu.h>

LOG_MODULE_REGISTER(can_gd32fd, CONFIG_CAN_LOG_LEVEL);

#define GD32_CAN_TX_MB 0U
#define GD32_CAN_RX_MB 1U

/* CAN core clock: CK_APB2 (300MHz at 600MHz operation) */
#define GD32_CAN_CLOCK_HZ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency) / 2

struct can_gd32fd_filter_slot {
	can_rx_callback_t cb;
	void *user_data;
	uint32_t id;
	uint32_t mask;
	bool extended;
	bool in_use;
};

struct can_gd32fd_data {
	struct k_sem tx_done;
	struct k_mutex tx_lock;
	struct k_spinlock filter_lock;
	can_tx_callback_t tx_cb;
	void *tx_user_data;
	bool started;
	bool tx_started;
	bool loopback;
	struct can_timing timing;
	struct can_timing data_timing;
	bool fd_mode;
	bool silent;
	uint32_t rx_buf[CAN_MAX_DLEN / sizeof(uint32_t)];
	struct can_gd32fd_filter_slot filters[CONFIG_CAN_GD32FD_MAX_FILTER];
};

struct can_gd32fd_config {
	struct can_driver_config common;
	uint32_t reg;
	uint16_t clkid;
	uint32_t can_idx;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(void);
};

static void can_gd32fd_enter_mode(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;
	can_operation_modes_enum mode;

	if (!data->started) {
		mode = CAN_DISABLE_MODE;
	} else if (data->loopback) {
		/* transmit internally looped back, bus pins ignored */
		mode = CAN_LOOPBACK_SILENT_MODE;
	} else if (data->silent) {
		mode = CAN_MONITOR_MODE;
	} else {
		mode = CAN_NORMAL_MODE;
	}

	if (can_operation_mode_enter(cfg->reg, mode) != SUCCESS) {
		LOG_ERR("failed to enter mode %u", (unsigned int)mode);
	}
}

static void can_gd32fd_apply_timing(const struct device *dev,
				    const struct can_timing *timing)
{
	const struct can_gd32fd_config *cfg = dev->config;
	can_parameter_struct para;

	can_struct_para_init(CAN_INIT_STRUCT, &para);

	para.internal_counter_source = CAN_TIMER_SOURCE_BIT_CLOCK;
	para.self_reception = DISABLE;
	para.mb_tx_order = CAN_TX_HIGH_PRIORITY_MB_FIRST;
	para.mb_tx_abort_enable = ENABLE;
	para.local_priority_enable = DISABLE;
	para.mb_rx_ide_rtr_type = CAN_IDE_RTR_FILTERED;
	para.mb_remote_frame = CAN_STORE_REMOTE_REQUEST_FRAME;
	para.rx_private_filter_queue_enable = DISABLE;
	para.edge_filter_enable = DISABLE;
	para.protocol_exception_enable = DISABLE;
	para.rx_filter_order = CAN_RX_FILTER_ORDER_MAILBOX_FIRST;
	para.memory_size = CAN_MEMSIZE_32_UNIT;
	/* accept-all mailbox public filter (from the vendor loopback example) */
	para.mb_public_filter = 0x5FFC0000U;

	para.resync_jump_width = (uint8_t)timing->sjw;
	para.prop_time_segment = (uint8_t)timing->prop_seg;
	para.time_segment_1 = (uint8_t)timing->phase_seg1;
	para.time_segment_2 = (uint8_t)timing->phase_seg2;
	para.prescaler = timing->prescaler;

	if (can_init(cfg->reg, &para) != SUCCESS) {
		LOG_ERR("can_init failed");
	}
}

/*
 * Configure the FD data phase (must be called while the controller is
 * in inactive/halt mode, alongside the other can_init-derived setup).
 * Enabling FD with 64-byte mailboxes also switches the message RAM
 * mailbox stride; the vendor helpers derive addresses from FDEN/MDSZ.
 */
static void can_gd32fd_apply_fd_config(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;

	if (!data->fd_mode) {
		CAN_CTL0(cfg->reg) &= ~CAN_CTL0_FDEN;
		return;
	}

	can_fd_parameter_struct fd;

	can_struct_para_init(CAN_FD_INIT_STRUCT, &fd);

	fd.iso_can_fd_enable = ENABLE;
	fd.bitrate_switch_enable = ENABLE;
	fd.mailbox_data_size = CAN_MAILBOX_DATA_SIZE_64_BYTES;
	/* secondary sample point at the data phase sample position */
	fd.tdc_enable = ENABLE;
	fd.tdc_offset = MIN(data->data_timing.prop_seg +
			    data->data_timing.phase_seg1, 31U);
	fd.prescaler = data->data_timing.prescaler;
	fd.resync_jump_width = data->data_timing.sjw;
	fd.prop_time_segment = data->data_timing.prop_seg;
	fd.time_segment_1 = data->data_timing.phase_seg1;
	fd.time_segment_2 = data->data_timing.phase_seg2;

	can_fd_config(cfg->reg, &fd);
}

static void can_gd32fd_install_rx_mb(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;
	can_mailbox_descriptor_struct rx;

	can_struct_para_init(CAN_MDSC_STRUCT, &rx);

	rx.code = CAN_MB_RX_STATUS_EMPTY;
	rx.ide = 0U;
	rx.rtr = 0U;
	rx.id = 0U; /* private filters default to accept-all */
	rx.data = data->rx_buf;

	can_mailbox_config(cfg->reg, GD32_CAN_RX_MB, &rx);
}

static int can_gd32fd_start(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;

	if (data->started) {
		return -EALREADY;
	}

	data->started = true;
	/* full re-initialization like the vendor loopback example: can_init
	 * from a clean state, then enter the mode, then arm the RX mailbox */
	can_gd32fd_apply_timing(dev, &data->timing);
	/*
	 * Mailbox private filters power up as an unknown non-zero mask
	 * (exact id match). Set the RX mailbox filter to "do not compare
	 * any bit" so it accepts every frame id; software filters apply.
	 * The MPF registers are only writable in halt/inactive mode.
	 */
	(void)can_operation_mode_enter(cfg->reg, CAN_INACTIVE_MODE);
	can_gd32fd_apply_fd_config(dev);
	/*
	 * The public filter mask (RMPUBF) powers up as a random RAM value
	 * and, with rx_private_filter_queue_enable disabled, filters every
	 * mailbox. Clear it (and the private filter of the RX mailbox) so
	 * that no bit takes part in the comparison: accept all frames and
	 * leave filtering to software.
	 */
	CAN_RMPUBF(cfg->reg) = 0U;
	can_private_filter_config(cfg->reg, GD32_CAN_RX_MB, 0U);
	can_gd32fd_enter_mode(dev);
	can_gd32fd_install_rx_mb(dev);
	can_interrupt_enable(cfg->reg, CAN_INT_MB0);
	can_interrupt_enable(cfg->reg, CAN_INT_MB1);

	return 0;
}

static int can_gd32fd_stop(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;

	if (!data->started) {
		return -EALREADY;
	}

	data->started = false;
	can_interrupt_disable(cfg->reg, CAN_INT_MB0);
	can_interrupt_disable(cfg->reg, CAN_INT_MB1);
	can_gd32fd_enter_mode(dev);

	return 0;
}

static int can_gd32fd_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_gd32fd_data *data = dev->data;

	if (data->started) {
		return -EBUSY;
	}

	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_FD)) != 0) {
		return -ENOTSUP;
	}

	data->loopback = (mode & CAN_MODE_LOOPBACK) != 0;
	data->silent = (mode & CAN_MODE_LISTENONLY) != 0;
	data->fd_mode = (mode & CAN_MODE_FD) != 0;

	return 0;
}

static int can_gd32fd_send(const struct device *dev,
			   const struct can_frame *frame,
			   k_timeout_t timeout,
			   can_tx_callback_t callback,
			   void *user_data)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;
	can_mailbox_descriptor_struct tx;
	uint32_t buf[CAN_MAX_DLEN / sizeof(uint32_t)] = {0};
	uint8_t bytes = can_dlc_to_bytes(MIN(frame->dlc, CANFD_MAX_DLC));
	bool fdf = (frame->flags & CAN_FRAME_FDF) != 0;
	int ret = 0;

	if (!data->started) {
		return -ENETDOWN;
	}

	if (frame->dlc > CANFD_MAX_DLC) {
		return -EINVAL;
	}

	if (!fdf && frame->dlc > CAN_MAX_DLC) {
		return -EINVAL;
	}

	if (fdf && (frame->flags & CAN_FRAME_RTR) != 0) {
		return -EINVAL;
	}

	if ((frame->flags & CAN_FRAME_BRS) != 0 && !fdf) {
		return -EINVAL;
	}

	if (!data->fd_mode && (frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS)) != 0) {
		return -ENOTSUP;
	}

	if (k_mutex_lock(&data->tx_lock, timeout) != 0) {
		return -EIO;
	}

	/*
	 * Wait for the previous transmission to leave the DATA/INACTIVE
	 * transition: a fresh mailbox starts in RX-inactive (code 0) and the
	 * first transmission may be issued directly, afterwards the mailbox
	 * returns to TX_INACTIVE once complete.
	 */
	if (data->tx_started) {
		uint32_t spins = 0;

		while (can_mailbox_code_get(cfg->reg, GD32_CAN_TX_MB) ==
		       CAN_MB_TX_STATUS_DATA) {
			if (spins++ > 100000U) {
				k_mutex_unlock(&data->tx_lock);
				return -EAGAIN;
			}
			k_yield();
		}
	}
	data->tx_started = true;

	memcpy(buf, frame->data, bytes);

	can_struct_para_init(CAN_MDSC_STRUCT, &tx);
	tx.code = CAN_MB_TX_STATUS_DATA;
	tx.ide = (frame->flags & CAN_FRAME_IDE) ? 1U : 0U;
	tx.rtr = (frame->flags & CAN_FRAME_RTR) ? 1U : 0U;
	tx.id = frame->id;
	tx.dlc = frame->dlc;
	tx.fdf = fdf ? 1U : 0U;
	tx.brs = (fdf && (frame->flags & CAN_FRAME_BRS)) ? 1U : 0U;
	tx.prio = 0U;
	tx.data = buf;
	tx.data_bytes = bytes;

	data->tx_cb = callback;
	data->tx_user_data = user_data;

	if (callback == NULL) {
		k_sem_reset(&data->tx_done);
	}

	/* writing the mailbox with code=DATA triggers the transmission */
	can_mailbox_config(cfg->reg, GD32_CAN_TX_MB, &tx);

	if (callback == NULL) {
		ret = k_sem_take(&data->tx_done, timeout);
		if (ret != 0) {
			ret = -EIO;
		}
	}

	k_mutex_unlock(&data->tx_lock);

	return ret;
}

static void can_gd32fd_dispatch_rx(const struct device *dev,
				   const can_mailbox_descriptor_struct *rx)
{
	struct can_gd32fd_data *data = dev->data;
	struct can_frame frame = {0};
	uint8_t bytes = MIN(rx->data_bytes, CAN_MAX_DLEN);
	bool delivered = false;

	frame.dlc = can_bytes_to_dlc(bytes);
	frame.id = rx->id;
	if (rx->ide != 0U) {
		frame.flags |= CAN_FRAME_IDE;
	}
	if (rx->rtr != 0U) {
		frame.flags |= CAN_FRAME_RTR;
	}
	if (rx->fdf != 0U) {
		frame.flags |= CAN_FRAME_FDF;
		if (rx->brs != 0U) {
			frame.flags |= CAN_FRAME_BRS;
		}
		if (rx->esi != 0U) {
			frame.flags |= CAN_FRAME_ESI;
		}
	}
	if (rx->data != NULL && bytes > 0) {
		memcpy(frame.data, rx->data, bytes);
	}

	k_spinlock_key_t key = k_spin_lock(&data->filter_lock);

	for (size_t i = 0; i < ARRAY_SIZE(data->filters); i++) {
		struct can_gd32fd_filter_slot *f = &data->filters[i];

		if (!f->in_use || f->extended != ((frame.flags & CAN_FRAME_IDE) != 0)) {
			continue;
		}

		if (((frame.id ^ f->id) & f->mask) != 0U) {
			continue;
		}

		f->cb(dev, &frame, f->user_data);
		delivered = true;
	}

	k_spin_unlock(&data->filter_lock, key);

	if (!delivered) {
		LOG_WRN("rx frame id=0x%x dropped (no filter match)", frame.id);
	}
}

static void can_gd32fd_isr(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;
	can_mailbox_descriptor_struct rx;

	if (can_flag_get(cfg->reg, CAN_FLAG_MB0) != RESET) {
		can_flag_clear(cfg->reg, CAN_FLAG_MB0);

		if (data->tx_cb != NULL) {
			can_tx_callback_t cb = data->tx_cb;

			data->tx_cb = NULL;
			cb(dev, 0, data->tx_user_data);
		} else {
			k_sem_give(&data->tx_done);
		}
	}

	if (can_flag_get(cfg->reg, CAN_FLAG_MB1) != RESET) {
		can_flag_clear(cfg->reg, CAN_FLAG_MB1);

		can_struct_para_init(CAN_MDSC_STRUCT, &rx);
		rx.data = data->rx_buf;
		if (can_mailbox_receive_data_read(cfg->reg, GD32_CAN_RX_MB, &rx) ==
		    SUCCESS) {
			can_gd32fd_dispatch_rx(dev, &rx);
		}

		/* rearm the receive-all mailbox */
		can_gd32fd_install_rx_mb(dev);
	}
}

static int can_gd32fd_add_rx_filter(const struct device *dev,
				    can_rx_callback_t callback,
				    void *user_data,
				    const struct can_filter *filter)
{
	struct can_gd32fd_data *data = dev->data;
	int ret = -ENOSPC;

	if ((filter->flags & ~CAN_FILTER_IDE) != 0) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&data->filter_lock);

	for (size_t i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (!data->filters[i].in_use) {
			data->filters[i].cb = callback;
			data->filters[i].user_data = user_data;
			data->filters[i].id = filter->id & filter->mask;
			data->filters[i].mask = filter->mask;
			data->filters[i].extended =
				(filter->flags & CAN_FILTER_IDE) != 0;
			data->filters[i].in_use = true;
			ret = (int)i;
			break;
		}
	}

	k_spin_unlock(&data->filter_lock, key);

	return ret;
}

static void can_gd32fd_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_gd32fd_data *data = dev->data;

	if (filter_id < 0 || (size_t)filter_id >= ARRAY_SIZE(data->filters)) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&data->filter_lock);

	data->filters[filter_id].in_use = false;
	k_spin_unlock(&data->filter_lock, key);
}

static int can_gd32fd_get_capabilities(const struct device *dev,
				       can_mode_t *caps)
{
	ARG_UNUSED(dev);

	*caps = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;

	if (IS_ENABLED(CONFIG_CAN_FD_MODE)) {
		*caps |= CAN_MODE_FD;
	}

	return 0;
}

static int can_gd32fd_set_timing(const struct device *dev,
				 const struct can_timing *timing)
{
	struct can_gd32fd_data *data = dev->data;

	if (data->started) {
		return -EBUSY;
	}

	data->timing = *timing;

	return 0;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_gd32fd_set_timing_data(const struct device *dev,
				      const struct can_timing *timing)
{
	struct can_gd32fd_data *data = dev->data;

	if (data->started) {
		return -EBUSY;
	}

	data->data_timing = *timing;

	return 0;
}
#endif /* CONFIG_CAN_FD_MODE */

static int can_gd32fd_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	*rate = GD32_CAN_CLOCK_HZ;

	return 0;
}

static int can_gd32fd_get_state(const struct device *dev, enum can_state *state,
				struct can_bus_err_cnt *err_cnt)
{
	const struct can_gd32fd_config *cfg = dev->config;

	if (state != NULL) {
		switch (can_error_state_get(cfg->reg)) {
		case CAN_ERROR_STATE_PASSIVE:
			*state = CAN_STATE_ERROR_PASSIVE;
			break;
		case CAN_ERROR_STATE_BUS_OFF:
			*state = CAN_STATE_BUS_OFF;
			break;
		default:
			*state = CAN_STATE_ERROR_ACTIVE;
			break;
		}
	}

	if (err_cnt != NULL) {
		can_error_counter_struct cnt;

		can_error_counter_get(cfg->reg, &cnt);
		err_cnt->tx_err_cnt = cnt.tx_errcnt;
		err_cnt->rx_err_cnt = cnt.rx_errcnt;
	}

	return 0;
}



static DEVICE_API(can, can_gd32fd_driver_api) = {
	.start = can_gd32fd_start,
	.stop = can_gd32fd_stop,
	.set_mode = can_gd32fd_set_mode,
	.set_timing = can_gd32fd_set_timing,
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_gd32fd_set_timing_data,
#endif
	.send = can_gd32fd_send,
	.add_rx_filter = can_gd32fd_add_rx_filter,
	.remove_rx_filter = can_gd32fd_remove_rx_filter,
	.get_capabilities = can_gd32fd_get_capabilities,
	.get_state = can_gd32fd_get_state,
	.set_state_change_callback = NULL,
	.get_core_clock = can_gd32fd_get_core_clock,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 32,
		.prop_seg = 64,
		.phase_seg1 = 32,
		.phase_seg2 = 32,
		.prescaler = 1024,
	},
#ifdef CONFIG_CAN_FD_MODE
	/* CAN_FDBT: DSJW 1..8, DPTS 0..31, DPBS1 1..8, DPBS2 2..8,
	 * DBAUDPSC 1..1024 */
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_data_max = {
		.sjw = 8,
		.prop_seg = 31,
		.phase_seg1 = 8,
		.phase_seg2 = 8,
		.prescaler = 1024,
	},
#endif /* CONFIG_CAN_FD_MODE */
};

static int can_gd32fd_init(const struct device *dev)
{
	const struct can_gd32fd_config *cfg = dev->config;
	struct can_gd32fd_data *data = dev->data;
	struct can_timing timing = {
		/* 1Mbit/s @ 300MHz CK_APB2, sample point 80% (vendor example) */
		.sjw = 1,
		.prop_seg = 2,
		.phase_seg1 = 5,
		.phase_seg2 = 2,
		.prescaler = 30,
	};
	struct can_timing calc;
	int ret;

	/* derive the initial timing from the devicetree bitrate when the
	 * common calculator finds a solution (1M -> prescaler 3 / 100tq) */
	if (can_calc_timing(dev, &calc, cfg->common.bitrate,
			    cfg->common.sample_point) == 0) {
		calc.sjw = MAX(1, MIN(calc.sjw, calc.phase_seg2));
		timing = calc;
	}

	data->timing = timing;

#ifdef CONFIG_CAN_FD_MODE
	{
		struct can_timing data_timing = {
			/* 5Mbit/s data phase @ 300MHz: prescaler 2, 30 tq,
			 * sample point 80% */
			.sjw = 4,
			.prop_seg = 15,
			.phase_seg1 = 8,
			.phase_seg2 = 6,
			.prescaler = 2,
		};

		if (can_calc_timing_data(dev, &calc, cfg->common.bitrate_data,
					 cfg->common.sample_point_data) == 0) {
			calc.sjw = MAX(1, MIN(calc.sjw, calc.phase_seg2));
			data_timing = calc;
		}

		data->data_timing = data_timing;
	}
#endif /* CONFIG_CAN_FD_MODE */

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	(void)clock_control_on(GD32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	/* CK_CAN from CK_APB2 (300MHz) */
	rcu_can_clock_config((can_idx_enum)cfg->can_idx, RCU_CANSRC_APB2);

	k_sem_init(&data->tx_done, 0, 1);
	k_mutex_init(&data->tx_lock);

	cfg->irq_config_func();

	return 0;
}

#define CAN_GD32FD_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);						\
	static void can_gd32fd_irq_config_##n(void)				\
	{									\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, message, irq),		\
			    DT_INST_IRQ_BY_NAME(n, message, priority),		\
			    can_gd32fd_isr,					\
			    DEVICE_DT_INST_GET(n), 0);				\
		irq_enable(DT_INST_IRQ_BY_NAME(n, message, irq));		\
	}									\
	static struct can_gd32fd_data can_gd32fd_data_##n;			\
	static const struct can_gd32fd_config can_gd32fd_config_##n = {	\
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(n, 125000, 1000000),	\
		.reg = DT_INST_REG_ADDR(n),					\
		.clkid = DT_INST_CLOCKS_CELL(n, id),				\
		/* CAN base + 0x1000 * index (CAN0..CAN2) */			\
		.can_idx = (DT_INST_REG_ADDR(n) - 0x4001A000U) / 0x1000U,	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),			\
		.irq_config_func = can_gd32fd_irq_config_##n,			\
	};									\
	CAN_DEVICE_DT_INST_DEFINE(n, can_gd32fd_init, NULL,			\
				  &can_gd32fd_data_##n,			\
				  &can_gd32fd_config_##n,			\
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,	\
				  &can_gd32fd_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_GD32FD_INIT)
