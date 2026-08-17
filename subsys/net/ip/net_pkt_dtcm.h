/*
 * Copyright (c) 2026 Kabirz.
 * SPDX-License-Identifier: Apache-2.0
 *
 * DTCM variants of the network packet pool / context array definitions.
 *
 * When CONFIG_NET_PKT_DTCM is set, the rx/tx net_pkt slabs, the rx/tx
 * net_buf pools (metadata + data) and the net_context array are placed
 * in the .dtcm_noinit section provided by the zephyr,dtcm chosen node
 * (e.g. STM32F4 CCM RAM). This frees the same amount of SRAM.
 *
 * Restriction: DTCM is only accessible by the CPU, not by DMA. Only
 * valid when all network device drivers use programmed I/O. See
 * CONFIG_NET_PKT_DTCM Kconfig help for details.
 */

#ifndef ZEPHYR_SUBSYS_NET_IP_NET_PKT_DTCM_H_
#define ZEPHYR_SUBSYS_NET_IP_NET_PKT_DTCM_H_

#include <zephyr/kernel.h>
#include <zephyr/toolchain.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_context.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond INTERNAL_HIDDEN */

#if defined(CONFIG_NET_PKT_DTCM)

/* Upstream K_MEM_SLAB_DEFINE() hardcodes __noinit_named(); use the
 * _IN_SECT form to point the slab buffer at .dtcm_noinit while the
 * k_mem_slab descriptor itself stays wherever STRUCT_SECTION_ITERABLE
 * places it. */
#define NET_PKT_SLAB_DEFINE_DTCM(name, count)					     \
	K_MEM_SLAB_DEFINE_IN_SECT(name, __dtcm_noinit_section,			     \
				  sizeof(struct net_pkt), count, 4);		     \
	NET_PKT_ALLOC_STATS_DEFINE(pkt_alloc_stats_##name, name)

/* Mirror of _NET_BUF_ARRAY_DEFINE() from net_buf.h with the metadata
 * array in .dtcm_noinit. Keep in sync with upstream when updating. */
#define _NET_BUF_ARRAY_DEFINE_IN_DTCM(_name, _count, _ud_size)			       \
	struct _net_buf_##_name { uint8_t b[sizeof(struct net_buf)];		       \
				  uint8_t ud[_ud_size]; } __net_buf_align;	       \
	BUILD_ASSERT(_ud_size <= UINT8_MAX);					       \
	BUILD_ASSERT(offsetof(struct net_buf, user_data) ==			       \
		     offsetof(struct _net_buf_##_name, ud), "Invalid offset");	       \
	BUILD_ASSERT(__alignof__(struct net_buf) ==				       \
		     __alignof__(struct _net_buf_##_name), "Invalid alignment");      \
	BUILD_ASSERT(sizeof(struct _net_buf_##_name) ==			       \
		     ROUND_UP(sizeof(struct net_buf) + _ud_size, __alignof__(struct net_buf)), \
		     "Size cannot be determined");				       \
	static struct _net_buf_##_name _net_buf_##_name[_count] __dtcm_noinit_section

/* Mirror of NET_BUF_POOL_FIXED_DEFINE() from net_buf.h with the
 * metadata array and the data array in .dtcm_noinit. Keep in sync with
 * upstream when updating. */
#define NET_BUF_POOL_FIXED_DEFINE_DTCM(_name, _count, _data_size, _ud_size, _destroy)     \
	_NET_BUF_ARRAY_DEFINE_IN_DTCM(_name, _count, _ud_size);		       \
	static uint8_t __dtcm_noinit_section					       \
	net_buf_data_##_name[_count][_data_size] __net_buf_align;		       \
	static const struct net_buf_pool_fixed net_buf_fixed_##_name = {	       \
		.data_pool = (uint8_t *)net_buf_data_##_name,			       \
	};									       \
	static const struct net_buf_data_alloc net_buf_fixed_alloc_##_name = {       \
		.cb = &net_buf_fixed_cb,					       \
		.alloc_data = (void *)&net_buf_fixed_##_name,			       \
		.max_alloc_size = _data_size,					       \
	};									       \
	static STRUCT_SECTION_ITERABLE(net_buf_pool, _name) =			       \
		NET_BUF_POOL_INITIALIZER(_name, &net_buf_fixed_alloc_##_name,	       \
					 _net_buf_##_name, _count, _ud_size,	       \
					 _destroy)

/* Array of plain objects (zero-initialized at boot by their users)
 * placed in .dtcm_noinit. */
#define NET_DEFINE_DTCM(_type, _name, _count)					       \
	static _type __dtcm_noinit_section _name[_count]

#endif /* CONFIG_NET_PKT_DTCM */

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_NET_IP_NET_PKT_DTCM_H_ */
