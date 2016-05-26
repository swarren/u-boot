/*
 * Copyright (c) 2016, NVIDIA CORPORATION.
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef _TEGRA186_BPMP_H
#define _TEGRA186_BPMP_H

int tegra186_bpmp_call(struct udevice *dev, uint32_t mrq,
		       void *tx_msg, uint32_t tx_size,
		       void *rx_msg, uint32_t rx_size);

#endif
