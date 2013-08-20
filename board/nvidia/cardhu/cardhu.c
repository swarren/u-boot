/*
 *  (C) Copyright 2010-2013
 *  NVIDIA Corporation <www.nvidia.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/gp_padctrl.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include "pinmux-config-cardhu.h"
#include <i2c.h>
#include <netdev.h>

#define PMU_I2C_ADDRESS		0x2D
#define MAX_I2C_RETRY		3

/*
 * Routine: pinmux_init
 * Description: Do individual peripheral pinmux configs
 */
void pinmux_init(void)
{
	pinmux_config_table(tegra3_pinmux_common,
		ARRAY_SIZE(tegra3_pinmux_common));

	pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	/* Initialize any non-default pad configs (APB_MISC_GP regs) */
	padgrp_config_table(cardhu_padctrl, ARRAY_SIZE(cardhu_padctrl));
}

#if defined(CONFIG_TEGRA_MMC)
/*
 * Do I2C/PMU writes to bring up SD card bus power
 *
 */
void board_sdmmc_voltage_init(void)
{
	uchar reg, data_buffer[1];
	int i;

	i2c_set_bus_num(0);	/* PMU is on bus 0 */

	/* TPS659110: LDO5_REG = 3.3v, ACTIVE to SDMMC1 */
	data_buffer[0] = 0x65;
	reg = 0x32;

	for (i = 0; i < MAX_I2C_RETRY; ++i) {
		if (i2c_write(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1))
			udelay(100);
	}

	/* TPS659110: GPIO7_REG = PDEN, output a 1 to EN_3V3_SYS */
	data_buffer[0] = 0x09;
	reg = 0x67;

	for (i = 0; i < MAX_I2C_RETRY; ++i) {
		if (i2c_write(PMU_I2C_ADDRESS, reg, 1, data_buffer, 1))
			udelay(100);
	}
}

/*
 * Routine: pin_mux_mmc
 * Description: setup the MMC muxes, power rails, etc.
 */
void pin_mux_mmc(void)
{
	/*
	 * NOTE: We don't do mmc-specific pin muxes here.
	 * They were done globally in pinmux_init().
	 */

	/* Bring up the SDIO1 power rail */
	board_sdmmc_voltage_init();
}
#endif	/* MMC */

#ifdef CONFIG_PCI_TEGRA
int tegra_pcie_board_init(void)
{
	unsigned int old_bus;
	u8 addr, data[1];
	int err;

	old_bus = i2c_get_bus_num();

	err = i2c_set_bus_num(0);
	if (err) {
		debug("failed to set I2C bus\n");
		return err;
	}

	/* TPS659110: LDO1_REG = 1.05V, ACTIVE */
	data[0] = 0x15;
	addr = 0x30;

	err = i2c_write(PMU_I2C_ADDRESS, addr, 1, data, 1);
	if (err) {
		debug("failed to set VDD supply\n");
		return err;
	}

	/* GPIO: PEX = 3.3V */
	err = gpio_request(GPIO_PL7, "PEX");
	if (err < 0)
		return err;

	gpio_direction_output(GPIO_PL7, 1);

	/* TPS659110: LDO2_REG = 1.05V, ACTIVE */
	data[0] = 0x15;
	addr = 0x31;

	err = i2c_read(PMU_I2C_ADDRESS, addr, 1, data, 1);
	if (err) {
		debug("failed to set AVDD supply\n");
		return err;
	}

	i2c_set_bus_num(old_bus);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	return pci_eth_init(bis);
}
#endif /* PCI */
