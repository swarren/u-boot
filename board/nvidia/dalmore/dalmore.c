/*
 * Copyright (c) 2010-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <common.h>
#include <asm-generic/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/gp_padctrl.h>
#include <asm/arch/pinmux.h>
#include "pinmux-config-dalmore.h"
#include <i2c.h>

#define BAT_I2C_ADDRESS		0x48	/* TPS65090 charger */
#define PMU_I2C_ADDRESS		0x58	/* TPS65913 PMU */

/*
 * Routine: pinmux_init
 * Description: Do individual peripheral pinmux configs
 */
void pinmux_init(void)
{
	pinmux_config_table(tegra114_pinmux_set_nontristate,
		ARRAY_SIZE(tegra114_pinmux_set_nontristate));

	pinmux_config_table(tegra114_pinmux_common,
		ARRAY_SIZE(tegra114_pinmux_common));

	pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	/* Initialize any non-default pad configs (APB_MISC_GP regs) */
	padgrp_config_table(dalmore_padctrl, ARRAY_SIZE(dalmore_padctrl));
}

/* Writes val to reg @ chip address pmu */
void i2c_write_pmic(uchar pmu, uchar reg, uchar val)
{
	uchar data_buffer[1];
	int ret;

	data_buffer[0] = val;

	ret = i2c_write(pmu, reg, 1, data_buffer, 1);
	if (ret)
		printf("%s: PMU i2c_write %02X<-%02X returned %d\n",
			__func__, reg, data_buffer[0], ret);
}

#if defined(CONFIG_TEGRA_MMC)
/*
 * Do I2C/PMU writes to bring up SD card bus power
 *
 */
void board_sdmmc_voltage_init(void)
{
	int ret = i2c_set_bus_num(0);	/* PMU is on bus 0 */
	if (ret)
		printf("%s: i2c_set_bus_num returned %d\n", __func__, ret);

	/* TPS65913: LDO9_VOLTAGE = 3.3V */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x61, 0x31);

	/* TPS65913: LDO9_CTRL = Active */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x60, 0x01);

	/* TPS65090: FET6_CTRL = enable output auto discharge, enable FET6 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x14, 0x03);
}

void board_vreg_init(void)
{
	int ret = i2c_set_bus_num(0);	/* PMU is on bus 0 */
	if (ret)
		printf("%s: i2c_set_bus_num returned %d\n", __func__, ret);

	/*
	 * Enable USB voltage: AVDD_USB_HDMI for AVDD_USB_AP
	 *			and AVDD_HDMI_AP
	 *   LDOUSB_VOLTAGE = 3.3v
	 *   LDOUSB_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x65, 0x31);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x64, 0x01);

	/*
	 * Enable HVDD_USB3 voltage: HVDD_USB3_AP
	 *   LDOLN_VOLTAGE = 3.3v
	 *   LDOLN_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x63, 0x31);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x62, 0x01);

	/*
	 * Enable additional VDD_1V1_CORE
	 *
	 *   SMPS7_CTRL: enable active: auto
	 *
	 *   VDD_CORE is provided by SMPS4_SW, 5 and 7 where
	 *   4 and 5 are enabled after power on.
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x30, 0x05);

	/*
	 * Set and enable AVDD_2V8_CAM1
	 *   LDO1_VOLTAGE = 2.8v
	 *   LDO1_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x51, 0x27);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x50, 0x01);

	/*
	 * Set and enable AVDD_2V8_CAM2
	 *   LDO2_VOLTAGE = 2.8v
	 *   LDO2_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x53, 0x27);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x52, 0x01);

	/*
	 * Set and enable AVDD_1V2 for VDDIO_HSIC_AP and AVDD_DSI_CSI_AP
	 *   LDO3_VOLTAGE = 1.2v
	 *   LDO3_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x55, 0x07);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x54, 0x01);

	/*
	 * Set and enable VPP_FUSE_APP
	 *   LDO4_VOLTAGE = 1.8v
	 *   LDO4_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x57, 0x13);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x56, 0x01);

	/*
	 * Set and enable VDD_1V2_LCD
	 *   LDO5_VOLTAGE = 1.2v (TPS65913)
	 *   LDO5_CTRL = Active
	 *
	 * Enable VDD_LCD_BL
	 *   VOUT1 (FET1) (TPS65090): auto discharge and enable
	 *
	 * Enable AVDD_LCD
	 *   VOUT4 (FET4) (TPS65090): auto discharge and enable
	 *
	 * Enable VDD_LVDS
	 *   VOUT5 (FET5) (TPS65090): auto discharge and enable
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x59, 0x07); /* LDO5_VOLTAGE */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x58, 0x01); /* LD05_CTRL */
	i2c_write_pmic(BAT_I2C_ADDRESS, 0x0F, 0x03); /* VOUT1 (FET1) */
	i2c_write_pmic(BAT_I2C_ADDRESS, 0x12, 0x03); /* VOUT4 (FET4) */
	i2c_write_pmic(BAT_I2C_ADDRESS, 0x13, 0x03); /* VOUT5 (FET5) */

	/*
	 * Set and enable VDD_SENSOR
	 *   LDO6_VOLTAGE = 2.85v
	 *   LDO6_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x5B, 0x28);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x5A, 0x01);

	/*
	 * Set and enable AVDD_2V8_CAM_AF1
	 *   LDO7_VOLTAGE = 2.8v
	 *   LDO7_CTRL = Active
	 */
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x5D, 0x27);
	i2c_write_pmic(PMU_I2C_ADDRESS, 0x5C, 0x01);

	/*
	 * Enable VDD_3V3_COM
	 *   VOUT7 (FET7) (TPS65090): auto discharge and enable
	 */
	i2c_write_pmic(BAT_I2C_ADDRESS, 0x15, 0x03);

	/* Enable LCD backlight */
	gpio_direction_output(DSI_PANEL_BL_EN_GPIO, 1);
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

	/* Bring up the SDIO3 power rail */
	board_sdmmc_voltage_init();
}
#endif /* MMC */
