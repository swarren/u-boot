/*
 * (C) Copyright 2012-2013,2015 Stephen Warren
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <config.h>
#include <dm.h>
#include <fdt_support.h>
#include <fdt_simplefb.h>
#include <lcd.h>
#include <mmc.h>
#include <asm/gpio.h>
#include <asm/arch/mbox.h>
#include <asm/arch/sdhci.h>
#include <asm/global_data.h>
#include <dm/platform_data/serial_pl01x.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

static const struct bcm2835_gpio_platdata gpio_platdata = {
	.base = BCM2835_GPIO_BASE,
};

U_BOOT_DEVICE(bcm2835_gpios) = {
	.name = "gpio_bcm2835",
	.platdata = &gpio_platdata,
};

static const struct pl01x_serial_platdata serial_platdata = {
#ifdef CONFIG_BCM2836
	.base = 0x3f201000,
#else
	.base = 0x20201000,
#endif
	.type = TYPE_PL011,
	.clock = 3000000,
};

U_BOOT_DEVICE(bcm2835_serials) = {
	.name = "serial_pl01x",
	.platdata = &serial_platdata,
};

struct msg_get_arm_mem {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_arm_mem get_arm_mem;
	u32 end_tag;
};

struct msg_get_board_rev {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_board_rev get_board_rev;
	u32 end_tag;
};

struct msg_get_mac_address {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_mac_address get_mac_address;
	u32 end_tag;
};

struct msg_set_power_state {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_set_power_state set_power_state;
	u32 end_tag;
};

struct msg_get_clock_rate {
	struct bcm2835_mbox_hdr hdr;
	struct bcm2835_mbox_tag_get_clock_rate get_clock_rate;
	u32 end_tag;
};

/* See comments in mbox.h for data source */
static const struct {
	const char *name;
	const char *fdtfile;
	bool has_onboard_eth;
} models[] = {
	[0] = {
		"Unknown model",
#ifdef CONFIG_BCM2836
		"bcm2836-rpi-other.dtb",
#else
		"bcm2835-rpi-other.dtb",
#endif
		false,
	},
#ifdef CONFIG_BCM2836
	[BCM2836_BOARD_REV_2_B] = {
		"2 Model B",
		"bcm2836-rpi-2-b.dtb",
		true,
	},
#else
	[BCM2835_BOARD_REV_B_I2C0_2] = {
		"Model B (no P5)",
		"bcm2835-rpi-b-i2c0.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C0_3] = {
		"Model B (no P5)",
		"bcm2835-rpi-b-i2c0.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_4] = {
		"Model B",
		"bcm2835-rpi-b.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_5] = {
		"Model B",
		"bcm2835-rpi-b.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_I2C1_6] = {
		"Model B",
		"bcm2835-rpi-b.dtb",
		true,
	},
	[BCM2835_BOARD_REV_A_7] = {
		"Model A",
		"bcm2835-rpi-a.dtb",
		false,
	},
	[BCM2835_BOARD_REV_A_8] = {
		"Model A",
		"bcm2835-rpi-a.dtb",
		false,
	},
	[BCM2835_BOARD_REV_A_9] = {
		"Model A",
		"bcm2835-rpi-a.dtb",
		false,
	},
	[BCM2835_BOARD_REV_B_REV2_d] = {
		"Model B rev2",
		"bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_REV2_e] = {
		"Model B rev2",
		"bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_REV2_f] = {
		"Model B rev2",
		"bcm2835-rpi-b-rev2.dtb",
		true,
	},
	[BCM2835_BOARD_REV_B_PLUS] = {
		"Model B+",
		"bcm2835-rpi-b-plus.dtb",
		true,
	},
	[BCM2835_BOARD_REV_CM] = {
		"Compute Module",
		"bcm2835-rpi-cm.dtb",
		false,
	},
	[BCM2835_BOARD_REV_A_PLUS] = {
		"Model A+",
		"bcm2835-rpi-a-plus.dtb",
		false,
	},
#endif
};

u32 rpi_board_rev = 0;

int dram_init(void)
{
	ALLOC_ALIGN_BUFFER(struct msg_get_arm_mem, msg, 1, 16);
	int ret;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_arm_mem, GET_ARM_MEMORY);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query ARM memory size\n");
		return -1;
	}

	gd->ram_size = msg->get_arm_mem.body.resp.mem_size;

	return 0;
}

static void set_fdtfile(void)
{
	const char *fdtfile;

	if (getenv("fdtfile"))
		return;

	fdtfile = models[rpi_board_rev].fdtfile;
	setenv("fdtfile", fdtfile);
}

static void set_usbethaddr(void)
{
	ALLOC_ALIGN_BUFFER(struct msg_get_mac_address, msg, 1, 16);
	int ret;

	if (!models[rpi_board_rev].has_onboard_eth)
		return;

	if (getenv("usbethaddr"))
		return;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_mac_address, GET_MAC_ADDRESS);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query MAC address\n");
		/* Ignore error; not critical */
		return;
	}

	eth_setenv_enetaddr("usbethaddr", msg->get_mac_address.body.resp.mac);

	return;
}

int misc_init_r(void)
{
	set_fdtfile();
	set_usbethaddr();
	return 0;
}

static int power_on_module(u32 module)
{
	ALLOC_ALIGN_BUFFER(struct msg_set_power_state, msg_pwr, 1, 16);
	int ret;

	BCM2835_MBOX_INIT_HDR(msg_pwr);
	BCM2835_MBOX_INIT_TAG(&msg_pwr->set_power_state,
			      SET_POWER_STATE);
	msg_pwr->set_power_state.body.req.device_id = module;
	msg_pwr->set_power_state.body.req.state =
		BCM2835_MBOX_SET_POWER_STATE_REQ_ON |
		BCM2835_MBOX_SET_POWER_STATE_REQ_WAIT;

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN,
				     &msg_pwr->hdr);
	if (ret) {
		printf("bcm2835: Could not set module %u power state\n",
		       module);
		return -1;
	}

	return 0;
}

static void get_board_rev(void)
{
	ALLOC_ALIGN_BUFFER(struct msg_get_board_rev, msg, 1, 16);
	int ret;
	const char *name;

	BCM2835_MBOX_INIT_HDR(msg);
	BCM2835_MBOX_INIT_TAG(&msg->get_board_rev, GET_BOARD_REV);

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg->hdr);
	if (ret) {
		printf("bcm2835: Could not query board revision\n");
		/* Ignore error; not critical */
		return;
	}

	/*
	 * For details of old-vs-new scheme, see:
	 * https://github.com/pimoroni/RPi.version/blob/master/RPi/version.py
	 * http://www.raspberrypi.org/forums/viewtopic.php?f=63&t=99293&p=690282
	 * (a few posts down)
	 */
	rpi_board_rev = msg->get_board_rev.body.resp.rev;
	if (rpi_board_rev & 0x800000)
		rpi_board_rev = (rpi_board_rev >> 4) & 0xff;
	if (rpi_board_rev >= ARRAY_SIZE(models)) {
		printf("RPI: Board rev %u outside known range\n",
		       rpi_board_rev);
		rpi_board_rev = 0;
	}
	if (!models[rpi_board_rev].name) {
		printf("RPI: Board rev %u unknown\n", rpi_board_rev);
		rpi_board_rev = 0;
	}

	name = models[rpi_board_rev].name;
	printf("RPI %s\n", name);
}

/* A bunch of stuff lifted from pinctrl-bcm2835.c in Linux */

/* GPIO register offsets */
#define GPFSEL0		0x0	/* Function Select */
#define GPSET0		0x1c	/* Pin Output Set */
#define GPCLR0		0x28	/* Pin Output Clear */
#define GPLEV0		0x34	/* Pin Level */
#define GPEDS0		0x40	/* Pin Event Detect Status */
#define GPREN0		0x4c	/* Pin Rising Edge Detect Enable */
#define GPFEN0		0x58	/* Pin Falling Edge Detect Enable */
#define GPHEN0		0x64	/* Pin High Detect Enable */
#define GPLEN0		0x70	/* Pin Low Detect Enable */
#define GPAREN0		0x7c	/* Pin Async Rising Edge Detect */
#define GPAFEN0		0x88	/* Pin Async Falling Edge Detect */
#define GPPUD		0x94	/* Pin Pull-up/down Enable */
#define GPPUDCLK0	0x98	/* Pin Pull-up/down Enable Clock */

#define FSEL_REG(p)		(GPFSEL0 + (((p) / 10) * 4))
#define FSEL_SHIFT(p)		(((p) % 10) * 3)
#define GPIO_REG_OFFSET(p)	((p) / 32)
#define GPIO_REG_SHIFT(p)	((p) % 32)

enum bcm2835_fsel {
	BCM2835_FSEL_GPIO_IN = 0,
	BCM2835_FSEL_GPIO_OUT = 1,
	BCM2835_FSEL_ALT0 = 4,
	BCM2835_FSEL_ALT1 = 5,
	BCM2835_FSEL_ALT2 = 6,
	BCM2835_FSEL_ALT3 = 7,
	BCM2835_FSEL_ALT4 = 3,
	BCM2835_FSEL_ALT5 = 2,
	BCM2835_FSEL_COUNT = 8,
	BCM2835_FSEL_MASK = 0x7,
};

static inline uint32_t bcm2835_gpio_rd(uint32_t reg)
{
	return readl((void *)(0x20200000 + reg));
}

static inline void bcm2835_gpio_wr(uint32_t reg, uint32_t val)
{
	writel(val, (void *)(0x20200000 + reg));
}

static inline void bcm2835_pinctrl_fsel_set(int pin, enum bcm2835_fsel fsel)
{
	uint32_t val = bcm2835_gpio_rd(FSEL_REG(pin));
	enum bcm2835_fsel cur = (val >> FSEL_SHIFT(pin)) & BCM2835_FSEL_MASK;

	if (cur == fsel)
		return;

	if (cur != BCM2835_FSEL_GPIO_IN && fsel != BCM2835_FSEL_GPIO_IN) {
		/* always transition through GPIO_IN */
		val &= ~(BCM2835_FSEL_MASK << FSEL_SHIFT(pin));
		val |= BCM2835_FSEL_GPIO_IN << FSEL_SHIFT(pin);
		bcm2835_gpio_wr(FSEL_REG(pin), val);
	}

	val &= ~(BCM2835_FSEL_MASK << FSEL_SHIFT(pin));
	val |= fsel << FSEL_SHIFT(pin);

	bcm2835_gpio_wr(FSEL_REG(pin), val);
}

void enable_jtag(void)
{
	bcm2835_pinctrl_fsel_set(4,  BCM2835_FSEL_ALT5);
	bcm2835_pinctrl_fsel_set(22, BCM2835_FSEL_ALT4);
	bcm2835_pinctrl_fsel_set(23, BCM2835_FSEL_ALT4);
	bcm2835_pinctrl_fsel_set(24, BCM2835_FSEL_ALT4);
	bcm2835_pinctrl_fsel_set(25, BCM2835_FSEL_ALT4);
	bcm2835_pinctrl_fsel_set(27, BCM2835_FSEL_ALT4);
}

int board_init(void)
{
	enable_jtag();

	get_board_rev();

	gd->bd->bi_boot_params = 0x100;

	return power_on_module(BCM2835_MBOX_POWER_DEVID_USB_HCD);
}

int board_mmc_init(bd_t *bis)
{
	ALLOC_ALIGN_BUFFER(struct msg_get_clock_rate, msg_clk, 1, 16);
	int ret;

	power_on_module(BCM2835_MBOX_POWER_DEVID_SDHCI);

	BCM2835_MBOX_INIT_HDR(msg_clk);
	BCM2835_MBOX_INIT_TAG(&msg_clk->get_clock_rate, GET_CLOCK_RATE);
	msg_clk->get_clock_rate.body.req.clock_id = BCM2835_MBOX_CLOCK_ID_EMMC;

	ret = bcm2835_mbox_call_prop(BCM2835_MBOX_PROP_CHAN, &msg_clk->hdr);
	if (ret) {
		printf("bcm2835: Could not query eMMC clock rate\n");
		return -1;
	}

	return bcm2835_sdhci_init(BCM2835_SDHCI_BASE,
				  msg_clk->get_clock_rate.body.resp.rate_hz);
}

int ft_board_setup(void *blob, bd_t *bd)
{
	/*
	 * For now, we simply always add the simplefb DT node. Later, we
	 * should be more intelligent, and e.g. only do this if no enabled DT
	 * node exists for the "real" graphics driver.
	 */
	lcd_dt_simplefb_add_node(blob);

	return 0;
}
