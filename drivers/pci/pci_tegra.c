/*
 * Copyright (c) 2010, CompuLab, Ltd.
 * Author: Mike Rapoport <mike@compulab.co.il>
 *
 * Based on NVIDIA PCIe driver
 * Copyright (c) 2008-2009, NVIDIA Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <errno.h>
#include <fdtdec.h>
#include <malloc.h>
#include <pci.h>

#include <asm/io.h>
#include <asm/gpio.h>

#include <linux/list.h>

DECLARE_GLOBAL_DATA_PTR;

int fdt_n_addr_cells(const void *fdt, int node)
{
	const u32 *prop;
	int len, parent;

	do {
		parent = fdt_parent_offset(fdt, node);
		if (parent >= 0)
			node = parent;

		prop = fdt_getprop(fdt, node, "#address-cells", &len);
		if (prop)
			return fdt32_to_cpup(prop);
	} while (parent >= 0);

	return 1;
}

int fdt_n_size_cells(const void *fdt, int node)
{
	const u32 *prop;
	int len, parent;

	do {
		parent = fdt_parent_offset(fdt, node);
		if (parent >= 0)
			node = parent;

		prop = fdt_getprop(fdt, node, "#size-cells", &len);
		if (prop)
			return fdt32_to_cpup(prop);
	} while (parent >= 0);

	return 1;
}

struct fdt_resource {
	fdt_addr_t start;
	fdt_addr_t end;
};

static inline fdt_size_t fdt_resource_size(const struct fdt_resource *res)
{
	return res->end - res->start + 1;
}

int fdt_get_resource(const void *fdt, int node, const char *property,
		     unsigned int index, struct fdt_resource *res)
{
	const fdt32_t *ptr, *end;
	unsigned int i = 0;
	int na, ns, len;

	na = fdt_n_addr_cells(fdt, node);
	ns = fdt_n_size_cells(fdt, node);

	ptr = fdt_getprop(fdt, node, property, &len);
	if (!ptr)
		return len;

	end = ptr + len / 4;

	while (ptr + na + ns <= end) {
		if (i == index) {
			res->start = fdt_addr_to_cpup(ptr);
			res->end = res->start + fdt_size_to_cpup(ptr + na) - 1;
			return 0;
		}

		ptr += na + ns;
		i++;
	}

	return -FDT_ERR_NOTFOUND;
}

int fdt_get_string_index(const void *fdt, int node, const char *property,
			 const char *string)
{
	const char *list, *end;
	int len, index = 0;

	list = fdt_getprop(fdt, node, property, &len);
	if (!list)
		return len;

	end = list + len;

	while (list < end) {
		int n = strlen(string);
		int m = strlen(list);

		if (n == m && memcmp(list, string, n) == 0)
			return index;

		list += max(n, m) + 1;
		index++;
	}

	return -FDT_ERR_NOTFOUND;
}

int fdt_get_named_resource(const void *fdt, int node, const char *property,
			   const char *names, const char *name,
			   struct fdt_resource *res)
{
	int index;

	index = fdt_get_string_index(fdt, node, names, name);
	if (index < 0)
		return index;

	return fdt_get_resource(fdt, node, property, index, res);
}

int fdtdec_pci_get_bdf(const void *fdt, int node, pci_dev_t *bdf)
{
	const fdt32_t *prop;
	int len;

	prop = fdt_getprop(fdt, node, "reg", &len);
	if (!prop)
		return len;

	*bdf = fdt32_to_cpup(prop) & 0xffffff;

	return 0;
}

#define AFI_AXI_BAR0_SZ	0x00
#define AFI_AXI_BAR1_SZ	0x04
#define AFI_AXI_BAR2_SZ	0x08
#define AFI_AXI_BAR3_SZ	0x0c
#define AFI_AXI_BAR4_SZ	0x10
#define AFI_AXI_BAR5_SZ	0x14

#define AFI_AXI_BAR0_START	0x18
#define AFI_AXI_BAR1_START	0x1c
#define AFI_AXI_BAR2_START	0x20
#define AFI_AXI_BAR3_START	0x24
#define AFI_AXI_BAR4_START	0x28
#define AFI_AXI_BAR5_START	0x2c

#define AFI_FPCI_BAR0	0x30
#define AFI_FPCI_BAR1	0x34
#define AFI_FPCI_BAR2	0x38
#define AFI_FPCI_BAR3	0x3c
#define AFI_FPCI_BAR4	0x40
#define AFI_FPCI_BAR5	0x44

#define AFI_CACHE_BAR0_SZ	0x48
#define AFI_CACHE_BAR0_ST	0x4c
#define AFI_CACHE_BAR1_SZ	0x50
#define AFI_CACHE_BAR1_ST	0x54

#define AFI_MSI_BAR_SZ		0x60
#define AFI_MSI_FPCI_BAR_ST	0x64
#define AFI_MSI_AXI_BAR_ST	0x68

#define AFI_CONFIGURATION		0xac
#define  AFI_CONFIGURATION_EN_FPCI	(1 << 0)

#define AFI_FPCI_ERROR_MASKS	0xb0

#define AFI_INTR_MASK		0xb4
#define  AFI_INTR_MASK_INT_MASK	(1 << 0)
#define  AFI_INTR_MASK_MSI_MASK	(1 << 8)

#define AFI_SM_INTR_ENABLE	0xc4
#define  AFI_SM_INTR_INTA_ASSERT	(1 << 0)
#define  AFI_SM_INTR_INTB_ASSERT	(1 << 1)
#define  AFI_SM_INTR_INTC_ASSERT	(1 << 2)
#define  AFI_SM_INTR_INTD_ASSERT	(1 << 3)
#define  AFI_SM_INTR_INTA_DEASSERT	(1 << 4)
#define  AFI_SM_INTR_INTB_DEASSERT	(1 << 5)
#define  AFI_SM_INTR_INTC_DEASSERT	(1 << 6)
#define  AFI_SM_INTR_INTD_DEASSERT	(1 << 7)

#define AFI_AFI_INTR_ENABLE		0xc8
#define  AFI_INTR_EN_INI_SLVERR		(1 << 0)
#define  AFI_INTR_EN_INI_DECERR		(1 << 1)
#define  AFI_INTR_EN_TGT_SLVERR		(1 << 2)
#define  AFI_INTR_EN_TGT_DECERR		(1 << 3)
#define  AFI_INTR_EN_TGT_WRERR		(1 << 4)
#define  AFI_INTR_EN_DFPCI_DECERR	(1 << 5)
#define  AFI_INTR_EN_AXI_DECERR		(1 << 6)
#define  AFI_INTR_EN_FPCI_TIMEOUT	(1 << 7)
#define  AFI_INTR_EN_PRSNT_SENSE	(1 << 8)

#define AFI_PCIE_CONFIG					0x0f8
#define  AFI_PCIE_CONFIG_PCIE_DISABLE(x)		(1 << ((x) + 1))
#define  AFI_PCIE_CONFIG_PCIE_DISABLE_ALL		0xe
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_MASK	(0xf << 20)
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_SINGLE	(0x0 << 20)
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_420	(0x0 << 20)
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_DUAL	(0x1 << 20)
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_222	(0x1 << 20)
#define  AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_411	(0x2 << 20)

#define AFI_FUSE			0x104
#define  AFI_FUSE_PCIE_T0_GEN2_DIS	(1 << 2)

#define AFI_PEX0_CTRL			0x110
#define AFI_PEX1_CTRL			0x118
#define AFI_PEX2_CTRL			0x128
#define  AFI_PEX_CTRL_RST		(1 << 0)
#define  AFI_PEX_CTRL_CLKREQ_EN		(1 << 1)
#define  AFI_PEX_CTRL_REFCLK_EN		(1 << 3)

#define AFI_PEXBIAS_CTRL_0		0x168

#define PADS_CTL_SEL		0x0000009C

#define PADS_CTL		0x000000A0
#define  PADS_CTL_IDDQ_1L	(1 <<  0)
#define  PADS_CTL_TX_DATA_EN_1L	(1 <<  6)
#define  PADS_CTL_RX_DATA_EN_1L	(1 << 10)

#define PADS_PLL_CTL_TEGRA20			0x000000B8
#define PADS_PLL_CTL_TEGRA30			0x000000B4
#define  PADS_PLL_CTL_RST_B4SM			(0x1 <<  1)
#define  PADS_PLL_CTL_LOCKDET			(0x1 <<  8)
#define  PADS_PLL_CTL_REFCLK_MASK		(0x3 << 16)
#define  PADS_PLL_CTL_REFCLK_INTERNAL_CML	(0x0 << 16)
#define  PADS_PLL_CTL_REFCLK_INTERNAL_CMOS	(0x1 << 16)
#define  PADS_PLL_CTL_REFCLK_EXTERNAL		(0x2 << 16)
#define  PADS_PLL_CTL_TXCLKREF_MASK		(0x1 << 20)
#define  PADS_PLL_CTL_TXCLKREF_DIV10		(0x0 << 20)
#define  PADS_PLL_CTL_TXCLKREF_DIV5		(0x1 << 20)
#define  PADS_PLL_CTL_TXCLKREF_BUF_EN		(0x1 << 22)

#define PADS_REFCLK_CFG0			0x000000C8
#define PADS_REFCLK_CFG1			0x000000CC

/*
 * Fields in PADS_REFCLK_CFG*. Those registers form an array of 16-bit
 * entries, one entry per PCIe port. These field definitions and desired
 * values aren't in the TRM, but do come from NVIDIA.
 */
#define PADS_REFCLK_CFG_TERM_SHIFT		2  /* 6:2 */
#define PADS_REFCLK_CFG_E_TERM_SHIFT		7
#define PADS_REFCLK_CFG_PREDI_SHIFT		8  /* 11:8 */
#define PADS_REFCLK_CFG_DRVI_SHIFT		12 /* 15:12 */

/* Default value provided by HW engineering is 0xfa5c */
#define PADS_REFCLK_CFG_VALUE \
	( \
		(0x17 << PADS_REFCLK_CFG_TERM_SHIFT)   | \
		(0    << PADS_REFCLK_CFG_E_TERM_SHIFT) | \
		(0xa  << PADS_REFCLK_CFG_PREDI_SHIFT)  | \
		(0xf  << PADS_REFCLK_CFG_DRVI_SHIFT)     \
	)

#define RP_VEND_XP	0x00000F00
#define  RP_VEND_XP_DL_UP	(1 << 30)

#define RP_LINK_CONTROL_STATUS			0x00000090
#define  RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE	0x20000000
#define  RP_LINK_CONTROL_STATUS_LINKSTAT_MASK	0x3fff0000

struct tegra_pcie;

struct tegra_pcie_port {
	struct tegra_pcie *pcie;

	struct fdt_resource regs;
	unsigned int num_lanes;
	unsigned int index;

	struct list_head list;
};

struct tegra_pcie_soc {
	unsigned int num_ports;
	unsigned long pads_pll_ctl;
	unsigned long tx_ref_sel;
	bool has_pex_clkreq_en;
	bool has_pex_bias_ctrl;
	bool has_cml_clk;
};

struct tegra_pcie {
	struct pci_controller hose;

	struct fdt_resource pads;
	struct fdt_resource afi;
	struct fdt_resource cs;

	struct fdt_resource prefetch;
	struct fdt_resource mem;
	struct fdt_resource io;

	struct list_head ports;
	unsigned long xbar;

	const struct tegra_pcie_soc *soc;
};

static inline struct tegra_pcie *to_tegra_pcie(struct pci_controller *hose)
{
	return container_of(hose, struct tegra_pcie, hose);
}

#define fdt_for_each_subnode(fdt, node, parent)		\
	for (node = fdt_first_subnode(fdt, parent);	\
	     node >= 0;					\
	     node = fdt_next_subnode(fdt, node))

static void afi_writel(struct tegra_pcie *pcie, unsigned long value,
		       unsigned long offset)
{
	writel(value, pcie->afi.start + offset);
}

static unsigned long afi_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(pcie->afi.start + offset);
}

static void pads_writel(struct tegra_pcie *pcie, unsigned long value,
			unsigned long offset)
{
	writel(value, pcie->pads.start + offset);
}

static unsigned long pads_readl(struct tegra_pcie *pcie, unsigned long offset)
{
	return readl(pcie->pads.start + offset);
}

static unsigned long rp_readl(struct tegra_pcie_port *port,
			      unsigned long offset)
{
	return readl(port->regs.start + offset);
}

static unsigned long tegra_pcie_conf_offset(pci_dev_t bdf, int where)
{
	return ((where & 0xf00) << 16) | (PCI_BUS(bdf) << 16) |
	       (PCI_DEV(bdf) << 11) | (PCI_FUNC(bdf) << 8) |
	       (where & 0xfc);
}

static int tegra_pcie_conf_address(struct tegra_pcie *pcie, pci_dev_t bdf,
				   int where, unsigned long *address)
{
	unsigned int bus = PCI_BUS(bdf);

	if (bus == 0) {
		unsigned int dev = PCI_DEV(bdf);
		struct tegra_pcie_port *port;

		list_for_each_entry(port, &pcie->ports, list) {
			if (port->index + 1 == dev) {
				*address = port->regs.start + (where & ~3);
				return 0;
			}
		}
	} else {
		*address = pcie->cs.start + tegra_pcie_conf_offset(bdf, where);
		return 0;
	}

	return -1;
}

static int tegra_pcie_read_conf(struct pci_controller *hose, pci_dev_t bdf,
				int where, u32 *value)
{
	struct tegra_pcie *pcie = to_tegra_pcie(hose);
	unsigned long address;
	int err;

	err = tegra_pcie_conf_address(pcie, bdf, where, &address);
	if (err < 0) {
		*value = 0xffffffff;
		return 1;
	}

	*value = readl(address);

	/* fixup root port class */
	if (PCI_BUS(bdf) == 0) {
		if (where == PCI_CLASS_REVISION) {
			*value &= ~0x00ff0000;
			*value |= PCI_CLASS_BRIDGE_PCI << 16;
		}
	}

	return 0;
}

static int tegra_pcie_write_conf(struct pci_controller *hose, pci_dev_t bdf,
				 int where, u32 value)
{
	struct tegra_pcie *pcie = to_tegra_pcie(hose);
	unsigned long address;
	int err;

	err = tegra_pcie_conf_address(pcie, bdf, where, &address);
	if (err < 0)
		return 1;

	writel(value, address);

	return 0;
}

static int tegra_pcie_port_parse_dt(const void *fdt, int node,
				    struct tegra_pcie_port *port)
{
	const u32 *addr;
	int len;

	addr = fdt_getprop(fdt, node, "assigned-addresses", &len);
	if (!addr) {
		error("tegra-pcie: property \"assigned-addresses\" not found\n");
		return -FDT_ERR_NOTFOUND;
	}

	port->regs.start = fdt32_to_cpup(addr + 2);
	port->regs.end = port->regs.start + fdt32_to_cpup(addr + 4);

	return 0;
}

static int tegra_pcie_get_xbar_config(const void *fdt, int node, u32 lanes,
				      unsigned long *xbar)
{
	enum fdt_compat_id id = fdtdec_lookup(fdt, node);

	switch (id) {
	case COMPAT_NVIDIA_TEGRA20_PCIE:
		switch (lanes) {
		case 0x00000004:
			debug("tegra-pcie: single-mode configuration\n");
			*xbar = AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_SINGLE;
			return 0;

		case 0x00000202:
			debug("tegra-pcie: dual-mode configuration\n");
			*xbar = AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_DUAL;
			return 0;
		}
		break;

	case COMPAT_NVIDIA_TEGRA30_PCIE:
		switch (lanes) {
		case 0x00000204:
			debug("tegra-pcie: 4x1, 2x1 configuration\n");
			*xbar = AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_420;
			return 0;

		case 0x00020202:
			debug("tegra-pcie: 2x3 configuration\n");
			*xbar = AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_222;
			return 0;

		case 0x00010104:
			debug("tegra-pcie: 4x1, 1x2 configuration\n");
			*xbar = AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_411;
			return 0;
		}

	default:
		break;
	}

	return -FDT_ERR_NOTFOUND;
}

static int tegra_pcie_parse_dt_ranges(const void *fdt, int node,
				      struct tegra_pcie *pcie)
{
	const u32 *ptr, *end;
	int len;

	ptr = fdt_getprop(fdt, node, "ranges", &len);
	if (!ptr) {
		error("tegra-pcie: missing \"ranges\" property\n");
		return -FDT_ERR_NOTFOUND;
	}

	end = ptr + len / 4;

	while (ptr < end) {
		struct fdt_resource *res = NULL;
		u32 space = fdt32_to_cpup(ptr);

		switch ((space >> 24) & 0x3) {
		case 0x01:
			res = &pcie->io;
			break;

		case 0x02: /* 32 bit */
		case 0x03: /* 64 bit */
			if (space & (1 << 30))
				res = &pcie->prefetch;
			else
				res = &pcie->mem;

			break;
		}

		if (res) {
			res->start = fdt32_to_cpup(ptr + 3);
			res->end = res->start + fdt32_to_cpup(ptr + 5);
		}

		ptr += 3 + 1 + 2;
	}

	debug("tegra-pcie: PCI regions:\n");
	debug("tegra-pcie:   I/O: %#x-%#x\n", pcie->io.start, pcie->io.end);
	debug("tegra-pcie:   non-prefetchable memory: %#x-%#x\n",
	      pcie->mem.start, pcie->mem.end);
	debug("tegra-pcie:   prefetchable memory: %#x-%#x\n",
	      pcie->prefetch.start, pcie->prefetch.end);

	return 0;
}

static int tegra_pcie_parse_port_info(const void *fdt, int node,
				      unsigned int *index,
				      unsigned int *lanes)
{
	pci_dev_t bdf;
	int err;

	err = fdtdec_get_int(fdt, node, "nvidia,num-lanes", 0);
	if (err < 0) {
		error("tegra-pcie: failed to parse \"nvidia,num-lanes\" property\n");
		return err;
	}

	*lanes = err;

	err = fdtdec_pci_get_bdf(fdt, node, &bdf);
	if (err < 0) {
		error("tegra-pcie: failed to parse \"reg\" property\n");
		return err;
	}

	*index = PCI_DEV(bdf) - 1;

	return 0;
}

static int tegra_pcie_parse_dt(const void *fdt, int node,
			       struct tegra_pcie *pcie)
{
	int err, subnode;
	u32 lanes = 0;

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "pads",
				     &pcie->pads);
	if (err < 0) {
		error("tegra-pcie: resource \"pads\" not found\n");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "afi",
				     &pcie->afi);
	if (err < 0) {
		error("tegra-pcie: resource \"afi\" not found\n");
		return err;
	}

	err = fdt_get_named_resource(fdt, node, "reg", "reg-names", "cs",
				     &pcie->cs);
	if (err < 0) {
		error("tegra-pcie: resource \"cs\" not found\n");
		return err;
	}

	err = tegra_pcie_parse_dt_ranges(fdt, node, pcie);
	if (err < 0) {
		error("tegra-pcie: failed to parse \"ranges\" property\n");
		return err;
	}

	fdt_for_each_subnode(fdt, subnode, node) {
		unsigned int index = 0, num_lanes = 0;
		struct tegra_pcie_port *port;

		err = tegra_pcie_parse_port_info(fdt, subnode, &index,
						 &num_lanes);
		if (err < 0) {
			error("tegra-pcie: failed to obtain root port info\n");
			continue;
		}

		lanes |= num_lanes << (index << 3);

		if (!fdtdec_get_is_enabled(fdt, subnode))
			continue;

		port = malloc(sizeof(*port));
		if (!port)
			continue;

		memset(port, 0, sizeof(*port));
		port->num_lanes = num_lanes;
		port->index = index;

		err = tegra_pcie_port_parse_dt(fdt, subnode, port);
		if (err < 0) {
			free(port);
			continue;
		}

		list_add_tail(&port->list, &pcie->ports);
		port->pcie = pcie;
	}

	err = tegra_pcie_get_xbar_config(fdt, node, lanes, &pcie->xbar);
	if (err < 0) {
		error("tegra-pcie: invalid lane configuration\n");
		return err;
	}

	return 0;
}

enum tegra_powergate {
	TEGRA_POWERGATE_CPU,
	TEGRA_POWERGATE_3D,
	TEGRA_POWERGATE_VENC,
	TEGRA_POWERGATE_PCIE,
	TEGRA_POWERGATE_VDEC,
	TEGRA_POWERGATE_L2,
	TEGRA_POWERGATE_MPE,
	TEGRA_POWERGATE_HEG,
	TEGRA_POWERGATE_SATA,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
	TEGRA_POWERGATE_CELP,
	TEGRA_POWERGATE_3D1,
};

#define PWRGATE_TOGGLE	0x30
#define  PWRGATE_TOGGLE_START (1 << 8)

#define PWRGATE_STATUS	0x38

static int tegra_powergate_set(enum tegra_powergate id, bool state)
{
	unsigned long value;
	bool old_state;

	value = readl(NV_PA_PMC_BASE + PWRGATE_STATUS);
	old_state = value & (1 << id);

	if (state == old_state)
		return 0;

	writel(PWRGATE_TOGGLE_START | id, NV_PA_PMC_BASE + PWRGATE_TOGGLE);

	return 0;
}

static int tegra_powergate_power_on(enum tegra_powergate id)
{
	return tegra_powergate_set(id, true);
}

static int tegra_powergate_power_off(enum tegra_powergate id)
{
	return tegra_powergate_set(id, false);
}

static int tegra_powergate_remove_clamping(enum tegra_powergate id)
{
	unsigned long value;

	if (id == TEGRA_POWERGATE_VDEC)
		value = 1 << TEGRA_POWERGATE_PCIE;
	else if (id == TEGRA_POWERGATE_PCIE)
		value = 1 << TEGRA_POWERGATE_VDEC;
	else
		value = 1 << id;

	writel(value, NV_PA_PMC_BASE + 0x34);

	return 0;
}

static int tegra_powergate_sequence_power_up(enum tegra_powergate id)
{
	int err;

	/* reset PCIe controller */
	writel(1 << 6, NV_PA_CLK_RST_BASE + 0x310);

	err = tegra_powergate_power_on(id);
	if (err < 0)
		return err;

	/* enable PCIe clock */
	writel(1 << 6, NV_PA_CLK_RST_BASE + 0x330);

	udelay(10);

	err = tegra_powergate_remove_clamping(id);
	if (err < 0)
		return err;

	udelay(10);

	/* take PCIe controller out of reset */
	writel(1 << 6, NV_PA_CLK_RST_BASE + 0x314);

	return 0;
}

int __weak tegra_pcie_board_init(void)
{
	return 0;
}

static int train_plle(void)
{
	unsigned int timeout = 2000;
	unsigned long value;

	value = readl(NV_PA_PMC_BASE + 0x1ac);
	value |= 1 << 5;
	writel(value, NV_PA_PMC_BASE + 0x1ac);

	value = readl(NV_PA_PMC_BASE + 0x1ac);
	value |= 1 << 4;
	writel(value, NV_PA_PMC_BASE + 0x1ac);

	value = readl(NV_PA_PMC_BASE + 0x1ac);
	value &= ~(1 << 5);
	writel(value, NV_PA_PMC_BASE + 0x1ac);

	do {
		value = readl(NV_PA_CLK_RST_BASE + 0x0ec);
		if (value & (1 << 15))
			break;

		udelay(100);
	} while (--timeout);

	if (timeout == 0) {
		error("tegra-pcie: timeout waiting for PLLE to become ready\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int setup_plle(void)
{
	unsigned int cpcon = 11, p = 18, n = 150, m = 1, timeout = 1000;
	unsigned long value;
	int err;

	/* disable PLLE clock */
	value = readl(NV_PA_CLK_RST_BASE + 0x0e8);
	value &= ~(1 << 31);
	value &= ~(1 << 30);
	writel(value, NV_PA_CLK_RST_BASE + 0x0e8);

	/* clear lock enable and setup field */
	value = readl(NV_PA_CLK_RST_BASE + 0x0ec);
	value &= ~(1 << 9 | 0xffff000c);
	writel(value, NV_PA_CLK_RST_BASE + 0x0ec);

	value = readl(NV_PA_CLK_RST_BASE + 0x0ec);
	if ((value & (1 << 15)) == 0) {
		err = train_plle();
		if (err < 0) {
			error("tegra-pcie: failed to train PLLE\n");
			return err;
		}
	}

	/* configure PLLE */
	value = readl(NV_PA_CLK_RST_BASE + 0x0e8);

	value &= ~(0x0f << 24);
	value |= cpcon << 24;

	value &= ~(0x3f << 16);
	value |= p << 16;

	value &= ~(0xff <<  8);
	value |= n << 8;

	value &= ~(0xff <<  0);
	value |= m << 0;

	writel(value, NV_PA_CLK_RST_BASE + 0x0e8);

	value = readl(NV_PA_CLK_RST_BASE + 0x0ec);
	value |= (0x7 << 16) | (1 << 9) | (0 << 2);
	writel(value, NV_PA_CLK_RST_BASE + 0x0ec);

	value = readl(NV_PA_CLK_RST_BASE + 0x068);
	value |= 0x7 << 10;
	writel(value, NV_PA_CLK_RST_BASE + 0x068);

	value = readl(NV_PA_CLK_RST_BASE + 0x0e8);
	value |= (1 << 31) | (1 << 30);
	writel(value, NV_PA_CLK_RST_BASE + 0x0e8);

	do {
		value = readl(NV_PA_CLK_RST_BASE + 0x0ec);
		if (value & (1 << 11))
			break;

		udelay(2);
	} while (--timeout);

	if (timeout == 0) {
		error("tegra-pcie: timeout waiting for PLLE to lock\n");
		return -ETIMEDOUT;
	}

	udelay(50);

	value = readl(NV_PA_CLK_RST_BASE + 0x068);
	value &= ~(0x03f << 24); /* incintrv */
	value |= 0x18 << 24;

	value &= ~(0x0ff << 16); /* inc */
	value |= 0x01 << 16;

	value &= ~(0x007 << 10); /* disable */
	value |= 0x00 << 10;

	value &= ~(0x1ff <<  0); /* max */
	value |= 0x24 << 0;
	writel(value, NV_PA_CLK_RST_BASE + 0x068);

	return 0;
}

static int tegra_pcie_power_on(struct tegra_pcie *pcie)
{
	const struct tegra_pcie_soc *soc = pcie->soc;
	unsigned long value;
	int err;

	/* reset PCIEXCLK logic, AFI controller and PCIe controller */
	writel(1 << 10, NV_PA_CLK_RST_BASE + 0x310);
	writel(1 <<  8, NV_PA_CLK_RST_BASE + 0x310);
	writel(1 <<  6, NV_PA_CLK_RST_BASE + 0x310);

	tegra_powergate_power_off(TEGRA_POWERGATE_PCIE);

	tegra_pcie_board_init();

	tegra_powergate_sequence_power_up(TEGRA_POWERGATE_PCIE);

	/* take AFI controller out of reset */
	writel(1 << 8, NV_PA_CLK_RST_BASE + 0x314);

	/* enable AFI clock */
	writel(1 << 8, NV_PA_CLK_RST_BASE + 0x330);

	if (soc->has_cml_clk) {
		/* enable CML clock */
		value = readl(NV_PA_CLK_RST_BASE + 0x48c);
		value |= (1 << 0);
		value &= ~(1 << 1);
		writel(value, NV_PA_CLK_RST_BASE + 0x48c);
	}

	err = setup_plle();
	if (err < 0) {
		error("PLLE setup failed\n");
		return err;
	}

	return 0;
}

static int tegra_pcie_enable_controller(struct tegra_pcie *pcie)
{
	const struct tegra_pcie_soc *soc = pcie->soc;
	struct tegra_pcie_port *port;
	unsigned int timeout = 300;
	unsigned long value;

	if (soc->has_pex_bias_ctrl)
		afi_writel(pcie, 0, AFI_PEXBIAS_CTRL_0);

	value = afi_readl(pcie, AFI_PCIE_CONFIG);
	value &= ~AFI_PCIE_CONFIG_SM2TMS0_XBAR_CONFIG_MASK;
	value |= AFI_PCIE_CONFIG_PCIE_DISABLE_ALL | pcie->xbar;

	list_for_each_entry(port, &pcie->ports, list)
		value &= ~AFI_PCIE_CONFIG_PCIE_DISABLE(port->index);

	afi_writel(pcie, value, AFI_PCIE_CONFIG);

	value = afi_readl(pcie, AFI_FUSE);
	value &= ~AFI_FUSE_PCIE_T0_GEN2_DIS;
	afi_writel(pcie, value, AFI_FUSE);

	/* initialize internal PHY, enable up to 16 PCIe lanes */
	pads_writel(pcie, 0, PADS_CTL_SEL);

	/* override IDDQ to 1 on all 4 lanes */
	value = pads_readl(pcie, PADS_CTL);
	value |= PADS_CTL_IDDQ_1L;
	pads_writel(pcie, value, PADS_CTL);

	/*
	 * Set up PHY PLL inputs select PLLE output as refclock, set TX
	 * ref sel to div10 (not div5).
	 */
	value = pads_readl(pcie, soc->pads_pll_ctl);
	value &= ~(PADS_PLL_CTL_REFCLK_MASK | PADS_PLL_CTL_TXCLKREF_MASK);
	value |= PADS_PLL_CTL_REFCLK_INTERNAL_CML | soc->tx_ref_sel;
	pads_writel(pcie, value, soc->pads_pll_ctl);

	/* take PLL out of reset */
	value = pads_readl(pcie, soc->pads_pll_ctl);
	value |= PADS_PLL_CTL_RST_B4SM;
	pads_writel(pcie, value, soc->pads_pll_ctl);

	/* configure the reference clock driver */
	value = PADS_REFCLK_CFG_VALUE | (PADS_REFCLK_CFG_VALUE << 16);
	pads_writel(pcie, value, PADS_REFCLK_CFG0);

	if (soc->num_ports > 2)
		pads_writel(pcie, PADS_REFCLK_CFG_VALUE, PADS_REFCLK_CFG1);

	do {
		value = pads_readl(pcie, soc->pads_pll_ctl);
		if (value & PADS_PLL_CTL_LOCKDET)
			break;

		udelay(2000);
	} while (--timeout);

	if (timeout == 0) {
		error("tegra-pcie: timeout waiting for PADS PLL to lock\n");
		return -ETIMEDOUT;
	}

	/* turn off IDDQ override */
	value = pads_readl(pcie, PADS_CTL);
	value &= ~PADS_CTL_IDDQ_1L;
	pads_writel(pcie, value, PADS_CTL);

	/* enable TX/RX data */
	value = pads_readl(pcie, PADS_CTL);
	value |= PADS_CTL_TX_DATA_EN_1L | PADS_CTL_RX_DATA_EN_1L;
	pads_writel(pcie, value, PADS_CTL);

	/* take the PCIEXCLK logic out of reset */
	writel(1 << 10, NV_PA_CLK_RST_BASE + 0x314);

	/* finally enable PCIe */
	value = afi_readl(pcie, AFI_CONFIGURATION);
	value |= AFI_CONFIGURATION_EN_FPCI;
	afi_writel(pcie, value, AFI_CONFIGURATION);

	value = AFI_INTR_EN_INI_SLVERR | AFI_INTR_EN_INI_DECERR |
		AFI_INTR_EN_TGT_SLVERR | AFI_INTR_EN_TGT_DECERR |
		AFI_INTR_EN_TGT_WRERR | AFI_INTR_EN_DFPCI_DECERR;
	afi_writel(pcie, value, AFI_AFI_INTR_ENABLE);

	value = 0xffffffff;
	afi_writel(pcie, value, AFI_SM_INTR_ENABLE);

	/* don't enable MSI */
	afi_writel(pcie, AFI_INTR_MASK_INT_MASK, AFI_INTR_MASK);

	/* disable all exceptions */
	afi_writel(pcie, 0, AFI_FPCI_ERROR_MASKS);

	return 0;
}

static void tegra_pcie_setup_translations(struct tegra_pcie *pcie)
{
	unsigned long fpci, axi, size;

	/* BAR 0: type 1 extended configuration space */
	fpci = 0xfe100000;
	size = fdt_resource_size(&pcie->cs);
	axi = pcie->cs.start;

	afi_writel(pcie, axi, AFI_AXI_BAR0_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR0_SZ);
	afi_writel(pcie, fpci, AFI_FPCI_BAR0);

	/* BAR 1: downstream I/O */
	fpci = 0xfdfc0000;
	size = fdt_resource_size(&pcie->io);
	axi = pcie->io.start;

	afi_writel(pcie, axi, AFI_AXI_BAR1_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR1_SZ);
	afi_writel(pcie, fpci, AFI_FPCI_BAR1);

	/* BAR 2: prefetchable memory */
	fpci = (((pcie->prefetch.start >> 12) & 0x0fffffff) << 4) | 0x1;
	size = fdt_resource_size(&pcie->prefetch);
	axi = pcie->prefetch.start;

	afi_writel(pcie, axi, AFI_AXI_BAR2_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR2_SZ);
	afi_writel(pcie, fpci, AFI_FPCI_BAR2);

	/* BAR 3: non-prefetchable memory */
	fpci = (((pcie->mem.start >> 12) & 0x0fffffff) << 4) | 0x1;
	size = fdt_resource_size(&pcie->mem);
	axi = pcie->mem.start;

	afi_writel(pcie, axi, AFI_AXI_BAR3_START);
	afi_writel(pcie, size >> 12, AFI_AXI_BAR3_SZ);
	afi_writel(pcie, fpci, AFI_FPCI_BAR3);

	/* NULL out the remaining BARs as they are not used */
	afi_writel(pcie, 0, AFI_AXI_BAR4_START);
	afi_writel(pcie, 0, AFI_AXI_BAR4_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR4);

	afi_writel(pcie, 0, AFI_AXI_BAR5_START);
	afi_writel(pcie, 0, AFI_AXI_BAR5_SZ);
	afi_writel(pcie, 0, AFI_FPCI_BAR5);

	/* map all upstream transactions as uncached */
	afi_writel(pcie, NV_PA_SDRAM_BASE, AFI_CACHE_BAR0_ST);
	afi_writel(pcie, 0, AFI_CACHE_BAR0_SZ);
	afi_writel(pcie, 0, AFI_CACHE_BAR1_ST);
	afi_writel(pcie, 0, AFI_CACHE_BAR1_SZ);

	/* MSI translations are setup only when needed */
	afi_writel(pcie, 0, AFI_MSI_FPCI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
	afi_writel(pcie, 0, AFI_MSI_AXI_BAR_ST);
	afi_writel(pcie, 0, AFI_MSI_BAR_SZ);
}

static unsigned long tegra_pcie_port_get_pex_ctrl(struct tegra_pcie_port *port)
{
	unsigned long ret = 0;

	switch (port->index) {
	case 0:
		ret = AFI_PEX0_CTRL;
		break;

	case 1:
		ret = AFI_PEX1_CTRL;
		break;

	case 2:
		ret = AFI_PEX2_CTRL;
		break;
	}

	return ret;
}

static void tegra_pcie_port_reset(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	/* pulse reset signel */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);

	udelay(2000);

	value = afi_readl(port->pcie, ctrl);
	value |= AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);
}

static void tegra_pcie_port_enable(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	/* enable reference clock */
	value = afi_readl(port->pcie, ctrl);
	value |= AFI_PEX_CTRL_REFCLK_EN;

	if (port->pcie->soc->has_pex_clkreq_en)
		value |= AFI_PEX_CTRL_CLKREQ_EN;

	afi_writel(port->pcie, value, ctrl);

	tegra_pcie_port_reset(port);
}

static bool tegra_pcie_port_check_link(struct tegra_pcie_port *port)
{
	unsigned int retries = 3;
	unsigned long value;

	do {
		unsigned int timeout = 200;

		do {
			value = rp_readl(port, RP_VEND_XP);
			if (value & RP_VEND_XP_DL_UP)
				break;

			udelay(2000);
		} while (--timeout);

		if (!timeout) {
			debug("tegra-pcie: link %u down, retrying\n",
			      port->index);
			goto retry;
		}

		timeout = 200;

		do {
			value = rp_readl(port, RP_LINK_CONTROL_STATUS);
			if (value & RP_LINK_CONTROL_STATUS_DL_LINK_ACTIVE)
				return true;

			udelay(2000);
		} while (--timeout);

retry:
		tegra_pcie_port_reset(port);
	} while (--retries);

	return false;
}

static void tegra_pcie_port_disable(struct tegra_pcie_port *port)
{
	unsigned long ctrl = tegra_pcie_port_get_pex_ctrl(port);
	unsigned long value;

	/* assert port reset */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_RST;
	afi_writel(port->pcie, value, ctrl);

	/* disable reference clock */
	value = afi_readl(port->pcie, ctrl);
	value &= ~AFI_PEX_CTRL_REFCLK_EN;
	afi_writel(port->pcie, value, ctrl);
}

static void tegra_pcie_port_free(struct tegra_pcie_port *port)
{
	list_del(&port->list);
	free(port);
}

static int tegra_pcie_enable(struct tegra_pcie *pcie)
{
	struct tegra_pcie_port *port, *tmp;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		debug("tegra-pcie: probing port %u, using %u lanes\n",
		      port->index, port->num_lanes);

		tegra_pcie_port_enable(port);

		if (tegra_pcie_port_check_link(port))
			continue;

		printf("tegra-pcie: link %u down, ignoring\n", port->index);

		tegra_pcie_port_disable(port);
		tegra_pcie_port_free(port);
	}

	return 0;
}

static const struct tegra_pcie_soc tegra20_pcie_soc = {
	.num_ports = 2,
	.pads_pll_ctl = PADS_PLL_CTL_TEGRA20,
	.tx_ref_sel = PADS_PLL_CTL_TXCLKREF_DIV10,
	.has_pex_clkreq_en = false,
	.has_pex_bias_ctrl = false,
	.has_cml_clk = false,
};

static const struct tegra_pcie_soc tegra30_pcie_soc = {
	.num_ports = 3,
	.pads_pll_ctl = PADS_PLL_CTL_TEGRA30,
	.tx_ref_sel = PADS_PLL_CTL_TXCLKREF_BUF_EN,
	.has_pex_clkreq_en = true,
	.has_pex_bias_ctrl = true,
	.has_cml_clk = true,
};

static int process_nodes(const void *fdt, int nodes[], unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		const struct tegra_pcie_soc *soc;
		struct tegra_pcie *pcie;
		enum fdt_compat_id id;
		int err;

		if (!fdtdec_get_is_enabled(fdt, nodes[i]))
			continue;

		id = fdtdec_lookup(fdt, nodes[i]);
		switch (id) {
		case COMPAT_NVIDIA_TEGRA20_PCIE:
			soc = &tegra20_pcie_soc;
			break;

		case COMPAT_NVIDIA_TEGRA30_PCIE:
			soc = &tegra30_pcie_soc;
			break;

		default:
			error("tegra-pcie: unsupported compatible: %s\n",
			      fdtdec_get_compatible(id));
			continue;
		}

		pcie = malloc(sizeof(*pcie));
		if (!pcie) {
			error("tegra-pcie: failed to allocate controller\n");
			continue;
		}

		memset(pcie, 0, sizeof(*pcie));
		pcie->soc = soc;

		INIT_LIST_HEAD(&pcie->ports);

		err = tegra_pcie_parse_dt(fdt, nodes[i], pcie);
		if (err < 0) {
			free(pcie);
			continue;
		}

		err = tegra_pcie_power_on(pcie);
		if (err < 0) {
			error("tegra-pcie: failed to power on\n");
			continue;
		}

		err = tegra_pcie_enable_controller(pcie);
		if (err < 0) {
			error("tegra-pcie: failed to enable controller\n");
			continue;
		}

		tegra_pcie_setup_translations(pcie);

		err = tegra_pcie_enable(pcie);
		if (err < 0) {
			error("tegra-pcie: failed to enable PCIe\n");
			continue;
		}

		pcie->hose.first_busno = 0;
		pcie->hose.current_busno = 0;
		pcie->hose.last_busno = 0;

		pci_set_region(&pcie->hose.regions[0], NV_PA_SDRAM_BASE,
			       NV_PA_SDRAM_BASE, gd->ram_size,
			       PCI_REGION_MEM | PCI_REGION_SYS_MEMORY);

		pci_set_region(&pcie->hose.regions[1], pcie->io.start,
			       pcie->io.start, fdt_resource_size(&pcie->io),
			       PCI_REGION_IO);

		pci_set_region(&pcie->hose.regions[2], pcie->mem.start,
			       pcie->mem.start, fdt_resource_size(&pcie->mem),
			       PCI_REGION_MEM);

		pci_set_region(&pcie->hose.regions[3], pcie->prefetch.start,
			       pcie->prefetch.start,
			       fdt_resource_size(&pcie->prefetch),
			       PCI_REGION_MEM | PCI_REGION_PREFETCH);

		pcie->hose.region_count = 4;

		pci_set_ops(&pcie->hose,
			    pci_hose_read_config_byte_via_dword,
			    pci_hose_read_config_word_via_dword,
			    tegra_pcie_read_conf,
			    pci_hose_write_config_byte_via_dword,
			    pci_hose_write_config_word_via_dword,
			    tegra_pcie_write_conf);

		pci_register_hose(&pcie->hose);

#ifdef CONFIG_PCI_SCAN_SHOW
		printf("PCI: Enumerating devices...\n");
		printf("---------------------------------------\n");
		printf("  Device        ID          Description\n");
		printf("  ------        --          -----------\n");
#endif

		pcie->hose.last_busno = pci_hose_scan(&pcie->hose);
	}

	return 0;
}

void pci_init_board(void)
{
	const void *fdt = gd->fdt_blob;
	int count, nodes[1];

	count = fdtdec_find_aliases_for_id(fdt, "pcie-controller",
					   COMPAT_NVIDIA_TEGRA30_PCIE,
					   nodes, ARRAY_SIZE(nodes));
	if (process_nodes(fdt, nodes, count))
		return;

	count = fdtdec_find_aliases_for_id(fdt, "pcie-controller",
					   COMPAT_NVIDIA_TEGRA20_PCIE,
					   nodes, ARRAY_SIZE(nodes));
	if (process_nodes(fdt, nodes, count))
		return;
}

int pci_skip_dev(struct pci_controller *hose, pci_dev_t dev)
{
	/*
	if (PCI_BUS(dev) == 0 && PCI_DEV(dev) == 0)
		return 1;
	*/

	if (PCI_BUS(dev) != 0 && PCI_DEV(dev) > 0)
		return 1;

	return 0;
}
