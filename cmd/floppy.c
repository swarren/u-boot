/*
 * TODO:
 * - Test on-chip pull-ups for RX lines. If these work, we don't need external components.
*/

#include <common.h>
#include <command.h>
#include <dm.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/gpio.h>

#define NAME_MOTOR	"gpio13"
#define NAME_SELECT	"gpio6"
#define NAME_DIRECTION	"gpio19"
#define NAME_STEP	"gpio26"
#define NAME_TRK00	"gpio20"
#define NAME_INDEX	"gpio21"
#define NAME_RDATA	"gpio9"

#define INDEX_MOTOR	0
#define INDEX_SELECT	1
#define INDEX_DIRECTION	2
#define INDEX_STEP	3
#define INDEX_TRK00	4
#define INDEX_INDEX	5
#define INDEX_RDATA	6

#define HWREG(offset) ((uint32_t *)((uint64_t)(0x3f000000 | (offset))))
//#define HWREG(offset) ((uint32_t *)((uint64_t)(0x7e000000 | (offset))))

static struct {
	const char *use_name;
	const char *gpio_name;
	int output;
	int init_out_data;
	unsigned int gpio;
	int requested;
} pins[] = {
	[INDEX_MOTOR] = {"floppy-motor", NAME_MOTOR, 1, 1, },
	[INDEX_SELECT] = {"floppy-select", NAME_SELECT, 1, 1, },
	[INDEX_DIRECTION] = {"floppy-direction", NAME_DIRECTION, 1, 1, },
	[INDEX_STEP] = {"floppy-step", NAME_STEP, 1, 1, },
	[INDEX_TRK00] = {"floppy-trk00", NAME_TRK00, 0, },
	[INDEX_INDEX] = {"floppy-index", NAME_INDEX, 0, },
	[INDEX_RDATA] = {"floppy-rdata", NAME_RDATA, 0, },
};
#define PIN(x) pins[INDEX_##x].gpio

static int get_pins(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(pins); i++)
		pins[i].requested = 0;

	for (i = 0; i < ARRAY_SIZE(pins); i++) {
		ret = gpio_lookup_name(pins[i].gpio_name, NULL, NULL, &pins[i].gpio);
		if (ret) {
			printf("gpio_lookup_name(%s): %d\n", pins[i].gpio_name, ret);
			return -1;
		}
		ret = gpio_request(pins[i].gpio, pins[i].use_name);
		if (ret) {
			printf("gpio_request(%d): %d\n", pins[i].gpio, ret);
			return -1;
		}
		pins[i].requested = 1;
		if (pins[i].output)
			gpio_direction_output(pins[i].gpio, pins[i].init_out_data);
		else
			gpio_direction_input(pins[i].gpio);
	}

	return 0;
}

static void put_pins(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pins); i++) {
		if (!pins[i].requested)
			continue;
		gpio_direction_input(pins[i].gpio);
		gpio_free(pins[i].gpio);
		pins[i].requested = 0;
	}
}

static void get_pinmux_spi(void)
{
	u32 val;

	// GPFSEL0
	val = readl(HWREG(0x200000));
	// GPIO9/SPI_MISO
	val &= ~(7 << 27);
	val |= 4 << 27;
	writel(val, HWREG(0x200000));

	// GPFSEL4
	val = readl(HWREG(0x200004));
	// GPIO10/SPI_MOSI
	val &= ~(7 << 0);
	val |= 4 << 0;
	// GPIO11/SPI_CLK
	val &= ~(7 << 3);
	val |= 4 << 3;
	writel(val, HWREG(0x200004));
}

static int do_floppy_trk00(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret, i;

	ret = get_pins();
	if (ret)
		goto out_put_pins;

	gpio_set_value(PIN(SELECT), 0);
	mdelay(1);
	gpio_set_value(PIN(DIRECTION), 1);
	mdelay(1);
	for (i = 0; i < 100; i++) {
		if (!gpio_get_value(PIN(TRK00)))
			break;
		printf("Step down %d\n", i);
		gpio_set_value(PIN(STEP), 0);
		mdelay(1);
		gpio_set_value(PIN(STEP), 1);
		mdelay(20);
	}
	printf("At track 0\n");
	gpio_set_value(PIN(DIRECTION), 0);
	mdelay(1);
	for (i = 0; i < 5; i++) {
		printf("Step up %d\n", i);
		gpio_set_value(PIN(STEP), 0);
		mdelay(1);
		gpio_set_value(PIN(STEP), 1);
		mdelay(20);
	}
	gpio_set_value(PIN(SELECT), 1);
	mdelay(1);

	ret = 0;

out_put_pins:
	put_pins();

	return ret;
}

U_BOOT_CMD(floppy_trk00, 1, 0, do_floppy_trk00,
	   "fiddle with a floppy drive",
	   "No parameters!");

static int do_floppy_index(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret, last_v, v, ctr;
	ulong ts, t;

	ret = get_pins();
	if (ret)
		goto out_put_pins;

	gpio_set_value(PIN(MOTOR), 0);
	mdelay(1000);
	gpio_set_value(PIN(SELECT), 0);
	mdelay(1);

	ts = get_timer(0);
	ctr = 0;
	last_v = -1;
	while (ctr < 10) {
		v = gpio_get_value(PIN(INDEX));
		if (v != last_v) {
			if (!v)
				ctr++;
			last_v = v;
		}

		t = get_timer(0);
		if ((t - ts) > 10000) {
			printf("Timeout\n");
			break;
		}
	}
	printf("Saw %d index\n", ctr);
	t = get_timer(0);
	printf("... in %lu ms\n", t - ts);

	gpio_set_value(PIN(SELECT), 1);
	mdelay(1);
	gpio_set_value(PIN(MOTOR), 1);
	mdelay(1);

	ret = 0;

out_put_pins:
	put_pins();

	return ret;
}

U_BOOT_CMD(floppy_index, 1, 0, do_floppy_index,
	   "fiddle with a floppy drive",
	   "No parameters!");

static int do_floppy_rdata(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret, index, i;
	uint32_t last_v, v;
	ulong ts, t;
	uint16_t data[1024];

	ret = get_pins();
	if (ret)
		goto out_put_pins;

	gpio_set_value(PIN(MOTOR), 0);
	mdelay(1000);
	gpio_set_value(PIN(SELECT), 0);
	mdelay(1);

	ts = get_timer(0);
	index = 0;
	data[index] = 0;
	v = 1;
	for (;;) {
		last_v = v;
		v = (readl(HWREG(0x200034)) >> 9) & 1;
		if ((v == last_v) && (data[index] < 0x7fff)) {
			data[index]++;
			continue;
		}
		data[index] |= last_v << 15;
		index++;
		if (index == ARRAY_SIZE(data))
			break;
		data[index] = 1;
	}
	t = get_timer(0);

	gpio_set_value(PIN(SELECT), 1);
	mdelay(1);
	gpio_set_value(PIN(MOTOR), 1);
	mdelay(1);

	printf("Data capture took %lu ms; at index %d\n", t - ts, index);
	for (i = 0; i < 20 /*ARRAY_SIZE(data)*/; i++)
		printf("data[%d]=0x%04x\n", i, data[i]);

	ret = 0;

out_put_pins:
	put_pins();

	return ret;
}

U_BOOT_CMD(floppy_rdata, 1, 0, do_floppy_rdata,
	   "fiddle with a floppy drive",
	   "No parameters!");

#define SPI0_CS		HWREG(0x204000)
#define SPI0_CS_RXF	BIT(20)
#define SPI0_CS_RXR	BIT(19)
#define SPI0_CS_TXD	BIT(18)
#define SPI0_CS_RXD	BIT(17)
#define SPI0_CS_DONE	BIT(16)
#define SPI0_CS_REN	BIT(12)
#define SPI0_CS_ADCS	BIT(11)
#define SPI0_CS_DMAEN	BIT(8)
#define SPI0_CS_TA	BIT(7)
#define SPI0_CS_CLR_RX	BIT(5)
#define SPI0_CS_CLR_TX	BIT(4)

#define SPI0_FIFO	HWREG(0x204004)

#define SPI0_CLK	HWREG(0x204008)

#define SPI0_DLEN	HWREG(0x20400c)

#define SPI0_LTOH	HWREG(0x204010)

#define SPI0_DC		HWREG(0x204014)

static int do_floppy_spi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uint32_t val;
	int i, tx, rx;
	ulong ts;

	get_pinmux_spi();

	// Max divider for easy capture
	writel(0, SPI0_CLK);

	// Hack to avoid inter-byte clock delay!
	// https://www.raspberrypi.org/forums/viewtopic.php?t=181154
	// https://raw.githubusercontent.com/juj/fbcp-ili9341/e109c434cbd7512311032cf33d8064d2a192945b/spi.cpp
	writel(2, SPI0_DLEN);

	val = SPI0_CS_TA | SPI0_CS_CLR_RX | SPI0_CS_CLR_TX;
	writel(val, SPI0_CS);

	ts = get_timer(0);
	tx = rx = 1024;
	for (i = 0; i < 16; i++) {
		writel(0xaaaaaaaaU, SPI0_FIFO);
		tx--;
	}
	while (tx || rx) {
		val = readl(SPI0_CS);
		if (tx && (val & SPI0_CS_TXD)) {
			writel(0xaaaaaaaaU, SPI0_FIFO);
			tx--;
		}
		if (rx && (val & SPI0_CS_RXD)) {
			readl(SPI0_FIFO);
			rx--;
		}
	}
	do {
		val = readl(SPI0_CS);
		if (val & SPI0_CS_DONE)
			break;
	} while (get_timer(0) - ts < 1000);
	writel(0, SPI0_CS);
	printf("SPI TX %lu ms, CS: 0x%08x\n", get_timer(0) - ts, val);

	return 0;
}

U_BOOT_CMD(floppy_spi, 1, 0, do_floppy_spi,
	   "fiddle with a floppy drive",
	   "No parameters!");

#define DMA_BASE		0x007000
#define DMA_CH_BASE(ch)		(DMA_BASE + ((ch) * 0x100))
#define DMA_REG(ch, r)		HWREG(DMA_CH_BASE(ch) + (r))

#define DMA_CS(ch)		DMA_REG(ch, 0x0)
#define DMA_CS_RESET		BIT(31)
#define DMA_CS_ABORT		BIT(30)
#define DMA_CS_WAIT_WRITES	BIT(28)
#define DMA_CS_PANIC_PRI_P	20
#define DMA_CS_PRI_P		16
#define DMA_CS_ERROR		BIT(8)
#define DMA_CS_WAITING_WRTIES	BIT(6)
#define DMA_CS_DREQ_STOPS_DMA	BIT(5)
#define DMA_CS_PAUSED		BIT(4)
#define DMA_CS_DREQ		BIT(3)
#define DMA_CS_INT		BIT(2)
#define DMA_CS_END		BIT(1)
#define DMA_CS_ACTIVE		BIT(0)

#define DMA_CONBLK_AD(ch)	DMA_REG(ch, 0x4)

#define DMA_TI(ch)		DMA_REG(ch, 0x8)
#define DMA_TI_NO_WIDE_BURSTS	BIT(26)
#define DMA_TI_WAITS_P		21
#define DMA_TI_PER_MAP_P	16
#define DMA_TI_BURST_LENGTH_P	12
#define DMA_TI_SRC_IGNORE	BIT(11)
#define DMA_TI_SRC_DREQ		BIT(10)
#define DMA_TI_SRC_WIDTH_128	BIT(9)
#define DMA_TI_SRC_INC		BIT(8)
#define DMA_TI_DEST_IGNORE	BIT(7)
#define DMA_TI_DEST_DREQ	BIT(6)
#define DMA_TI_DEST_WIDTH_128	BIT(5)
#define DMA_TI_DEST_INC		BIT(4)
#define DMA_TI_WAIT_RESP	BIT(3)
#define DMA_TI_2DMODE		BIT(1)
#define DMA_TI_INTEN		BIT(0)

#define DMA_SOURCE_AD(ch)	DMA_REG(ch, 0xc)

#define DMA_DEST_AD(ch)		DMA_REG(ch, 0x10)

#define DMA_TXFR_LEN(ch)	DMA_REG(ch, 0x14)

#define DMA_STRIDE(ch)		DMA_REG(ch, 0x18)

#define DMA_NEXTCONBK(ch)	DMA_REG(ch, 0x1c)

#define DMA_DEBUG(ch)		DMA_REG(ch, 0x20)

#define N_TX_BYTES	128U
#define ITERS		1 // SPI doesn't restart on next "header" in FIFO:-(
static int do_floppy_spi_dma(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	uint32_t *spi_data_tx, *spi_data_rx;
	uint32_t *dma_desc_tx, *dma_desc_rx;
	int ret, i;
	uint32_t val;
	ulong ts;

	get_pinmux_spi();

	spi_data_tx = memalign(ARCH_DMA_MINALIGN, (N_TX_BYTES + 4) * ITERS);
	if (!spi_data_tx) {
		printf("memalign(spi_data_tx) failed\n");
		return 1;
	}
	printf("spi_data_tx %p\n", spi_data_tx);
	spi_data_tx[0] = (N_TX_BYTES << 16) | SPI0_CS_TA;
	memset(&spi_data_tx[1], 0xaa, N_TX_BYTES);
#if ITERS > 1
	spi_data_tx[1 + (N_TX_BYTES / 4)] = (N_TX_BYTES << 16) | SPI0_CS_TA;
	memset(&spi_data_tx[1 + (N_TX_BYTES / 4) + 1], 0xaa, N_TX_BYTES);
#endif
	flush_cache((unsigned long)spi_data_tx, (N_TX_BYTES + 4)) * ITERS;

	dma_desc_tx = memalign(ARCH_DMA_MINALIGN, N_TX_BYTES + 4);
	if (!dma_desc_tx) {
		printf("memalign(dma_desc_tx) failed\n");
		ret = 1;
		goto out_free_spi_data_tx;
	}
	printf("dma_desc_tx %p\n", dma_desc_tx);
	dma_desc_tx[0] =
		(6 << DMA_TI_PER_MAP_P) |
		DMA_TI_SRC_INC |
		DMA_TI_DEST_DREQ |
		DMA_TI_WAIT_RESP;
	dma_desc_tx[1] = ((uint64_t)spi_data_tx) & 0x7fffffff;
	dma_desc_tx[2] = 0x7e204004; //SPI0_FIFO;
	dma_desc_tx[3] = (N_TX_BYTES + 4) * ITERS;
	dma_desc_tx[4] = 0; /* 2D STRIDE */
	dma_desc_tx[5] = 0; /* NEXTCONBK */
	dma_desc_tx[6] = 0; /* Reserved */
	dma_desc_tx[7] = 0; /* Reserved */
	flush_cache((unsigned long)dma_desc_tx, ARCH_DMA_MINALIGN);

	spi_data_rx = memalign(ARCH_DMA_MINALIGN, N_TX_BYTES * ITERS);
	if (!spi_data_rx) {
		printf("memalign(spi_data_rx) failed\n");
		ret = 1;
		goto out_free_dma_desc_tx;
	}
	printf("spi_data_rx %p\n", spi_data_rx);
	flush_cache((unsigned long)spi_data_rx, N_TX_BYTES);

	dma_desc_rx = memalign(ARCH_DMA_MINALIGN, N_TX_BYTES);
	if (!dma_desc_rx) {
		printf("memalign(dma_desc_rx) failed\n");
		ret = 1;
		goto out_free_spi_data_rx;
	}
	printf("dma_desc_rx %p\n", dma_desc_rx);
	dma_desc_rx[0] =
		(7 << DMA_TI_PER_MAP_P) |
		DMA_TI_DEST_INC |
		DMA_TI_SRC_DREQ;
	dma_desc_rx[1] = 0x7e204004; //SPI0_FIFO;
	dma_desc_rx[2] = ((uint64_t)spi_data_rx) & 0x7fffffff;
	dma_desc_rx[3] = N_TX_BYTES * ITERS;
	dma_desc_rx[4] = 0; /* 2D STRIDE */
	dma_desc_rx[5] = 0; /* NEXTCONBK */
	dma_desc_rx[6] = 0; /* Reserved */
	dma_desc_rx[7] = 0; /* Reserved */
	flush_cache((unsigned long)dma_desc_rx, ARCH_DMA_MINALIGN);

	/* SPI setup */
	writel(8, SPI0_CLK);
	val = SPI0_CS_DMAEN | SPI0_CS_CLR_RX | SPI0_CS_CLR_TX;
	writel(val, SPI0_CS);
	readl(SPI0_CS);

	/* DMA RX setup (ch 1) */
	writel(DMA_CS_RESET, DMA_CS(1));
	readl(DMA_CS(1));
	mdelay(1); // FIXME: Busy wait instead
	writel(((uint64_t)dma_desc_rx) & 0x7fffffff, DMA_CONBLK_AD(1));
	val =
		DMA_CS_WAIT_WRITES |
		(0xf << DMA_CS_PANIC_PRI_P) |
		(0xf << DMA_CS_PRI_P) |
		DMA_CS_INT |
		DMA_CS_END |
		DMA_CS_ACTIVE;
	writel(val, DMA_CS(1));

	/* DMA TX setup (ch 0) */
	writel(DMA_CS_RESET, DMA_CS(0));
	readl(DMA_CS(0));
	mdelay(1); // FIXME: Busy wait instead
	writel(((uint64_t)dma_desc_tx) & 0x7fffffff, DMA_CONBLK_AD(0));
	val =
		DMA_CS_WAIT_WRITES |
		(0xf << DMA_CS_PANIC_PRI_P) |
		(0xf << DMA_CS_PRI_P) |
		DMA_CS_INT |
		DMA_CS_END |
		DMA_CS_ACTIVE;
	writel(val, DMA_CS(0));

	ts = get_timer(0);
	do {
		val = readl(DMA_CS(1));
		if (val & DMA_CS_END)
			break;
	} while (get_timer(0) - ts < 5000);
	writel(0, SPI0_CS);
	printf("SPI TX %lu ms\n", get_timer(0) - ts);
	for (i = 0; i < 2; i++) {
		printf("DMA_CS(%d):        0x%08x\n", i, readl(DMA_CS(i)));
		printf("DMA_CONBLK_AD(%d): 0x%08x\n", i, readl(DMA_CONBLK_AD(i)));
		printf("DMA_TI(%d):        0x%08x\n", i, readl(DMA_TI(i)));
		printf("DMA_SOURCE_AD(%d): 0x%08x\n", i, readl(DMA_SOURCE_AD(i)));
		printf("DMA_DEST_AD(%d):   0x%08x\n", i, readl(DMA_DEST_AD(i)));
		printf("DMA_TXFR_LEN(%d):  0x%08x\n", i, readl(DMA_TXFR_LEN(i)));
		printf("DMA_STRIDE(%d):    0x%08x\n", i, readl(DMA_STRIDE(i)));
		printf("DMA_NEXTCONBK(%d): 0x%08x\n", i, readl(DMA_NEXTCONBK(i)));
		printf("DMA_DEBUG(%d):     0x%08x\n", i, readl(DMA_DEBUG(i)));
	}
	printf("SPI_CS:           0x%08x\n", readl(SPI0_CS));
	printf("SPI_DLEN:         0x%08x\n", readl(SPI0_DLEN));

	ret = 0;

//out_free_dma_desc_rx:
	free(dma_desc_rx);
out_free_spi_data_rx:
	free(spi_data_rx);
out_free_dma_desc_tx:
	free(dma_desc_tx);
out_free_spi_data_tx:
	free(spi_data_tx);

	return ret;
}

U_BOOT_CMD(floppy_spi_dma, 1, 0, do_floppy_spi_dma,
	   "fiddle with a floppy drive",
	   "No parameters!");

