#include "gennum.h"

#include <linux/types.h>
#include <linux/module.h>
#include <linux/io.h>

static inline uint8_t reverse_bits8(uint8_t x)
{
	x = ((x >> 1) & 0x55) | ((x & 0x55) << 1);
	x = ((x >> 2) & 0x33) | ((x & 0x33) << 2);
	x = ((x >> 4) & 0x0f) | ((x & 0x0f) << 4);

	return x;
}

static uint32_t unaligned_bitswap_le32(const uint32_t *ptr32)
{
	static uint32_t tmp32;
	static uint8_t *tmp8 = (uint8_t *) &tmp32;
	static uint8_t *ptr8;

	ptr8 = (uint8_t *) ptr32;

	*(tmp8 + 0) = reverse_bits8(*(ptr8 + 0));
	*(tmp8 + 1) = reverse_bits8(*(ptr8 + 1));
	*(tmp8 + 2) = reverse_bits8(*(ptr8 + 2));
	*(tmp8 + 3) = reverse_bits8(*(ptr8 + 3));

	return tmp32;
}

int gennum_loader(void __iomem *bar4, const void *data, int size8)
{
	int ctrl = 0;
	int i;
	int done = 0;
	int wrote = 0;
	int size32 = (size8 + 3) >> 2;
	const uint32_t *data32 = data;

	/*
	 * The synchronous reset (by setting bit 6 of FCL_CTRL must only be
	 * applied when the FCL_CLK_DIV register is set to 0 or some registers
	 * that run off the divided clock will not be reset. This prevents
	 * locking out the software during a reset.
	 */
	writel(0x00, bar4 + GN412X_FCL_CLK_DIV);
	writel(0x40, bar4 + GN412X_FCL_CTRL); /* Synchronous reset */

	/* confirm the reset */
	i = readl(bar4 + GN412X_FCL_CTRL);
	if (i != 0x40) {
		printk(KERN_ERR "%s: %i: error\n", __func__, __LINE__);
		return -EIO;
	}
	writel(0x00, bar4 + GN412X_FCL_CTRL); /* clear the reset bit */
	writel(0x00, bar4 + GN412X_FCL_IRQ); /* clear pending irq */

	/*
	 * If our data is not 4-byte-aligned, we need to write the
	 * last-byte-count into the FCL_CTRL register
	 */
	switch(size8 & 3) {
		case 3: ctrl = 0x116; break;
		case 2: ctrl = 0x126; break;
		case 1: ctrl = 0x136; break;
		case 0: ctrl = 0x106; break;
	}
	writel(ctrl, bar4 + GN412X_FCL_CTRL);

	writel(0x00, bar4 + GN412X_FCL_CLK_DIV); /* again? maybe 1 or 2? */
	writel(0x00, bar4 + GN412X_FCL_TIMER_CTRL); /* disable FCL timer */

	/*
	 * Set the timer value (TIMER_0 is least significant bits). IRQ is
	 * set when the timer reaches this value
	 */
	writel(0x10, bar4 + GN412X_FCL_TIMER_0);
	writel(0x00, bar4 + GN412X_FCL_TIMER_1);

	/*
	 * Set delay before data and clock is applied by FCL
	 * after SPRI_STATUS is detected being assert.
	 */
	writel(0x08, bar4 + GN412X_FCL_TIMER2_0); /* delay before data/clk */
	writel(0x00, bar4 + GN412X_FCL_TIMER2_1);
	writel(0x17, bar4 + GN412X_FCL_EN); /* output enable */

	ctrl |= 0x01; /* start FSM configuration */
	writel(ctrl, bar4 + GN412X_FCL_CTRL);

	while(size32 > 0)
	{
		/* Check to see FPGA configuation status */
		i = readl(bar4 + GN412X_FCL_IRQ);
		if ((i & 8) && wrote) {
			done = 1;
			printk("%s: %i: done after %i\n", __func__, __LINE__,
				wrote);
		} else if ((i & 0x4) && !done) {
			printk("%s: %i: error after %i\n", __func__, __LINE__,
				wrote);
			return -EIO;
		}

		/* Wait until at least 1/2 of the fifo is empty */
		while (readl(bar4 + GN412X_FCL_IRQ)  & (1<<5));

		/* Write a few dwords into FIFO at a time. */
		for (i = 0; size32 && i < 32; i++) {
			writel(unaligned_bitswap_le32(data32),
				bar4 + GN412X_FCL_FIFO);
			data32++;
			size32--;
			wrote++;
		}
	}

	writel(0x186, bar4 + GN412X_FCL_CTRL); /* last data written */

	/* Checking for the interrupt is left to the caller */
	return wrote;
}
