#ifndef CERN_SPEC_H
#define CERN_SPEC_H

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/wishbone.h>

#define SPEC_FLAG_IRQREQUEST	(1 << 0)

#define SPEC_DEFAULT_DMABUFSIZE	(1 << 20)	/* 1MB */
#define SPEC_MAX_DMABUFSIZE	(1024 * 4096)

struct spec_dev {
	struct pci_dev *pdev;
	struct device *dev;
	struct cdev cdev;

	struct resource *area[3];
	void *remap[3];

	unsigned int flags;

	struct work_struct work;
	const struct firmware *fw;

	void *dmabuf;

	struct mutex mutex;
	unsigned int usecount;

	struct timespec irqtime;
	unsigned long irqcount;
	wait_queue_head_t irq_queue;

	struct wb_bus wb_bus;
	int bus_registered;
};

#define bus_to_spec_dev(bus) \
	container_of(bus, struct spec_dev, wb_bus)

#endif

struct spec_fw {
	int fwlen;
	void *data;
};

/* Offsets for BAR areas in llseek() and/or ioctl */
#define SPEC_BAR_0		0x00000000
#define SPEC_BAR_2		0x20000000
#define SPEC_BAR_4		0x40000000
#define SPEC_BAR_BUF		0xc0000000

#ifdef __KERNEL__

#define SPEC_IS_DMABUF(addr)	((addr) >= SPEC_BAR_BUF)
#define SPEC_GET_BAR(x)		((x) >> 28)
#define SPEC_SET_BAR(x)		((x) << 28)
#define SPEC_GET_OFF(x)		((x) & 0x0fffffff)

static inline int spec_is_valid_bar(unsigned long address)
{
	int bar = SPEC_GET_BAR(address);
	return bar == 0 || bar == 2 || bar == 4 || bar == 0x0c;
}

static inline int spec_is_dmabuf_bar(unsigned long address)
{
	int bar = SPEC_GET_BAR(address);
	return bar == 0x0c;
}

#endif

#define __SPEC_IOC_MAGIC 'S'

#define SPEC_LOAD_FIRMWARE	_IOW(__SPEC_IOC_MAGIC, 0, struct spec_fw)
#define SPEC_LOAD_DEMO_CONFIG	_IOW(__SPEC_IOC_MAGIC, 1, struct spec_fw)

#endif /* CERN_SPEC_H */
