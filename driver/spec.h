#ifndef CERN_SPEC_H
#define CERN_SPEC_H

#include <linux/types.h>
#include <linux/ioctl.h>

#ifdef __KERNEL__

#define SPEC_FLAG_IRQREQUEST	(1 << 0)

struct spec_dev {
	struct pci_dev *pdev;
	struct device *dev;
	struct cdev cdev;

	struct resource *area[3];
	void *remap[3];

	unsigned int flags;

	struct work_struct work;
	const struct firmware *fw;

	struct mutex mutex;
	unsigned int usecount;

	struct timespec irqtime;
	unsigned long irqcount;
	wait_queue_head_t irq_queue;
};

#endif

struct spec_fw {
	int fwlen;
	void *data;
};

#define __SPEC_IOC_MAGIC 'S'

#define SPEC_LOADFW	_IOW(__SPEC_IOC_MAGIC, 0, struct spec_fw)

#endif /* CERN_SPEC_H */
