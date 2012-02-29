/*
 * This file is part of the Simple PCI Express Carrier (SPEC) Linux driver.
 * More information available at: http://www.ohwr.org/projects/spec
 *
 * Copyright (c) 2011 Alessandro Rubini <rubini@gnudd.com>
 * Copyright (c) 2011 Manohar Vanga <manohar.vanga@cern.ch>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/fs.h>

#include "spec.h"
#include "gennum.h"

#ifndef SPECMOD_VERSION
#define SPECMOD_VERSION			0x2011
#endif

#define SPEC_MAX_MINORS			32

#define SPEC_DEBUG			1
#define SPEC_MAX_FWNAME_SIZE		50

#define PCI_VENDOR_ID_CERN		0x1a39
#define PCI_DEVICE_ID_CERN_SPEC_V2	0x0004

#define GENNUM_REGS_BAR			4

const char *driver_name = "spec";

static struct class *spec_class;
static int spec_major;
static int spec_minors[SPEC_MAX_MINORS];
static DEFINE_MUTEX(spec_minors_mutex);

static char *fwname = NULL;
module_param(fwname, charp, 0000);
MODULE_PARM_DESC(fwname, "The name of the firmware to load automatically");

static int dmasize = SPEC_DEFAULT_DMABUFSIZE;
module_param_named(bufsize, dmasize, int, 0);

#ifdef SPEC_DEMO_CONFIG
struct firmware config_space;
#endif

uint8_t spec_wb_read8(struct wb_bus *bus, wb_addr_t addr)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	return readb(sdev->remap[0] + addr);
}

uint16_t spec_wb_read16(struct wb_bus *bus, wb_addr_t addr)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	return readw(sdev->remap[0] + addr);
}

uint32_t spec_wb_read32(struct wb_bus *bus, wb_addr_t addr)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	return readl(sdev->remap[0] + addr);
}

uint64_t spec_wb_read64(struct wb_bus *bus, wb_addr_t addr)
{
	uint32_t upper, lower;
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	lower = readl(sdev->remap[0] + addr);
	upper = readl(sdev->remap[0] + addr + 4);

	return (((uint64_t)upper << 32) | lower);
}

void spec_wb_write8(struct wb_bus *bus, wb_addr_t addr, uint8_t val)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	writeb(val, sdev->remap[0] + addr);
}

void spec_wb_write16(struct wb_bus *bus, wb_addr_t addr, uint16_t val)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	writew(val, sdev->remap[0] + addr);
}

void spec_wb_write32(struct wb_bus *bus, wb_addr_t addr, uint32_t val)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	writel(val, sdev->remap[0] + addr);
}

void spec_wb_write64(struct wb_bus *bus, wb_addr_t addr, uint64_t val)
{
	struct spec_dev *sdev = bus_to_spec_dev(bus);

	writel(val, sdev->remap[0] + addr);
	writel(val >> 32, sdev->remap[0] + addr);
}

void *spec_memcpy_from_wb(struct wb_bus *bus, wb_addr_t addr, void *buf, size_t len)
{
	int i;
	uint8_t *bytes = (uint8_t *)buf;
	for (i = 0; i < len; i++)
		*bytes++ = spec_wb_read8(bus, addr + i);
	return buf;
}

void *spec_memcpy_to_wb(struct wb_bus *bus, wb_addr_t addr, const void *buf, size_t len)
{
	int i;
	uint8_t *bytes = (uint8_t *)buf;
	for (i = 0; i < len; i++)
		spec_wb_write8(bus, *bytes++, addr + i);
	return (void *)buf;
}

void *spec_wb_read_cfg(struct wb_bus *bus, wb_addr_t addr, void *buf, size_t len)
{
#ifdef SPEC_DEMO_CONFIG
	memcpy(buf, &config_space.data[addr], len);
	return buf;
#endif
	return NULL;
}

struct wb_ops spec_wb_ops = {
	.read8 = spec_wb_read8,
	.read16 = spec_wb_read16,
	.read32 = spec_wb_read32,
	.read64 = spec_wb_read64,
	.write8 = spec_wb_write8,
	.write16 = spec_wb_write16,
	.write32 = spec_wb_write32,
	.write64 = spec_wb_write64,
	.memcpy_from_wb = spec_memcpy_from_wb,
	.memcpy_to_wb = spec_memcpy_to_wb,
	.read_cfg = spec_wb_read_cfg,
};

irqreturn_t spec_irq_handler(int irq, void *devid)
{
	struct spec_dev *dev = (struct spec_dev *)devid;

	getnstimeofday(&dev->irqtime);
	dev->irqcount++;
	disable_irq_nosync(irq);
	wake_up_interruptible(&dev->irq_queue);

	return IRQ_HANDLED;
}

int spec_gennum_load(struct spec_dev *dev, const void *data, int size)
{
	int i, done = 0, wrote = 0;
	int err;
	unsigned long j;
	void __iomem *bar4 = dev->remap[2];

	/* Ok, now call register access, which lived elsewhere */
	wrote = gennum_loader(bar4, data, size);
	if (wrote < 0)
		return wrote;

	j = jiffies + 2 * HZ;
	while(!done) {
		i = readl(bar4 + GN412X_FCL_IRQ);
		if (i & 0x8) {
			pr_info(KBUILD_MODNAME
				": %s: done after %i\n", __func__,
				wrote);
			done = 1;
		} else if( (i & 0x4) && !done) {
			pr_info(KBUILD_MODNAME
				": %s: error after %i\n", __func__,
				wrote);
			return -ETIMEDOUT;
		}
		if (time_after(jiffies, j)) {
			pr_info(KBUILD_MODNAME
				": %s: timeout after %i\n", __func__,
				wrote);
			return -ETIMEDOUT;
		}
	}

	/*
	 * Register the device as a wishbone controller
	 *
	 * Here, we would in practice, read the location of the SDWB header
	 * from a register in PCI memory. For now we just use a demo firmware
	 * with the header information to test things out.
	 */
	dev->wb_bus.name = (char *)driver_name;
	dev->wb_bus.owner = THIS_MODULE;
	dev->wb_bus.ops = &spec_wb_ops;

	/* The wb_cfggen tool sets the header base address to a default of 0 */
	dev->wb_bus.sdwb_header_base = 0;

	err = wb_register_bus(&dev->wb_bus);
	if (err) {
		pr_err(KBUILD_MODNAME ": failed to register wishbone bus\n");
		return err;
	}

	dev->bus_registered = 1;

	return 0;
}

void spec_complete_firmware(const struct firmware *fw, void *context)
{
	struct spec_dev *dev = (struct spec_dev *)context;
	int err;

	dev->fw = fw;

	if (fw) {
		pr_info(KBUILD_MODNAME ": %s: %p: size %i (0x%x), data %p\n",
			__func__, fw, fw ? fw->size : 0, fw ? fw->size : 0,
			fw ? fw->data : 0);
	} else {
		pr_warning(KBUILD_MODNAME ": %s: no firmware\n", __func__);
		return;
	}

	err = spec_gennum_load(dev, fw->data, fw->size);
	if (err)
		pr_err(KBUILD_MODNAME ": %s: loading returned error %i\n",
			__func__, err);

	release_firmware(dev->fw);
	dev->fw = NULL;
}

void spec_load_firmware(struct work_struct *work)
{
	struct spec_dev *dev = container_of(work, struct spec_dev, work);
	struct pci_dev *pdev = dev->pdev;
	static char fwname[SPEC_MAX_FWNAME_SIZE];
	int err;

	if (!(*fwname)) {
		dev_err(&pdev->dev, "no fwname specified\n");
		return;
	}
	if (SPEC_DEBUG)
		pr_info(KBUILD_MODNAME ": %s: %s\n", __func__, fwname);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	err = request_firmware_nowait(THIS_MODULE, 1, fwname, &pdev->dev,
		dev, spec_complete_firmware);
#else
	err = request_firmware_nowait(THIS_MODULE, 1, fwname, &pdev->dev,
		GFP_KERNEL, dev, spec_complete_firmware);
#endif
	pr_info(KBUILD_MODNAME ": %s: request_firmware returned %i\n",
		__func__, err);
}

void spec_request_firmware(struct spec_dev *dev)
{
	if (dev->fw) {
		pr_err(KBUILD_MODNAME
			": firmware loading already in progress\n");
		return;
	}
	schedule_work(&dev->work);
}

static unsigned int __devinit spec_get_free(void)
{
	unsigned int i;

	for (i = 0; i < SPEC_MAX_MINORS; i++)
		if (spec_minors[i] == 0)
			break;

	return i;
}

static int spec_open(struct inode *inode, struct file *f)
{
	struct spec_dev *dev = (struct spec_dev *)container_of(inode->i_cdev,
					struct spec_dev, cdev);

	f->private_data = dev;
	mutex_lock(&dev->mutex);
	dev->usecount++;
	mutex_unlock(&dev->mutex);

	return 0;
}

static int spec_release(struct inode *inode, struct file *f)
{
	struct spec_dev *dev = f->private_data;

	mutex_lock(&dev->mutex);
	dev->usecount--;
	mutex_unlock(&dev->mutex);

	return 0;
}

static int spec_mmap(struct file *f, struct vm_area_struct *vma)
{
	return -EOPNOTSUPP;
}

static ssize_t spec_read(struct file *f, char __user *buf, size_t count,
		loff_t *offp)
{
	struct spec_dev *dev = f->private_data;
	void *base;
	loff_t pos = *offp;
	int bar, off, size;
	u32 low, high;

	bar = SPEC_GET_BAR(pos) / 2; /* index in the array */
	off = SPEC_GET_OFF(pos);
	if (0)
		pr_info(KBUILD_MODNAME ": %s: pos %llx = bar %x off %x\n",
			__func__, pos, bar*2, off);
	if (!spec_is_valid_bar(pos))
		return -EINVAL;

	/* reading the DMA buffer is trivial, so do it first */
	if (SPEC_IS_DMABUF(pos)) {
		base = dev->dmabuf;
		if (off >= dmasize)
			return 0; /* EOF */
		if (off + count > dmasize)
			count = dmasize - off;
		if (copy_to_user(buf, base + off, count))
			return -EFAULT;
		*offp += count;
		return count;
	}

	/* inexistent or I/O ports: EINVAL */
	if (!dev->remap[bar])
		return -EINVAL;
	base = dev->remap[bar];

	/* valid on-board area: enforce sized access if size is 1,2,4,8 */
	size = dev->area[bar]->end + 1 - dev->area[bar]->start;
	if (off >= size)
		return -EIO; /* it's not memory, an error is better than EOF */
	if (count + off > size)
		count = size - off;
	switch (count) {
		case 1:
			if (put_user(readb(base + off), (u8 *)buf))
				return -EFAULT;
			break;
		case 2:
			if (put_user(readw(base + off), (u16 *)buf))
				return -EFAULT;
			break;
		case 4:
			if (put_user(readl(base + off), (u32 *)buf))
				return -EFAULT;
			break;
		case 8:
			low = readl(base + off);
			high = readl(base + off + 1);
			if (put_user(low + ((u64)high << 32), (u64 *)buf))
				return -EFAULT;
			break;
		default:
			if (copy_to_user(buf, base + off, count))
				return -EFAULT;
	}
	*offp += count;
	return count;
}

static ssize_t spec_write(struct file *f, const char __user *buf, size_t count,
		loff_t *offp)
{
	struct spec_dev *dev = f->private_data;
	void *base;
	loff_t pos = *offp;
	int bar, off, size;
	union {u8 d8; u16 d16; u32 d32; u64 d64;} data;
	bar = SPEC_GET_BAR(pos) / 2; /* index in the array */
	off = SPEC_GET_OFF(pos);
	if (!spec_is_valid_bar(pos))
		return -EINVAL;

	/* writing the DMA buffer is trivial, so do it first */
	if (SPEC_IS_DMABUF(pos)) {
		base = dev->dmabuf;
		if (off >= dmasize)
			return -ENOSPC;
		if (off + count > dmasize)
			count = dmasize - off;
		if (copy_from_user(base + off, buf, count))
			return -EFAULT;
		*offp += count;
		return count;
	}

	/* inexistent or I/O ports: EINVAL */
	if (!dev->remap[bar])
		return -EINVAL;
	base = dev->remap[bar];

	/* valid on-board area: enforce sized access if size is 1,2,4,8 */
	size = dev->area[bar]->end + 1 - dev->area[bar]->start;
	if (off >= size)
		return -EIO; /* it's not memory, an error is better than EOF */
	if (count + off > size)
		count = size - off;
	switch (count) {
		case 1:
			if (get_user(data.d8, (u8 *)buf))
				return -EFAULT;
			writeb(data.d8, base + off);
			break;
		case 2:
			if (get_user(data.d16, (u16 *)buf))
				return -EFAULT;
			writew(data.d16, base + off);
			break;
		case 4:
			if (get_user(data.d32, (u32 *)buf))
				return -EFAULT;
			writel(data.d32, base + off);
			break;
		case 8:
			/* while put_user_8 exists, get_user_8 does not */
			if (copy_from_user(&data.d64, buf, count))
				return -EFAULT;
			writel(data.d64, base + off);
			writel(data.d64 >> 32, base + off + 4);
			break;
		default:
			if (copy_from_user(base + off, buf, count))
				return -EFAULT;
	}
	*offp += count;
	return count;
}



static long spec_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct spec_dev *dev = f->private_data;
	void __user *argp = (void __user *)arg;
	struct spec_fw fw;
	int ret = 0;
	unsigned char *fwbuf;

	mutex_lock(&dev->mutex);

	switch (cmd) {
#ifdef SPEC_DEMO_CONFIG
	case SPEC_LOAD_DEMO_CONFIG:
		if (copy_from_user(&fw, argp, sizeof(fw))) {
			ret = -EFAULT;
			break;
		}
		if (fw.fwlen == 0) {
			ret = -EFAULT;
			break;
		}
		pr_info(KBUILD_MODNAME ": %s: firmware: %d bytes\n",
			__func__, fw.fwlen);
		fwbuf = kmalloc(fw.fwlen, GFP_KERNEL);
		if (fwbuf == NULL) {
			ret = -ENOMEM;
			break;
		}
		if (copy_from_user(fwbuf, fw.data, fw.fwlen)) {
			pr_err(KBUILD_MODNAME
				": %s: failed to copy firmware from user space\n",
				__func__);
			ret = -EFAULT;
			kfree(fwbuf);
			break;
		}
		config_space.size = fw.fwlen;
		config_space.data = fwbuf;
		pr_info(KBUILD_MODNAME ": loaded demo config space firmware\n");
		break;
#endif
	case SPEC_LOAD_FIRMWARE:
#ifdef SPEC_DEMO_CONFIG
		if (config_space.size == 0) {
			pr_err(KBUILD_MODNAME ": demo config space not set. "
				"use SPEC_LOAD_DEMO_CONFIG ioctl.\n");
			ret = -EFAULT;
			break;
		}
#endif
		if (copy_from_user(&fw, argp, sizeof(fw))) {
			ret = -EFAULT;
			break;
		}
		if (fw.fwlen == 0) {
			ret = -EFAULT;
			break;
		}
		pr_info(KBUILD_MODNAME ": %s: firmware: %d bytes\n",
			__func__, fw.fwlen);
		fwbuf = kmalloc(fw.fwlen, GFP_KERNEL);
		if (fwbuf == NULL) {
			ret = -ENOMEM;
			break;
		}
		if (copy_from_user(fwbuf, fw.data, fw.fwlen)) {
			pr_err(KBUILD_MODNAME
				": %s: failed to copy firmware from user space\n",
				__func__);
			ret = -EFAULT;
			kfree(fwbuf);
			break;
		}
		ret = spec_gennum_load(dev, (const void *)fwbuf,
			fw.fwlen);
		if (ret < 0) {
			pr_err(KBUILD_MODNAME
				": %s: failed to load firmware: error %d\n",
				__func__, ret);
			kfree(fwbuf);
			break;
		}
		pr_info(KBUILD_MODNAME
			": %s: loaded firmware successfully\n", __func__);

		kfree(fwbuf);
		break;
	}

	mutex_unlock(&dev->mutex);

	return ret;
}

struct file_operations spec_file_ops = {
	.open = spec_open,
	.release = spec_release,
	.mmap = spec_mmap,
	.read = spec_read,
	.write = spec_write,
	.unlocked_ioctl = spec_ioctl,
};

static int spec_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int i;
	int ret;
	int minor;
	struct spec_dev *dev;

	dev = kzalloc(sizeof(struct spec_dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "failed to allocate spec_dev\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	dev->dev = &pdev->dev;
	dev->pdev = pdev;
	pci_set_drvdata(pdev, dev);
	INIT_WORK(&dev->work, spec_load_firmware);
	init_waitqueue_head(&dev->irq_queue);
	cdev_init(&dev->cdev, &spec_file_ops);
	mutex_init(&dev->mutex);

	if (dmasize > SPEC_MAX_DMABUFSIZE) {
		pr_warning(KBUILD_MODNAME
			": %s: DMA buffer size too big, using 0x%x\n",
			__func__,
			SPEC_MAX_DMABUFSIZE);
		dmasize = SPEC_MAX_DMABUFSIZE;
	}

	dev->dmabuf = __vmalloc(dmasize, GFP_KERNEL | __GFP_ZERO,
			PAGE_KERNEL);
	if (!dev->dmabuf) {
		ret = -ENOMEM;
		goto err_dmaalloc;
	}

	mutex_lock(&spec_minors_mutex);
	minor = spec_get_free();
	if (minor == SPEC_MAX_MINORS) {
		dev_err(&pdev->dev, "too many devices!\n");
		ret = -EIO;
		goto err_dis;
	}
	mutex_unlock(&spec_minors_mutex);

	if (pdev->irq > 0) {
		i = request_irq(pdev->irq, spec_irq_handler, IRQF_SHARED,
				driver_name, dev);
		if (i < 0)
			pr_err(KBUILD_MODNAME
				"can't request irq %i, error %i\n",
				pdev->irq, i);
		else
			dev->flags |= SPEC_FLAG_IRQREQUEST;
	}

	ret = pci_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable SPEC device\n");
		goto err_enable;
	}

	ret = pci_request_regions(pdev, driver_name);
	if (ret) {
		dev_err(&pdev->dev, "failed to request PCI resources\n");
		goto err_resource;
	}

	/* Save BAR0, BAR2 and BAR4 and try to remap them */
	for (i = 0; i < 3; i++) {
		struct resource *r = &pdev->resource[2*i];
		if (!r->start)
			continue;
		dev->area[i] = r;
		if (r->flags & IORESOURCE_MEM)
			dev->remap[i] = ioremap(r->start,
						r->end - r->start + 1);
		if (SPEC_DEBUG) {
			pr_info(KBUILD_MODNAME
				": %s: BAR%i: %llx-%llx (size: 0x%llx) - %08lx\n",
				__func__, i,
				(long long)r->start,
				(long long)r->end,
				(long long)(r->end - r->start + 1),
				r->flags);
		}
	}

	ret = cdev_add(&dev->cdev, MKDEV(spec_major, minor), 1);
	if (ret) {
		dev_err(&pdev->dev, "chardev registration failed\n");
		goto err_cdev;
	}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29))
	if (IS_ERR(device_create(spec_class, &pdev->dev,
				MKDEV(spec_major, minor), NULL,
				"spec%u", minor)))
#else
	if (IS_ERR(device_create(spec_class, &pdev->dev,
				MKDEV(spec_major, minor), "spec%u", minor)))
#endif
		dev_err(&pdev->dev, "can't create device\n");

	if (fwname)
		spec_request_firmware(dev);

	return 0;

err_cdev:
	for (i = 0; i < 3; i++)
		iounmap(dev->remap[i]);
err_resource:
	pci_disable_device(pdev);
err_enable:
	if (dev->flags & SPEC_FLAG_IRQREQUEST)
		free_irq(pdev->irq, dev);
		/* FIXME: Reenable if we are shared */
	spec_minors[minor] = 0;
	mutex_unlock(&spec_minors_mutex);
err_dis:
	kfree(dev->dmabuf);
err_dmaalloc:
	kfree(dev);
err_alloc:
	return ret;
}

static void spec_remove(struct pci_dev *pdev)
{
	int i;
	struct spec_dev *dev = pci_get_drvdata(pdev);
	unsigned int minor = MINOR(dev->cdev.dev);

	if (dev->bus_registered)
		wb_unregister_bus(&dev->wb_bus);

	device_destroy(spec_class, MKDEV(spec_major, minor));

	cdev_del(&dev->cdev);

	if (dev->flags & SPEC_FLAG_IRQREQUEST)
		free_irq(pdev->irq, dev);
		/* FIXME: Reenable if we are shared */

	for (i = 0; i < 3; i++)
		iounmap(dev->remap[i]);
	release_firmware(dev->fw);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	vfree(dev->dmabuf);
	kfree(dev);
}

static const struct pci_device_id spec_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CERN, PCI_DEVICE_ID_CERN_SPEC_V2) },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, spec_ids);

static struct pci_driver spec_pci_driver = {
	.name = "spec",
	.id_table = spec_ids,
	.probe = spec_probe,
	.remove = __devexit_p(spec_remove),
};

static int __init spec_init(void)
{
	int ret;
	dev_t dev;

	spec_class = class_create(THIS_MODULE, "spec");
	if (IS_ERR(spec_class)) {
		ret = PTR_ERR(spec_class);
		pr_err(KBUILD_MODNAME ": can't register spec class\n");
		goto err;
	}

	ret = alloc_chrdev_region(&dev, 0, SPEC_MAX_MINORS, "spec");
	if (ret) {
		pr_err(KBUILD_MODNAME
			": can't register character device\n");
		goto err_attr;
	}
	spec_major = MAJOR(dev);

	ret = pci_register_driver(&spec_pci_driver);
	if (ret) {
		pr_err(KBUILD_MODNAME ": can't register pci driver\n");
		goto err_unchr;
	}

	pr_info(KBUILD_MODNAME ": %s: SPEC driver version %08x"
		": init OK\n", __func__, SPECMOD_VERSION);

	return 0;

err_unchr:
	unregister_chrdev_region(dev, SPEC_MAX_MINORS);
err_attr:
	class_destroy(spec_class);
err:
	return ret;
}

static void __exit spec_exit(void)
{
	pci_unregister_driver(&spec_pci_driver);

	unregister_chrdev_region(MKDEV(spec_major, 0), SPEC_MAX_MINORS);

	class_destroy(spec_class);

#ifdef SPEC_DEMO_CONFIG
	if (config_space.size != 0)
		kfree(config_space.data);
#endif
}

module_init(spec_init);
module_exit(spec_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC Linux Driver");
MODULE_LICENSE("GPL");
