/* BAR4 0x808*/

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

#include "spec.h"
#include "gennum.h"

#define SPEC_VERSION			"22082011"
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
MODULE_PARM_DESC(fwname, "The name of the firmware to load");

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
			printk("%s: %i: done after %i\n", __func__, __LINE__,
				wrote);
			done = 1;
		} else if( (i & 0x4) && !done) {
			printk("%s: %i: error after %i\n", __func__, __LINE__,
				wrote);
			return -ETIMEDOUT;
		}
		if (time_after(jiffies, j)) {
			printk("%s: %i: tout after %i\n", __func__, __LINE__,
				wrote);
			return -ETIMEDOUT;
		}
	}
	return 0;
}

void spec_complete_firmware(const struct firmware *fw, void *context)
{
	struct spec_dev *dev = (struct spec_dev *)context;
	int err;

	dev->fw = fw;

	if (fw) {
		pr_info("%s: %p: size %i (0x%x), data %p\n", __func__, fw,
			fw ? fw->size : 0, fw ? fw->size : 0,
			fw ? fw->data : 0);
	} else {
		pr_warning("%s: no firmware\n", __func__);
		return;
	}

	err = spec_gennum_load(dev, fw->data, fw->size);
	if (err)
		pr_err("%s: loading returned error %i\n", __func__, err);

	release_firmware(dev->fw);
	dev->fw = NULL;
}

void spec_load_firmware(struct work_struct *work)
{
	struct spec_dev *dev = container_of(work, struct spec_dev, work);
	struct pci_dev *pdev = dev->pdev;
	static char fwname[SPEC_MAX_FWNAME_SIZE];
	int err;

	if (!fwname) {
		dev_err(&pdev->dev, "no fwname specified\n");
		return;
	}
	if (SPEC_DEBUG)
		printk("%s: %s\n", __func__, fwname);

	err = request_firmware_nowait(THIS_MODULE, 1, fwname, &pdev->dev,
		dev, spec_complete_firmware);
	printk("request_firmware returned %i\n", err);
}

void spec_request_firmware(struct spec_dev *dev)
{
	if (dev->fw) {
		pr_err("firmware loading already in progress\n");
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

	printk("Spec Open!\n");

	return 0;
}

static int spec_release(struct inode *inode, struct file *f)
{
	struct spec_dev *dev = f->private_data;

	mutex_lock(&dev->mutex);
	dev->usecount--;
	mutex_unlock(&dev->mutex);

	printk("Spec Close!\n");

	return 0;
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
	case SPEC_LOADFW:
		if (copy_from_user(&fw, argp, sizeof(fw))) {
			ret = -EFAULT;
			break;
		}
		if (fw.fwlen == 0) {
			ret = -EFAULT;
			break;
		}
		printk(KERN_INFO KBUILD_MODNAME "firmware: %d bytes\n",
			fw.fwlen);
		fwbuf = kmalloc(fw.fwlen, GFP_KERNEL);
		if (fwbuf == NULL) {
			ret = -ENOMEM;
			break;
		}
		if (copy_from_user(fwbuf, fw.data, fw.fwlen)) {
			printk(KERN_ERR KBUILD_MODNAME
				"failed to copy firmware from user space\n");
			ret = -EFAULT;
			kfree(fwbuf);
			break;
		}
		ret = gennum_loader(dev->remap[2], (const void *)fwbuf,
			fw.fwlen);
		if (ret < 0) {
			printk(KERN_ERR KBUILD_MODNAME
				"failed to load firmware: error %d\n", ret);
			kfree(fwbuf);
			break;
		}

		kfree(fwbuf);
		break;
	}

	mutex_unlock(&dev->mutex);

	return ret;
}

struct file_operations spec_file_ops = {
	.open = spec_open,
	.release = spec_release,
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
			printk(KERN_ERR KBUILD_MODNAME
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
			printk("%s: BAR%i: %llx-%llx (size: 0x%llx) - %08lx\n",
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
	kfree(dev);
err_alloc:
	return ret;
}

static void spec_remove(struct pci_dev *pdev)
{
	int i;
	struct spec_dev *dev = pci_get_drvdata(pdev);
	unsigned int minor = MINOR(dev->cdev.dev);

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
		printk(KERN_ERR "can't register spec class\n");
		goto err;
	}

	ret = alloc_chrdev_region(&dev, 0, SPEC_MAX_MINORS, "spec");
	if (ret) {
		printk(KERN_ERR "can't register character device\n");
		goto err_attr;
	}
	spec_major = MAJOR(dev);

	ret = pci_register_driver(&spec_pci_driver);
	if (ret) {
		printk(KERN_ERR "can't register pci driver\n");
		goto err_unchr;
	}

	printk(KERN_INFO "SPEC Driver, version " SPEC_VERSION ", init OK\n");

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
}

module_init(spec_init);
module_exit(spec_exit);

MODULE_AUTHOR("Manohar Vanga");
MODULE_DESCRIPTION("CERN SPEC Linux Driver");
MODULE_LICENSE("GPL");
