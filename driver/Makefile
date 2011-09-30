SPEC_VERSION=0x$(shell date +%d%m%Y)

obj-m += cernspec.o

cernspec-objs += spec.o gennum.o

EXTRA_CFLAGS=-DSPECMOD_VERSION=$(SPEC_VERSION)

CPU?=L865
KVER?=2.6.24.7-rt27
KERN_DIR?=/acc/sys/$(CPU)/usr/src/kernels/$(KVER)

.PHONY: all clean

all:
	$(MAKE) -C $(KERN_DIR) M=`pwd` modules

clean:
	$(MAKE) -C $(KERN_DIR) M=`pwd` clean
