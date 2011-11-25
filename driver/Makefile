SPEC_VERSION=0x$(shell date +%Y%m%d)

obj-m += cernspec.o

cernspec-objs += spec.o gennum.o

EXTRA_CFLAGS=-DSPECMOD_VERSION=$(SPEC_VERSION) -I$(obj)/../../wishbone/simulation/include -DSPEC_DEMO_CONFIG -DSPEC_DEBUG

CPU?=L865
KVER?=2.6.24.7-rt27
LINUX?=/acc/sys/$(CPU)/usr/src/kernels/$(KVER)

.PHONY: all clean

all:
	$(MAKE) -C $(LINUX) M=`pwd` modules

clean:
	$(MAKE) -C $(LINUX) M=`pwd` clean
