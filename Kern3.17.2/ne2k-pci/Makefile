#
# Makefile for the 8390 network device drivers.
#

obj-$(CONFIG_NE2K_PCI) += ne2k-pci.o

ne2k-pci-objs := ne2k-pci_main.o 8390.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order
