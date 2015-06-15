#
# Makefile for the VIA device drivers.
#

obj-$(CONFIG_VIA_RHINE) += via-rhine.o

via-rhine-objs := via-rhine_main.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order

