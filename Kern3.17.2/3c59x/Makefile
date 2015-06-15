#
# Makefile for the 3Com Ethernet device drivers
#

obj-$(CONFIG_VORTEX) += 3c59x.o

3c59x-objs := 3c59x_main.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order
