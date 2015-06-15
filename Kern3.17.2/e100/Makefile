#
# Makefile for the Intel network device drivers.
#

obj-$(CONFIG_E100) += e100.o

e100-objs := e100_main.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order

