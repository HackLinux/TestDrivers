#
# Makefile for the IC Plus device drivers
#

obj-$(CONFIG_IP1000) += ipg.o

ipg-objs := ipg_main.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order
