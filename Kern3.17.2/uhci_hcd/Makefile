#
# Makefile for USB Host Controller Drivers
#

obj-$(CONFIG_USB_UHCI_HCD)	+= uhci_hcd.o

uhci_hcd-objs := uhci-hcd_main.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.c *.symvers *.order
