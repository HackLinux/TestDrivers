#
# Makefile for the USB Mass Storage device drivers.
#
# 15 Aug 2000, Christoph Hellwig <hch@infradead.org>
# Rewritten to use lists instead of if-statements.
#

obj-$(CONFIG_USB_STORAGE)	+= usb_storage.o

usb_storage-y := scsiglue.o protocol.o transport.o usb.o
usb_storage-y += initializers.o sierra_ms.o option_ms.o
usb_storage-y += usual-tables.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.c *.symvers *.order
