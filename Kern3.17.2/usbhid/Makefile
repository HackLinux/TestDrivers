#
# Makefile for the USB input drivers
#

# Multipart objects.
usbhid-y	:= hid-core.o hid-quirks.o

ifeq ($(CONFIG_USB_HIDDEV),y)
	usbhid-y    += hiddev.o
endif
ifeq ($(CONFIG_HID_PID),y)
	usbhid-y    += hid-pidff.o
endif

obj-$(CONFIG_USB_HID)		+= usbhid.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.c *.symvers *.order

