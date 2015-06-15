#
# Makefile for the Intel(R) PRO/1000 ethernet driver
#

obj-$(CONFIG_E1000E) += e1000e.o

e1000e-objs := 82571.o ich8lan.o 80003es2lan.o \
	       mac.o manage.o nvm.o phy.o \
	       param.o ethtool.o netdev.o ptp.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order
