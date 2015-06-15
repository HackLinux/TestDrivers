obj-$(CONFIG_E1000) += e1000.o

e1000-objs := e1000_main.o e1000_hw.o e1000_ethtool.o e1000_param.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf *.ko *.o *.mod.* *.symvers *.order
