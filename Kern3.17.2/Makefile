obj-$(CONFIG_E100) += e100/
obj-$(CONFIG_VORTEX) += 3c59x/
obj-$(CONFIG_E1000) += e1000/
obj-$(CONFIG_E1000E) += e1000e/
obj-$(CONFIG_IGB) += igb/
obj-$(CONFIG_IP1000) += ipg/
obj-$(CONFIG_MOUSE_PS2) += psmouse/
obj-$(CONFIG_NE2K_PCI) += ne2k-pci/
obj-$(CONFIG_R8169) += r8169/
obj-$(CONFIG_USB_UHCI_HCD) += uhci_hcd/
obj-$(CONFIG_USB_STORAGE) += usb_storage/
obj-$(CONFIG_8139TOO) += 8139too/
obj-$(CONFIG_SKY2) += sky2/
obj-$(CONFIG_USB_HID) += usbhid/
obj-$(CONFIG_VIA_RHINE) += via-rhine/

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
clean:
	rm -rf */*.ko */*.o */*.mod.* */*.symvers */*.order */.*
