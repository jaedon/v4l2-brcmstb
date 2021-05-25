# Makefile -- For the linux kernel driver module

#
# By default, the build is done against the running linux kernel source.
# To build against a different kernel source tree, set KDIR:
#
#	make KDIR=/path/to/kernel/source

ifneq ($(KERNELRELEASE),)

obj-m += brcmstb-v4l2.o

else

KDIR	?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

endif
