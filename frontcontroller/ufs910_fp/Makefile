ifeq ($(KERNELRELEASE),)
DRIVER_TOPDIR:=$(shell pwd)/..
include $(DRIVER_TOPDIR)/kernel.make
else
obj-m += ufs910_fp.o
obj-m += ufs910_14w_button.o
obj-m += ufs910_14w_led.o

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif
endif

