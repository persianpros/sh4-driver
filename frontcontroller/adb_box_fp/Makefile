ifeq ($(KERNELRELEASE),)
DRIVER_TOPDIR:=$(shell pwd)/..
include $(DRIVER_TOPDIR)/kernel.make
else

adb_fp-objs += adb_box_fp.o
adb_fp-objs += adb_box_pt6302.o
adb_fp-objs += adb_box_pt6958.o
obj-m       += adb_fp.o

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif
endif

