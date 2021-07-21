ifeq ($(KERNELRELEASE),)
DRIVER_TOPDIR:=$(shell pwd)/..
include $(DRIVER_TOPDIR)/kernel.make
else
ufs910_fp-objs += ufs910_fp_main.o
ufs910_fp-objs += ufs910_fp_file.o
ufs910_fp-objs += ufs910_fp_procfs.o

obj-m += ufs910_fp.o

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif
endif

