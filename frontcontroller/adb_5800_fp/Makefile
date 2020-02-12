ifeq ($(KERNELRELEASE),)
	DRIVER_TOPDIR:=$(shell pwd)/..
	include $(DRIVER_TOPDIR)/kernel.make
else
	adb_5800_fp-objs += adb_5800_fp_main.o
	adb_5800_fp-objs += adb_5800_fp_file.o
	adb_5800_fp-objs += adb_5800_fp_procfs.o
	obj-m += adb_5800_fp.o

	ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
		ccflags-y += $(CCFLAGSY)
	else
		CFLAGS += $(CCFLAGSY)
	endif
endif

