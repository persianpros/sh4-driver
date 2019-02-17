ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    cuberevo_micom-objs += cuberevo_micom_file.o
    cuberevo_micom-objs += cuberevo_micom_asc.o
    cuberevo_micom-objs += cuberevo_micom_procfs.o
    cuberevo_micom-objs += cuberevo_micom_main.o

    obj-m += cuberevo_micom.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif
endif

