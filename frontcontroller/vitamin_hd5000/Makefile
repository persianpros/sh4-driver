ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    vitamin_micom-objs += vitamin_micom_file.o
    vitamin_micom-objs += vitamin_micom_asc.o
    vitamin_micom-objs += vitamin_micom_procfs.o
    vitamin_micom-objs += vitamin_micom_main.o

    obj-m += vitamin_micom.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif
endif

