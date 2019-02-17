ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    kathrein_micom-objs += kathrein_micom_file.o
    kathrein_micom-objs += kathrein_micom_asc.o
    kathrein_micom-objs += kathrein_micom_procfs.o
    kathrein_micom-objs += kathrein_micom_main.o

    obj-m += kathrein_micom.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif

endif

