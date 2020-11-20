ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    opt9600_fp-objs += opt9600_fp_file.o
    opt9600_fp-objs += opt9600_fp_asc.o
    opt9600_fp-objs += opt9600_fp_main.o

    obj-m += opt9600_fp.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif
endif

