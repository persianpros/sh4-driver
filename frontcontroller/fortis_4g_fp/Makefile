ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    ifdef FOREVER_NANOSMART
        fortis_4g_fp-objs += fortis_4g_fp_et6226.o
    endif
    ifdef FOREVER_9898HD
        fortis_4g_fp-objs += fortis_4g_fp_et16315.o
    endif
    ifdef DP7001
        fortis_4g_fp-objs += fortis_4g_fp_et6226.o
    endif
    ifdef FOREVER_3434HD
        fortis_4g_fp-objs += fortis_4g_fp_et16315.o
    endif
    ifdef FOREVER_2424HD
        fortis_4g_fp-objs += fortis_4g_fp_et6226.o
    endif
    ifdef GPV8000
        fortis_4g_fp-objs += fortis_4g_fp_et16315.o
    endif
    ifdef EP8000
        fortis_4g_fp-objs += fortis_4g_fp_et16315.o
    endif
    ifdef EPP8000
        fortis_4g_fp-objs += fortis_4g_fp_et16315.o
    endif
    fortis_4g_fp-objs += fortis_4g_fp_main.o
    fortis_4g_fp-objs += fortis_4g_fp_procfs.o

    obj-m += fortis_4g_fp.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif
endif

