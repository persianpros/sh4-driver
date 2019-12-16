ifeq ($(KERNELRELEASE),)
    DRIVER_TOPDIR:=$(shell pwd)/..
    include $(DRIVER_TOPDIR)/kernel.make
else
    ifdef FOREVER_NANOSMART
        fortis_4g_fp-objs += fortis_4g_i2c.o
    endif
    ifdef DP7001
        fortis_4g_fp-objs += fortis_4g_i2c.o
    endif
    ifdef FOREVER_9898HD
        fortis_4g_fp-objs += fortis_4g_pio.o
    endif
    ifdef FOREVER_2424HD
        fortis_4g_fp-objs += fortis_4g_i2c.o
    endif
    ifdef GPV8000
        fortis_4g_fp-objs += fortis_4g_pio.o
    endif
    ifdef EP8000
        fortis_4g_fp-objs += fortis_4g_pio.o
    endif
    ifdef EPP8000
        fortis_4g_fp-objs += fortis_4g_pio.o
    endif
    fortis_4g_fp-objs += fortis_4g_main.o

    obj-m += fortis_4g_fp.o

    ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
        ccflags-y += $(CCFLAGSY)
    else
        CFLAGS += $(CCFLAGSY)
    endif
endif
