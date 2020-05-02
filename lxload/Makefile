#
# Makefile for LX firmware loader.
#
ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

obj-m       := lxload.o
lxload-objs := lxloader.o

