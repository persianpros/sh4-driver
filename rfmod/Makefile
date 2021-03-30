TREE_ROOT?=$(DRIVER_TOPDIR)/

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

obj-m := rfmod.o
rfmod-objs := 74t1.o rfmod_core.o 
