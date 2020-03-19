#
# Makefile for module fan controlling on adb_box
#
ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

obj-m += adb_fan.o
adb_fan-objs += adb_box_fan.o
