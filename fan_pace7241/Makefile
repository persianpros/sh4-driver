#
# Makefile for module fan controlling on pace7241
#
ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

obj-m += fan_ctrl.o
fan_ctrl-objs += pace7241_fan.o
