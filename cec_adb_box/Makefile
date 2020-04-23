#
# Makefile for HDMI CEC control on ADB ITI-5800
#
ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

obj-m += cec_ctrl.o
cec_ctrl-objs += cec.o
