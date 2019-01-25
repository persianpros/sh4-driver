CCFLAGSY += -DDEBUG_DVB

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

dvb-it913x-objs := it913x.o it913x-fe.o 
obj-m := dvb-it913x.o


EXTRA_CFLAGS += -DLINUX -DCONFIG_IT913X_USB
EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core
EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-usb
EXTRA_CFLAGS += -Idrivers/media/dvb/frontends
