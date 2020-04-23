TREE_ROOT?=$(DRIVER_TOPDIR)/
CONFIG_STGFB_PATH?=$(DRIVER_TOPDIR)/stgfb/stmfb
CCFLAGSY += -I$(TREE_ROOT)/linux/include/linux/stm/
CCFLAGSY += -I$(TREE_ROOT)/linux/drivers/media/video/stm/
CCFLAGSY += -I$(KERNEL_LOCATION)/drivers/media/dvb/dvb-core
CCFLAGSY += -I$(CONFIG_STGFB_PATH)/include
CCFLAGSY += -DDEBUG_DVB
CCFLAGSY += -I$(TREE_ROOT)/include
ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

fe_core-objs += dibx000_common.o
fe_core-objs += dib0070.o
fe_core-objs += dib7000p.o

#fe_core-objs += mxl111sf-gpio.o
fe_core-objs += mxl111sf-phy.o
fe_core-objs += mxl111sf-demod.o
fe_core-objs += mxl111sf-tuner.o
fe_core-objs += mxl111sf.o

fe_core-objs +=	stv090x.o
fe_core-objs +=	stv6110x.o
fe_core-objs +=	core.o

obj-m        += fe_core.o
