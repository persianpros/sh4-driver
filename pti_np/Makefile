TREE_ROOT?=$(DRIVER_TOPDIR)/
CONFIG_STGFB_PATH?=$(DRIVER_TOPDIR)/stgfb/stmfb
CCFLAGSY += -I$(KERNEL_LOCATION)/drivers/media/dvb/dvb-core 
CCFLAGSY += -I$(CONFIG_STGFB_PATH)/include 
CCFLAGSY += -DDEBUG_DVB
CCFLAGSY += -I$(TREE_ROOT)/include 
CCFLAGSY += -I$(TREE_ROOT)/player2/linux/include 
CCFLAGSY += -I$(TREE_ROOT)/player2/player/standards
CCFLAGSY += -O2

#player2_211
CCFLAGSY += -I$(TREE_ROOT)/player2/player/wrapper/
CCFLAGSY += -I$(TREE_ROOT)/player2/linux/include/linux/stm/

ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

pti-objs:=	pti_main.o \
		pti_session.o \
		pti_slot.o \
		pti_buffer.o \
		pti_hal.o \
		pti_descrambler.o \
		pti_process.o \
		pti_filter.o

obj-m += pti.o
