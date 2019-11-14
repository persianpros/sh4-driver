#ARCH = sh
#CROSS_COMPILE = sh4-linux-
#INSTALL_MOD_PATH = /mnt/rootfs_nfs
#KDIR = /opt/STM/STLinux-2.3/devkit/build/build-mb442/kernel
#PWD = $(shell pwd)

EXTRA_CFLAGS = -DEXPORT_SYMTAB -D__TDT__

##CFLAGS: dummy functions need this.
CFLAGS += -Idrivers/media/dvb/dvb-core

obj-m      += qboxhd_generic.o

#all:
	#make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

#clean:
	#-rm -f *.o *.ko .*.cmd .*.flags *.mod.c

 #-include $(KDIR)/Rules.make
