#ARCH = sh 
#CROSS_COMPILE = sh4-linux- 
#INSTALL_MOD_PATH = /opt/STM/STLinux-2.2/devkit/sh4/target
#INSTALL_MOD_PATH = /mnt/tmp/mnt/stb7109/

# Old kernel 
#INSTALL_MOD_PATH = /mnt/tmp/mnt/bck_stb7109_090908/stb7109
#KDIR = $(INSTALL_MOD_PATH)/lib/modules/$(CURRENT)/build
#CURRENT = 2.6.17.14_stm22_0039-COCOREF_GOLD_7109-STFAE


#INSTALL_MOD_PATH = /opt/STM/STLinux-2.3/devkit/sh4/target
#INSTALL_MOD_PATH = /00_qbox_hd/rootFS/

#KDIR = /opt/STM/STLinux-2.3/devkit/build/build-mb442/kernel


#PWD = $(shell pwd)
#EXTRA_CFLAGS = -DEXPORT_SYMTAB

#TEST_CFLAGS = -O2 -Wall
TEST_CFLAGS = -Wall

TEST_TARGET = fpanel


obj-m      += fpanel.o


#all:
	#make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules 
#	@cp -v fpga_drv.ko $(INSTALL_MOD_PATH)/home

test:
	$(CROSS_COMPILE)gcc $(TEST_CFLAGS) $(TEST_TARGET).c -o $(TEST_TARGET)

#	@cp -v $(TEST_TARGET) $(INSTALL_MOD_PATH)/home


#clean:
	#-rm -f *.o *.ko .*.cmd .*.flags *.mod.c $(TEST_TARGET)
 
#-include $(KDIR)/Rules.make
