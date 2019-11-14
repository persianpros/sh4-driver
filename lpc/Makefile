#ARCH = sh 
#CROSS_COMPILE = sh4-linux-


#INSTALL_MOD_PATH = /opt/STM/STLinux-2.3/devkit/sh4/target
#CURRENT = 2.6.23.16_stm23_0109-1-mb442
#CURRENT = 2.6.23.17_stm23_0109-1-qboxhd
#KDIR = $(INSTALL_MOD_PATH)/lib/modules/$(CURRENT)/build
#KDIR = /opt/STM/STLinux-2.3/devkit/build/build-mb442/kernel

#PWD = $(shell pwd)
#EXTRA_CFLAGS = -DEXPORT_SYMTAB

TEST_CFLAGS = -O2 -Wall
TEST_TARGET = lpc_test
APP_TARGET = lpc_app


obj-m      += lpc_mod.o
lpc_mod-objs := lpc_i2c.o time_funcs.o

#all:
	#make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

test:
	$(CROSS_COMPILE)gcc $(TEST_CFLAGS) $(TEST_TARGET).c -o $(TEST_TARGET)

app:
	$(CROSS_COMPILE)gcc $(TEST_CFLAGS) $(APP_TARGET).c -o $(APP_TARGET)

#clean:
	#rm -rf *.o *.ko .*.cmd .*.flags *.mod.c  *~ .tmp_versions Module.symvers $(TEST_TARGET)
 
 #-include $(KDIR)/Rules.make
