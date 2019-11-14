#
# @file      Makefile for the LCD driver
#			 It calls the Makefile for the daemon
# @author    Pedro Aguilar
#
# Copyright (c) 2008 Duolabs S.r.l.
#
# Changelog:
# Date    Author      Comments
# ------------------------------------------------------------------------
# 020408  paguilar    Original
# 100608  paguilar    Move user-space app to dir /daemon
#

#ARCH = sh
#CROSS_COMPILE = sh4-linux-
#INSTALL_MOD_PATH = /opt/STM/STLinux-2.3/devkit/sh4/target
#CURRENT = 2.6.23.16_stm23_0109-1-qboxhd
#KDIR = $(INSTALL_MOD_PATH)/lib/modules/$(CURRENT)/build
#KDIR = /opt/STM/STLinux-2.3/devkit/build/build-mb442/kernel
#PWD = $(shell pwd)
EXTRA_CFLAGS = -DEXPORT_SYMTAB


obj-m      := lcd.o
lcd-objs := lcd_drv.o

#all:
	#make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KDIR) M=$(PWD) modules

lcd_on_off_brightness:
	$(CROSS_COMPILE)gcc -O2 -Wall $@.c -o $@
	$(CROSS_COMPILE)strip $@

#clean:
		#-rm -f *.o *.ko .*.cmd .*.flags *.mod.c lcd_on_off_brightness

 #-include $(KDIR)/Rules.make

