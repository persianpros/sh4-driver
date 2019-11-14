ifeq ($(BOARD_TYPE),qboxhd)

EXTRA_CFLAGS = -DEXPORT_SYMTAB

TEST_CFLAGS = -O2 -Wall
TEST_TARGET = starci2win_test

obj-m      += starci2win_mod.o
starci2win_mod-objs := starci2win_i2c.o starci2win.o ci_aux_functions.o ci_cis_functions.o

else
endif

