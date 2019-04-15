TREE_ROOT?=$(DRIVER_TOPDIR)/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/avl2108/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/avl6222/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/cx24116/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/tda10023/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/zl10353/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/stv0288/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/frontends/tda10024_mxl201/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/lnbs/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/platform/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/socket/
CCFLAGSY += -I$(TREE_ROOT)/frontends/multituner/include


ifneq (,$(findstring 2.6.3,$(KERNELVERSION)))
ccflags-y += $(CCFLAGSY)
else
CFLAGS += $(CCFLAGSY)
endif

lnb_pio-objs +=       lnbs/lnb_pio.o
obj-m += lnb_pio.o
lnbh221-objs +=       lnbs/lnbh221.o
obj-m += lnbh221.o
lnb_a8293-objs +=       lnbs/lnb_a8293.o
obj-m += lnb_a8293.o

socket-objs +=       socket/socket.o
obj-m += socket.o

frontend_platform-objs +=       platform/frontend_platform.o
obj-m += frontend_platform.o

ifdef ATEVIO7500
avl2108-objs +=        frontends/avl2108/avl2108.o
avl2108-objs +=        frontends/avl2108/stv6110a.o
avl2108-objs +=        frontends/avl2108/stv6306.o
obj-m += avl2108.o

tda10024-objs +=        frontends/tda10024_mxl201/tda10024.o
tda10024-objs +=        frontends/tda10024_mxl201/mxl201.o
obj-m += tda10024.o

cxd2820-objs +=        frontends/cxd2820_tda18272/cxd2820.o
cxd2820-objs +=        frontends/cxd2820_tda18272/tda18272.o
obj-m += cxd2820.o
endif

ifdef UFS913
avl6222-objs +=        frontends/avl6222/avl6222.o
avl6222-objs +=        frontends/avl6222/stv6110a.o
obj-m += avl6222.o
endif

ifdef CUBEREVO
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_MINI
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_MINI2
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_250HD
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_9500HD
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_2000HD
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_MINI_FTA
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o

else ifdef CUBEREVO_3000HD
cx24116-objs +=        frontends/cx24116/cx24116.o
obj-m += cx24116.o

stv090x-objs +=        frontends/stv090x/stv090x.o
stv090x-objs +=        frontends/stv090x/stv6110x.o
stv090x-objs +=        frontends/stv090x/core.o
obj-m += stv090x.o

tda10023-objs +=        frontends/tda10023/tda10023.o
tda10023-objs +=        frontends/tda10023/core.o
obj-m += tda10023.o

zl10353-objs +=        frontends/zl10353/zl10353.o
zl10353-objs +=        frontends/zl10353/core.o
obj-m += zl10353.o

stv0288-objs +=        frontends/stv0288/stv0288.o
stv0288-objs +=        frontends/stv0288/core.o
obj-m += stv0288.o
endif
