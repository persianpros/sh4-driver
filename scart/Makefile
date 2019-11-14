ifeq ($(BOARD_TYPE),qboxhd)
scart-objs +=		stv6414/stv6414.o
obj-m += scart.o
else 
ifeq ($(BOARD_TYPE),qboxhd_mini)
scart_mini-objs +=	scart_pins/scart_mini.o
obj-m += scart_mini.o
endif
endif


