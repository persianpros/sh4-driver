EXTRA_CFLAGS = -DEXPORT_SYMTAB

TEST_CFLAGS = -O2 -Wall
TEST_TARGET = scart_mini_test

SLIMAPP_CFLAGS = -O2 -Wall
SLIMAPP_TARGET = scart_mini_slimapp

obj-m      += scart_mini.o


test:
	$(CROSS_COMPILE)gcc $(TEST_CFLAGS) $(TEST_TARGET).c -o $(TEST_TARGET)

slimapp:
	$(CROSS_COMPILE)gcc $(SLIMAPP_CFLAGS) $(SLIMAPP_TARGET).c -o $(SLIMAPP_TARGET)

