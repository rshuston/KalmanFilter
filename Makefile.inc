CC = gcc
CFLAGS = -g -Wall

LD = gcc
LDFLAGS =
TEST_LDFLAGS =

AR = ar
ARFLAGS = rcs

PKG_CONFIG = $(shell which pkg-config)
ifneq ($(PKG_CONFIG), "")
CFLAGS += $(shell $(PKG_CONFIG) --cflags check)
TEST_LDFLAGS += $(shell $(PKG_CONFIG) --libs check)
endif

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@ -MMD

%.tstbl.o : %.c
	$(CC) -c $(CFLAGS) -DTESTABLE $< -o $@

%_test.exe : %_test.o %.tstbl.o
	$(LD) $(LDFLAGS) $^ $(TEST_LDFLAGS) -o $@

.PRECIOUS: %.o %.tstbl.o
