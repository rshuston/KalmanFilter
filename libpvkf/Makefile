include ../Makefile.inc

LIB = libpvkf.a

SRC = PVKalmanFilter.c
OBJS = $(SRC:.c=.o)

TEST_SRC = $(SRC:.c=_test.c)
TEST_OBJS = $(TEST_SRC:.c=.o)
TEST_EXES =  $(TEST_SRC:.c=.exe)

.PHONY: default force tests runtests clean clobber

default: $(LIB)

force:

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

PVKalmanFilter_test.exe: PVKalmanFilter_test.o PVKalmanFilter.o
	$(LD) $(LDFLAGS) $^ $(TEST_LDFLAGS) -o $@

tests: force
	@$(MAKE) $(TEST_EXES)

runtests: tests
	@for t in $(TEST_EXES); do (./$$t); done

clean:
	rm -rf $(OBJS) $(OBJS:.o=.d) $(TEST_OBJS) $(TEST_OBJS:.o=.d)

clobber:
	rm -f $(OBJS) $(OBJS:.o=.d) $(TEST_OBJS) $(TEST_OBJS:.o=.d) $(TEST_EXES) $(LIB)

-include $(wildcard *d)
