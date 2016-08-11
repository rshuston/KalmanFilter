include Makefile.inc

EXE = runner.exe

APP_LIB = app.a

APP_SRC = app.c
APP_OBJS = $(APP_SRC:.c=.o)

APP_TEST_SRC = $(APP_SRC:.c=_test.c)
APP_TEST_OBJS =  $(APP_TEST_SRC:.c=.o)
APP_TEST_EXES =  $(APP_TEST_SRC:.c=.exe)

COMPONENT_DIRS = libpvkf
COMPONENT_LIBS = $(foreach dir, $(COMPONENT_DIRS), $(dir)/$(dir).a)

CFLAGS += $(addprefix -I, $(COMPONENT_DIRS))

.PHONY: default force all tests alltests runtests runalltests deps clean clobber

default: $(EXE)

force:

$(EXE): main.o $(APP_LIB) $(COMPONENT_LIBS)
	$(LD) $(LDFLAGS) $^ -o $(EXE)

$(APP_LIB): $(APP_OBJS)
	$(AR) $(ARFLAGS) $@ $^

$(COMPONENT_LIBS): force
	$(MAKE) -C $(dir $@) $(notdir $@)

app_test.exe: app_test.o $(APP_LIB) $(COMPONENT_LIBS)
	$(LD) $(LDFLAGS) $^ $(TEST_LDFLAGS) -o $@

all: $(EXE) alltests

tests: force
	$(MAKE) $(APP_TEST_EXES)

alltests: tests
	for d in $(COMPONENT_DIRS); do (cd $$d; $(MAKE) tests ); done

runtests: tests
	for t in $(APP_TEST_EXES); do (./$$t); done

runalltests: runtests
	for d in $(COMPONENT_DIRS); do (cd $$d; $(MAKE) runtests ); done

deps:
	$(CC) -MM $(CFLAGS) main.c $(APP_SRC) $(APP_TEST_SRC) > Makefile.deps
	for d in $(COMPONENT_DIRS); do (cd $$d; $(MAKE) deps ); done

clean:
	rm -f main.o $(APP_OBJS) $(APP_TEST_OBJS)
	for d in $(COMPONENT_DIRS); do (cd $$d; $(MAKE) clean ); done

clobber:
	rm -f main.o $(APP_OBJS) $(APP_TEST_OBJS) $(APP_LIB) $(EXE) $(APP_TEST_EXES)
	for d in $(COMPONENT_DIRS); do (cd $$d; $(MAKE) clobber ); done

-include Makefile.deps
