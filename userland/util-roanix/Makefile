
CFLAGS ?= -O2 -pipe
LDFLAGS ?=

PROGRAMS = init dmesg pthread-test

all: $(PROGRAMS)

pthread-test: LDLIBS = -lpthread

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(PROGRAMS)

.PHONY: all clean
