
CFLAGS ?= -O2 -pipe
LDFLAGS ?=

PROGRAMS = init dmesg

all: $(PROGRAMS)

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

clean:
	rm -f $(PROGRAMS)

.PHONY: all clean
