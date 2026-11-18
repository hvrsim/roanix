CFLAGS ?= -O2 -pipe
LDFLAGS ?=

PROGRAMS = dmesg

all: $(PROGRAMS)

%: %.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f $(PROGRAMS)

.PHONY: all clean
