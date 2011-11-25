TOOLS=fwload testread testwrite configload

CFLAGS=-I../driver -I../../wishbone/simulation/include -ggdb -Wall -Wextra

all: $(TOOLS)

%.elf: %.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf *.o $(TOOLS)
