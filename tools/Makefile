TOOLS=fwload.elf testread.elf testwrite.elf

CFLAGS=-I../driver -ggdb -Wall -Wextra

all: $(TOOLS)

%.elf: %.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf *.o $(TOOLS)
