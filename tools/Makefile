TOOLS=fwload

all: $(TOOLS)

fwload: fwload.c
	gcc -ggdb fwload.c -I../driver -o fwload

clean:
	rm -rf *.o $(TOOLS)
