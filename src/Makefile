CFLAGS=-Werror -Wall -Os

all: xvcd

xvcd: serial_jtag.o xvcd.o
	$(CC) -o $@ $^ $(CFLAGS)

test: serial_jtag.o test.o
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -Rf *.o
	
mrproper: clean
	rm -f xvcd
	rm -f test