#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <spec.h>

int main(int argc, char *argv[])
{
	int fd;
	int num;
	char devname[20];
	int val;
	int writeb = 0xdeadbeef;
	int readb;

	if (argc != 2) {
		printf("usage: %s <spec_num>\n", argv[0]);
		printf("spec_num is used to create /dev/spec<num>\n");
		exit(1);
	}

	num = atoi(argv[1]);
	sprintf(devname, "/dev/spec%d", num);

	fd = open(devname, O_RDWR);
	if (fd < 0) {
		printf("failed to open %s\n", devname);
		return fd;
	}
	
	lseek(fd, SPEC_BAR_4 + 0x854, SEEK_SET);
	read(fd, &val, sizeof(val));
	lseek(fd, SPEC_BAR_4 + 0x854, SEEK_SET);
	write(fd, &writeb, sizeof(writeb));
	lseek(fd, SPEC_BAR_4 + 0x854, SEEK_SET);
	read(fd, &readb, sizeof(readb));
	printf("Wrote: %08x, Read Back: %08x\n", writeb, readb);
	writeb = 0xa5a5a5a5;
	lseek(fd, SPEC_BAR_4 + 0x854, SEEK_SET);
	write(fd, &writeb, sizeof(writeb));
	lseek(fd, SPEC_BAR_4 + 0x854, SEEK_SET);
	read(fd, &readb, sizeof(readb));
	printf("Wrote: %08x, Read Back: %08x\n", writeb, readb);
	write(fd, &val, sizeof(val));

	close(fd);
	return 0;
}
