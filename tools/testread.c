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
	
	lseek(fd, SPEC_BAR_4 + 0x0, SEEK_SET);
	read(fd, &val, sizeof(val));
	printf("Vendor: %04x\n", val & 0xffff);
	printf("Device: %04x\n", (val>>16) & 0xffff);

	close(fd);
	return 0;
}
