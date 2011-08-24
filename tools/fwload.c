#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <spec.h>

int main(int argc, char *argv[])
{
	int fd;
	int ret;
	struct spec_fw fw;
	FILE *file;
	int len;
	char *buffer;
	int num;
	char devname[20];

	if (argc != 3) {
		printf("usage: %s <spec_num> <firmware>\n", argv[0]);
		printf("spec_num is used to create /dev/spec<num>\n");
		exit(1);
	}

	num = atoi(argv[1]);
	sprintf(devname, "/dev/spec%d", num);

	/* read in the firmware file */
	file = fopen(argv[2], "rw");

	fseek(file, 0, SEEK_END);
	len = ftell(file);
	fseek(file, 0, SEEK_SET);

	buffer = (char *)malloc(len);
	if (!buffer) {
		fprintf(stderr, "Memory error!");
		fclose(file);
		return;
	}

	fread(buffer, len, 1, file);
	fclose(file);

	/* load firmware */
	fw.fwlen = len;
	fw.data = buffer;
	
	fd = open(devname, O_RDWR);
	if (fd < 0)
		return fd;
	
	ioctl(fd, SPEC_LOADFW, &fw);

	free(buffer);

	close(fd);
	return ret;
}
