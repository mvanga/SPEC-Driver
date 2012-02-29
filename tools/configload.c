/*
 * This file is part of the Simple PCI Express Carrier (SPEC) Linux driver.
 * More information available at: http://www.ohwr.org/projects/spec
 *
 * Copyright (c) 2011 Alessandro Rubini <rubini@gnudd.com>
 * Copyright (c) 2011 Manohar Vanga <manohar.vanga@cern.ch>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <spec.h>

int main(int argc, char *argv[])
{
	int fd;
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
		return -1;
	}

	fread(buffer, len, 1, file);
	fclose(file);

	/* load firmware */
	fw.fwlen = len;
	fw.data = buffer;
	
	fd = open(devname, O_RDWR);
	if (fd < 0)
		return fd;
	
	ioctl(fd, SPEC_LOAD_DEMO_CONFIG, &fw);

	free(buffer);

	close(fd);
	return 0;
}
