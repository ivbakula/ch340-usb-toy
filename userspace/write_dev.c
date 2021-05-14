#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>

#define CH340_SET_BAUDRATE	1

int main(int argc, char *argv[])
{
	int fd, retval;
	ssize_t w;
	char buffer[16] = "teuta";

	if (argc < 2) {
		fprintf(stderr, "Usage ./a.out /dev/usbTTL(n)\n");
		exit(-1);
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("Unable to open file ");
		exit(fd);
	}

	retval = ioctl(fd, CH340_SET_BAUDRATE, 115200);
	if (retval < 0) {
		perror("ioctl() ");
		exit(retval);
	}

	w = write(fd, buffer, sizeof(buffer));
	if (w < 0) {
		perror("write() ");
		exit(w);
	}

	close(fd);
	return 0;
}
