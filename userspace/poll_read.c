#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <string.h>

#define CH340_SET_BAUDRATE	1
#define CH340_GET_VENDORDATA	2

int main(int argc, char *argv[])
{
	struct pollfd pfd;
	char buf[64] = {'\0'};
	int ready;
	unsigned short dev_attached;

	if (argc < 2) {
		fprintf(stderr, "Usage: ./a.out /dev/usbTTL[0-9]\n");
		exit(-1);
	}

	pfd.fd = open(argv[1], O_RDONLY);
	if (pfd.fd < 0) {
		perror("Error ");
		exit(pfd.fd);
	}
	
	pfd.events = POLLIN | POLLRDNORM | POLLHUP;
	dev_attached = 1;
	while (dev_attached) {
		ready = poll(&pfd, 1, -1);
		if (ready == -1) {
			perror("poll: ");
			exit(EXIT_FAILURE);
		}
		if (pfd.revents != 0) {
			if (pfd.revents & (POLLIN | POLLRDNORM)) {
				size_t s = read(pfd.fd, buf, sizeof(buf));
				if(s == -1) {
					perror("read: ");
					exit(EXIT_FAILURE);
				}
				printf("%s", buf);
			} else if(pfd.revents & POLLHUP){
				printf("closing fd\n");
				if(close(pfd.fd) == -1) {
					perror("close");
					exit(EXIT_FAILURE);
				}
				dev_attached = 0;
			}
		}
		memset(buf, '\0', sizeof(buf));
	}
	return 0;
}
