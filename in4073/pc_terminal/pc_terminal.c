/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

#include "joystick.h"

#define JS_DEV "/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-joystick"
#define JS_DEV_RES "/dev/input/by-id/usb-Microntek_USB_Joystick-joystick"

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>

int serial_device = 0;
int fd_RS232;
fd_set set;
short axis[4];
struct timeval timeout;

void rs232_open(void)
{
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ESLBOARD", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime
	assert(fd_RS232>=0);

  	result = isatty(fd_RS232);
  	assert(result == 1);

  	name = ttyname(fd_RS232);
  	assert(name != 0);

  	result = tcgetattr(fd_RS232, &tty);
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // added timeout

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */

	timeout.tv_sec = 0;
	timeout.tv_usec = 10000;

	FD_ZERO(&set);
	FD_SET(fd_RS232, &set);
}


void 	rs232_close(void)
{
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}



int	rs232_getchar_nb()
{
	int 		result;
	unsigned char 	c;
		
	int rv = select(fd_RS232 + 1, &set, &set, &set, &timeout);
	if (rv == -1) {
		perror("select");
		result = 0;
	}
	else if (FD_ISSET(fd_RS232, &set))
		result = 0;
	else 
		result = read(fd_RS232, &c, 1);

	if (result == 0)
		return -1;
	else
	{
		assert(result == 1);
		return (int) c;
	}
}


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int 	rs232_putchar(char c)
{
	int result;

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);

	assert(result == 1);
	return result;
}


//Pavel Rapoport
//Packing input from joystick and sending it to FCB

int send_axis_input() {
	uint8_t arr[8];
	arr[0] = axis[0] & 0xFF;
	arr[1] = (axis[0] >> 8) & 0xFF;
	arr[2] = axis[1] & 0xFF;
	arr[3] = (axis[1] >> 8) & 0xFF;
	arr[4] = axis[2] & 0xFF;
	arr[5] = (axis[2] >> 8) & 0xFF;
	arr[6] = axis[3] & 0xFF;
	arr[7] = (axis[3] >> 8) & 0xFF;
	for (int i = 0; i < 8; ++i)
		rs232_putchar(arr[i]);
	return 0;
}

int find_sqrt(int arg) {
        int result = 200;
        if (result * result < arg)
                while (result * result < arg)
                        ++result;
        else
                while (result * result > arg)
                        --result;
        return result;
}




//Pavel Rapoport
//Main function edited for regular polling of joystick for input data

/*
 *----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	char		c;
	int 		js_fd;
	struct js_event js;
	struct timeval 	start;
	struct timeval	tm1, tm2;

	for (int i = 0; i < 4; ++i) {
		axis[i] = 0;
	}

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	term_puts("\nConnecting joystick...\n");


	if ((js_fd = open(JS_DEV_RES, O_RDONLY)) < 0) {
		term_puts("\nFailed to connect joystick\n");
		//exit(1);
	}
	gettimeofday(&tm1, NULL);
	gettimeofday(&start, NULL);

	fcntl(js_fd, F_SETFL, O_NONBLOCK);

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	long long diff;

	int ae[4];
	int b = 1;
	int d = 1;
	for (;;)
	{
		gettimeofday(&tm2, NULL);
		diff = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;
		if (diff > 15) {
			int intrCount = 0;
			gettimeofday(&tm1, NULL);
			diff = 1000 * (tm2.tv_sec - start.tv_sec) + (tm2.tv_usec - start.tv_usec) / 1000;
			//fprintf(stderr, "%d ", diff);
			while (read(js_fd, &js, sizeof(struct js_event)) ==
                                        sizeof(struct js_event))  {
                        	switch(js.type & ~JS_EVENT_INIT) {
                                	case JS_EVENT_BUTTON:
						if (js.value) {
							//TODO: send command for this button to FCB
						}
                                        	break;
                                	case JS_EVENT_AXIS:
						//if (js.number < 4)
                                        		axis[js.number] = js.value;
                                        	break;
                        	}
				++intrCount;
                	}
			//fprintf(stderr, "%d\t%d\t%d\t%d\t%d\n", intrCount, axis[0], axis[1], axis[2], axis[3]);
			send_axis_input();

			ae[0] = find_sqrt((b*axis[1] - d*axis[3] - b*axis[2])/(4*b*d));   // A
			ae[1] = find_sqrt((b*axis[2] - d*axis[3] - 2*d*axis[0])/(4*b*d)); // B
 			ae[2] = find_sqrt((-2*d*axis[1] - d*axis[3] - b*axis[2])/(4*b*d)); // C
			ae[3] = find_sqrt((b*axis[2] - d*axis[3] + 2*d*axis[0])/(4*b*d)); // D

			fprintf(stderr, "%d\t%d\t%d\t%d\n", ae[0], ae[1], ae[2], ae[3]);

		//if ((c = term_getchar_nb()) != -1) 
		//	rs232_putchar(c);
		}

		//if ((c = rs232_getchar_nb()) != -1)
		//	term_putchar(c);
	}

	term_exitio();
	rs232_close();
	close(js_fd);
	term_puts("\n<exit>\n");

	return 0;
}

