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

int serial_device = 0;
int fd_RS232;
int axis[6];
int button[12];

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

int sendInput(int *axisData) {
	return 0;
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
	int 		fd;
	struct js_event js;
	time_t 		last_button_pressed;

	last_button_pressed = time(NULL);
	

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	term_puts("\nConnecting joystick...\n");


	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		term_puts("\nFailed to connect joystick\n");
		//exit(1);
	}

	fcntl(fd, F_SETFL, O_NONBLOCK);

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	for (;;)
	{
		while (read(fd, &js, sizeof(struct js_event)) ==
                                        sizeof(struct js_event))  {

                        /* register data
                         */
                        // fprintf(stderr,".");
                        switch(js.type & ~JS_EVENT_INIT) {
                                case JS_EVENT_BUTTON:
                                        button[js.number] = js.value;
                                        break;
                                case JS_EVENT_AXIS:
                                        axis[js.number] = js.value;
                                        break;
                        }
                }
		if ((c = sendInput(axis)) < 0) 
			perror("Error: failed to send input data to FCB\n");
		
		if ((c = term_getchar_nb()) != -1) 
			rs232_putchar(c);

		if ((c = rs232_getchar_nb()) != -1)
			term_putchar(c);
		
		if ((time(NULL) - last_button_pressed) > 0.015) {
			last_button_pressed = time(NULL);
			for (int i = 0; i < 10; ++i)
				if (button[i]) {
					char str[1];
					sprintf(str, "%d", i);
					rs232_putchar(str[0]);
				}
		}
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

