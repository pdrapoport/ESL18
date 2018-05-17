/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#define PC

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "msgprocess.h"

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

void rs232_open(void)
{
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

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
int pc2drone(uint8_t *msg){
    int result;
    int msglen = cmd2len(msg[1]);
	//fprintf(stderr,"msglen: %d\n",msglen);
	do {
		result = (int) write(fd_RS232, msg, msglen);
	} while (result == 0);

	assert(result == msglen);
	return result;
}

void process_key(uint8_t c)
{
	uint8_t msg[MAXMSG];
	msg[0] = (char)c;
	uint8_t *payload;
	//fprintf(stderr,"%04x\n",msg[0]);
	switch(msg[0]){
		//motor control
		case 'd': //motor 0 up
		case 'c': //motor 0 down
		case 'f': //motor 1 up
		case 'v': //motor 1 down
		case 'g': //motor 2 up
		case 'b': //motor 2 down
		case 'h': //motor 3 up
		case 'n': //motor 4 down

		//lift, roll, pitch, yaw control
		case 'a': //lift up
		case 'z': //lift down
		case 'q': //yaw down
		case 'w': //yaw up
		case 'u': //yaw control p up
		case 'j': //yaw control p down
		case 'i': //roll, pitch control p1 up
		case 'k': //roll, pitch control p1 down
		case 'o': //roll, pitch control p2 up
		case 'l': //roll, pitch control p2 down
			payload = makePayload(PWKB, msg);
			break;

		//mode
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
			payload = makePayload(PWMODE, msg);
			break;
		
		//test case
		case 'y':
			msg[0] = 0x01;
			msg[1] = 0x56;
			msg[2] = 0x00;
			msg[3] = 0x2C;
			msg[4] = 0x01;
			msg[5] = 0x4D;
			msg[6] = 0x00;
			msg[7] = 0x16;
			payload = makePayload(PWMOV, msg);
			break;

		//arrow and escape
		case 27:
			term_getchar_nb();
			switch(term_getchar_nb()){
				case 65:
					//arrow up, pitch down
					msg[0] = 43;
					break;

				case 66:
					//arrow down, pitch up
					msg[0] = 95;
					break;

				case 68:
					//arrow left, roll up
					msg[0] = 40;
					break;
				
				case 67:
					//arrow right, roll down
					msg[0] = 41;
					break;

				default:
					//escape, abort
					msg[0] = 27;
					break;
			}
			payload = makePayload(PWKB, msg);
			break;
		
		default:
			msg[0] = '/';
			payload = makePayload(PWKB, msg);
			break;
	}

	pc2drone(payload);
	free(payload);
	//fprintf(stderr,"sent %c\n",msg[0]);
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	char	c;

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");
	initProtocol();
	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	for (;;)
	{
		if ((c = term_getchar_nb()) != -1){
			fprintf(stderr, "char: %c\n",c);
			process_key(c);
			
		}

		if ((c = rs232_getchar_nb()) != -1)
			term_putchar(c);

	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

