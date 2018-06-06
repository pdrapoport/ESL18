/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 **------------------------------------------------------------
 */

#define PC

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "msgprocess.h"

#include "joystick.h"

#define JS_DEV "/dev/input/by-id/usb-Logitech_Logitech_Extreme_3D-joystick"
#define JS_DEV_RES "/dev/input/by-id/usb-Microntek_USB_Joystick-joystick"

/*------------------------------------------------------------
 * console I/O
 **------------------------------------------------------------
 */
struct termios savetty;
FILE *fp;  //printing to a file just to check the filters

void    term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void    term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void    term_puts(char *s)
{
	fprintf(stderr, "char: %s\n",s);
}


void    term_putchar(char c)
{
	putc(c,stderr);
	fprintf(fp,"%c",c);  //printing to a file just to check the filters
}

int     term_getchar_nb()
{
	static unsigned char line [2];

	if (read(0,line,1)) // note: destructive read
		return (int) line[0];

	return -1;
}

int     term_getchar()
{
	int c;

	while ((c = term_getchar_nb()) == -1)
		;
	return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 **------------------------------------------------------------
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
int fd_RS232, js_fd;
fd_set set;
struct js_event js;

int axis[6];
int button[12];
struct timeval timeout;

void rs232_open(void)
{
	char            *name;
	int result;
	struct termios tty;

	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime
	printf("%d\n",fd_RS232);
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
	tty.c_cc[VTIME] = 0; // added timeout //0 for NON-BLOCKING MODE but a lot of keys are missed by the protocol

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */

	timeout.tv_sec = 0;
	timeout.tv_usec = 10000;

	FD_ZERO(&set);
	FD_SET(fd_RS232, &set);
}


void    rs232_close(void)
{
	int result;

	result = close(fd_RS232);
	assert (result==0);
}



int     rs232_getchar_nb()
{
	int result;
	unsigned char c;

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


int     rs232_getchar()
{
	int c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int     rs232_putchar(char c)
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

void process_joystick(uint8_t but){
	uint8_t msg[PWKBLEN - ADDBYTES];
	uint8_t *payload;
	switch(but) {
	case 0:
		msg[0] = '1';
		break;
	case 1:
		msg[0] = '0';
		break;
	case 2:
		msg[0] = '3';
		break;
	case 6:
		msg[0] = '2';
		break;
	case 7:
		msg[0] = '4';
		break;
	case 8:
		msg[0] = '5';
		break;
	case 9:
		msg[0] = '6';
		break;
	case 10:
		msg[0] = '7';
		break;
	case 11:
		msg[0] = '8';
		break;
	}
	payload = makePayload(PWKB, msg);
	pc2drone(payload);
	free(payload);
}

void process_key(uint8_t c)
{
	uint8_t msg[PWMOVLEN - ADDBYTES];
	msg[0] = (char)c;
	uint8_t *payload;
	//fprintf(stderr,"%04x\n",msg[0]);
	switch(msg[0]) {
	//motor control
	case 'd':         //motor 0 up
	case 'c':         //motor 0 down
	case 'f':         //motor 1 up
	case 'v':         //motor 1 down
	case 'g':         //motor 2 up
	case 'b':         //motor 2 down
	case 'h':         //motor 3 up
	case 'n':         //motor 4 down
	case 'm':         //b constant up
	case ',':         //b constant down
	case '.':         //d constant up
	case '/':         //d constant down

	//lift, roll, pitch, yaw control
	case 'a':         //lift up
	case 'z':         //lift down
	case 'q':         //yaw down
	case 'w':         //yaw up
	case 'u':         //yaw control p up
	case 'j':         //yaw control p down
	case 'i':         //roll, pitch control p1 up
	case 'k':         //roll, pitch control p1 down
	case 'o':         //roll, pitch control p2 up
	case 'l':         //roll, pitch control p2 down
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
	case 'p':
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
		switch(term_getchar_nb()) {
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

void sendLRPY(int16_t lift, int16_t roll, int16_t pitch, int16_t yaw){
	uint8_t *payload;
	uint8_t msg[PWMOVLEN - ADDBYTES];
	msg[0] = highByte(lift);
	msg[1] = lowByte(lift);
	msg[2] = highByte(roll);
	msg[3] = lowByte(roll);
	msg[4] = highByte(pitch);
	msg[5] = lowByte(pitch);
	msg[6] = highByte(yaw);
	msg[7] = lowByte(yaw);

	payload = makePayload(PWMOV, msg);
	pc2drone(payload);
	free(payload);
}

void checkJoystick() {
	while (read(js_fd,&js,sizeof(struct js_event)) ==
	       sizeof(struct js_event))  {
		switch(js.type & ~JS_EVENT_INIT) {
		case JS_EVENT_BUTTON:
			button[js.number] = js.value;
			if(button[js.number] == 1) process_joystick(js.number);
			break;
		case JS_EVENT_AXIS:
			axis[js.number] = js.value;
			break;
		}
		//if (errno != EAGAIN) {
		//	perror("\njs: error reading (EAGAIN)");
		//	exit (1);
		//}
	}
}

/*----------------------------------------------------------------
 * main -- execute terminal
 **----------------------------------------------------------------
 */

int main(int argc, char **argv)
{
	char c;
	struct timeval start;
	struct timeval tm1, tm2;
	long long diff;
	long long absdiff;
	bool exit = false;

	for (int i = 0; i < 4; ++i) {
		axis[i] = 0;
	}
	fp = fopen("file.txt","w");
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");
	initProtocol();

	term_puts("\nConnecting joystick...\n");


	//if ((js_fd = open(JS_DEV_RES, O_RDONLY)) < 0) {
	if ((js_fd = open(JS_DEV, O_RDONLY)) < 0) {
		term_puts("\nFailed to connect joystick\n");
		//exit(1);
	}
	gettimeofday(&tm1, NULL);
	gettimeofday(&start, NULL);

	fcntl(js_fd, F_SETFL, O_NONBLOCK);

	/* discard any incoming text
	 */
	//while ((c = rs232_getchar_nb()) != -1)
	//fputc(c,stderr);

	/* send & receive
	 */


	for (;; )
	{
		if ((c = term_getchar_nb()) != -1) {
			fprintf(stderr, "char: %c\n",c);
			process_key(c);
			if (c == 'e')
				exit = true;
		}
		gettimeofday(&tm2, NULL);
		diff = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;
		absdiff = 1000 * (tm2.tv_sec - start.tv_sec) + (tm2.tv_usec - start.tv_usec) / 1000;
		if (diff >= 20 && absdiff >= 3000) {
			gettimeofday(&tm1, NULL);
			//fprintf(stderr, "diff = %llu | absdiff = %llu\n", diff, absdiff);
			//checkJoystick();
			axis[3] = 0;
			sendLRPY(axis[0], axis[1], axis[2],((-1) * axis[3] / 2) + 16384);

			//printf()			// for (int i = 0; i < 4; ++i) {
			//      axis[i]++;
			// }
			//if ((c = term_getchar_nb()) != -1)
			//	rs232_putchar(c);
		}

		if ((c = rs232_getchar_nb()) != -1)
			term_putchar(c);
		if (exit)
			break;
	}

	term_exitio();
	rs232_close();
	close(js_fd);
	fclose(fp);

	term_puts("\n<exit>\n");

	return 0;
}
