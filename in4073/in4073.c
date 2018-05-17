/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#define DRONE2PC
//#include "msg2payload.h"

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c){
	switch (c)
	{
		//motor control
		case 'd':
			ae[0] += 10;
			break;
		case 'c':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'f':
			ae[1] += 10;
			break;
		case 'v':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'g':
			ae[2] += 10;
			break;
		case 'b':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'h':
			ae[3] += 10;
			break;
		case 'n':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		
		//lift, roll, pitch, yaw control
		case 'a': 
			//lift up
			break;
		case 'z': 
			//lift down
			break;
		case 'q': 
			//yaw down
			break;
		case 'w': 
			//yaw up
			break;
		case 'u': 
			//yaw control p up
			break;
		case 'j': 
			//yaw control p down
			break;
		case 'i': 
			//roll, pitch control p1 up
			break;
		case 'k': 
			//roll, pitch control p1 down
			break;
		case 'o': 
			//roll, pitch control p2 up
			break;
		case 'l': 
			//roll, pitch control p2 down
			break;

		case '0':
			//mode 0
			break;
		case '1':
			//mode 1
			break;
		case '2':
			//mode 2
			break;
		case '3':
			//mode 3
			break;
		case '4':
			//mode 4
			break;
		case '5':
			//mode 5
			break;
		case '6':
			//mode 6
			break;
		case '7':
			//mode 7
			break;
		case '8':
			//mode 8
			break;
		
		
		case 43:
			//pitch down
			break;
		case 95:
			//pitch up
			break;
		case 40:
			//roll up
			break;
		case 41:
			//roll down
			break;
		case 27:
			demo_done = true;
			break;
		default:
			nrf_gpio_pin_toggle(RED);
			break;
	}
}

void processRecMsg(){
	if(recBuff != 0){
		uint8_t idCmd = receivedMsg[1].idCmd;
		int msglen = cmd2len(idCmd);
		uint8_t msg[MAXMSG];
		int j = 0;
		for(j= 0;j< msglen-ADDBYTES;j++){
			//printf("%04x ",receivedMsg[i].msg[j]),
			msg[j] = receivedMsg[1].msg[j];
		}

		switch(idCmd){
			case PWMODE:
				
				break;
			case PWMOV:
				//printf("PWMOV\n");
				changeMov(msg);
				break;
			case DWLOG:
				
				break;
			case DWMODE:
				
				break;
			case PRMODE:
				
				break;
			case PWKB:
				changeKbParam(msg);
				break;
			default:
				printf("ERROR\n");
				break;
		}
		slideRecMsg(1);
	}
	
}

void changeMode();
void changeMov(uint8_t *msg){
	/* int j = 0;
	int msglen = cmd2len(PWMOV);
	for(j = 0; j < msglen-ADDBYTES; printf("%04x ",msg[j]),j++);
	printf("\n"); */
	int16_t mot1, mot2, mot3, mot4;
	mot1 = (int16_t)combineByte(msg[0], msg[1]);
	mot2 = (int16_t)combineByte(msg[2], msg[3]);
	mot3 = (int16_t)combineByte(msg[4], msg[5]);
	mot4 = (int16_t)combineByte(msg[6], msg[7]);
	ae[0] = mot1;
	ae[1] = mot2;
	ae[2] = mot3;
	ae[3] = mot4;
}
void changeKbParam(uint8_t *msg){
	process_key((uint8_t)msg[0]);
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	ble_init();
	initProtocol();

	uint32_t counter = 0;
	demo_done = false;
	//uint16_t i = 0;

	while (!demo_done)
	{
		//if (rx_queue.count) process_key( dequeue(&rx_queue) );
		receivePkt();
		

		if (check_timer_flag()) 
		{
			
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			//printf("adc req\n");
			read_baro();
			//printf("read baro\n");
			processPkt();
			//printf("processpkt\n");
			//printf("test %d\n",i++);
			processRecMsg();
			//printf("processrecmsg\n");
			
			/* printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure); */

			clear_timer_flag();
			//printf("cleartimerflag\n");
		}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
