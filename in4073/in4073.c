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
 ****------------------------------------------------------------------
 */

#include "in4073.h"

#define DRONE2PC

enum states state;

void initValues(){
	b = 1;
	d = 10;
	p = 10;
	p1 = 50;
	p2 = 100;

	demo_done = false;
	state = Safe_Mode;
	sp_avg = 0;
	sq_avg = 0;
	sr_avg = 0;
	sax_avg = 0;
	say_avg = 0;
	saz_avg = 0;
	motors_off = true;
	calibration_done = false;
	no_failure = true;
	last_rec_pkt = get_time_us();
    flash_full = false;
    read_completed = false;
    start_logging = false;
    //diff = 0;
	for (int i = 0; i<4; i++)
		axis_offset[i] = 0;
}

/*------------------------------------------------------------------
 * FSM FCB
 ****------------------------------------------------------------------
 */

const char* getCurrentState(enum states state)
{
	switch (state)
	{
	case Safe_Mode: return "Safe_Mode";
	case Panic_Mode: return "Panic_Mode";
	case Manual_Mode: return "Manual_Mode";
	case Calibration_Mode: return "Calibration_Mode";
	case Yaw_Mode: return "Yaw_Mode";
	case Full_Mode: return "Full_Mode";
	case Raw_Mode: return "Raw_Mode";
	case Height_Mode: return "Height_Mode";
	case Wireless_Mode: return "Wireless_Mode";
	default: return "Error";
	}
}

bool checkJS(){
	return (axis[0] || axis[1] || axis[2] || axis[3] > 50);
}

bool checkMotor(){
	return (ae[0] || ae[1] || ae[2] || ae[3]);
}

void step(enum states *state, int c) {
	switch (*state) {
	// SAFE MODE
	case Safe_Mode:
		no_failure = true;
		switch (c) {
		case '2':
			if (!checkMotor() && !checkJS() && no_failure) {
				*state = Manual_Mode;
				printf("Manual_Mode Selected\n");
			}
			break;

		case '3':
			if (!checkMotor() && !checkJS() && no_failure) {
				*state = Calibration_Mode;
				printf("Calibration_Mode Selected\n");
			}
			break;

		case '4':
			if (!checkMotor() && !checkJS() && no_failure && calibration_done && DMP) {
				*state = Yaw_Mode;
				printf("Yaw_Mode Selected\n");
			}
            else if (!DMP){
                DMP = true;
                imu_init(DMP, 100);
                calibration_done = false;
                *state = Safe_Mode;
            }
			break;

		case '5':
			if (!checkMotor() && !checkJS() && no_failure && calibration_done && DMP) {
				*state = Full_Mode;
				printf("Full_Mode Selected\n");
			}
            else if (!DMP){
                DMP = true;
                imu_init(DMP, 100);
                calibration_done = false;
                *state = Safe_Mode;
            }
			break;

		case '6':
			if (!checkMotor() && !checkJS() && no_failure && calibration_done && !DMP) {
				*state = Raw_Mode;
				printf("Raw_Mode Selected\n");
			}
            else if (DMP){
                DMP = false;
                imu_init(DMP, 500);
                calibration_done = false;
                *state = Safe_Mode;
            }
			break;

		case '7':
			if (!checkMotor() && !checkJS() && no_failure && calibration_done) {
				*state = Height_Mode;
				printf("Height_Mode Selected\n");
			}
			break;

		case '8':
			if (!checkMotor() && !checkJS() && no_failure && calibration_done) {
				*state = Wireless_Mode;
				printf("Wireless_Mode Selected\n");
			}
			break;

		default:
			printf("No mode selected\n");
			break;
		}
		break;

	// MANUAL MODE
	case Manual_Mode:
		if (!no_failure) {
			*state = Panic_Mode;
			printf("Panic_Mode Selected\n");
		}
		else if (c == '0' && !checkMotor() && !checkJS()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
		}
		else{
			printf("No mode selected\n");
		}
		break;

	// YAW MODE
	case Yaw_Mode:
		if (!no_failure) {
			*state = Panic_Mode;
			printf("Panic_Mode Selected\n");
		}
		else if (c == '0' && !checkMotor()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
		}
		else{
			printf("No mode selected\n");
		}
		break;

	// FULL MODE
	case Full_Mode:
		if (!no_failure)
			*state = Panic_Mode;
		// Call for Panic_Mode function required
		else if (c == '0' && !checkMotor()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
			// Call for Safe_Mode function required
		}
		else{
			printf("No mode selected\n");
		}
		break;

	// RAW MODE
	case Raw_Mode:
		if (!no_failure)
			*state = Panic_Mode;
		// Call for Panic_Mode function required
		else if (c == '0' && !checkMotor()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
			// Call for Safe_Mode function required
		}
		else{
			printf("No mode selected\n");
		}
		break;

	// HEIGHT MODE
	case Height_Mode:
		if (!no_failure)
			*state = Panic_Mode;
		// Call for Panic_Mode function required
		else if (c == '0' && !checkMotor()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
			// Call for Safe_Mode function required
		}
		else{
			printf("No mode selected\n");
		}
		break;

	// WIRELESS MODE
	case Wireless_Mode:
		if (!no_failure)
			*state = Panic_Mode;
		// Call for Panic_Mode function required
		else if (c == '0' && !checkMotor()) {
			*state = Safe_Mode;
			printf("Safe_Mode Selected\n");
			// Call for Safe_Mode function required
		}
		else{
			printf("No mode selected\n");
		}
		break;

	case Panic_Mode:
		nrf_gpio_pin_toggle(RED);
		if (c == '0' && !checkMotor() && ((bat_volt > 1110) || (bat_volt < 650)))
			*state = Safe_Mode;
		break;

	case Calibration_Mode:
		break;
	}
}

void apply_offset_js_axis(){
	int32_t temp;
	for(int i = 0; i<4; i++) {
		temp = axis[i]+axis_offset[i];
		if (temp < -32767)
			axis[i] = -32767;
		else if (temp>32767)
			axis[i] = 32767;
		else
			axis[i] = temp;
	}
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 ****------------------------------------------------------------------
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
	case 'm':
		//d++;
		break;
	case ',':
		//d--;
		//if(d < 1) d = 1;
		break;
	case '.':
		//b++;
		break;
	case '/':
		//b--;
		//if(b < 1) b = 1;
		break;

	//lift, roll, pitch, yaw control
	case 'a':
		//lift up
		if (axis_offset[3]<31767)
			axis_offset[3] += 1000;
		break;
	case 'z':
		//lift down
		if (axis_offset[3]>-31767)
			axis_offset[3] -= 1000;
		break;
	case 'q':
		//yaw down
		if (axis_offset[2]> -31767)
			axis_offset[2] -= 1000;
		break;
	case 'w':
		//yaw up
		if (axis_offset[2]<31767)
			axis_offset[2] += 1000;
		break;
	case 'u':
		//yaw control p up
		p+=10;
		break;
	case 'j':
		//yaw control p down
		if (p > 10) p-=10;
		break;
	case 'i':
		//roll, pitch control p1 up
		p1+=1;
		break;
	case 'k':
		//roll, pitch control p1 down
		if(p1 > 0) p1-=1;
		break;
	case 'o':
		//roll, pitch control p2 up
		p2+=1;
		break;
	case 'l':
		//roll, pitch control p2 down
		if(p2 > 1) p2-=1;
		break;
	case 43:
		//pitch down
		if (axis_offset[1]<31767)
			axis_offset[1] += 1000;
		break;
	case 95:
		//pitch up
		if (axis_offset[1]> -31767)
			axis_offset[1] -= 1000;
		break;
	case 40:
		//roll up
		if (axis_offset[0]<31767)
			axis_offset[0] += 1000;
		break;
	case 41:
		//roll down
		if (axis_offset[0]> -31767)
			axis_offset[0] -= 1000;
		break;
	case 27:
		demo_done = true;
		break;

	//modes
	case '1':
		nrf_gpio_pin_toggle(RED);
		no_failure = false;
		step(&state,'1');
		break;
	case '2':
		step(&state,'2');
		break;
	case '3':
		step(&state,'3');
		break;
	case '4':
		step(&state,'4');
		break;
	case '5':
		step(&state,'5');
		break;
	case '6':
		step(&state,'6');
		break;
	case '7':
		step(&state,'7');
		break;
	case '8':
		step(&state,'8');
		break;
	case '0':
		step(&state,'0');
		break;
	case 'p':
		//printf("%s\n",getCurrentState(state));
        if (!flash_full){
            printf("STARTING LOGGING\n");
            start_logging = true;
        }
		break;
	default:
		nrf_gpio_pin_toggle(RED);
		break;
	}
}


// Author: Vincent Bejach
/* Implement the FSM defined for the communication protocol.
 * Reads from the global variable recChar, and remove part of its content when a packet is done being processed or when some bytes are thrown away.
 * Outputs the message of the packet being processed in the global receivedMsg array. The fnished processing is indicated by the flag messageComplete being set to true.
 */
void processPkt() {
	receivePkt();
	while (readIndex < buffCount) {
		switch (packState) {
		case wait:
			//printf("\nWAIT!\n");
			//printf("READ %02X\n", recChar[readIndex]);
			if (recChar[readIndex] == STARTBYTE) {
				//printf("START\n");
				++readIndex;
				packState = first_byte_received;
			}
			else {
				slideMsg(1);
			}
			break;
		case first_byte_received:
			msglen = cmd2len(recChar[readIndex++]);
			packState = receiveMsg;
			if (msglen == 0) {
				slideMsg(1);
				packState = wait;
			}
			//printf("\nFIRST!\n");
			break;
		case receiveMsg:
			if (readIndex < msglen - 1) {
				++readIndex;
			}
			else {
				packState = CRC_Check;
			}
			//printf("\nRECV\n");
			break;
		case CRC_Check:
			if (checkCRC(recChar, msglen)) {
				receivedMsg[++recBuff] = getPayload(msglen);
				//printf("\nRECEIVED MESSAGE: ");
				// for (int k = 0; k < msglen; ++k) {
				//     printf("%02X ", recChar[k]);
				// }
				// printf("\n");
				processRecMsg();
				// if (buffCount > 13) {
				//     printf("oldStartByte: %02X\n", recChar[13]);
				// }
				// printf("%d/%d -> ", readIndex, buffCount);
				slideMsg(msglen);
				// printf("%d/%d\n", readIndex, buffCount);
				// if (buffCount > 0) {
				//     printf("currentStartByte: %02X\n", recChar[0]);
				// }
				packState = wait;
			}
			else {
				//printf("\nCRC FAIL!\n");
				slideMsg(1);
				packState = wait;
			}
			// printf("\nCRC!\n");
			break;
		case panic:
			//TODO: Fall on the floor and cry "AAAAAAAAAAAAAAAAAAAAAAAAAAA!!!"
			//panic_on = true;
			break;
		default:
			packState = panic;
		}
	}
}

void processRecMsg(){
	if(recBuff != 0) {
		uint8_t idCmd = receivedMsg[1].idCmd;
		int msglen = cmd2len(idCmd);
		uint8_t msg[MAXMSG];
		int j = 0;
		for(j= 0; j< msglen-ADDBYTES; j++) {
			//printf("%04x ",receivedMsg[i].msg[j]),
			msg[j] = receivedMsg[1].msg[j];
		}

		switch(idCmd) {
		case PWMODE:
			//printf("PWMODE\n");
			changeMode(msg);
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
		last_rec_pkt = get_time_us();
		slideRecMsg(1);
	}

}

void changeMode(uint8_t *msg){
	process_key((uint8_t)msg[0]);
}

void changeMov(uint8_t *msg){
	// int j = 0;
	// int msglen = cmd2len(PWMOV);
	// for(j = 0; j < msglen-ADDBYTES; printf("%04x ",msg[j]),j++);
	// printf("\n");
	//int16_t mot1, mot2, mot3, mot4;

	axis[0] = (int16_t)combineByte(msg[0], msg[1]);
	axis[1] = (int16_t)combineByte(msg[2], msg[3]);
	axis[2] = (int16_t)combineByte(msg[4], msg[5]);
	axis[3] = (int16_t)combineByte(msg[6], msg[7]);
	apply_offset_js_axis();
}

void changeKbParam(uint8_t *msg){
	process_key((uint8_t)msg[0]);
}

void checkMotors() {
	uint8_t data[8];
	for(int i = 0; i < 8; ++i) {
		data[i] = dequeue(&rx_queue);
	}
	for(int i = 0; i < 4; ++i) {
		ae[i] = data[2*i] | (data[2*i+1] << 8);
	}
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 ****------------------------------------------------------------------
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
	initValues();
	//dmp_enable_gyro_cal(0); //Disables the calibration of the gyro data in the DMP
	uint8_t packet[42];
    bool connection_lost = false;
	long connection_start_time = get_time_us() + 2350000;
	//uint32_t tm2, tm1;
	uint32_t counter = 0;
    DMP = true;
	//tm1 = get_time_us();
	while (!demo_done)
	{
        connection_lost = false;
        //if (rx_queue.count) process_key( dequeue(&rx_queue) );
        processPkt();


        if (check_timer_flag()) //50 ms
        {
            if (counter++%10 == 0) nrf_gpio_pin_toggle(BLUE);  //Blink blue LED every 0.5 secs

    		adc_request_sample();
    		if (bat_volt < 500) {
    		    //state = Panic_Mode;
    		}
    		read_baro();
    		processRecMsg();

    		if((get_time_us() > connection_start_time) && (get_time_us() - last_rec_pkt > 100000) && !connection_lost) {
    			//state = Panic_Mode;
    			nrf_gpio_pin_toggle(YELLOW);
    			connection_lost = true;
    		}

            // Prinf data in terminal every 100ms
            //This will be replaced by telemetry
            if (counter%2 == 0) {
                printf("%10ld | %2d | ", get_time_us(), state);
                printf("%5d | %3d %3d %3d %3d | ",axis[3],ae[0],ae[1],ae[2],ae[3]);
                if (DMP){
                    printf("%6d %6d %6d | ", phi-phi_avg, theta-theta_avg, psi-psi_avg);
                    printf("%6d %6d %6d | ", sp-sp_avg, sq-sq_avg, sr-sr_avg);
                    printf("%6d %6d %6d | ", sax-sax_avg, say-say_avg, saz-saz_avg);

                }
                else{
                    printf("%6d %6d %6d | ", f_d.phi_kalman, f_d.theta_kalman, psi-psi_avg);
                    printf("%6d %6d %6d | ", sp-sp_avg, sq-sq_avg, f_d.sr_filtered);
                    printf("%6d %6d %6d | ", f_d.sax_filtered, f_d.say_filtered, saz-saz_avg);
                }
                printf("%4d | %4ld | %6ld | %2d | %2d | %2d | %2d | %2d \n", bat_volt, temperature, pressure, b, d, p, p1, p2);
            }
            clear_timer_flag();

		}

		if (check_sensor_int_flag())
		{
            //tm1 = get_time_us();
			// Run Filters
			if (!DMP){
                get_raw_sensor_data();

				filter = say_butterworth;
				f_d.say_filtered = butterworth_filter(say-say_avg,&filter);
				filter = kalman_phi;
				f_d.phi_kalman = kalman_filter(f_d.say_filtered,sp-sp_avg,&filter);

				filter = sax_butterworth;
				f_d.sax_filtered = butterworth_filter(sax-sax_avg,&filter);
				filter = kalman_theta;
				f_d.theta_kalman = kalman_filter(f_d.sax_filtered,sq-sq_avg,&filter);

				filter = sr_butterworth;
				f_d.sr_filtered = butterworth_filter(sr-sr_avg,&filter);
			}
            else
                get_dmp_data();

			run_filters_and_control(&state);
            //tm2 = get_time_us();
            //diff = tm2-tm1;

            if (!flash_full && start_logging)
                write_packet_flash();
		}
	}

    while(!read_completed){
        read_packet_flash(packet);
        print_to_terminal(packet);
        nrf_delay_ms(10);
    }

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
