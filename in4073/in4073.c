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

enum states state;

void initValues(){
  b = 1;
  d = 10;
  p = 1610;
  p1 = 50;
  p2 = 110;

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
    connection_lost = false;

    for (int i = 0; i < 4; i++)
        axis_offset[i] = 0;
}

/*------------------------------------------------------------------
 * FSM FCB
 *------------------------------------------------------------------
 */


bool checkJS(){
    return (axis[0] || axis[1] || axis[2] || axis[3] > 50);
}

bool checkMotor() {
    return (ae[0] || ae[1] || ae[2] || ae[3]);
}

void step(enum states * state, int c) {
    switch ( * state) {
        // SAFE MODE
        case Safe_Mode:
            no_failure = true;
            switch (c){
                case '2':
                    if (!checkMotor() && !checkJS() && no_failure){
                        *state = Manual_Mode;
                    }
                    break;

                case '3':
                    if (!checkMotor() && !checkJS() && no_failure){
                        *state = Calibration_Mode;
                    }
                    break;

                case '4':
                    if (!checkMotor() && !checkJS() && no_failure && calibration_done){
                        *state = Yaw_Mode;
                    }
                    break;

                case '5':
                    if (!checkMotor() && !checkJS() && no_failure && calibration_done){
                        *state = Full_Mode;
                    }
                    break;

                case '6':
                    if (!checkMotor() && !checkJS() && no_failure && calibration_done){
                        *state = Raw_Mode;
                    }
                    break;

                case '7':
                    if (!checkMotor() && !checkJS() && no_failure && calibration_done){
                        *state = Height_Mode;
                    }
                    break;

                case '8':
                    if (!checkMotor() && !checkJS() && no_failure && calibration_done){
                        *state = Wireless_Mode;
                    }
                    break;

                default:
                    break;
            }
            break;

        // MANUAL MODE
        case Manual_Mode:
            if (!no_failure){
                *state = Panic_Mode;
            }
            else if (c == '0' && !checkMotor() && !checkJS()){
                *state = Safe_Mode;
            }
            else{
            }
            break;

        // YAW MODE
        case Yaw_Mode:
            if (!no_failure){
                *state = Panic_Mode;
            }
            else if (c == '0' && !checkMotor()){
                *state = Safe_Mode;
            }
            else{
            }
            break;

      // FULL MODE
      case Full_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && !checkMotor()){
          *state = Safe_Mode;
          // Call for Safe_Mode function required
        }
        else{
        }
        break;

      // RAW MODE
      case Raw_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && !checkMotor()){
          *state = Safe_Mode;
          // Call for Safe_Mode function required
        }
        else{
        }
        break;

      // HEIGHT MODE
      case Height_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && !checkMotor()){
          *state = Safe_Mode;
          // Call for Safe_Mode function required
        }
        else{
        }
        break;

      // WIRELESS MODE
      case Wireless_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && !checkMotor()){
          *state = Safe_Mode;
          // Call for Safe_Mode function required
        }
        else{
        }
        break;

        case Panic_Mode:
            nrf_gpio_pin_toggle(RED);
            if (c == '0' && !checkMotor() && ((bat_volt > 1000) || (bat_volt < 650)))
                * state = Safe_Mode;
            break;

        case Calibration_Mode:
            break;
    }
}

void apply_offset_js_axis() {
    int32_t temp;
    for (int i = 0; i < 4; i++) {
        temp = axis[i] + axis_offset[i];
        if (temp < -32767)
            axis[i] = -32767;
        else if (temp > 32767)
            axis[i] = 32767;
        else
            axis[i] = temp;
    }
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c) {
    switch (c) {
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
            d++;
            break;
        case ',':
            d--;
            if (d < 1) d = 1;
            break;
        case '.':
            b++;
            break;
        case '/':
            b--;
            if (b < 1) b = 1;
            break;

        //lift, roll, pitch, yaw control
        case 'a':
            //lift up
            if (axis_offset[3] < 31767)
                axis_offset[3] += 1000;
            break;
        case 'z':
            //lift down
            if (axis_offset[3] > -31767)
                axis_offset[3] -= 1000;
            break;
        case 'q':
            //yaw down
            if (axis_offset[2] > -31767)
                axis_offset[2] -= 1000;
            break;
        case 'w':
            //yaw up
            if (axis_offset[2] < 31767)
                axis_offset[2] += 1000;
            break;
        case 'u':
            //yaw control p up
            p += 10;
            break;
        case 'j':
            //yaw control p down
            if (p > 10) p -= 10;
            break;
        case 'i':
            //roll, pitch control p1 up
            p1 += 1;
            break;
        case 'k':
            //roll, pitch control p1 down
            if (p1 > 0) p1 -= 1;
            break;
        case 'o':
            //roll, pitch control p2 up
            p2 += 1;
            break;
        case 'l':
            //roll, pitch control p2 down
            if (p2 > 1) p2 -= 1;
            break;
        case 43:
            //pitch down
            if (axis_offset[0] < 31767)
                axis_offset[0] += 1000;
            break;
        case 95:
            //pitch up
            if (axis_offset[0] > -31767)
                axis_offset[0] -= 1000;
            break;
        case 40:
            //roll up
            if (axis_offset[1] < 31767)
                axis_offset[1] += 1000;
            break;
        case 41:
            //roll down
            if (axis_offset[1] > -31767)
                axis_offset[1] -= 1000;
            break;
        case 27:
            demo_done = true;
            break;

        //modes
        case '1':
            nrf_gpio_pin_toggle(RED);
            no_failure = false;
            step( & state, '1');
            break;
        case '2':
            step( & state, '2');
            break;
        case '3':
            step( & state, '3');
            break;
        case '4':
            step( & state, '4');
            break;
        case '5':
            step( & state, '5');
            break;
        case '6':
            step( & state, '6');
            break;
        case '7':
            step( & state, '7');
            break;
        case '8':
            step( & state, '8');
            break;
        case '0':
            step( & state, '0');
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
          sendErrMsg(3);
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

void sendTelemetryPacket() {
    uint8_t packet[42];
    uint32_t timestamp = get_time_us();
    packet[0] = fourthByte(timestamp);
    packet[1] = thirdByte(timestamp);
    packet[2] = secondByte(timestamp);
    packet[3] = lowByte(timestamp);
    packet[4] = state;
    packet[5] = calibration_done;
    for(int i = 0; i < 4; ++i) {
        packet[6 + 2 * i] = highByte(ae[i]);
        packet[6 + 2 * i + 1] = lowByte(ae[i]);
    }
    packet[14] = highByte(phi - phi_avg);
    packet[15] = lowByte(phi - phi_avg);
    packet[16] = highByte(theta - theta_avg);
    packet[17] = lowByte(theta - theta_avg);
    packet[18] = highByte(psi - psi_avg);
    packet[19] = lowByte(psi - psi_avg);
    packet[20] = highByte(sp - sp_avg);
    packet[21] = lowByte(sp - sp_avg);
    packet[22] = highByte(sq - sq_avg);
    packet[23] = lowByte(sq - sq_avg);
    packet[24] = highByte(sr - sr_avg);
    packet[25] = lowByte(sr - sr_avg);
    packet[26] = highByte(sax - sax_avg);
    packet[27] = lowByte(sax - sax_avg);
    packet[28] = highByte(say - say_avg);
    packet[29] = lowByte(say - say_avg);
    packet[30] = highByte(saz - saz_avg);
    packet[31] = lowByte(saz - saz_avg);
    packet[32] = highByte(bat_volt);
    packet[33] = lowByte(bat_volt);
    packet[34] = fourthByte(0);
    packet[35] = thirdByte(0);
    packet[36] = highByte(p1);
    packet[37] = lowByte(p1);
    packet[38] = fourthByte(0);
    packet[39] = thirdByte(0);
    packet[40] = highByte(p2);
    packet[41] = lowByte(p2);
    uint8_t *msg = makePayload(DWTEL, packet);
    for(int i = 0; i < 48; ++i) {
        uart_put(msg[i]);
    }
    free(msg);
}

void sendErrMsg(uint8_t errNum){
    uint8_t packet[1];
    packet[0] = errNum;
    msglen = cmd2len(DWERR);
    uint8_t *msg = makePayload(DWERR, packet);
    for(int i = 0; i < 6; ++i){
        uart_put(msg[i]);
    }
    free(msg);
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */

int main(void)
{
    bool connection_lost = false;
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


	long connection_start_time = get_time_us()+ 1000000;

    uint32_t counter = 0;
    uint8_t bat_counter = 0;
    uint32_t sum_bat_volt = 1060*8;
    uint32_t lts = get_time_us();

    //tm1 = get_time_us();
	while (!demo_done)
	{
        connection_lost = false;
  		processPkt();

  		/*if (rx_queue.count) {
  			checkMotors();
  		}*/
  		if (check_timer_flag()) //40 ms
  		{
            if (counter++ % 20 == 0) nrf_gpio_pin_toggle(BLUE);

            adc_request_sample();

            //Battery check function
            sum_bat_volt += bat_volt;
            if(!(bat_counter++ % 8)){
                sum_bat_volt = sum_bat_volt>>3;
                if(sum_bat_volt < 550){
                    state = Panic_Mode;
                    sendErrMsg(4);
                }
                sum_bat_volt = 0;
                }

            //processRecMsg();

            if ((get_time_us() > connection_start_time) && (get_time_us() - last_rec_pkt > 100000) && !connection_lost) {
                state = Panic_Mode;
                nrf_gpio_pin_toggle(YELLOW);
                connection_lost = true;
            }
  			read_baro();
  			//printf("read baro\n");

  			//printf("test %d\n",i++);
  			//printf("processrecmsg\n");
			//printf("%10ld | %2d | ", get_time_us(), state);
			//printf("%5d | %3d %3d %3d %3d | ",axis[3],ae[0],ae[1],ae[2],ae[3]);
			//printf("%6d %6d %6d | ", phi-phi_avg, theta-theta_avg, psi-psi_avg);
			//printf("%6d %6d %6d | ", sp-sp_avg, sq-sq_avg, sr-sr_avg);
            //printf("%6d %6d %6d | ", sax-sax_avg, say-say_avg, saz-saz_avg);
			//printf("%4d | %4ld | %6ld | %2d | %2d | %2d \n", bat_volt, temperature, pressure, b, d, p);
  			clear_timer_flag();
            // printf("%10ld | %2d | ", get_time_us(), state);
            // printf("%5d | %3d %3d %3d %3d | ", axis[3], ae[0], ae[1], ae[2], ae[3]);
            // printf("%6d %6d %6d | ", phi - phi_avg, theta - theta_avg, psi - psi_avg);
            // printf("%6d %6d %6d | ", sp - sp_avg, sq - sq_avg, sr - sr_avg);
            // printf("%6d %6d %6d | ", sax - sax_avg, say - say_avg, saz - saz_avg);
            // printf("%4d | %4ld | %6ld | %2d | %2d | %2d | %2d | %2d \n", bat_volt, temperature, pressure, b, d, p, p1, p2);

        }

        if ((get_time_us() - lts) > 100000) {
            sendTelemetryPacket();
            lts = get_time_us();
        }

  		if (check_sensor_int_flag())
  		{
            //tm2 = get_time_us();
            //diff = (tm2 - tm1)/ 1000;
            //printf("%4ld \n ", diff);
            //tm1 = tm2;
  			get_dmp_data();
  			run_filters_and_control(&state);
  		}


  	}
	nrf_delay_ms(100);

    NVIC_SystemReset();
}