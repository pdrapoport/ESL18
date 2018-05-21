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

enum states {
  Safe_Mode,            // Mode 0
  Panic_Mode,           // Mode 1
  Manual_Mode,          // Mode 2
  Calibration_Mode,     // Mode 3
  Yaw_Mode,             // Mode 4
  Full_Mode,            // Mode 5
  Raw_Mode,             // Mode 6
  Height_Mode,          // Mode 7
  Wireless_Mode         // Mode 8
} state;

bool motors_off = true; // Update according to the readings
bool no_failure = true; // Update
bool calibration_done = false; // Update after the calibration is done
enum states state;

/*------------------------------------------------------------------
 * FSM FCB
 *------------------------------------------------------------------
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

void step(enum states *state, int c) {
  switch (*state) {

    // SAFE MODE
    case Safe_Mode:
        switch (c){
            case '1':
                *state = Panic_Mode;
                printf("PANIC!!!!!\n");
                panic_mode();
                break;

            case '2':
                if (motors_off && no_failure){
                    *state = Manual_Mode;
                    printf("Manual_Mode Selected\n");
                    // Call for Manual_Mode function required
                }
                break;

          case '3':
            if (motors_off && no_failure){
              *state = Calibration_Mode;
              printf("Calibration_Mode Selected\n");
              calibration_mode();
              printf("Calibration performed\n");
              calibration_done = true; //Change after the calibration is done
              if (motors_off && calibration_done){
                *state = Safe_Mode;
                printf("Safe_Mode Selected\n");
              }
            }
            break;

          case '4':
            if (motors_off && no_failure && calibration_done){
              *state = Yaw_Mode;
              printf("Yaw_Mode Selected\n");
              // Call for Yaw_Mode function required
            }
            break;

          case '5':
            if (motors_off && no_failure && calibration_done){
              *state = Full_Mode;
              printf("Full_Mode Selected\n");
              // Call for Full_Mode function required
            }
            break;

          case '6':
            if (motors_off && no_failure && calibration_done){
              *state = Raw_Mode;
              printf("Raw_Mode Selected\n");
              // Call for Raw_Mode function required
            }
            break;

          case '7':
            if (motors_off && no_failure && calibration_done){
              *state = Height_Mode;
              printf("Height_Mode Selected\n");
              // Call for Height_Mode function required
            }
            break;

          case '8':
            if (motors_off && no_failure && calibration_done){
              *state = Wireless_Mode;
              printf("Wireless_Mode Selected\n");
              // Call for Full_Mode function required
            }
            break;

          default:
            printf("No mode selected\n");
            break;
      }
      break;

      // MANUAL MODE
      case Manual_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && motors_off){
          *state = Safe_Mode;
          printf("Safe_Mode Selected\n");
          // Call for Safe_Mode function required
        }
        else{
          printf("No mode selected\n");
        }
        break;

      // YAW MODE
      case Yaw_Mode:
        if (!no_failure)
          *state = Panic_Mode;
          // Call for Panic_Mode function required
        else if (c == '0' && motors_off){
          *state = Safe_Mode;
          printf("Safe_Mode Selected\n");
          // Call for Safe_Mode function required
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
        else if (c == '0' && motors_off){
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
        else if (c == '0' && motors_off){
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
        else if (c == '0' && motors_off){
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
        else if (c == '0' && motors_off){
          *state = Safe_Mode;
          printf("Safe_Mode Selected\n");
          // Call for Safe_Mode function required
        }
        else{
          printf("No mode selected\n");
        }
        break;

	case Panic_Mode:
	  break;

	case Calibration_Mode:
	  break;
    }
}

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
		case '1':
			nrf_gpio_pin_toggle(RED);
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
            printf("Go to safe mode\n");
            break;
        case 'p':
            printf("%s\n",getCurrentState(state));
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
                     if (msglen == 0){
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
                     if(checkCRC(recChar, msglen)) {
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
    //printf("%10ld | ", get_time_us());
    printf("ax_0 = %6d | ax_1 = %6d | ax_2 = %6d | ax_3 = %6d\n", axis[0], axis[1], axis[2], axis[3]);
    if (state == Manual_Mode)
        manual_mode();
}

void changeKbParam(uint8_t *msg){
	process_key((uint8_t)msg[0]);
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */

void checkMotors() {
	uint8_t data[8];
	for(int i = 0; i < 8; ++i) {
		data[i] = dequeue(&rx_queue);
	}
	for(int i = 0; i < 4; ++i) {
		ae[i] = data[2*i] | (data[2*i+1] << 8);
	}
}

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
	state = Safe_Mode;
    phi_avg = 0, theta_avg = 0, psi_avg = 0;

	while (!demo_done)
	{
  		//if (rx_queue.count) process_key( dequeue(&rx_queue) );
  		processPkt();

  		/*if (rx_queue.count) {
  			checkMotors();
  		}*/
  		if (check_timer_flag())
  		{

  			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

  			adc_request_sample();
  			//printf("adc req\n");
  			read_baro();
  			//printf("read baro\n");

  			//printf("test %d\n",i++);
  			processRecMsg();
  			//printf("processrecmsg\n");

			//printf("%10ld | ", get_time_us());
			//printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			//printf("%6d %6d %6d | ", phi-phi_avg, theta-theta_avg, psi-psi_avg);
			//printf("%6d %6d %6d | ", sp, sq, sr);
			//printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

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
