/*------------------------------------------------------------------
 *  in4073.c -- drone main file
 *  Modified from in4073.c by I. Protonotarios
 *
 * 	Group 7:
 *  - Pavel Rapoport
 * 	- Antonio Rueda
 * 	- Haris Suwignyo
 * 	- Vincent Bejach
 *
 * 	TU Delft
 *
 *  June 2018
 *
 *****------------------------------------------------------------------
 */

#include "in4073.h"

#define DRONE2PC

enum states state;

// Author: Vincent Bejach
// Initialize all the variables value
void initValues() {
    b = 1;
    d = 10;
    p = 1610;
    p1 = 50;
    p2 = 110;
    flash_full = false;
    read_completed = false;
    start_logging = false;

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


// Author: Haris Suwignyo
// Function to check if the joystick is in neutral position. Return true if joystick is not in neutral position, and vice versa
bool checkJS() {
    return (axis[0] || axis[1] || axis[2] || axis[3] > 50);
}

// Author: Haris Suwignyo
// Function to check if the motor is spinning. Return true if motor is spinning, and vice versa
bool checkMotor() {
    return (ae[0] || ae[1] || ae[2] || ae[3]);
}

// Author: Antonio Rueda
// Function to implement drone state's FSM and also some feature check
void step(enum states *state, int c) {
    switch (*state) {
    // SAFE MODE
		case Safe_Mode:
			no_failure = true;
			switch (c) {
				case '2':
					if (!checkMotor() && !checkJS() && no_failure) {
						*state = Manual_Mode;
					}
					break;

				case '3':
					if (!checkMotor() && !checkJS() && no_failure) {
						*state = Calibration_Mode;
					}
					break;

				case '4':
					if (!checkMotor() && !checkJS() && no_failure && calibration_done && DMP) {
						*state = Yaw_Mode;
						printf("Yaw_Mode Selected\n");
					}
					else if (!DMP) {
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
					else if (!DMP) {
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
					else if (DMP) {
						DMP = false;
						imu_init(DMP, 500);
						calibration_done = false;
						*state = Safe_Mode;
					}
					break;

				case '7':
					if (!checkMotor() && !checkJS() && no_failure && calibration_done) {
						*state = Height_Mode;
					}
					break;

				case '8':
					if (!checkMotor() && !checkJS() && no_failure && calibration_done) {
						*state = Wireless_Mode;
					}
					break;

				default:
					break;
			}
			break;

		// MANUAL MODE
		case Manual_Mode:
			if (!no_failure) {
				*state = Panic_Mode;
			}
			else if (c == '0' && !checkMotor() && !checkJS()) {
				*state = Safe_Mode;
			}
			break;

		// YAW MODE
		case Yaw_Mode:
			if (!no_failure) {
				*state = Panic_Mode;
			}
			else if (c == '0' && !checkMotor()) {
				*state = Safe_Mode;
			}
			break;

		// FULL MODE
		case Full_Mode:
			if (!no_failure)
				*state = Panic_Mode;
			// Call for Panic_Mode function required
			else if (c == '0' && !checkMotor()) {
				*state = Safe_Mode;
				// Call for Safe_Mode function required
			}
			break;

		// RAW MODE
		case Raw_Mode:
			if (!no_failure)
				*state = Panic_Mode;
			// Call for Panic_Mode function required
			else if (c == '0' && !checkMotor()) {
				*state = Safe_Mode;
				// Call for Safe_Mode function required
			}
			break;

		// HEIGHT MODE
		case Height_Mode:
			if (!no_failure)
				*state = Panic_Mode;
			// Call for Panic_Mode function required
			else if (c == '0' && !checkMotor()) {
				*state = Safe_Mode;
				// Call for Safe_Mode function required
			}
			break;

		// WIRELESS MODE
		case Wireless_Mode:
			if (!no_failure)
				*state = Panic_Mode;
			// Call for Panic_Mode function required
			else if (c == '0' && !checkMotor()) {
				*state = Safe_Mode;
				// Call for Safe_Mode function required
			}
			break;

		case Panic_Mode:
			nrf_gpio_pin_toggle(RED);
			if (c == '0' && !checkMotor() && ((bat_volt > 1050) || (bat_volt < 650)))
				*state = Safe_Mode;
			break;

		case Calibration_Mode:
			break;
    }
}

// Author: Antonio Rueda
// Function to apply offset to joystick input (roll, pitch, yaw, lift)
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

// Author: Haris Suwignyo
// Function to process keyboard input
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
			if (axis_offset[1] > -31767)
				axis_offset[1] -= 1000;
			break;
		case 95:
			//pitch up
			if (axis_offset[1] < 31767)
				axis_offset[1] += 1000;
			break;
		case 40:
			//roll up
			if (axis_offset[0] > -31767)
				axis_offset[0] -= 1000;
			break;
		case 41:
			//roll down
			if (axis_offset[0] < 31767)
				axis_offset[0] += 1000;
			break;
		case 27:
			state = Panic_Mode;
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
			if (!flash_full) {
				printf("STARTING LOGGING\n");
				start_logging = true;
			}
			break;
		default:
			nrf_gpio_pin_toggle(RED);
			break;
    }
}

// Author: Pavel Rapoport
// Function to receive packet from PC, store the received packet, and process the whole received packet.
void processPkt() {
    receivePkt();
    while (readIndex < buffCount) {
        switch (packState) {
        case wait:
            //Wait for a start byte

            //printf("\nWAIT!\n");
            //printf("READ %02X\n", recChar[readIndex]);
            if (recChar[readIndex] == STARTBYTE) {
                //printf("START\n");
                ++readIndex;
                packState = first_byte_received;
            }
            else {
                //Slide the byte, start all over again
                slideMsg(1);
            }
            break;
        case first_byte_received:
            //Extract the message length

            msglen = cmd2len(recChar[readIndex++]);
            packState = receiveMsg;
            if (msglen == 0) {
                //Slide the byte, start all over again
                slideMsg(1);
                packState = wait;
            }
            //printf("\nFIRST!\n");
            break;
        case receiveMsg:
            if (readIndex < msglen - 1) {
                //Keep processing message according to its message length
                ++readIndex;
            }
            else {
                //Move to CRC_Check when reached the final byte of CRC
                packState = CRC_Check;
            }
            //printf("\nRECV\n");
            break;
        case CRC_Check:
            if (checkCRC(recChar, msglen)) {
                //Store the received message to receivedMsg buffer
                receivedMsg[++recBuff] = getPayload(msglen);

                //printf("\nRECEIVED MESSAGE: ");
                // for (int k = 0; k < msglen; ++k) {
                //     printf("%02X ", recChar[k]);
                // }
                // printf("\n");

                //Process the received message
                processRecMsg();
                // if (buffCount > 13) {
                //     printf("oldStartByte: %02X\n", recChar[13]);
                // }
                // printf("%d/%d -> ", readIndex, buffCount);

                //Slide the whole message
                slideMsg(msglen);
                // printf("%d/%d\n", readIndex, buffCount);
                // if (buffCount > 0) {
                //     printf("currentStartByte: %02X\n", recChar[0]);
                // }

                //Wait for the next start byte
                packState = wait;
            }
            else {
                //printf("\nCRC FAIL!\n");
                //sendErrMsg(3);

                //Slide the first byte, start all over again
                slideMsg(1);
                packState = wait;
            }
            // printf("\nCRC!\n");
            break;
        case panic:
            //panic_on = true;
            break;
        default:
            packState = wait;
        }
    }
}

// Author: Vincent Bejach
// Function to process received message in receivedMsg buffer
void processRecMsg() {
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

// Author: Haris Suwignyo
// Function to process mode input message from PC
void changeMode(uint8_t *msg) {
    process_key((uint8_t)msg[0]);
}

// Author: Haris Suwignyo
// Function to process joystick input from PC
void changeMov(uint8_t *msg) {
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

// Author: Haris Suwignyo
// Function to process parameter input message from PC, same as changeMode()
void changeKbParam(uint8_t *msg) {
    process_key((uint8_t)msg[0]);
}

// Author: Pavel Rapoport
// Function to send telemetry packet from Drone to PC
void sendTelemetryPacket() {
    uint8_t packet[42];
    uint32_t timestamp = get_time_us();
    packet[0] = fourthByte(timestamp);
    packet[1] = thirdByte(timestamp);
    packet[2] = secondByte(timestamp);
    packet[3] = lowByte(timestamp);
    packet[4] = state;
    packet[5] = !flash_full && start_logging;
    for(int i = 0; i < 4; ++i) {
        packet[6 + 2 * i] = highByte(ae[i]);
        packet[6 + 2 * i + 1] = lowByte(ae[i]);
    }
    if (DMP) {
        packet[14] = highByte(phi - phi_avg);
        packet[15] = lowByte(phi - phi_avg);
        packet[16] = highByte(theta - theta_avg);
        packet[17] = lowByte(theta - theta_avg);
        packet[20] = highByte(sp-sp_avg);
        packet[21] = lowByte(sp-sp_avg);
        packet[22] = highByte(sq_avg);
        packet[23] = lowByte(sq_avg);
    }
    else {
        packet[14] = highByte(f_d.phi_kalman);
        packet[15] = lowByte(f_d.phi_kalman);
        packet[16] = highByte(f_d.theta_kalman);
        packet[17] = lowByte(f_d.theta_kalman);
        packet[20] = highByte(p_kalman);
        packet[21] = lowByte(p_kalman);
        packet[22] = highByte(q_kalman);
        packet[23] = lowByte(q_kalman);
    }
    packet[18] = highByte(psi - psi_avg);
    packet[19] = lowByte(psi - psi_avg);
    // packet[20] = highByte(sp-sp_avg); //COMMENT THIS OUT IN CASE OF USING P/Q_KALMAN
    // packet[21] = lowByte(sp-sp_avg);
    // packet[22] = highByte(sq_avg);
    // packet[23] = lowByte(sq_avg);
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

// Author: Haris Suwignyo
// Function to send error message from Drone to PC, for debugging purposes
void sendErrMsg(uint8_t errNum) {
    uint8_t packet[1];
    packet[0] = errNum;
    uint8_t msglen = cmd2len(DWERR);
    uint8_t *msg = makePayload(DWERR, packet);
    for(int i = 0; i < 6; ++i) {
        uart_put(msg[i]);
    }
    free(msg);
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *****------------------------------------------------------------------
 */

int main(void)
{
    //bool connection_lost = false;
    uart_init();
    gpio_init();
    timers_init();
    adc_init();
    twi_init();
    imu_init(true, 100);
    baro_init();
    spi_flash_init();
    //ble_init();
    initProtocol();
    initValues();
    //dmp_enable_gyro_cal(0); //Disables the calibration of the gyro data in the DMP
    DMP = true;

	//Drone starting time, for connection check
    long connection_start_time = get_time_us()+ 1000000;

	//Battery check variables
    uint32_t counter = 0;
    uint8_t bat_counter = 0;
    uint32_t sum_bat_volt = 1060*8;

    uint32_t lts = get_time_us();
    uint8_t packet[20];

    //tm1 = get_time_us();
    while (!demo_done || !motors_off)
    {
        connection_lost = false;
        processPkt();

        if (check_timer_flag()) //40 ms
        {
            if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

            adc_request_sample();
			
			// Author: Haris Suwignyo
            /* Function to check the battery voltage.
			 * Battery readings are averaged to avoid spiking voltage drop when motor is spinning at high speed.
			 * This is done to avoid the drone going to panic mode when the average battery voltage is still safe.
			 */
			  
            sum_bat_volt += bat_volt;
            if(!(bat_counter++ % 8)) {
                sum_bat_volt = sum_bat_volt>>3;
                if(sum_bat_volt < 550) {
                    state = Panic_Mode;
                    //sendErrMsg(4);
                }
                sum_bat_volt = 0;
            }

            processRecMsg();

			// Author: Vincent Bejach
			/* Function to check the connection between drone and PC. 
			 * If we don't receive any packet within 100ms, assume that the connection broke.
			 * Go to panic mode if the connection broke.
			 */
            if ((get_time_us() > connection_start_time) && (get_time_us() - last_rec_pkt > 100000) && !connection_lost) {
                state = Panic_Mode;
                nrf_gpio_pin_toggle(YELLOW);
                connection_lost = true;
            }
            read_baro();
            clear_timer_flag();
        }


		// Author: Pavel Rapoport
		// Function to send telemetry data every 100ms
        if ((get_time_us() - lts) > 100000) {
            sendTelemetryPacket();
            lts = get_time_us();
        }

		// Author: Antonio Rueda
		// Function to do all the filtering and control
        if (check_sensor_int_flag())
        {
            // Run Filters
            if (!DMP) {
                get_raw_sensor_data();
                //tm1 = get_time_us();
                filter = kalman_phi;
                f_d.phi_kalman = kalman_filter(say-say_avg,sp-sp_avg,&filter);
                //tm2 = get_time_us()-tm1;
                filter = kalman_theta;
                f_d.theta_kalman = kalman_filter(sax-sax_avg,sq-sq_avg,&filter);
                filter = sr_butterworth;
                f_d.sr_filtered = butterworth_filter(sr-sr_avg,&filter);
            }
            else
                get_dmp_data();

            run_filters_and_control(&state);

			// Write the measured data to flash memory
            if (!flash_full && start_logging) {
                write_packet_flash();
            }
        }
    }

	// Author: Antonio Rueda
	// Function to print logging after we finished with the demo
    while(!read_completed) {
        read_packet_flash(packet);
        print_to_terminal(packet);
        nrf_delay_ms(10);
    }

    nrf_delay_ms(100);
    NVIC_SystemReset();
}
