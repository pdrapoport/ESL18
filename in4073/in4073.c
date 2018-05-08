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
              // Call for Calibration_Mode function required
              //delay(2000);
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
void process_key(uint8_t c)
{
	printf("Recv key: %d\n", c);
	switch (c)
	{
		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		case '1':
			nrf_gpio_pin_toggle(RED);
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
      printf("%s\n",getCurrentState(state));
      break;
		default:
			nrf_gpio_pin_toggle(RED);
	}
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

	uint32_t counter = 0;
	demo_done = false;
	state = Safe_Mode;
	while (!demo_done)
	{
		if (rx_queue.count) process_key( dequeue(&rx_queue) );

		if (check_timer_flag())
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();

			//printf("%10ld | ", get_time_us());
			//printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			//printf("%6d %6d %6d | ", phi, theta, psi);
			//printf("%6d %6d %6d | ", sp, sq, sr);
			//printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

			clear_timer_flag();
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
