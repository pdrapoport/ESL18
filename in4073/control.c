/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "math.h"


//Variables for calibration
int16_t sp_array[64], sq_array[64], sr_array[64];
int16_t sax_array[64], say_array[64], saz_array[64];
int i = 0;
int sp_sum = 0, sq_sum = 0, sr_sum = 0;
int sax_sum = 0, say_sum = 0, saz_sum = 0;

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void run_filters_and_control(enum states *state){
	int16_t lift, roll, pitch, yaw;

	lift = roll = pitch = yaw = 0;
	switch (*state) {
    	case Safe_Mode:
	  		break;
		case Manual_Mode:
			roll = axis[0]; //L roll
			pitch = axis[1]; //M pitch
			yaw = axis[2]; //N yaw
			lift = axis[3]; //Z lift
			break;
		case Calibration_Mode:
			sp_sum += sp;
			sq_sum += sq;
			sr_sum += sr;
			sax_sum += sax;
			say_sum += say;
			saz_sum += saz;
			i++;
			if (i == 50){
				sp_avg = sp_sum / i;
				sq_avg = sq_sum / i;
				sr_avg = sr_sum / i;
				sax_avg = sax_sum / i;
				say_avg = say_sum / i;
				saz_avg = saz_sum / i;
				printf("Calibration performed\n");
				calibration_done = true;
				if (motors_off && calibration_done){
				  *state = Safe_Mode;
				  printf("Safe_Mode Selected\n");
				  i = 0;
				  sp_sum = 0;
				  sq_sum = 0;
				  sr_sum = 0;
				  sax_sum = 0;
				  say_sum = 0;
				  saz_sum = 0;
				}
			}
			break;
		case Yaw_Mode:
			roll = axis[0]; //L roll
			pitch = axis[1]; //M pitch
			lift = axis[3]; //Z lift
			yaw = p * (5 * axis[2] - sr);
			break;
		case Full_Mode:
			break;
		case Raw_Mode:
			break;
		case Height_Mode:
			break;
		case Wireless_Mode:
			break;
		case Panic_Mode:
			for (int j = 0; j<4; j++){
				ae[j] -= 1;
				if (ae[j] <= 0)
					ae[j] = 0;
			}
			break;
	}

	if (*state != Panic_Mode){
		ae[0] = sqrt((2*d*pitch + d*lift - b*yaw)/(4*b*d));  // A
		ae[1] = sqrt((b*yaw + d*lift - 2*d*roll)/(4*b*d));  // B
		ae[2] = sqrt((-2*d*pitch + d*lift - b*yaw)/(4*b*d)); // C
		ae[3] = sqrt((b*yaw + d*lift + 2*d*roll)/(4*b*d));  // D

		for (int i = 0; i < 4; i++){
			ae[i] = ae[i]*4; //Scaling Factor
			if (ae[i] >= 500)
				ae[i] = 500;
			else if (lift > 5910 && ae[i] <= 152)
				ae[i] = 152;
		}

		//printf("ae_0 = %6d | ae_2 = %6d | ae_2 = %6d | ae_3 = %6d\n", ae[0], ae[1], ae[2], ae[3]);
	}
	update_motors();
}
