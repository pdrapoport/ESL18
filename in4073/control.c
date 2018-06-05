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
int phi_sum = 0, theta_sum = 0, psi_sum = 0;



void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
	motors_off = true;
	for (int i = 0; i < 4; ++i) {
		motors_off &= (motor[i] == 0);
	}
}

void run_filters_and_control(enum states *state){
	int32_t lift, roll, pitch, yaw;
	lift = roll = pitch = yaw = 0;
	switch (*state) {
    	case Safe_Mode:
	  		break;

		case Manual_Mode:
			roll = axis[0]*30; //L roll
			pitch = axis[1]*30; //M pitch
			yaw = axis[2]*30; //N yaw
			lift = axis[3]*30; //Z lift
			break;

		case Calibration_Mode:
			sp_sum += sp;
			sq_sum += sq;
			sr_sum += sr;
			sax_sum += sax;
			say_sum += say;
			saz_sum += saz;
			phi_sum += phi;
			theta_sum += theta;
			psi_sum += psi;
			i++;
			if (i == 50){
				sp_avg = sp_sum / i;
				sq_avg = sq_sum / i;
				sr_avg = sr_sum / i;
				sax_avg = sax_sum / i;
				say_avg = say_sum / i;
				saz_avg = saz_sum / i;
				phi_avg = phi_sum / i;
				theta_avg = theta_sum / i;
				psi_avg = psi_sum / i;
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
				  phi_sum = 0;
				  theta_sum = 0;
				  psi_sum = 0;
				}
			}
			break;

		case Yaw_Mode:
			roll = axis[0]*30;
			pitch = axis[1]*30;
			yaw = p * (axis[2]*30 - (sr-sr_avg));
			lift = axis[3]*30;
			break;

		case Full_Mode:
			pitch = p1 * (axis[1]*30 - (theta-theta_avg)) - p2*(sq-sq_avg);
			roll = p1 * (axis[0]*30 - (phi-phi_avg)) - p2*(sp-sp_avg);
			yaw = p * (axis[2]*30 - (sr-sr_avg));
			lift = axis[3]*30;
			break;

		case Raw_Mode:
            // Run Filters
            // filter = say_butterworth;
            // f_d.say_filtered = butterworth_filter(say-say_avg,&filter);
            //filter = kalman_phi;
            //f_d.phi_kalman = kalman_filter(f_d.say_filtered,sp-sp_avg,kalman_phi);

            // filter = sax_butterworth;
            // f_d.sax_filtered = butterworth_filter(sax-sax_avg,&filter);
            // filter = kalman_theta;
            // f_d.theta_kalman = kalman_filter(f_d.sax_filtered,sq-sq_avg,&filter);

            // filter = sr_butterworth;
            // f_d.sr_filtered = butterworth_filter(sr-sr_avg,&filter);

            //Control with filtered Data
            pitch = p1 * (axis[1]*30 - f_d.theta_kalman) - p2*(sq-sq_avg);
            roll = p1 * (axis[0]*30 - f_d.phi_kalman) - p2*(sp-sp_avg);
            yaw = p * (axis[2]*30 - f_d.sr_filtered);
            lift = axis[3]*30;
			break;

		case Height_Mode:
			//insert control here
			break;

		case Wireless_Mode:
			//insert control here
			break;

		case Panic_Mode:
			; //to avoid the static int below case
			static int k = 0;
			if(k++ % 3 == 0){
				for (int j = 0; j<4; j++){
					ae[j] -= 1;
					if (ae[j] <= 0)
						ae[j] = 0;
				}
			}
			//check if the motor has turned off
			if(!motor[0] && !motor[1] && !motor[2] && !motor[3]) {no_failure = true;motors_off = true;}
			break;
	}

	if (*state != Panic_Mode){
		ae[0] = sqrt((2*d*pitch + d*lift - b*yaw)/(4*b*d));  // A
		ae[1] = sqrt((b*yaw + d*lift - 2*d*roll)/(4*b*d));  // B
		ae[2] = sqrt((-2*d*pitch + d*lift - b*yaw)/(4*b*d)); // C
		ae[3] = sqrt((b*yaw + d*lift + 2*d*roll)/(4*b*d));  // D

		for (int i = 0; i < 4; i++){
			//ae[i] = ae[i]*6; //Scaling Factor
			if (ae[i] >= 500)
				ae[i] = 500;
			else if (lift > 5910 && ae[i] <= 152)
				ae[i] = 152;
			else if (ae[i]<0)
				ae[i] = 0;
		}

		//printf("ae_0 = %6d | ae_2 = %6d | ae_2 = %6d | ae_3 = %6d\n", ae[0], ae[1], ae[2], ae[3]);
	}
	update_motors();
}
