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

int b = 1;
int d = 1;

int find_sqrt(int arg) {
	int result = 200;
	if (result * result < arg)
		while (result * result < arg)
			++result;
	else
		while (result * result > arg)
			--result;
	return result;
}

//SQRT -> Can we use this code? or do we need to create or own sqrt??
uint16_t isqrt16 (uint16_t n) // OR isqrt16 ( uint16 n ) OR  isqrt8 ( uint8 n ) - respectively [ OR overloaded as isqrt (uint?? n) in C++ ]
{
    register uint16_t root, remainder, place;

    root = 0;
    remainder = n;
    place = 0x4000; // OR place = 0x4000; OR place = 0x40; - respectively

    while (place > remainder)
        place = place >> 2;
    while (place)
    {
        if (remainder >= root + place)
        {
            remainder = remainder - root - place;
            root = root + (place << 1);
        }
        root = root >> 1;
        place = place >> 2;
    }
    if(remainder>root) root++;
    return root;
}

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	//update_motors();
}

void manual_mode()
{
	ae[0] = isqrt16((2*d*axis[1] - d*axis[3] - b*axis[2])/(4*b*d));  // A
	ae[1] = isqrt16((b*axis[2] - d*axis[3] - 2*d*axis[0])/(4*b*d));  // B
	ae[2] = isqrt16((-2*d*axis[1] - d*axis[3] - b*axis[2])/(4*b*d)); // C
	ae[3] = isqrt16((b*axis[2] - d*axis[3] + 2*d*axis[0])/(4*b*d));  // D

	for (int i = 0; i < 4; i++){
		ae[i] = ae[i]*2;
		if (ae[i] >= 500)
			ae[i] = 500;
	}

	printf("ae_0 = %6d | ae_2 = %6d | ae_2 = %6d | ae_3 = %6d\n", ae[0], ae[1], ae[2], ae[3]);
	//update_motors();
}

void panic_mode(){
	uint32_t start_time, current_time;
	for (int i = 0; i<4; i++){
		ae[i] = 500;
	}
	start_time = get_time_us();
	printf("Entering Panic_Mode\n");
	while (ae[0] != 0 || ae[1] != 0 || ae[2] != 0 || ae[3] != 0){
		current_time = get_time_us();
		if ((current_time - start_time)/1000 >= 100){
			for (int i = 0; i<4; i++){
				ae[i] -= 10;
			}
			printf("ae_0 = %6d | ae_2 = %6d | ae_2 = %6d | ae_3 = %6d\n", ae[0], ae[1], ae[2], ae[3]);
			//update_motors();
			start_time = current_time;
		}
	}
}

void calibration_mode(){
	uint32_t start_time, current_time;
	int16_t phi_array[10], theta_array[10], psi_array[10];
	int i = 0;
    int phi_sum = 0, theta_sum = 0, psi_sum = 0;

	start_time = get_time_us();
	current_time = get_time_us();

	//Delay of 25 secs
	while ((current_time - start_time)/1000 <= 25000)
		current_time = get_time_us();

    while (i<10){
		if (check_timer_flag()){
			clear_timer_flag();
			if (check_sensor_int_flag()){
				get_dmp_data();
				//run_filters_and_control();
				phi_array[i] = phi;
				theta_array[i] = theta;
				psi_array[i] = psi;
				i++;
			}
		}
	}

	for (i = 0; i < 10; i++){
		phi_sum += phi_array[i];
		theta_sum += theta_array[i];
		psi_sum += psi_array[i];
	}
	phi_avg = phi_sum / i;
	theta_avg = theta_sum / i;
	psi_avg = psi_sum / i;

	// printf("phi_avg = %d\n", phi_avg);
	// printf("theta_avg = %d\n", theta_avg);
	// printf("psi_avg = %d\n", psi_avg);
}
