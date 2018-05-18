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
	update_motors();
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

	printf("ae_0 = %d | ae_1 = %d | ae_2 = %d | ae_3 = %d\n", ae[0], ae[1], ae[2], ae[3]);
	//update_motors();
}
