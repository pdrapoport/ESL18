/*------------------------------------------------------------------
 *  filters.c -- implementation of filtering algorithm
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
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum filters filter;

// Taken from fixed_order2_butterworth10.c from the project resource page (http://wwwtmp.st.ewi.tudelft.nl/koen/cs4140/Resources/dsp.tar)
/*----------------------------------------------------------------
 * float2fix -- convert float to fixed point 18+14 bits
 ********----------------------------------------------------------------
 */
int float2fix(float x) {
    int y;

    y = x * (1 << 14);
    return y;
}

// Taken from fixed_order2_butterworth10.c from the project resource page (http://wwwtmp.st.ewi.tudelft.nl/koen/cs4140/Resources/dsp.tar)
/*----------------------------------------------------------------
 * fix2float -- convert fixed 18+14 bits to float
 ********----------------------------------------------------------------
 */
float fix2float(int x) {
    float y;

    y = ((float)x) / (1 << 14);
    return y;
}

// Taken from https://spin.atomicobject.com/2012/03/15/simple-fixed-point-math/
int32_t fixed_mul_14(int32_t x, int32_t y)
{
    return ((int64_t)x * (int64_t)y) / (1 << 14);
}
// Taken from https://spin.atomicobject.com/2012/03/15/simple-fixed-point-math/
int32_t fixed_div_14(int32_t x, int32_t y)
{
    return ((int64_t)x * (1 << 14)) / y;
}

// Author: Antonio Rueda
// Function to initialize butterworth filter's variables
void initialize_butterworth() {
    for (int n = 0; n < 2; n++) {
        x_1[n] = 0;
        y_1[n] = 0;
        x_2[n] = 0;
        y_2[n] = 0;
        x_3[n] = 0;
        y_3[n] = 0;
    }
}

// Author: Antonio Rueda
// Function to initialize kalman filter's variables
void initialize_kalman() {
    for (int n = 0; n < 1; n++) {
        p_bias[n] = 0;
        p_sensor[n] = 0;
        q_bias[n] = 0;
        q_sensor[n] = 0;
    }
}

// Author: Antonio Rueda
// Function to do the butterworth filter second order
int16_t butterworth_filter(int16_t raw_data, enum filters *filter ) {
    int i;
    int bb[3];
    int  yy0;
    unsigned int a[3];
    //Butterworth LowPass Filter: Cutt-off at 10 Hz and FS = 500Hz
    //Calculated using Matlab: [a,b] = butter(2, fc/(fs/2))
    a[0] = float2fix(0.0036);
    a[1] = float2fix(0.0072);
    a[2] = float2fix(0.0036);
    bb[0] = float2fix(1);
    bb[1] = float2fix(-1.8227);
    bb[2] = float2fix(0.8372);

    switch (*filter) {
		case sr_butterworth:
			for (i = 2; i > 0; i--) {
				x_3[i] = x_3[i-1];
				y_3[i] = y_3[i-1];
			}
			x_3[0] = raw_data;
			x0 = float2fix(x_3[0]);
			x1 = float2fix(x_3[1]);
			x2 = float2fix(x_3[2]);
			yy1 = float2fix(y_3[1]);
			yy2 = float2fix(y_3[2]);
			break;
		case kalman_phi:
			break;
		case kalman_theta:
			break;
    }

    yy0 = fixed_mul_14(a[0],x0)+fixed_mul_14(a[1],x1)+fixed_mul_14(a[2],x2)-fixed_mul_14(bb[1],yy1)-fixed_mul_14(bb[2],yy2);

    switch (*filter) {
		case sr_butterworth:
			y_3[0] = fix2float(yy0);
			return y_3[0];
		case kalman_phi:
			break;
		case kalman_theta:
			break;
    }
    return 0;
}

// Author: Antonio Rueda
// Function to do the kalman filter
int16_t kalman_filter(int16_t filtered, int16_t vel_read, enum filters *filter) {
    static int k = 0;
    static int j = 0;
    unsigned int C1 = float2fix(256);
    unsigned int C2 = float2fix(1000000);
    unsigned int p2phi = float2fix(0.002);

    switch (*filter) {
		case kalman_phi:
			p_bias[0] = p_bias[1];
			p_sensor[0] = p_sensor[1];
			p_sensor[1] = vel_read;
			p_kalman = float2fix(p_sensor[0] - p_bias[0]);
			phi_kalman = phi_kalman + fixed_mul_14(p_kalman,p2phi);
			if (k != 0)
				phi_error = phi_kalman - float2fix(filtered);
			else
				k++;

			phi_kalman = phi_kalman - fixed_div_14(phi_error,C1);
			p_bias[1] = float2fix(p_bias[0]) + fixed_div_14(fixed_div_14(phi_error,p2phi),C2);//COMMENT THIS OUT
			//p_bias[1] = float2fix(p_bias[0]) + fixed_div_14(phi_error,C2);
			p_bias[1] = fix2float(p_bias[1]);
			p_kalman = fix2float(p_kalman);
			return fix2float(phi_kalman);

		case kalman_theta:
			q_bias[0] = q_bias[1];
			q_sensor[0] = q_sensor[1];
			q_sensor[1] = vel_read;
			q_kalman = float2fix(q_sensor[0] - q_bias[0]);
			theta_kalman = theta_kalman + fixed_mul_14(q_kalman,p2phi);
			if (j != 0)
				theta_error = theta_kalman - float2fix(filtered);
			else
				j++;
			theta_kalman = theta_kalman - fixed_div_14(theta_error,C1);
			q_bias[1] = float2fix(q_bias[0]) + fixed_div_14(fixed_div_14(theta_error,p2phi),C2); //COMMENT THIS OUT
			// q_bias[1] = float2fix(q_bias[0]) + fixed_div_14(theta_error,C2);
			q_bias[1] = fix2float(q_bias[1]);
			q_kalman = fix2float(q_kalman);
			return fix2float(theta_kalman);

		case sr_butterworth:
			break;
    }
    return 0;
}
