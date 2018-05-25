#include "in4073.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

double sample;
double x[3], y[3];
double data[2];
double p_sensor[2], p_bias[2];

int bb[3];
int a[3];

int x0, x1, x2, y_1, y_2, y_0;
int phip;
int p2phi;

int C1,C2;
int p_kalman, phi_kalman, phi_error;

/*----------------------------------------------------------------
 * float2fix -- convert float to fixed point 18+14 bits
 *******----------------------------------------------------------------
 */
int float2fix(double x) {
	int y;

	y = x * (1 << 14);
	return y;
}

/*----------------------------------------------------------------
 * fix2float -- convert fixed 18+14 bits to float
 *******----------------------------------------------------------------
 */
double fix2float(int x) {
	double y;

	y = ((double)x) / (1 << 14);
	return y;
}

//FROM https://spin.atomicobject.com/2012/03/15/simple-fixed-point-math/
int32_t fixed_mul_14(int32_t x, int32_t y)
{
	return ((int64_t)x * (int64_t)y) / (1 << 14);
}

int32_t fixed_div_14(int32_t x, int32_t y)
{
	return ((int64_t)x * (1 << 14)) / y;
}

void initialize_butterworth(){
	for (int n = 0; n < 2; n++) {
		x[n] = 0;
		y[n] = 0;
	}

	//Butterworth LowPass Filter: Cutt-off at 10 Hz and FS = 100Hz
	a[0] = float2fix(0.067455273889072);
	a[1] = float2fix(0.134910547778144);
	a[2] = float2fix(0.067455273889072);
	bb[0] = float2fix(1);
	bb[1] = float2fix(-1.142980502539901);
	bb[2] = float2fix(0.412801598096189);
}

void initialize_integrator(){
	for (int n = 0; n < 1; n++) {
		data[n] = 0;
	}
	p2phi = float2fix(0.01); //SAMPLE TIME???
}

void initialize_kalman(){
	C1 = float2fix(256);
	C2 = float2fix(1000000);
	for (int n = 0; n < 1; n++) {
		p_bias[n] = 0;
		p_sensor[n] = 0;
		p_sensor[n] = 0;
	}
	p2phi = float2fix(0.01); //SAMPLE TIME???
}

int butterworth_filter(int raw_data) {
	int i;

	for (i = 2; i > 0; i--) {
		x[i] = x[i-1];
		y[i] = y[i-1];
	}

	x[0] = raw_data;

	//printf("RAW = %d ",raw_data);
	x0 = float2fix(x[0]);
	x1 = float2fix(x[1]);
	x2 = float2fix(x[2]);
	y_1 = float2fix(y[1]);
	y_2 = float2fix(y[2]);

	y_0 = fixed_mul_14(a[0],x0)+fixed_mul_14(a[1],x1)+fixed_mul_14(a[2],x2)-fixed_mul_14(bb[1],y_1)-fixed_mul_14(bb[2],y_2);
	y[0] = fix2float(y_0);

	//printf("FILTERED = %f ",y[0]);
	return y[0];
}

int compute_phi(int p_filtered){

	data[1] = data[0];
	data[0] = p_filtered;

	//printf("p_filtered = %f  ",data[1]);

	phip = phip + fixed_mul_14(float2fix(data[1]),p2phi);

	//printf("FIXED = %f \n",fix2float(phip));
	return fix2float(phip);
}

int kalman_filter(int phi_filtered, int p_read){
	static int k = 0;

	p_bias[0] = p_bias[1];

	p_sensor[0] = p_sensor[1];
	p_sensor[1] = p_read;

	p_kalman = float2fix(p_sensor[0] - p_bias[0]);
	phi_kalman = phi_kalman + fixed_mul_14(p_kalman,p2phi);
	if (k != 0)
		phi_error = phi_kalman - float2fix(phi_filtered);
	else
		k++;

	phi_kalman = phi_kalman - fixed_div_14(phi_error,C1);
	p_bias[1] = float2fix(p_bias[0]) + fixed_div_14(fixed_div_14(phi_error,p2phi),C2);
	p_bias[1] = fix2float(p_bias[1]);
	//printf("P_KALMAN = %f \n",fix2float(p_kalman));
	//printf("phi_kalman_FIXED = %f \n",fix2float(phi_kalman));
	//printf("PHI_ERROR = %f \n",fix2float(phi_error));
	//printf("P_BIAS = %f \n",p_bias[1]);
	return fix2float(phi_kalman);

}
