#include <HLS_MATH.h>
//#include <math.h>
#include "calc.h"

unsigned int CalculatePhase(float T_X, float T_Y, float T_Z, char Offset){
#pragma HLS INTERFACE s_axilite port=return bundle=crtls
#pragma HLS INTERFACE s_axilite port=T_X bundle=crtls
#pragma HLS INTERFACE s_axilite port=T_Y bundle=crtls
#pragma HLS INTERFACE s_axilite port=T_Z bundle=crtls
#pragma HLS INTERFACE s_axilite port=Offset bundle=crtls

	float Delay;
	float Xdist_sq;
	float Ydist_sq;
	float zInt = T_Z;
	float Zdist_sq = (zInt - Height) * (zInt - Height);
	unsigned int a;
	unsigned int output=0;
	for (int i=0; i< 4; i++){
		//#pragma HLS PIPELINE
		Xdist_sq = T_X - XTransducerPositions[i + Offset];
		Ydist_sq = T_Y - YTransducerPositions[i + Offset];
		Xdist_sq = Xdist_sq * Xdist_sq;
		Ydist_sq = Ydist_sq * Ydist_sq;
		Delay = sqrtf( Xdist_sq + Ydist_sq + Zdist_sq) / 1000;
		Delay = 1-(mod(Delay,Wavelength) / Wavelength);
		Delay =  (Delay * 256);
		a = (roundf(Delay));
		if (a > 255){
			a = 0;
		}
		output += a << (8*i);
	}

	return output;

}

float mod(float numerator, float divisor){
	if (divisor > numerator)
		return numerator;
	else if (divisor == numerator)
		return 0;
	float remainder=0;
	float integer =0;
	remainder = modff(numerator/divisor,&integer);
	return remainder * divisor;
}
