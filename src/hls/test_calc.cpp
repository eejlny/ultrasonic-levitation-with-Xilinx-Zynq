#include <stdio.h>
#include "calc.h"



int main(){
	printf("Testing core");
	unsigned int A = CalculatePhase(66,66,100,0);

	printf("Phase is: %d, %d, %d, %d",((A>>24)&0xFF), ((A>>16)&0xFF), ((A>>8)&0xFF), ((A>>0)&0xFF));
	return 0;



}

