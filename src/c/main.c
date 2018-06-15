/*
 * main.c: Application for writing pre-generated phases to the UPAC
 * developed by : William Beasley and Brenda Gatusch
 * date : 14/09/17
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "platform.h"
#include <xparameters.h>
#include <xil_io.h>
#include "UPAC.h"
#include <math.h>
#include <xscugic.h>
#include <Xil_exception.h>
#include "xtime_l.h"
#include <xgpio.h>
#include <xscutimer.h>
#include "xuartps.h"
#include "xil_printf.h"
#include "xcalculatephase.h"
#include "xadcps.h"
#include "xil_types.h"
#include "PmodHYGRO.h"

#define TIMER_FREQ_HZ 100000000
#define ULTRASONIC_DRIVER_BASE_ADDR 0x43C00000
#define ULTRASONIC_DRIVER_HIGH_ADDR 0x43C0FFFF
#define TIMER_LOAD_VALUE 0xFFFFFFFF
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GPIO_INTERRUPT_ID XPS_GPIO_INT_ID
#define TICKS_PER_SECOND 666666687.0/2
#define TIMER_OVERHEAD 0.1
#define CHANNEL_NUM 64
#define REG_NUM 16
#define GPIO_DEVICE_ID XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_DEVICE_Ultrasonic_ID 1
#define XPAR_AXI_XADC_0_DEVICE_ID 0
#define Height 12.0
#define UseHardware 0
#define FREQUENCY 40000
#define INTC XScuGic
#define UART_DEVICE_ID 0
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID XPAR_XUARTPS_1_INTR

//Transducers Positions in phased array (units: mm)
const double XTransducerPositions[64] = {8.25,8.25,8.25,8.25,8.25,8.25,8.25,8.25,24.75,24.75,24.75,24.75,24.75,24.75,24.75,24.75,41.25,41.25,41.25,41.25,41.25,41.25,41.25,41.25,57.75,57.75,57.75,57.75,57.75,57.75,57.75,57.75,74.25,74.25,74.25,74.25,74.25,74.25,74.25,74.25,90.75,90.75,90.75,90.75,90.75,90.75,90.75,90.75,107.25,107.25,107.25,107.25,107.25,107.25,107.25,107.25,123.75,123.75,123.75,123.75,123.75,123.75,123.75,123.75};
const double YTransducerPositions[64] = {8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75,8.25,24.75,41.25,57.75,74.25,90.75,107.25,123.75};

//Function Declarations
void WriteOutputMask(u32 Mask[2]);
void UPAC_Write(u32 Match[REG_NUM]);
void CalculatePhase(double T_X, double T_Y, double T_Z, int *Output);
void CalculatePhaseHW(float T_X, float T_Y, float T_Z, int *Output);
void GetTransducers(double T_X,double T_Y,double T_Z, u32* Match);
u32 getPhase(int RegNumber, int *Delays);
void SetTransducers(double T_X,double T_Y,double T_Z);
int hls_init();
void hls_start(void *InstancePtr);
void hls_ISR(void *InstancePtr);
int hls_setup();
u32 GetTimerValue(u32 TimerIntrId,u16 Mode);
u32 start_timer();
u32 stop_timer();
u32 read_timer();
u32 get_microseconds(u32 Time1, u32 Time2);
void GetEchoData(XAdcPs *XADCInstPtr, float SamplesVaux4[400], float SamplesVaux10[400]);
int FindCoordinate(int count4, int count10, int coord);
int UartInterruptSet(INTC *IntcInstPtr, XUartPs *UartInstPtr, u16 DeviceId, u16 UartIntrId);
static int SetupInterruptSystem(INTC *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId);
void UartHandler(void *CallBackRef);
void init_XADC();
void PmodHYGROInit();
void PmodHYGROGetData();
void Test_Speed(double step_size, int delay_val);

//Initialise Hardware objects
XScuTimer Timer;
XGpio Gpio;
XGpio Triangulation;
XCalculatephase HLSPhaseCalc;
XScuGic ScuGic;
XUartPs UartPs;
INTC InterruptController;
PmodHYGRO HygroSensor;

int xtraStep =3000;
double CurrentPos[3] = {66,66,100};
volatile static int PhaseCalcDone = 0;
static XAdcPs XADCMonInst;
XAdcPs *XADCInstPtr;
XAdcPs_Config *ConfigPtr;
int Status_GPIO, Status_ADC, Status_UART;
int menu_select;
int return_menu;
int setoptions;
int signx, signy;
int startX, startY;
float speed_sound;
float wavelength;
float temp_degc, hum_perrh;

int main(void)
{

    //Variable declarations
    signx = 1;
    signy = 1;

	int distance;
    int X, Y, reqX, reqY, X_closedloop;
    startX = 63;
    startY = 63;
    X_closedloop = 66;
    distance = 7;
    double Z = 100;

	float Samples4 [400];
	float Samples10 [400];
    int count4, count10;
    float peak4, peak10;
    float threshold = 0.11;

    count4 = 0;
    count10 = 0;
    peak4 = 0.0;
    peak10 = 0.0;

	temp_degc = 0.0;
	hum_perrh = 0.0;
    wavelength = 0.00850725;

	setoptions = 0;
	menu_select = 0;
	return_menu = 1;
	char c;
	char echo_enable;
	int option;
	int delay;
	double step;

	//Initialise the platform
    init_platform();

    //Initialise XADC
    init_XADC();

    //Setup UART
    UartInterruptSet(&InterruptController, &UartPs, UART_DEVICE_ID, UART_INT_IRQ_ID);
    XUartPs_SetRecvTimeout(&UartPs, 0);

    //Write Output Mask, Enable all transducers
	u32 Mask[2] = {0xFFFFFFFF,0xFFFFFFFF};
    WriteOutputMask(Mask);

    //Populate Control Register with banket 1s
    Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0xFFFFFFFF);

    //Initalise GPIO
    Status_GPIO = XGpio_Initialize(&Gpio, GPIO_DEVICE_ID);
    if (Status_GPIO != XST_SUCCESS){
    	return XST_FAILURE;
    }

    //If using HLS block, initialise and setup interrupts
    if (UseHardware == 1){
    	hls_init(&HLSPhaseCalc);
    	hls_setup();
    }

    //Initialise Pmod HYGRO sensor
    PmodHYGROInit();

	while(1){

		while(menu_select < 1 || menu_select > 7 || return_menu == 1){

			XUartPs_SetRecvTimeout(&UartPs, 0);

		    //Start Menu:
            printf ("\n\rUltrasonic Phased Array Menu\n\r");
            printf("1.Fixed Coordinate\n\r2.Move in X-axis\n\r3.Move in Y-axis\n\r4.Move in XY-plane\n\r5.Closed loop system\n\r6.Set speed of sound\n\r7.Test speed\n\r");

            while (scanf("%d", &menu_select) == 0){
                getchar();
            }

            while((c = getchar()) != '\r');

            if (menu_select < 1 || menu_select > 7){
            	printf("Not an option\n\r");
            }
            else
            {
            	printf ("Selected: %i\n\r", menu_select);
            }

            setoptions = 0;
            return_menu = 0;
            echo_enable = 'n';
    	    signx = 1;
    	    signy = 1;
    	    startY = 63;
		}

        for (int i = 0; i < distance; i++)
        {
        	X = startX + i*signx;

            for (int j = 0; j < distance; j++)
            {
            	Y = startY + j*signy;

            	Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0xFFFFFFFF);

                switch (menu_select)
                {

                    //Fixed Coordinate Option
                    case 1:
            	        if(setoptions == 0){
            		        printf("Requested X-coordinate (between 50 and 70):\n\r");
            		        scanf("%d",&reqX);
            		        printf("Requested Y-coordinate (between 50 and 70):\n\r");
            		        scanf("%d",&reqY);


            		        if(reqX < 50){
            		        	reqX = 50;
            		        }

            		        if(reqX > 70){
            		        	reqX = 70;
            		        }

            		        if(reqY < 50){
            		        	reqY = 50;
            		        }

            		        if(reqY > 70){
            		        	reqY = 70;
            		        }


            		        printf("Coordinates: (%i,%i,100)\n\r", reqX, reqY);
            	        }

                        for (int j = 0; j < 1000; j++)
                        {
        	                SetTransducers(reqX,reqY,Z);
        	                usleep(5000);
                        }
        	            break;

        	        //Move in X-axis option
                    case 2:
                    	if (setoptions == 0)
                    	{
            				printf("Enable feedback? (y/n)\n\r");
            				scanf("%c", &echo_enable);
            				while((c = getchar()) != '\r');

            				if(echo_enable == 'y'){
            					printf("Echo enabled\n\r");
            				}
            				else{
            					printf("Echo disabled\n\r");
            				}
                    	}

                        for (int j = 0; j < 1000; j++)
                        {
                        	SetTransducers(Y,66,Z);
                        	usleep(5000);
                        }
        	            break;

        	        //Move in Y-axis Option
                    case 3:
                        for (int j = 0; j < 1000; j++)
                        {
                        	SetTransducers(65,Y,Z);
                        	usleep(5000);
                        }
        	            break;

                    //Move in XY-plane Option
                    case 4:
                        for (int j = 0; j < 1000; j++)
                        {
                        	SetTransducers(X,Y,Z);
                        	usleep(2000);
                        }
                        break;

                    //Closed Loop System Option
                    case 5:
                       	if((peak4 > threshold && peak4 > peak10)){
                        	while(X_closedloop < 69){

                        		X_closedloop++;

                                for (int j = 0; j < 1000; j++)
                                {
                                	SetTransducers(X_closedloop,66,Z);
                                    usleep(2000);
                                }
                        	}
                        }
                        else if(peak10 > threshold && peak10 > peak4){
                        	while(X_closedloop > 63){

                        		X_closedloop--;

                                for (int j = 0; j < 1000; j++)
                                {
                                	SetTransducers(X_closedloop,66,Z);
                                    usleep(2000);
                                }
                        	}
                        }
                        echo_enable = 'y';
                    	break;

                    //Set Speed of Sound Option
                    case 6:
                    	while(option < 1 || option > 2){

                        	printf("1.Default speed of sound\n\r2.Adjust to temperature\n\r");
                            while (scanf("%d", &option) == 0){
                                getchar();
                            }

                            while((c = getchar()) != '\r');

                            if (option == 1){
                            	speed_sound = 340.29;
                            	wavelength = speed_sound/FREQUENCY;
                            	printf("Speed of sound: %f  Wavelength: %f\n\r", speed_sound, wavelength);
                            }
                            else if(option == 2)
                            {
                            	PmodHYGROGetData();
                            	speed_sound = 331.3 + 0.6*temp_degc;
                            	wavelength = speed_sound/FREQUENCY;
                            	printf("Temperature: %.2f degC  Humidity: %.2f RH\n\r", temp_degc, hum_perrh);
                            	printf("Speed of sound: %f  Wavelength: %f\n\r", speed_sound, wavelength);
                            }
                            else
                            {
                            	printf("Not an option\n\r");
                            }
                    	}
                    	option = 0;
                    	return_menu = 1;

                    	break;

                    // Test Speed of Levitated Object Option
                    case 7:
                    	printf("Set step (mm):\n\r");
         		        scanf("%lf",&step);
        		        printf("Set delay factor (50 to 8000):\n\r");
        		        scanf("%d",&delay);

        		        if(step < 1 || step > 10){
        		        	step = 1;
        		        }

        		        if(delay < 1 || delay > 8000){
        		        	delay = 2000;
        		        }

        		        printf("Step: %lf   Delay: %i\n\r", step, delay);

                        Test_Speed(step,delay);
                        return_menu = 1;

                    	break;

                    // Default Option
                    default:
                        for (int j = 0; j < 1000; j++)
                        {
                        	SetTransducers(66,66,Z);
                        	usleep(5000);
                        }
                    	break;
                }

                // Check if user hit key to return to main menu
                if(return_menu == 1){
                	break;
                }

            	if (setoptions == 0)
            	{
    				printf("Press any key to return to this menu\n\r");
    				setoptions = 1;
            	}

            	// UART time out interrupt
                XUartPs_SetRecvTimeout(&UartPs,255);

                // Echo Wave Reception
                if(echo_enable == 'y')
                {
                    for (int j = 0; j < 1000; j++)
                    {
                    	if(menu_select == 2){
                        	SetTransducers(Y,66,Z);
                        	usleep(5000);
                            usleep(5000);
                    	}
                    	else if(menu_select == 5){
                        	SetTransducers(X_closedloop,66,Z);
                        	usleep(5000);
                            usleep(5000);
                    	}

                        if(return_menu == 1){
                        	break;
                        }
                    }

                    // Disable transducer array for 3.8 ms to quiet the acoustic waves around system
                    Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0x00000000);
                    usleep(3800);

                    // Enable transducer array for 300 us to create a small burst of acoustic waves
                    Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0xFFFFFFFF);
                    usleep(300);

                    // Disable transducer array before reading ADC captured samples
                    Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0x00000000);

                    // Read echo waves from two receivers Vaux4 and Vaux10
                    GetEchoData(XADCInstPtr, Samples4, Samples10);

                    // Enable transducer array
                    Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 64, 0xFFFFFFFF);

                    count4 = 0;
                    count10 = 0;
                    peak4 = 0.0;
                    peak10 = 0.0;

                    // Obtain peak values between ADC samples 150 and 200
                    for (int m = 0; m < 400; m++)
                    {
                        if (m >= 150 && m <= 200)
                        {
                        	if (Samples4[m] > peak4){
                        		peak4 = Samples4[m];
                        		count4 = m;
                        	}

                        	if (Samples10[m] > peak10){
                        		peak10 = Samples10[m];
                        		count10 = m;
                        	}
                        }
                    }

                    // For Option 2 "Move in X-axis", estimate position of levitated object
                    if(menu_select == 2){
                		FindCoordinate(count4, count10, Y);
                    }

                    // For Option 5 "Closed Loop System", display peak values and ADC sample number of peaks for both receivers
                    else if(menu_select == 5)
                    {
                       	printf("CurrentX: %i  count4: %i  count10: %i  peak4: %.3f  peak10: %.3f\n\r", X_closedloop, count4, count10, peak4, peak10);
                    }

                }

            }

            if(return_menu == 1){
            	break;
            }

            startY = Y;
            signy = -1*signy;
        }

        startX = X;
        signx = -1*signx;

    }

	// Not reached in program:
    cleanup_platform();
    return 0;
}


void init_XADC(){

	 XADCInstPtr = &XADCMonInst;

    //XADC initialization
     ConfigPtr = XAdcPs_LookupConfig(XPAR_AXI_XADC_0_DEVICE_ID);

     Status_ADC = XAdcPs_CfgInitialize(XADCInstPtr,ConfigPtr,ConfigPtr->BaseAddress);
     if(XST_SUCCESS != Status_ADC){
         print("ADC INIT FAILED\n\r");
     }

     //ADC self test
     Status_ADC = XAdcPs_SelfTest(XADCInstPtr);
}

//Writes the output mask into the UPAC mask registers
void WriteOutputMask(u32 Mask[2]){
	Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 68, Mask[0]);
	Xil_Out32(ULTRASONIC_DRIVER_BASE_ADDR + 72, Mask[1]);
}


//Returns what each register needs to contain.
u32 getPhase(int RegNumber, int *Delays){
	u32 temp =0;
	temp |= Delays[0+(RegNumber * 4)];
	temp |= Delays[1+(RegNumber * 4)] << 8;
	temp |= Delays[2+(RegNumber * 4)] << 16;
	temp |= Delays[3+(RegNumber * 4)] << 24;
	return temp;
}


//Write the phase shift values to the UPAC
void UPAC_Write(u32 Match[REG_NUM]){
	for(int i=0; i< REG_NUM;i++)
		UPAC_mWriteReg(ULTRASONIC_DRIVER_BASE_ADDR ,i*4, Match[i]);
}


//Software version of CalculatePhase: Calculate the phase delays for a target position
void CalculatePhase(double T_X, double T_Y, double T_Z, int *Output){

	double ChannelDelays[CHANNEL_NUM] ={0};
	int i=0;
	for(i=0; i< CHANNEL_NUM; i++){

		// Calculate distance between transducer elements to focal point
		ChannelDelays[i] = sqrt(pow((T_X - XTransducerPositions[i]),2) + pow((T_Y - YTransducerPositions[i]),2) + pow((T_Z - Height),2)) / 1000;

		// Phase shift value equal to remainder of distance divided by wavelength. Expressed as fraction of wavelength.
		ChannelDelays[i] = 1-(fmod(ChannelDelays[i],wavelength) / wavelength);

		// Creating twin-trap: Apply additional half-wavelength phase shift to upper half of array
		if (i > 31){
			ChannelDelays[i] = ChannelDelays[i] + 0.5;

			if(ChannelDelays[i] >= 1)
			{
				ChannelDelays[i] = ChannelDelays[i] - 1;
			}
		}

		// Convert final phase shift values into 8 bit numbers
		ChannelDelays[i] =  (ChannelDelays[i] * 256)  ;

		if (round(ChannelDelays[i]) > 255){
			Output[i] = 0;
		}
		else{
			Output[i] = (int)round(ChannelDelays[i]);
		}

	}

	return;
}


//Converts a float to its u32 equivalent for HLS block
u32 float_to_u32(float val){
	u32 result;
	union {
		float value;
		unsigned char bytes[4];
	}data;
	data.value = val;
	result = (data.bytes[3] << 24) + (data.bytes[2] << 16) + (data.bytes[1] << 8) + (data.bytes[0]);
	return result;
}


//Converts u32 from HLS into a float value
float u32_to_float(u32 val){
	union{
		float float_value;
		unsigned char bytes[4];
	}data;
	data.bytes[3] = (val >> 24) & 0xFF;
	data.bytes[2] = (val >> 16) & 0xFF;
	data.bytes[1] = (val >> 8) & 0xFF;
	data.bytes[0] = (val >> 0) & 0xFF;
	return data.float_value;
}


//Set the Transducers to target a focal point at X,Y,Z
void SetTransducers(double T_X,double T_Y,double T_Z){
	//Update the current Pos (can be used to prevent repetitive writing)
	if (CurrentPos[0] != T_X || CurrentPos[1] != T_Y || CurrentPos[2] == T_Z){
		CurrentPos[0] = T_X;
		CurrentPos[1] = T_Y;
		CurrentPos[2] = T_Z;
    int Delays[CHANNEL_NUM] = {0};
    u32 Match[REG_NUM] = {0};
    //Calucalte the phase delays using the HLS block or SW
    if (UseHardware == 1){

    	CalculatePhaseHW(T_X,T_Y,T_Z,Delays);


    }else{
    	//Time1 = start_timer();
    	CalculatePhase(T_X,T_Y,T_Z,Delays);
    	//Time2 = stop_timer();
    	//Elapsed = get_microseconds(Time1,Time2);
    }

    for(int i=0; i< REG_NUM; i++)
    	Match[i] = getPhase(i,Delays);
    //Write to transducers
    UPAC_Write(Match);
	}

}


//Calculates the phase delays, but wont update the transducers
void GetTransducers(double T_X,double T_Y,double T_Z ,u32* Match){
	int Delays[CHANNEL_NUM] = {0};
    CalculatePhase(T_X,T_Y,T_Z,Delays);
    for(int i=0; i< REG_NUM; i++)
    	Match[i] = getPhase(i,Delays);
}


//Initialise the CalculatePhase HLS block
int hls_init(){
    XCalculatephase_Config *cfg;
    int Status =0;

    cfg = XCalculatephase_LookupConfig(XPAR_CALCULATEPHASE_0_DEVICE_ID);
    if (!cfg)
    	return XST_FAILURE;
    Status = XCalculatephase_CfgInitialize(&HLSPhaseCalc, cfg);
    if (Status != XST_SUCCESS)
    	return XST_FAILURE;
    Status = XCalculatephase_Initialize(&HLSPhaseCalc, XPAR_XCALCULATEPHASE_0_DEVICE_ID);
    return Status;

}


//Starts the calculation and enables the interrupt
void hls_start(void *InstancePtr){
	XCalculatephase *pCalculator = (XCalculatephase *)InstancePtr;
	XCalculatephase_InterruptEnable(pCalculator,1);
	XCalculatephase_InterruptGlobalEnable(pCalculator);
	XCalculatephase_Start(pCalculator);
}


//ISR that runs each time the HLS block finishes it's calculation
void hls_ISR(void *InstancePtr){
	XCalculatephase *pCalculator = (XCalculatephase *)InstancePtr;

	//Disable global int
	XCalculatephase_InterruptGlobalDisable(pCalculator);
	//Disable local int
	XCalculatephase_InterruptDisable(pCalculator, 0xFFFFFFFF);
	// Clear
	XCalculatephase_InterruptClear(pCalculator,1);
	//Calculation complete
	PhaseCalcDone = 1;
}


//Setup Interrupt controller and register hsl_ISR as the ISR
int hls_setup(){
	int result;
	XScuGic_Config *pCfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (pCfg == NULL)
		return XST_FAILURE;
	result = XScuGic_CfgInitialize(&ScuGic, pCfg, pCfg->CpuBaseAddress);
	if (result != XST_SUCCESS)
		return result;
	result = XScuGic_SelfTest(&ScuGic);
	if (result != XST_SUCCESS)
		return result;
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &ScuGic);
	Xil_ExceptionEnable();
	result = XScuGic_Connect(&ScuGic,XPAR_FABRIC_CALCULATEPHASE_0_INTERRUPT_INTR,(Xil_InterruptHandler)hls_ISR,&HLSPhaseCalc);
	if (result != XST_SUCCESS)
		return result;
	XScuGic_Enable(&ScuGic, XPAR_FABRIC_CALCULATEPHASE_0_INTERRUPT_INTR);
	return XST_SUCCESS;
}

// //Hardware version of CalculatePhase: Calculate the phase delays for a target position
void CalculatePhaseHW(float T_X, float T_Y, float T_Z, int *Output){
		int i=0, val=0;
		XCalculatephase_Set_T_Z(&HLSPhaseCalc, float_to_u32(T_Z));
		XCalculatephase_Set_T_X(&HLSPhaseCalc, float_to_u32(T_X));
		XCalculatephase_Set_T_Y(&HLSPhaseCalc, float_to_u32(T_Y));

		for(i=0; i< CHANNEL_NUM; i+=4){

			while(!XCalculatephase_IsReady(&HLSPhaseCalc));
			XCalculatephase_Set_Offset(&HLSPhaseCalc, (i));
			hls_start(&HLSPhaseCalc);
			while(!PhaseCalcDone);
			val = XCalculatephase_Get_return(&HLSPhaseCalc);
			Output[i] = (val & 0xFF);
			Output[i + 1] = ((val>>8) & 0xFF);
			Output[i + 2] = ((val>>16) & 0xFF);
			Output[i + 3] = ((val>>24) & 0xFF);
			PhaseCalcDone = 0;
		}
}


u32 GetTimerValue(u32 TimerIntrId,u16 Mode)
{
    int                 Status;
    XScuTimer_Config    *ConfigPtr;
    volatile u32     CntValue  = 0;
    XScuTimer           *TimerInstancePtr = &Timer;

    switch(Mode){

       // Initialize the Private Timer so that it is ready to use
       case 0:
    	   ConfigPtr = XScuTimer_LookupConfig(TimerIntrId);

    	   Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtr, ConfigPtr->BaseAddr);

    	   if (Status != XST_SUCCESS){
    	       return XST_FAILURE;
    	   }

    	   // Load the timer prescaler register.
    	   XScuTimer_SetPrescaler(TimerInstancePtr, 0);

    	   // Load the timer counter register.
    	   XScuTimer_LoadTimer(TimerInstancePtr, 0xFFFFFFFF);

    	   // Start the timer counter and read start value
    	   XScuTimer_Start(TimerInstancePtr);
    	   CntValue = XScuTimer_GetCounterValue(TimerInstancePtr);

           break;

       // Read stop value and stop the timer counter
       case 1:
           CntValue = XScuTimer_GetCounterValue(TimerInstancePtr);
           XScuTimer_Stop(TimerInstancePtr);
           break;

       case 2:
    	   CntValue = XScuTimer_GetCounterValue(TimerInstancePtr);
           break;

       default:
    	   CntValue = XScuTimer_GetCounterValue(TimerInstancePtr);
    }

    return CntValue;
}


u32 start_timer(){
	return GetTimerValue(XPAR_PS7_SCUTIMER_0_DEVICE_ID,0);
}


u32 stop_timer(){
	return GetTimerValue(XPAR_PS7_SCUTIMER_0_DEVICE_ID,1);
}


u32 read_timer(){
	return GetTimerValue(XPAR_PS7_SCUTIMER_0_DEVICE_ID,2);
}


u32 get_microseconds(u32 Start_Time, u32 Stop_Time){
	u32 temp = ((Start_Time - Stop_Time)/(666666687.0/2))*1000000.0;
	if (temp < 0){
		temp = 0;
	}
	return temp;
}

// Obtain two echo waves of 400 samples each
void GetEchoData(XAdcPs *XADCInstPtr, float SamplesVaux4[400], float SamplesVaux10[400]){

	//Voltage readings
	u32 Vaux4ADCData, Vaux10ADCData;
	u32 Vaux4RawData, Vaux10RawData;
	float Vaux4Data, Vaux10Data;


    for (int m = 0; m < 400; m++)
    {
  	    // Capture ADC Data
		Vaux4ADCData = XAdcPs_GetAdcData(XADCInstPtr, XADCPS_CH_AUX_MIN+4);
	    Vaux10ADCData = XAdcPs_GetAdcData(XADCInstPtr, XADCPS_CH_AUX_MIN+10);

	    // 2's complement performed for negative samples and made positive
	    if(Vaux4ADCData > 32767)
	    {
	        Vaux4RawData = Vaux4ADCData ^ 0xFFFF;
	        Vaux4RawData += 1;
	        Vaux4Data = (((float)(Vaux4RawData))*(-1.0f))/65536.0f;
	    }
	    else
	    {
	        Vaux4Data = (((float)(Vaux4ADCData))*(1.0f))/65536.0f;
	    }

	    SamplesVaux4[m] = Vaux4Data;

	    // 2's complement performed for negative samples and made positive
	    if(Vaux10ADCData > 32767)
	    {
	        Vaux10RawData = Vaux10ADCData ^ 0xFFFF;
	        Vaux10RawData += 1;
	        Vaux10Data = (((float)(Vaux10RawData))*(-1.0f))/65536.0f;
	    }
	    else
	    {
		    Vaux10Data = (((float)(Vaux10ADCData))*(1.0f))/65536.0f;
	    }

	    SamplesVaux10[m] = Vaux10Data;
    }
}

// Estimate position of levitated object based on statistical analysis using ADC sample number of echo peak values
int FindCoordinate(int count4, int count10, int coord)
{

	int max_count;

	if(count4 > count10){
		max_count = count4;
	}
	else{
		max_count = count10;
	}

    if(max_count <= 159){
    	printf("CurrentX: %i  Guessed options: 63,64,65,69  count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 164){
    	printf("CurrentX: %i  Guessed options: Undefined    count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 169){
    	printf("CurrentX: %i  Guessed options: 67,68,69     count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 174){
    	printf("CurrentX: %i  Guessed options: 64,67,68,69  count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 179){
    	printf("CurrentX: %i  Guessed options: Undefined    count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 184 ){
    	printf("CurrentX: %i  Guessed options: Undefined    count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 189){
    	printf("CurrentX: %i  Guessed options: 64,65,66,67  count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else if(max_count <= 194){
    	printf("CurrentX: %i  Guessed options: 64,65,68,69  count4: %i  count10: %i\n\r", coord, count4, count10);
    }
    else
    {
    	printf("CurrentX: %i  Guessed options: Undefined    count4: %i  count10: %i\n\r", coord, count4, count10);
    }

    return coord;
}

// UART interrupt to check if user has hit key and return to user main menu
int UartInterruptSet(INTC *IntcInstPtr, XUartPs *UartInstPtr, u16 DeviceId, u16 UartIntrId)
{
	int Status;
	XUartPs_Config *Config;
	u32 IntrMask;

	//Initialize the UART driver

	Config = XUartPs_LookupConfig(DeviceId);
	if (NULL == Config) {
		return XST_FAILURE;
	}

	Status = XUartPs_CfgInitialize(UartInstPtr, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//Check hardware build
	Status = XUartPs_SelfTest(UartInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//Connect the UART to the interrupt subsystem
	Status = SetupInterruptSystem(IntcInstPtr, UartInstPtr, UartIntrId);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//Setup the handlers for the UART
	XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)UartHandler, UartInstPtr);

	//Enable the interrupt of the UART

	IntrMask = XUARTPS_IXR_TOUT;

	XUartPs_SetInterruptMask(UartInstPtr, IntrMask);
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);

	//Set the receiver timeout
	//XUartPs_SetRecvTimeout(UartInstPtr, 100);

	return XST_SUCCESS;
}


void UartHandler(void *CallBackRef)
{
	XUartPs_Config *Config;

	Config = XUartPs_LookupConfig(UART_DEVICE_ID);

	if (XUartPs_IsReceiveData(Config->BaseAddress)) {
		return_menu = 1;
		menu_select = 0;
		setoptions = 0;
	}

}


static int SetupInterruptSystem(INTC *IntcInstancePtr, XUartPs *UartInstancePtr, u16 UartIntrId)
{
	int Status;

	XScuGic_Config *IntcConfig;

	// Initialize the interrupt controller driver
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig, IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	//Connect interrupt handler to hardware interrupt handling logic of processor
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler) XScuGic_InterruptHandler, IntcInstancePtr);

	// Connect device driver handler to be called when interrupt occurs
	Status = XScuGic_Connect(IntcInstancePtr, UartIntrId, (Xil_ExceptionHandler) XUartPs_InterruptHandler,(void *) UartInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Enable the interrupt for the device
	XScuGic_Enable(IntcInstancePtr, UartIntrId);

	//Enable interrupts
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}


// Initialize the temperature and humidity sensor driver
void PmodHYGROInit()
{
    HYGRO_begin(
    	&HygroSensor,
		0x43C40000,
    	0x40, // chip address of PmodHYGRO IIC
		0x43C30000,
    	0,
    	100000000 // the clock frequency of the AXI bus, used to convert timer data
	);
}


// Read the temperature and humidity of the environment
void PmodHYGROGetData()
{
	temp_degc = HYGRO_getTemperature(&HygroSensor);
	hum_perrh = HYGRO_getHumidity(&HygroSensor);
	usleep(5000);
	usleep(5000);
}


// Use step size and delay factor to move levitated object at specific speed
void Test_Speed(double step_size, int delay_val)
{

	u32 Start_Time, Stop_Time, Total_Time;
	double speed;
    int X;

    for (int i = 0; i < 2; i++)
    {
        X = 60;
        Total_Time = 0;

        for (int j = 0; j < 1000; j++)
        {
            SetTransducers(X,66,100);
            usleep(5000);
            usleep(5000);
        }

        while(X < 69)
        {
        	X = X + step_size;

            for (int j = 0; j < 1000; j++)
            {
                Start_Time = start_timer();
                SetTransducers(X,66,100);
            	usleep(delay_val);
            	Stop_Time = stop_timer();
            	Total_Time = Total_Time + get_microseconds(Start_Time, Stop_Time);
            }
        }

        while(X > 60)
        {
        	X = X - step_size;

            for (int j = 0; j < 1000; j++)
            {
                Start_Time = start_timer();
                SetTransducers(X,66,100);
            	usleep(delay_val);
            	Stop_Time = stop_timer();
            	Total_Time = Total_Time + get_microseconds(Start_Time, Stop_Time);
            }
        }

        speed = (18.0 / Total_Time) * 1000000.0;

        printf("Step: %0.3lf  Delay: %i  Speed: %0.3lf mm/s   Time: %lu us\n\r", step_size, delay_val, speed, Total_Time);

    }

}
