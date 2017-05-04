//*****************************************************************************
//
// Robot.c - user programs, File system, stream data onto disk
//*****************************************************************************

// Jonathan W. Valvano 3/7/17, valvano@mail.utexas.edu
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file to do Lab 4
// as long as the basic functionality is simular
// 1) runs on your Lab 2 or Lab 3
// 2) implements your own eFile.c system with no code pasted in from other sources
// 3) streams real-time data from robot onto disk
// 4) supports multiple file reads/writes
// 5) has an interpreter that demonstrates features
// 6) interactive with UART input, and switch input

// LED outputs to logic analyzer for OS profile 
// PF1 is preemptive thread switch
// PF2 is periodic task
// PF3 is SW1 task (touch PF4 button)

// Button inputs
// PF0 is SW2 task 
// PF4 is SW1 button input

// Analog inputs
// PE3 sequencer 3, channel 3, J8/PE0, sampling in DAS(), software start
// PE0 timer-triggered sampling, channel 0, J5/PE3, 50 Hz, processed by Producer
//******Sensor Board I/O*******************
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) connected to PB0
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// HC-SR04 Ultrasonic Range Finder 
// J9X  Trigger0 to PB7 output (10us pulse)
// J9X  Echo0    to PB6 T0CCP0
// J10X Trigger1 to PB5 output (10us pulse)
// J10X Echo1    to PB4 T1CCP0
// J11X Trigger2 to PB3 output (10us pulse)
// J11X Echo2    to PB2 T3CCP0
// J12X Trigger3 to PC5 output (10us pulse)
// J12X Echo3    to PF4 T2CCP0

// Ping))) Ultrasonic Range Finder 
// J9Y  Trigger/Echo0 to PB6 T0CCP0
// J10Y Trigger/Echo1 to PB4 T1CCP0
// J11Y Trigger/Echo2 to PB2 T3CCP0
// J12Y Trigger/Echo3 to PF4 T2CCP0

// IR distance sensors
// J5/A0/PE3
// J6/A1/PE2
// J7/A2/PE1
// J8/A3/PE0  

// ESP8266
// PB1 Reset
// PD6 Uart Rx <- Tx ESP8266
// PD7 Uart Tx -> Rx ESP8266

// Free pins (debugging)
// PF3, PF2, PF1 (color LED)
// PD3, PD2, PD1, PD0, PC4

#include <string.h> 
#include <stdio.h>
#include <stdint.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "ADC.h"
#include "can0.h"
#include "usonic.h"
#include "PWM.h"
#include "ST7735.h"
#include "Drive.h"
#include "arctan.h"

#define SENSORFREQ 100		//50 Hz data collection

#define PF0  (*((volatile unsigned long *)0x40025004))
#define PF1  (*((volatile unsigned long *)0x40025008))
#define PF2  (*((volatile unsigned long *)0x40025010))
#define PF3  (*((volatile unsigned long *)0x40025020))
#define PF4  (*((volatile unsigned long *)0x40025040))
  
#define PD0  (*((volatile unsigned long *)0x40007004))
#define PD1  (*((volatile unsigned long *)0x40007008))
#define PD2  (*((volatile unsigned long *)0x40007010))
#define PD3  (*((volatile unsigned long *)0x40007020))

uint8_t NumCreated = 0;

void PortD_Init(void){ 
  SYSCTL_RCGCGPIO_R |= 0x08;       // activate port D
  while((SYSCTL_PRGPIO_R&0x08)==0){};      
  GPIO_PORTD_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTD_AFSEL_R &= ~0x0F;   // disable alt funct on PD3-0
  GPIO_PORTD_DEN_R |= 0x0F;     // enable digital I/O on PD3-0
  GPIO_PORTD_PCTL_R = ~0x0000FFFF;
  GPIO_PORTD_AMSEL_R &= ~0x0F;;      // disable analog functionality on PD
}  
#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units
  
//************SW1Push*************
// Called when SW1 Button pushed
// background threads execute once and return
void SW1Push(void){

}

//************SW2Push*************
// Called when SW2 Button pushed
// background threads execute once and return
void SW2Push(void){

}



#ifndef MOTOR_BOARD
char* IRStrings[NUM_IR] = {"IR FR: ", "IR RF: ", "IR LF: ", "IR FL: "};
uint16_t ADCValues[NUM_IR];
void IRSend(void){
	uint8_t message[8];
	ADC0_SS2_4Channels_TimerTriggered_Init(BUSCLK/SENSORFREQ);	//20 Hz data collection
	while(1){
		//GPIO_PORTF_DATA_R ^= 0x04;
		ADC_GetData(ADCValues);
		// Message Format: MESSAGE_NUMBER, RESERVED, 6 bytes of ADC data
		for(int i=0; i<NUM_IR; i++){
			message[2*i] = (ADCValues[i]&0x00FF);
			message[2*i+1] = ((ADCValues[i]&0xFF00)>>8);
			ST7735_Message(DISPLAY_NUMBER_1, i, IRStrings[i], ADCValues[i]);
		}
		CAN0_SendData(message, IR_ID);
	}
}

char* USONICStrings[4] = {"US 0: ", "US 1: ", "US 2: ", "US 3: "};
uint32_t USONICValues[NUM_USONIC];
void USONICSend(void){
	uint8_t message[8];
	USONIC_Init();
	OS_AddPeriodicThread(&USONIC_Start, BUSCLK/SENSORFREQ, 0); //20 Hz data Collection
	while(1){
		//PF1 ^= 0x02;
		USONIC_GetData(USONICValues);
		
		for(int i=0; i<NUM_USONIC; i++){
			message[2*i] = (USONICValues[i]&0x00FF);
			message[2*i+1] = ((USONICValues[i]&0xFF00)>>8);
			ST7735_Message(DISPLAY_NUMBER_2, i, USONICStrings[i], USONICValues[i]);
			
		}
	  CAN0_SendData(message, USONIC_ID);
	}
}
#endif

#ifdef MOTOR_BOARD

uint16_t min(uint16_t x, uint16_t y){
	if(x<y) return x;
	return y;
}

/*
// Potential State Machine
struct RobotState {
  // doesn't make too much sense if it is just this simple, but 
  // the idea is we also have some additional logic in the main MotorController loop
  int8_t angle;
  int8_t speed;
}

RobotState[] RobotFSM = {
  RobotState StraightAway = {0, MAXSPEED * 2 / 3},
  RobotState TurnLeft = {45, MAXSPEED / 3},
  RobotState TurnRight = {-45, MAXSPEED / 3}
};



*/

#define LEFT90LOOPS 10
#define SPEED_TEST_LOOPS 20

#define MAX_NUM_NEXT_STATES 16
#define NUM_STATES 12

#define STRAIGHT 0
#define STRAIGHT_NL 1
#define STRAIGHT_NR 2
#define LEFT90 3
#define LEFT60 4
#define LEFT45 5
#define LEFT30 6
#define RIGHT30 7
#define RIGHT45 8
#define RIGHT60 9
#define RIGHT90 10
#define WAIT 11

uint8_t StraightNS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = STRAIGHT;
	if((FR<200) && (FL<200)){                                       // very close in front
    if((LF<200) && (RF<200) && (LS<200) && (RS<200)){
      nextState = WAIT;
		} else if (((LA + LS +LF)/3) > ((RA + RS+RF)/3)){
			nextState = LEFT90;
		} else {
		  nextState = RIGHT90;
		}
	}  else if((FR<200) || (FL<200)){                               // very close on one side of the front
		if((FR< 200) && (FL > 275) && (LA > 350) && (LF > 350)){
			nextState = LEFT60;
		} else if((FL< 200) && (FR > 275) && (RA > 250) && (RF >350)){
			nextState = RIGHT60;
		} else if (((LA + LS +LF)/3) > ((RA + RS+RF)/3)){
			nextState = LEFT90;
		} else {
		  nextState = RIGHT90;
		}
  } else if((RA <200) || (LA < 200)){                             // very close on angle sensor, but not front
		if((RA <100) && (LA < 100)){
			nextState = WAIT;
	  } else if( RA < 70){
			nextState = LEFT60;
		} else if (LA < 70){
			nextState = RIGHT60;
		} else if (RA <200){
			nextState = LEFT30;
		} else if (LA < 200){
			nextState = RIGHT30;
		}
	} else if((FR<350) && (FL<350)){  								              // sorta close in front
    if((LF<200) && (RF<200) && (LS<200) && (RS<200)){
      nextState = WAIT;
		} else if ((RA>LA) && (RA > 400)){
			nextState = RIGHT45; 
		} else if ((LA>RA) && (LA > 400)){
			nextState = LEFT45;
		} else if ((RF>LF) && (RS>LS)&& (RA > 400)){
		  nextState = RIGHT90;	
		} else if ((LF>RF) && (LS>RS)&& (LA > 400)){
			nextState = LEFT90;
		} else if ( RS < LS){
			nextState = LEFT90;
		} else {
			nextState = RIGHT90;
		}
  } else if((LF < 200) || (LS < 200)){			                      // close to someting on left
		nextState = RIGHT30;
	} else if ((RF < 200) || (RS < 200) ){		                      // close to something on right
	  nextState = LEFT30;
	} else if ((FR > 350) && (FL > 350) && (RA > 400) && (LA > 400)){// open road ahead
	  nextState = STRAIGHT;
  } else if ((RA > LA ) &&  (RA > 500)){ // consider removing 
    nextState = STRAIGHT;//RIGHT45; //45R
	} else if ((LA>RA) && (LA > 500)){     // consider removing
	  nextState = STRAIGHT;//LEFT45; // 45L
	}
	return nextState;
}


uint8_t StraightNoLeftNS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = STRAIGHT;
	if((LF < 300) && (LS < 300)){ 
		return STRAIGHT;   
	} else if((LF < 200) || (LS < 200)){			// close to someting on left
		nextState = LEFT30;
	}
	return nextState;
}

uint8_t StraightNoRightNS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = STRAIGHT;	
	if((RF < 300) && (RS < 300)){ 
		return STRAIGHT;   
	} else if((LF < 200) || (LS < 200)){			// close to someting on left
		nextState = RIGHT45;
	}	
	return nextState;
}

uint8_t Left90NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = LEFT90;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	} else if (LA > 300){
		nextState = LEFT60;
	}
  return nextState;
}

uint8_t Left60NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = LEFT60;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	} else if ((LA > 300) && (LF > 300) && (LS > 200)){
		nextState = LEFT60;
	}
  return nextState;
}

uint8_t Left45NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
 	uint16_t nextState = LEFT45;
	//if(
	if((LA > FL) && (LA > FR) && (LA > RA)){
		nextState = LEFT45;
	} else	if((FL > 350) && (FR > 350)){
		nextState = STRAIGHT;
	}
  return nextState;
}

uint8_t Right45NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = RIGHT45;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	}
  return nextState;
}

uint8_t Right60NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = RIGHT60;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	}
  return nextState;
}

uint8_t Right90NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = RIGHT90;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	}  
	return nextState;
}

uint8_t Left30NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = LEFT30;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	}  
	return nextState;
}

uint8_t Right30NS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = RIGHT30;
	if(FL > 350 && FR > 350){
		nextState = STRAIGHT;
	}
  return nextState;
}

uint8_t WaitNS(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	uint16_t nextState = STRAIGHT;
	if((LF<200) && (RF<200) && (FR<200) && (FL<200) && (LS<200) && (RS<200) && (RA<200) && (LA<200)){
	  nextState = WAIT;
	} else if ((FR > 300) && (FL > 300)) {
		nextState = STRAIGHT;
	} else if ((RF > 300) && (RS > 300)){
		nextState = RIGHT90;
	} else if ((LF > 300) && (LS > 300)){
		nextState = LEFT90;
	}
  return nextState;
}

typedef struct State {
	int8_t speed;
	int8_t angle;
	uint8_t time; 									// may or may not be needed
	char* description;							// will be used for debugging
	uint8_t (*findNextState)(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t); // should use the sensor inputs to determine the next state
} State;

State FSM[NUM_STATES] = {
	{MAXSPEED*2/3, 0, 1, "Straight", StraightNS},  // State 0
	{MAXSPEED*2/3, 0, 1, "Straight No Left", StraightNS},  // State 1
	{MAXSPEED*2/3, 0, 1, "Straight No Right", StraightNS},  // State 2
	{MAXSPEED*2/3, -90, 1, "-90", StraightNS},  // State 3
	{MAXSPEED*2/3, -60, 1, "-60", StraightNS},  // State 4
	{MAXSPEED*2/3, -45, 1, "-45", StraightNS},  // State 5
	{MAXSPEED*2/3, -30, 1, "-30", StraightNS},  // State 6
	{MAXSPEED*2/3, 30, 1, "30", StraightNS},  // State 7
	{MAXSPEED*2/3, 45, 1, "45", StraightNS},  // State 8 
	{MAXSPEED*2/3, 60, 1, "60", StraightNS},  // State 9
	{MAXSPEED*2/3, 90, 1, "90", StraightNS},  // State 10
	{0       ,  0, 1, "Wait", StraightNS},   // State 11
};

typedef struct RefState {
	uint8_t (*stateRef)(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
} RefState;

// not sure if brackets necessary here, but might as well
RefState RefFSM[NUM_STATES] =
{
	{StraightNS},
	{StraightNoLeftNS},
	{StraightNoRightNS},
	{Left90NS},
	{Left60NS},
	{Left45NS},
	{Left30NS},
	{Right30NS},
	{Right45NS},
	{Right60NS},
	{Right90NS},
	{WaitNS}
};

void RunFSM(uint16_t LF, uint16_t FR, uint16_t FL, uint16_t RF, uint16_t LS, uint16_t RS, uint16_t RA, uint16_t LA){
	static uint8_t currentStateNumber;
	currentStateNumber = FSM[currentStateNumber].findNextState(LF, FR, FL, RF, LS, RS, RA, LA); // go to next state
	// FSM[currentStateNumber].findNextState = RefFSM[currentStateNumber].stateRef; // if we want to use other states
	if(currentStateNumber != STRAIGHT) Drive(FSM[currentStateNumber].speed, FSM[currentStateNumber].angle);				// update drive
	else Drive_PIControlDirection(min(LF, LS) - min(RF, RS));
}

uint8_t SensorData[NUMMSGS*NUM_SENSORBOARDS][MSGLENGTH];
uint8_t IRData[NUMMSGS*NUM_SENSORBOARDS][MSGLENGTH];
void MotorController(void){
	uint8_t debug = 0, max_speed_test = 0, turn_test = 1; // change to 0 to run FSM
	uint16_t counter = 0;
	Drive_Init();
	while(1){
			CAN0_GetMail(SensorData);
			uint16_t IRF, IFR, IFL, ILF, URM, URF, ULF, ULM;
			int32_t angle, speed;
//			uint16_t rightMin, leftMin, frontMin;
			IFR = ((SensorData[0][1]&0x00FF)<<8) + SensorData[0][0];
			IRF = ((SensorData[0][3]&0x00FF)<<8) + SensorData[0][2];
			ILF = ((SensorData[0][5]&0x00FF)<<8) + SensorData[0][4];
			IFL = ((SensorData[0][7]&0x00FF)<<8) + SensorData[0][6];
			URM = ((SensorData[1][1]&0x00FF)<<8) + SensorData[1][0];
			URF = ((SensorData[1][3]&0x00FF)<<8) + SensorData[1][2];
			ULF = ((SensorData[1][5]&0x00FF)<<8) + SensorData[1][4];
		  ULM = ((SensorData[1][7]&0x00FF)<<8) + SensorData[1][6];
			/*speed = MAXSPEED/2;
			if( RF < 150){
				angle = -45;
			}
			else if( LF < 150){
				angle = 45;
			}
			else if(FR <400 && FL < 400){
				if(RF > LF){
					angle = 90;
				}
				else{
					angle = -90;
				}
			}
			else if(FR > FL + 100){
				if(RF > 200){
					angle = 45;
				}
				else{
					angle = 0;
				}
			}
			else if(FL > FR + 100){
				if(LF > 200){
					angle = -45;
				}
				else{
					angle = 0;
				}
			}
			else{
				angle = 0;
			}
			leftMin = min(FL, LF);
			rightMin = min(FR, RF);
			frontMin = min(F, min(FL, FR));
			if (leftMin < rightMin){
        angle = 45;
			}
			else{
        angle = -45 ;
			}				
			if (frontMin < 100){
        speed *= -1;
			}

			if (speed < 0){
        angle *= -1;
      }
			Drive(speed, angle);*/
			if(debug){
				// test turn
				if(turn_test){
					// turn 90 degrees left
					if(counter >= SPEED_TEST_LOOPS){
						Drive_Stop();
					}else{
						Drive( FSM[LEFT90].speed, FSM[LEFT90].angle); // set up some measuring sticks?
						counter++;
					}
					// turn 90 degrees right
				}else if(max_speed_test){
					  if(counter >= SPEED_TEST_LOOPS){
							Drive_Stop();
						}else{
							Drive(MAXSPEED, 0); // set up some measuring sticks?
						  counter++;
						}
				}
				// test max speed
			}else{
				RunFSM(ILF, IFR, IFL, IRF, ULM, URM, URF, ULF);
			}
			/*if(IFR < 100){
				Drive(MAXSPEED/2, -45);
			}
			else if(IFL < 100){
				Drive(MAXSPEED/2, 45);
			}
			else{
			int16_t C1 = 707;
			int16_t C2 = -707;
			int32_t b = URM*1000;
			int32_t a = URF;
			angle = 90 - arctan(b + a*C2, a*C1);
			Drive_PIControlDirection(angle);
			}*/
		
	}
}

void Stop(void){
	OS_Sleep(180000);
	Drive_Stop();
}

#endif

//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
unsigned long Idlecount=0;
void IdleTask(void){ 
  while(1) { 
    Idlecount++;        // debugging 
  }
}


// NOTE: Define MOTOR_BOARD
int main(void){        // lab 4 real main
	OS_Init();           // initialize, disable interrupts
  PortD_Init();  // user debugging profile
	
	
	
//********initialize communication channels
	CAN0_Open();  

//*******attach background tasks***********
  
	
	
	NumCreated += OS_AddThread(&IdleTask,128,7);  // runs when nothing useful to do
	
	
	#ifdef MOTOR_BOARD
		OS_AddSW1Task(&SW1Push,2);    // PF4
		OS_AddSW2Task(&SW2Push,3);   // PF0
		NumCreated += OS_AddThread(&Stop, 128, 0);
		NumCreated += OS_AddThread(&MotorController, 128, 1);
	#else
		Output_Init();
		NumCreated += OS_AddThread(&USONICSend, 128, 1);
		NumCreated += OS_AddThread(&IRSend, 128, 1);	
	#endif

  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}
