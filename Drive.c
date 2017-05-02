// Drive.c
// Runs on TM4C123

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "PWM.h"
#include "Drive.h"

#define POWERMIN 400
#define POWERMAX 12100
#define SERVOMID 1875

uint16_t power, direction;
uint8_t stopped = 0;

void Drive_Init(void){
	direction = 0; //going straight
	power = POWERMIN;
	Left_InitB(12500, power, direction);          // initialize PWM0, 100 Hz
  Right_Init(12500, power, direction);   // initialize PWM0, 100 Hz
  Servo_Init(25000, SERVOMID);
}



//takes in an angle from -45 to 45 degrees, changes servo PWM Duty cycle
void Drive_WheelDirection(int8_t dir){
	int8_t newDir;
	uint16_t duty;
	if(dir < -45){
		newDir = -45;
	}
	else if(dir > 45){
		newDir = 45;
	}
	else{
		newDir = dir;
	}
	duty = ((45  - newDir) * 125)/9 + 1250;
	Servo_Duty(duty);
}

void Drive_DifferentialTurn(int8_t dir){	
	Drive_WheelDirection(dir);
	if(dir < 0){
		Left_DutyB(power/2, direction);
	}
	else{
		Right_Duty(power/2,direction);
	}
	
}

void Drive_SteepDifferentialTurn(int8_t dir){
	Drive_WheelDirection(dir);
	if(dir < 0){
		Left_DutyB(power/2, 1 - direction);
	}
	else{
		Right_Duty(power/2, 1 - direction);
	}
}

//takes in a speed from -MAXSPEED to MAXSPEED
void Drive_Speed(int8_t speed){
	int8_t newSpeed;
	uint32_t newDuty;
	if(speed < 0){
		direction = 1;
		newSpeed = speed*-1;
	}
	else{
		direction = 0;
		newSpeed = speed;
	}
	if(newSpeed > MAXSPEED){
		newSpeed = MAXSPEED;
	}
	newDuty = newSpeed*POWERMAX/MAXSPEED;
	if(newDuty < POWERMIN){
		newDuty = POWERMIN;
	}
	if(newDuty > POWERMAX){
		newDuty = POWERMAX;
	}
	power = newDuty;
	Left_DutyB(newDuty, direction);
	Right_Duty(newDuty, direction);
	
}


#define IMAX  11000
#define IMIN	-500

#define UMAX POWERMAX
#define UMIN POWERMAX/4

#define EKIdT  (Error*200)/100
#define EKP    (Error*200)/10 
#define MDIF 1000

void Drive_PIControlDirection(int32_t Error){
	static int32_t IL = POWERMAX;
	static int32_t IR = POWERMAX;
	IL -= EKIdT;
	IR += EKIdT;
	if(IL<IMIN) IL = IMIN;
	if(IL>IMAX) IL = IMAX;
	if(IR<IMIN) IR = IMIN;
	if(IR>IMAX) IR = IMAX;
	
	int32_t UL = IL - EKP;
	int32_t UR = IR + EKP;
	
	/*uint8_t dirL = UL < 0;
	uint8_t dirR = UR < 0;
	
	if(dirL) UL*=-1;
	if(dirR) UR*=-1;*/
	
	if(UL<UMIN) UL = UMIN;
	if(UR<UMIN) UR = UMIN;
	if(UL>UMAX) UL = UMAX;
	if(UR>UMAX) UR = UMAX;
	
	/*Left_Duty(UL, dirL);
	Right_DutyB(UR, dirR);*/
	if(!stopped){
	Left_DutyB(UL, 0);
	Right_Duty(UR, 0);
	}
}

void Drive(int8_t speed, int8_t dir){
	if(!stopped){
	Drive_Speed(speed);
	if(dir > 45 || dir < -45){
		Drive_SteepDifferentialTurn(dir);
	}
	else{
		Drive_DifferentialTurn(dir);
	}
}
}

void Drive_Stop(void){
	Left_DutyB(POWERMIN, 0);
	Right_Duty(POWERMIN, 0);
	stopped = 1;
}

