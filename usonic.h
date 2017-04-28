// Timer.h
// Runs on TM4C123
// Provide pulse measurement functions for Timer0, Timer1, Timer2 and Timer3,

// Daniel Valvano, Jonathan Valvano
// January 19, 2017 

/* This example accompanies the books
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2016
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers", 
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2017
 Copyright 2017 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
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


#ifndef __USONIC_H
#define __USONIC_H  1


#define NUM_HCSR04 2
#define NUM_PING 1

#define NUM_USONIC (NUM_HCSR04 + NUM_PING)


// Subroutine to wait 10 usec
// Inputs: None
// Outputs: None
// Notes: ...
void DelayWait10us(void);


//------------USONIC_Init------------
// Initialize Timer3A in edge time mode to request interrupts on
// the both edges of PB2 (T3CCP0).  The interrupt service routine
// acknowledges the interrupt records the time.
// PB3 GPIO output
// Input: none
// Output: none
void USONIC_Init(void);

//------------Timer3_StartHCSR04------------
// start HCSR04 ultrasonic distance measurement
// 10us Pulse output on PB3 GPIO output
// Input: none
// Output: none
void USONIC_StartHCSR04(void);

//------------Timer3_StartPing------------
// start Ping))) ultrasonic distance measurement
// 1) Make PB2 GPIO output
// 2) 5us Pulse output on PB2 GPIO output
// 3) Make PB2 input capture Timer input
// Input: none
// Output: none
void USONIC_StartPing(void);

void USONIC_Start(void);

//------------Timer3_Read------------
// read ultrasonic distance measurement
// Input: none
// Output: 0 if not ready, pulse width in 12.5ns time if ready
uint32_t USONIC_Read(void); 

//------------Cycles2milliInch------------
// convert time in bus cycles to distance in 0.001 in
// speed of sound is 13,560 in/sec
// Input: cycles 
// Output: distance in 0.001in
uint32_t Cycles2milliInch(uint32_t cycles);

//------------Cycles2millimeter------------
// convert time in bus cycles to distance in mm
// speed of sound is 340.29 m/sec
// Input: cycles 
// Output: distance in mm
uint32_t Cycles2millimeter(uint32_t cycles); 

void USONIC_GetData(uint32_t data[NUM_USONIC]);

#endif
