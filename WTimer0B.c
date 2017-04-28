// WTimer0B.c
// Runs on LM4F120/TM4C123
// Use WTimer0B in periodic mode to request interrupts at a particular
// period.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015
   Volume 1, Program 9.8

  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Volume 2, Program 7.5, example 7.6

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "OS.h"


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*WTimer0BPeriodicTask)(void);   // user function


// ***************** WTimer0B_Init ****************
// Activate WTIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
// Outputs: none
void WTimer0B_Init(void(*task)(void), uint32_t period, uint32_t priority){long sr; //Used for periodic Task 1
	sr = StartCritical();
	SYSCTL_RCGCWTIMER_R |= 0x01;   //  activate WTIMER0
	WTimer0BPeriodicTask = task;          // user function
	WTIMER0_CTL_R = (WTIMER0_CTL_R&~0x00001F00);    // disable Wtimer0B during setup
 	WTIMER0_CFG_R = 0x00000004;    // configure for 32-bit timer mode
  WTIMER0_TBMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  WTIMER0_TBPR_R = 0;            // prescale value for trigger
	WTIMER0_ICR_R = 0x00000100;    // 6) clear WTIMER0B timeout flag
	WTIMER0_TBILR_R = (period)-1;    // start value for trigger
  WTIMER0_IMR_R = (WTIMER0_IMR_R&~0x00001F00)|0x00000100;    // enable timeout interrupts
	NVIC_PRI23_R = (NVIC_PRI23_R&0x00FFFFFF)| (priority << 29); //set priority  
	NVIC_EN2_R = 0x80000000;              // enable interrupt 95 in NVIC
	WTIMER0_CTL_R |= 0x00000100;   // enable wtimer0B 32-b, periodic
	EndCritical(sr);
}

void WideTimer0B_Handler(void){
	WTIMER0_ICR_R |= 0x0100; //acknowledge WTimer0B interrupt
  (*WTimer0BPeriodicTask)();                // execute user task
}
