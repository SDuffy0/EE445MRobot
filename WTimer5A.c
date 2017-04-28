// WTimer5A.c
// Runs on LM4F120/TM4C123
// Use WTimer5A in periodic mode to request interrupts at a particular
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
void (*WTimer5APeriodicTask)(void);   // user function


// ***************** WTimer0A_Init ****************
// Activate WTIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
// Outputs: none
void WTimer5A_Init(void(*task)(void), uint32_t period, uint32_t priority){long sr; //Used for periodic Task 1
	sr = StartCritical();
	SYSCTL_RCGCWTIMER_R |= 0x20;   //  activate WTIMER5
	WTimer5APeriodicTask = task;          // user function
	WTIMER5_CTL_R = (WTIMER5_CTL_R&~0x0000001F);    // disable Wtimer0A during setup
  WTIMER5_CFG_R = 0x00000004;             // configure for 64-bit timer mode
  WTIMER5_TAMR_R = 0x00000012;   // configure for periodic mode, count-up
  WTIMER5_TAPR_R = 0;            // prescale value for trigger
	WTIMER5_ICR_R = 0x00000001;    // 6) clear WTIMER5A timeout flag
	WTIMER5_TAILR_R = (period)-1;    // start value for trigger
  WTIMER5_IMR_R = (WTIMER5_IMR_R&~0x0000001F)|0x00000001;    // enable timeout interrupts
	NVIC_PRI26_R = (NVIC_PRI26_R&0xFFFFFF00)|0x00000000; // 8) priority 0
  NVIC_EN3_R = 0x00000100;        // 9) enable interrupt 19 in NVIC
	WTIMER5_CTL_R |= 0x00000001;   // enable Wtimer5A 32-b, periodic
	EndCritical(sr);
}

void WideTimer5A_Handler(void){
	WTIMER5_ICR_R |= 0x01; // acknowledge wtimer5A timeout
  (*WTimer5APeriodicTask)();                // execute user task
}
