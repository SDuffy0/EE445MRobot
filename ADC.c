// ADCT0ATrigger.c
// Runs on LM4F120/TM4C123
// Provide a function that initializes Timer0A to trigger ADC
// SS3 conversions and request an interrupt when the conversion
// is complete.
// Daniel Valvano
// May 2, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
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
#include "ADC.h"
#include "OS.h"
#include "Filter.h"

#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable

#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAOTE         0x00000020  // GPTM TimerA Output Trigger
                                            // Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low

#define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
#define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
#define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
#define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
#define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select mask
#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer
#define ADC_SSPRI_SS3_4TH       0x00003000  // fourth priority
#define ADC_SSPRI_SS2_3RD       0x00000200  // third priority
#define ADC_SSPRI_SS1_2ND       0x00000010  // second priority
#define ADC_SSPRI_SS0_1ST       0x00000000  // first priority
#define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select
#define ADC_SSFIFO3_DATA_M      0x00000FFF  // Conversion Result Data mask
#define ADC_PC_SR_M             0x0000000F  // ADC Sample Rate
#define ADC_PC_SR_125K          0x00000001  // 125 ksps
#define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                            // Gating Control

#define NUM_SAMPLES 0x0

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// ADCSWTrigger.c
// Runs on LM4F120/TM4C123
// Provide functions that sample ADC inputs PE4, PE5 using SS2
// to be triggered by software and trigger two conversions,
// wait for them to finish, and return the two results.
// Daniel Valvano
// May 2, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
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


// There are many choices to make when using the ADC, and many
// different combinations of settings will all do basically the
// same thing.  For simplicity, this function makes some choices
// for you.  When calling this function, be sure that it does
// not conflict with any other software that may be running on
// the microcontroller.  Particularly, ADC0 sample sequencer 2
// is used here because it takes up to four samples, and two
// samples are needed.  Sample sequencer 2 generates a raw
// interrupt when the second conversion is complete, but it is
// not promoted to a controller interrupt.  Software triggers
// the ADC0 conversion and waits for the conversion to finish.
// If somewhat precise periodic measurements are required, the
// software trigger can occur in a periodic interrupt.  This
// approach has the advantage of being simple.  However, it does
// not guarantee real-time.
//
// A better approach would be to use a hardware timer to trigger
// the ADC conversion independently from software and generate
// an interrupt when the conversion is finished.  Then, the
// software can transfer the conversion result to memory and
// process it after all measurements are complete.
Sema4Type adcDataReady;
void ADC0_SS2_4Channels_TimerTriggered_Init(uint32_t period){
  volatile uint32_t delay;
  SYSCTL_RCGCADC_R |= 0x00000001; // 1) activate ADC0
  SYSCTL_RCGCTIMER_R |= 0x10;   // 4) activate timer4
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // 1) activate clocks
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;

  // Ain0 is on PE3, Ain1 is on PE2,  Ain2 is on PE1, Ain3 is on PE0
  GPIO_PORTE_DIR_R &= ~0x0F;  // 3.0) make PE3-0 inputs
  GPIO_PORTE_AFSEL_R |= 0x0F; // 4.0) enable alternate function on PE3-0
  GPIO_PORTE_DEN_R &= ~0x0F;  // 5.0) disable digital I/O on PE3-0
  GPIO_PORTE_AMSEL_R |= 0x0F; // 6.0) enable analog functionality on PE3-0

  ADC0_PC_R &= ~0xF;              // 8) clear max sample rate field
  ADC0_PC_R |= 0x1;               //    configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;          // 9) Sequencer 3 is lowest priority

	// TO DO: Check this to make sure that 1 is turned in
  // should change this to Timer2 in the future
  // Porbably need to also start SYSCTL clock for timer2?
  SYSCTL_RCGCADC_R |= 0x04;     // activate ADC0
  TIMER4_CTL_R = 0x00000000;    // disable timer4A during setup
  TIMER4_CTL_R |= 0x00000020;   // enable timer4A trigger to ADC
  TIMER4_CFG_R = 0;             // configure for 32-bit timer mode
  TIMER4_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  TIMER4_TAPR_R = 0;            // prescale value for trigger
  TIMER4_TAILR_R = period-1;    // start value for trigger
  TIMER4_IMR_R = 0x00000000;    // disable all interrupts
  TIMER4_CTL_R |= 0x00000001;   // enable timer4A 32-b, periodic, no interrupts
  ADC0_PC_R = 0x01;         // configure for 125K samples/sec
  ADC0_SSPRI_R = 0x3210;    // sequencer 0 is highest, sequencer 3 is lowest
  ADC0_ACTSS_R &= ~0x04;    // disable sample sequencer 2
  
	ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFF0000)+0x5555; // timer trigger event
  
  ADC0_SAC_R = ADC_SAC_AVG_64X;
  ADC0_SSMUX2_R = 0x0123;
	                     
  ADC0_SSCTL2_R = 0x6000;          // set flag and end                       
                 
  ADC0_IM_R |= 0x04;             // enable SS2 interrupts
  ADC0_ACTSS_R |= 0x04;              // enable sample sequencer 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFF00FF)|0x00004000; //priority 2
  NVIC_EN0_R = 1<<16;              // enable interrupt 16 in NVIC for SS2
	OS_InitSemaphore(&adcDataReady, 0);
}

uint32_t ADC2millimeter(uint32_t adcSample){
  if(adcSample<494) return 799; // maximum distance 80cm
  return (268130/(adcSample-159));  
}

uint16_t static ADCValues[NUM_IR];
AddFilter(0)
AddFilter(1)
AddFilter(2)
AddFilter(3)
void ADC0Seq2_Handler(void){
  ADC0_ISC_R = 0x0004;            	// acknowledge completion
	ADCValues[0] = ADC2millimeter(Filter0(ADC0_SSFIFO2_R&0xFFF));
	ADCValues[1] = ADC2millimeter(Filter1(ADC0_SSFIFO2_R&0xFFF));
	ADCValues[2] = ADC2millimeter(Filter2(ADC0_SSFIFO2_R&0xFFF));
	ADCValues[3] = ADC2millimeter(Filter3(ADC0_SSFIFO2_R&0xFFF));
	OS_bSignal(&adcDataReady);
}

void ADC_GetData(uint16_t data[NUM_IR]){
	OS_Wait(&adcDataReady);
	for(int i = 0; i<NUM_IR; i++){
		data[i] = ADCValues[i];
	}
}
