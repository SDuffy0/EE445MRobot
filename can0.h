// *********Can0.h ***************
// Runs on LM4F120/TM4C123
// Use CAN0 to communicate on CAN bus
// CAN0Rx PE4 (8) I TTL CAN module 0 receive.
// CAN0Tx PE5 (8) O TTL CAN module 0 transmit.

// Jonathan Valvano
// May 2, 2015

/* This example accompanies the books
   Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers, Volume 3,  
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2015

   Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers, Volume 2
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

#ifndef __CAN0_H__
#define __CAN0_H__ 1
#define CAN_BITRATE             1000000

#define MSGLENGTH 8
#define NUM_SENSORBOARDS 1

#define IR_ID 0
#define NUM_IRMSGS 1
#define USONIC_ID (IR_ID+NUM_IRMSGS)
#define NUM_USONICMSGS 1

#define NUMMSGS (NUM_IRMSGS+NUM_USONICMSGS)

#ifdef SENSOR_BOARD_1
	#define SENSOR_BOARD_NUMBER 0
#endif
#ifdef SENSOR_BOARD_2
	#define SENSOR_BOARD_NUMBER 1
#endif

#define XMT_MSG_NUM SENSOR_BOARD_NUMBER*NUMMSGS + ID + 1


// Initialize CAN port
void CAN0_Open(void);

#ifdef MOTOR_BOARD
// if receive data is ready, gets the data 
// if no receive data is ready, it waits until it is ready
void CAN0_GetMail(uint8_t data[NUMMSGS*NUM_SENSORBOARDS][MSGLENGTH]);
#endif

#ifndef MOTOR_BOARD
// send 4 bytes of data to other microcontroller 
void CAN0_SendData(uint8_t data[MSGLENGTH], uint8_t ID);
#endif


#endif //  __CAN0_H__

