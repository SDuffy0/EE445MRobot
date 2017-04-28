#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "OS.h"
#include "SysTick.h"
#include "PLL.h"
#include "ST7735.h"
#include "WTimer0A.h"
#include "WTimer0B.h"
#include "WTimer5A.h"


#define PRISCHED 1

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
																						
																						
#define PF2             (*((volatile uint32_t *)0x40025010))
#define PF1             (*((volatile uint32_t *)0x40025008))																						

#define MAXFIFOSIZE 128
																						
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
long StartCritical(void);
void EndCritical(long primask);
void StartOS(void);

uint8_t NumThreads;
#define MAXNUMTHREADS 10
#define STACKSIZE 200

tcbType tcbs[MAXNUMTHREADS];
tcbType *RunPt;
tcbType *NextPt;
int32_t Stacks[MAXNUMTHREADS][STACKSIZE];
int16_t CurrentID;

tcbType *SleepListStart;
tcbType *SleepListEnd;
tcbType *LinkinPointer;
uint32_t OSMsCount;

void(*PeriodicTask1)(void);
void(*PeriodicTask2)(void);
void(*SW1Task)(void);
void(*SW2Task)(void);


//Adds thread to circular linked list of active threads
void LinkThread(tcbType *threadPt){
	long status = StartCritical();
	if(NumThreads == 0){
		threadPt->next = threadPt;
		threadPt->prev = threadPt;
		LinkinPointer = threadPt;
		RunPt = threadPt;
	}
	else{
		threadPt->next = LinkinPointer->next; // Insert thread into linked list
		LinkinPointer->next = threadPt;
		threadPt->prev = LinkinPointer;
		threadPt->next->prev = threadPt; //set prev for thread after current to current
	}
	NumThreads++;
	/*#ifdef PRISCHED
		if(threadPt->priority < RunPt->priority){
			OS_Suspend();										//switch to new thread if higher priority
		}
	#endif*/
	EndCritical(status);
}

//Removes thread from circular linked list of active threads
void UnlinkThread(tcbType *threadPt){
	long status = StartCritical();
	NumThreads--;
	if(NumThreads > 0){
		threadPt->prev->next = threadPt->next;	//if no threads left there is no need to change pointers
		threadPt->next->prev = threadPt->prev;
		LinkinPointer = threadPt->next;
		OS_SelectNextThread();
	}
	threadPt->next = 0;	//clear next for Sleeping and Blocking lists
	threadPt->prev = 0;	//clear prev for Sleeping and Blocking lists
	OS_SwitchThread();
	EndCritical(status);
}

//Appends to end of doubly-linked list
void PushToList(tcbType *threadPt, tcbType **listStartPt, tcbType **listEndPt){
	if(*listStartPt){
		(*listEndPt)->next = threadPt;
		threadPt->prev = *listEndPt;
		*listEndPt = threadPt;
	}
	else{
		*listStartPt = threadPt;
		*listEndPt = threadPt;
	}
}

//returns first element in list
tcbType* PopFromList(tcbType **listStartPt, tcbType **listEndPt){
	tcbType *threadPt = *listStartPt;
	*listStartPt = threadPt->next;
	if(*listStartPt == 0){
		*listEndPt = 0;
	}
	return threadPt;
}

//adds an element to an ordered doubly-linked list
void AddToPriorityList(tcbType *threadPt, tcbType **listStartPt, tcbType **listEndPt){
	if(*listStartPt){
		tcbType *curThread = *listStartPt;
		while(curThread->next && (curThread->priority <= threadPt->priority)){	//skip threads ordered by priority
			curThread = curThread->next;
		}
		if(curThread->priority > threadPt->priority){		//cur thread is lower priority than thread to be inserted
			threadPt->next = curThread;	
			threadPt->prev = curThread->prev;
			curThread->prev = threadPt;
			if(*listStartPt == curThread){								//check if inserting at start of the list
				*listStartPt = threadPt;	
			}
		}
		else{																						//arrived to end of list
			(*listEndPt)->next = threadPt;
			threadPt->prev = *listEndPt;
			*listEndPt = threadPt;
		}
	}
	else{																							//list is empty
		*listStartPt = threadPt;
		*listEndPt = threadPt;
	}	
}

//removes a given thread from a linked list
void RemoveFromList(tcbType *threadPt, tcbType **listStartPt, tcbType **listEndPt){
	if(threadPt->prev){
		threadPt->prev->next = threadPt->next;
	}
	else{
		*listStartPt = threadPt->next;
	}
	if(threadPt->next){
		threadPt->next->prev = threadPt->prev;
	}
	else{
		*listEndPt = threadPt->prev;
	}
}

long delay;
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= 0x20; // activate port F
  while((SYSCTL_PRGPIO_R&0x20)==0){}; // allow time for clock to start 
	delay = 0;
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTF_CR_R |= 0x11;
  GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF0, PF4
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void GPIOPortF_Handler(void){
	if(GPIO_PORTF_MIS_R&0x10){
		GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
		SW1Task();
	}
	if(GPIO_PORTF_MIS_R&0x01){
		GPIO_PORTF_ICR_R = 0x01;      // acknowledge flag4
		SW2Task();
	}
}

void DecrementSleep(void){
	//PF1 ^= 0x02;
	//PF1 ^= 0x02;
	OSMsCount += 1;
	tcbType *curThread = SleepListStart;
	while(curThread){
		tcbType *nextThread = curThread->next;
		curThread->sleep--;
		if(curThread->sleep == 0){
			RemoveFromList(curThread, &SleepListStart, &SleepListEnd);
			LinkThread(curThread);
		}
		curThread = nextThread;
	}
	
	//PF1 ^= 0x02;
}


// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void){
	DisableInterrupts();
  PLL_Init(Bus80MHz);         // set processor clock to 80 MHz
	Output_Init();
	WTimer5A_Init(&DecrementSleep, BUSCLK/1000, 1); //sleep priority 1
	PortF_Init();
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 SysTick
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // priority 7 PendSV
	CurrentID = 4;
	NumThreads = 0;
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	long status = StartCritical();
	semaPt->Value = value;
	semaPt->BlockedListEnd = 0;
	semaPt->BlockedListStart = 0;
	EndCritical(status);
}


void Block(tcbType *threadPt, Sema4Type *semaPt){
	UnlinkThread(threadPt);	//take tcb out of circular linked list
	#ifdef PRISCHED
		AddToPriorityList(threadPt, &(semaPt->BlockedListStart), &(semaPt->BlockedListEnd));
	#else
		PushToList(threadPt, &(semaPt->BlockedListStart), &(semaPt->BlockedListEnd));	//add tcb to end of blocked linked list
	#endif
}
// ******** OS_Wait ************
//decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt) {
  long status = StartCritical();
  (semaPt->Value) = (semaPt->Value) - 1;
  if (semaPt->Value < 0){
		Block(RunPt, semaPt);
  }
  EndCritical(status);
}

void WakeUp(Sema4Type *semaPt){
	LinkThread(PopFromList(&(semaPt->BlockedListStart), &(semaPt->BlockedListEnd)));
}
// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt) {
  long status = StartCritical();
  (semaPt->Value) = (semaPt->Value) + 1; //add 1 to semaphore
  if (semaPt->Value <= 0){
  	WakeUp(semaPt);
  }
  EndCritical(status);
}


// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt) {
  long status = StartCritical();
  (semaPt->Value) = (semaPt->Value) - 1;
  if (semaPt->Value < 0){
		Block(RunPt, semaPt);
  }
  EndCritical(status);
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
	long status = StartCritical();
  if (semaPt -> Value < 0){
		(semaPt->Value) = (semaPt->Value) + 1; //add 1 to semaphore
  	WakeUp(semaPt);
  }
  EndCritical(status);
}
		

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority){
	long status = StartCritical();
	tcbType *unusedThread;
	int numThread;
	for(numThread = 0; numThread < MAXNUMTHREADS; numThread++){
		if(tcbs[numThread].id == 0){
			break;
		}
	}
	if(numThread == MAXNUMTHREADS){
		EndCritical(status);
		return 0;
	}
	unusedThread = &tcbs[numThread];
	unusedThread->id = CurrentID;	//Current ID is incremented forever for different IDs
	unusedThread->priority = priority;
	unusedThread->sleep = 0;
	LinkThread(unusedThread);
	SetInitialStack(numThread);		//initialize stack
	Stacks[numThread][STACKSIZE-2] = (int32_t)(task); //  set PC for Task
	CurrentID+=1;
	EndCritical(status);
  return 1;
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void){
	return RunPt->id;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int NumPeriodicThreads = 0;
int OS_AddPeriodicThread(void(*task)(void),unsigned long period, unsigned long priority){
	long status = StartCritical();
	if(NumPeriodicThreads >= 2){
		EndCritical(status);
		return 0;
	}
	if (NumPeriodicThreads == 0){//Wide Timer0A
		WTimer0A_Init(task, period, priority);
	}
	else{	//Wide Timer0B
		WTimer0B_Init(task, period, priority);
	}
	NumPeriodicThreads++;
	EndCritical(status);
	return 1; 
}


//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority){
	SW1Task = task;
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority << 21); // (g) priority 2
	GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
	return 0;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority){
	SW2Task = task;
	NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|(priority << 21);
	GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF0
	return 0;
}



// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	long status = StartCritical();
	tcbType *sleepyThread = RunPt;
	sleepyThread->sleep = sleepTime;
	UnlinkThread(sleepyThread);
	PushToList(sleepyThread, &SleepListStart, &SleepListEnd);
	EndCritical(status);
	//OS_SwitchThread();
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	long status = StartCritical();
	RunPt->id = 0; //set id to dead
	UnlinkThread(RunPt);
	EndCritical(status);
	//OS_SwitchThread();		//send to graveyard	
}



// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void){
	NVIC_ST_CURRENT_R = 0;
	NVIC_INT_CTRL_R = 0x04000000; //Trigger SysTick
}


int Fifo[MAXFIFOSIZE];
int PutPt;
int GetPt;
int FifoSize;
int FifoNumElements;
Sema4Type DataRoomLeft;
Sema4Type DataAvailable;
Sema4Type FifoMutex;
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size){
	long sr = StartCritical();      // make atomic
	OS_InitSemaphore(&DataAvailable, 0);
  PutPt = 0; // Empty
	GetPt = 0; // Empty
	FifoSize = size;
	FifoNumElements = 0;
  EndCritical(sr);
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data){
	/*OS_Wait(&DataRoomLeft);
	OS_Wait(&FifoMutex);*/
	if(FifoNumElements == FifoSize){
		return 0;
	}
	long sr = StartCritical();
	Fifo[PutPt] = data;
	PutPt = (PutPt + 1) % FifoSize;
	FifoNumElements++;
	EndCritical(sr);
	//OS_Signal(&FifoMutex);
	OS_Signal(&DataAvailable);
	return 1;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void){
	OS_Wait(&DataAvailable);
	//OS_Wait(&FifoMutex);
	long sr = StartCritical();
	unsigned long data = Fifo[GetPt];
	GetPt = (GetPt + 1) % FifoSize;
	FifoNumElements--;
	EndCritical(sr);
	//OS_Signal(&FifoMutex);
	//OS_Signal(&DataRoomLeft);
	return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void){
	return DataAvailable.Value;
}

Sema4Type BoxFree;
Sema4Type DataValid;
unsigned long MailBox;

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void){
	OS_InitSemaphore(&BoxFree, 1);
	OS_InitSemaphore(&DataValid, 0);
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data){
	OS_Wait(&BoxFree);
	MailBox = data;
	OS_Signal(&DataValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void){
	OS_Wait(&DataValid);
	unsigned long data = MailBox;
	OS_Signal(&BoxFree);
	return data;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in ms units
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void){
	return WTIMER5_TAR_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in ms units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop){
	long difference = stop - start;
	if(difference < 0){
		difference += 0xFFFFFFFF;
	}
	return difference;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void){
	OSMsCount = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void){
	return OSMsCount;
}



//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice){
	NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
	StartOS();                   // start on the first task
}

void OS_SwitchThread(void){
	NVIC_INT_CTRL_R = 0x10000000; //Trigger PendSV
}

void OS_SelectNextThread(void){
	long status = StartCritical();
	if(RunPt->next){	//avoid selecting next thread if runpointer got unliked so RunPt->next is 0 and NextPt has been chosen
		#ifdef PRISCHED
			tcbType *maxPtr = RunPt->next;
			tcbType *curPtr = maxPtr;
			 // find maximum priority, assume first one is max and compare the rest against it
			for (int i = 0; i < NumThreads-1; i++){
				curPtr = curPtr->next;
				if (curPtr->priority < maxPtr->priority){
					maxPtr = curPtr;
				}
			}
			 // set next pointer to TCB with highest priority
			NextPt = maxPtr;
		#else
			NextPt = RunPt->next;
		#endif
	}
	EndCritical(status);
}
