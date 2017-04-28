// filename **********TCB.H***********
// Real Time Operating System for Labs 2 and 3 
// Thomas C. Sweeney, Sean Duffy 2/2/17
// EE445M/EE380L.6 
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file


#include "stdint.h"

// struct from program 3.3 in the book
#define MAX_NUM_THREADS 10 // maximum 
#define STACKSIZE 100

typedef enum TSE{
	ACTIVE,
//	RUNNING,
	BLOCKED,
	SLEEPING,
	DEAD,
} THREAD_STATUS_ENUM;


typedef struct tcb tcbType;

struct tcb {
  int32_t *sp; // pointer to stack, valid for threads not running
  tcbType* next; // 
	tcbType* last;
  THREAD_STATUS_ENUM status; // showing resources that this thread needs or wants
  uint32_t sleep_counter; // implements sleep mode
  uint32_t threadNumber; // thread number, type, or name
  uint32_t age; // how long this thread has been active
  uint8_t priority; // not used in round robin scheduler
	Sema4Type* blockPt;
};

//typedef enum PTSE{
//	PTSE_ACTIVE,
//	PTSE_BLOCKED,
//	PTSE_SLEEPING,
//	PTSE_DEAD,
//} PERIODIDC_THREAD_STATUS_ENUM;
//	
//typedef struct{
//	void(*task)(void);
//	uint32_t priority;
//	uint32_t period;
//	PERIODIDC_THREAD_STATUS_ENUM status;
//	uint32_t lastTime;
//	uint32_t MaxJitter;
//} PTCBType;

#define MAX_NUM_PERIODIC_THREADS 2

